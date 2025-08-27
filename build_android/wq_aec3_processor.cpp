#include "wq_aec3_processor.h"

#include <algorithm>
#include <cstring>

// WebRTC includes
#include "audio_processing/audio_frame.h"

namespace webrtc_aec3_tts {

WqAec3Processor::WqAec3Processor() 
    : initialized_(false),
      stream_config_(kSampleRateHz, kChannels, false) {
}

WqAec3Processor::~WqAec3Processor() {
    Destroy();
}

bool WqAec3Processor::Initialize() {
    if (initialized_) {
        return true;
    }

    try {
        // Initialize AEC3 configuration with default settings optimized for production
        aec3_config_ = webrtc::EchoCanceller3Config();
        
        // Optimize for TTS echo cancellation - production-grade ERLE settings
        aec3_config_.filter.export_linear_aec_output = true;
        aec3_config_.filter.enable_shadow_filter_output_usage = true;
        aec3_config_.filter.conservative_initial_phase = false;
        aec3_config_.filter.initial_state_seconds = 1.5f;  // Faster initial convergence
        
        // Enhanced delay estimation for better ERLE
        aec3_config_.delay.default_delay = 0;  // Start with 0, let AEC3 auto-detect
        aec3_config_.delay.delay_estimate_smoothing = 0.6f;  // More aggressive tracking
        aec3_config_.delay.delay_candidate_detection_threshold = 0.15f;  // More sensitive detection
        aec3_config_.delay.delay_selection_thresholds.initial = 12;  // Faster initial detection
        aec3_config_.delay.delay_selection_thresholds.converged = 2;   // Maintain convergence
        
        // ERLE settings for better echo suppression
        aec3_config_.erle.max_l = 8.0f;  // Allow higher ERLE in low frequencies
        aec3_config_.erle.max_h = 4.0f;  // Allow higher ERLE in high frequencies
        aec3_config_.erle.onset_detection = true;
        
        // Suppressor tuning for aggressive echo suppression
        aec3_config_.suppressor.normal_tuning.mask_lf.enr_transparent = 0.3f;
        aec3_config_.suppressor.normal_tuning.mask_lf.enr_suppress = 0.25f;  // More aggressive
        aec3_config_.suppressor.normal_tuning.mask_hf.enr_transparent = 0.07f;
        aec3_config_.suppressor.normal_tuning.mask_hf.enr_suppress = 0.08f;  // More aggressive
        aec3_config_.suppressor.normal_tuning.max_inc_factor = 2.5f;
        aec3_config_.suppressor.normal_tuning.max_dec_factor_lf = 0.2f;
        
        // Initialize audio processing configuration for 48kHz
        // Note: Using EchoControl directly, no need for AudioProcessing::Config
        // Direct EchoControl usage - no AudioProcessing config needed

        // Create AEC3 factory and echo controller
        aec3_factory_ = std::make_unique<webrtc::EchoCanceller3Factory>(aec3_config_);
        echo_controller_ = aec3_factory_->Create(kSampleRateHz, kChannels, kChannels);
        
        if (!echo_controller_) {
            return false;
        }

        // Create high-pass filter
        high_pass_filter_ = std::make_unique<webrtc::HighPassFilter>(kSampleRateHz, kChannels);

        // Initialize audio buffers
        if (!InitializeAudioBuffers()) {
            return false;
        }

        initialized_ = true;
        return true;

    } catch (const std::exception& e) {
        return false;
    }
}

void WqAec3Processor::Destroy() {
    if (!initialized_) {
        return;
    }

    echo_controller_.reset();
    aec3_factory_.reset();
    high_pass_filter_.reset();
    
    render_audio_buffer_.reset();
    capture_audio_buffer_.reset();
    linear_output_buffer_.reset();
    
    ClearCleanAudioBuffer();
    
    initialized_ = false;
}

bool WqAec3Processor::InitializeAudioBuffers() {
    try {
        // Create audio buffers following the demo.cc pattern
        render_audio_buffer_ = std::make_unique<webrtc::AudioBuffer>(
            stream_config_.sample_rate_hz(), stream_config_.num_channels(),
            stream_config_.sample_rate_hz(), stream_config_.num_channels(),
            stream_config_.sample_rate_hz(), stream_config_.num_channels());

        capture_audio_buffer_ = std::make_unique<webrtc::AudioBuffer>(
            stream_config_.sample_rate_hz(), stream_config_.num_channels(),
            stream_config_.sample_rate_hz(), stream_config_.num_channels(),
            stream_config_.sample_rate_hz(), stream_config_.num_channels());

        // Linear output buffer for enhanced processing
        constexpr int kLinearOutputRateHz = 16000;
        linear_output_buffer_ = std::make_unique<webrtc::AudioBuffer>(
            kLinearOutputRateHz, stream_config_.num_channels(),
            kLinearOutputRateHz, stream_config_.num_channels(),
            kLinearOutputRateHz, stream_config_.num_channels());

        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool WqAec3Processor::ProcessRenderAudio(const int16_t* render_data, size_t length) {
    if (!initialized_ || !render_data || length != kFrameSize) {
        return false;
    }

    try {
        // Create AudioFrame from render data
        webrtc::AudioFrame render_frame;
        render_frame.UpdateFrame(0, render_data, length, kSampleRateHz, 
                                webrtc::AudioFrame::kNormalSpeech, 
                                webrtc::AudioFrame::kVadActive, kChannels);

        // Copy to audio buffer
        render_audio_buffer_->CopyFrom(&render_frame);
        
        // Process render signal (reference signal for echo cancellation)
        render_audio_buffer_->SplitIntoFrequencyBands();
        echo_controller_->AnalyzeRender(render_audio_buffer_.get());
        render_audio_buffer_->MergeFrequencyBands();

        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool WqAec3Processor::ProcessCaptureAudio(const int16_t* capture_data, int16_t* output_data, size_t length) {
    if (!initialized_ || !capture_data || !output_data || length != kFrameSize) {
        return false;
    }

    try {
        // Create AudioFrame from capture data
        webrtc::AudioFrame capture_frame;
        capture_frame.UpdateFrame(0, capture_data, length, kSampleRateHz,
                                 webrtc::AudioFrame::kNormalSpeech,
                                 webrtc::AudioFrame::kVadActive, kChannels);

        // Copy to audio buffer
        capture_audio_buffer_->CopyFrom(&capture_frame);

        // Analyze capture signal
        echo_controller_->AnalyzeCapture(capture_audio_buffer_.get());

        // Process capture signal (remove echo)
        capture_audio_buffer_->SplitIntoFrequencyBands();
        
        // Apply high-pass filter
        high_pass_filter_->Process(capture_audio_buffer_.get(), true);
        
        // CRITICAL: Set audio buffer delay (let AEC3 auto-detect optimal delay)
        // Don't force to 0 - let the delay estimator work properly
        // echo_controller_->SetAudioBufferDelay(0);
        
        // Apply echo cancellation
        echo_controller_->ProcessCapture(capture_audio_buffer_.get(), 
                                       linear_output_buffer_.get(), false);
        
        capture_audio_buffer_->MergeFrequencyBands();

        // Copy processed audio back to output
        capture_audio_buffer_->CopyTo(&capture_frame);
        std::memcpy(output_data, capture_frame.data(), length * sizeof(int16_t));

        // Store clean audio for later retrieval (convert int16 to float with proper normalization)
        std::vector<float> float_frame(length);
        for (size_t i = 0; i < length; ++i) {
            // Proper int16 to float conversion with full dynamic range preservation
            float_frame[i] = static_cast<float>(output_data[i]) / 32767.0f;  // Use 32767 not 32768
        }
        AddCleanAudioFrame(float_frame.data(), length);

        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool WqAec3Processor::GetMetrics(double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms) {
    if (!initialized_ || !echo_return_loss || !echo_return_loss_enhancement || !delay_ms) {
        return false;
    }

    try {
        auto metrics = echo_controller_->GetMetrics();
        *echo_return_loss = metrics.echo_return_loss;
        *echo_return_loss_enhancement = metrics.echo_return_loss_enhancement;
        *delay_ms = metrics.delay_ms;
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

void WqAec3Processor::SetAudioBufferDelay(int delay_ms) {
    if (initialized_) {
        echo_controller_->SetAudioBufferDelay(delay_ms);
    }
}

void WqAec3Processor::AddCleanAudioFrame(const float* audio_data, size_t length) {
    if (!audio_data || length == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(clean_audio_mutex_);
    
    std::vector<float> frame(audio_data, audio_data + length);
    clean_audio_frames_.push_back(std::move(frame));
    
    // Limit buffer size to prevent memory issues (keep last 1000 frames = ~10 seconds)
    if (clean_audio_frames_.size() > 1000) {
        clean_audio_frames_.erase(clean_audio_frames_.begin());
    }
}

size_t WqAec3Processor::GetCleanAudioBuffer(std::vector<std::vector<float>>& audio_frames) {
    std::lock_guard<std::mutex> lock(clean_audio_mutex_);
    
    audio_frames = clean_audio_frames_;
    size_t frame_count = clean_audio_frames_.size();
    
    return frame_count;
}

void WqAec3Processor::ClearCleanAudioBuffer() {
    std::lock_guard<std::mutex> lock(clean_audio_mutex_);
    clean_audio_frames_.clear();
}

std::vector<uint8_t> WqAec3Processor::GetCleanAudioAsBytes() {
    std::lock_guard<std::mutex> lock(clean_audio_mutex_);
    
    std::vector<uint8_t> result;
    if (clean_audio_frames_.empty()) {
        return result;
    }
    
    // Convert float samples to int16 PCM bytes with proper clamping
    result.reserve(clean_audio_frames_.size() * kFrameSize * sizeof(int16_t));
    
    for (const auto& frame : clean_audio_frames_) {
        for (float sample : frame) {
            // Clamp sample to valid range [-1.0, 1.0] to prevent overflow
            sample = std::max(-1.0f, std::min(1.0f, sample));
            
            // Convert float [-1.0, 1.0] to int16 [-32767, 32767] (avoid -32768 for symmetry)
            int16_t pcm_sample;
            if (sample >= 0.0f) {
                pcm_sample = static_cast<int16_t>(sample * 32767.0f);
            } else {
                pcm_sample = static_cast<int16_t>(sample * 32767.0f);
            }
            
            // Add bytes in little-endian format
            result.push_back(static_cast<uint8_t>(pcm_sample & 0xFF));
            result.push_back(static_cast<uint8_t>((pcm_sample >> 8) & 0xFF));
        }
    }
    
    return result;
}

void WqAec3Processor::ProcessAudioFrame(webrtc::AudioBuffer* buffer) {
    if (!buffer) {
        return;
    }
    
    buffer->SplitIntoFrequencyBands();
    // Additional processing can be added here if needed
    buffer->MergeFrequencyBands();
}

} // namespace webrtc_aec3_tts