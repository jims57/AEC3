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
        
        // Optimize for TTS echo cancellation
        aec3_config_.filter.export_linear_aec_output = true;
        aec3_config_.delay.default_delay = 5;  // Typical mobile delay
        aec3_config_.delay.delay_estimate_smoothing = 0.9f;  // Stable delay estimation
        
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
        
        // Apply echo cancellation
        echo_controller_->ProcessCapture(capture_audio_buffer_.get(), 
                                       linear_output_buffer_.get(), false);
        
        capture_audio_buffer_->MergeFrequencyBands();

        // Copy processed audio back to output
        capture_audio_buffer_->CopyTo(&capture_frame);
        std::memcpy(output_data, capture_frame.data(), length * sizeof(int16_t));

        // Store clean audio for real-time buffer
        AddCleanAudioFrame(capture_audio_buffer_->channels()[0], length);

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

void WqAec3Processor::ProcessAudioFrame(webrtc::AudioBuffer* buffer) {
    if (!buffer) {
        return;
    }
    
    buffer->SplitIntoFrequencyBands();
    // Additional processing can be added here if needed
    buffer->MergeFrequencyBands();
}

} // namespace webrtc_aec3_tts