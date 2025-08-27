#include "wq_aec3_processor.h"
#include "webrtc_compat.h"
#include <android/log.h>

#define LOG_TAG "WebRTC_AEC3_TTS"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)

namespace webrtc_aec3_tts {

// ========== TimedFrame Implementation ==========

TimedFrame::TimedFrame(const int16_t* samples, size_t size, uint64_t id) 
    : data(samples, samples + size), timestamp(std::chrono::high_resolution_clock::now()), frame_id(id) {}

// ========== WqAec3Processor Implementation ==========

WqAec3Processor::WqAec3Processor() : 
    frame_counter_(0), 
    last_delay_estimation_(0), 
    current_optimal_delay_ms_(kStreamDelay), 
    delay_estimation_counter_(0),
    total_render_frames_(0), 
    total_capture_frames_(0),
    timing_sync_enabled_(true),
    initialization_frames_(0),
    is_initialization_complete_(false),
    current_delay_ms_(kStreamDelay),
    manual_delay_ms_(0),
    
    // ERLE CONFIGURATION
    erle_max_l_(25.0f),          // Low-freq ERLE limit: 25dB (vs default 4dB) - Higher = stronger low-freq echo suppression
    erle_max_h_(15.0f),          // High-freq ERLE limit: 15dB (vs default 1.5dB) - Higher = stronger high-freq echo suppression  
    erle_min_(0.1f),             // Minimum ERLE: 0.1dB (vs default 1dB) - Lower = more aggressive minimum suppression
    
    // FILTER CONFIGURATION
    filter_length_blocks_(25),                    // Filter length in blocks (default 13) - Higher = better echo learning but slower adaptation
    filter_leakage_converged_(0.000005f),         // Leakage when converged (default 0.00005f) - Lower = more stable when converged
    filter_leakage_diverged_(0.005f),             // Leakage when diverged (default 0.05f) - Lower = faster recovery from divergence
    filter_error_floor_(0.001f),                  // Error floor (default 0.001f) - Lower = more sensitive error detection
    filter_error_ceil_(2.0f),                     // Error ceiling (default 2.0f) - Higher = more tolerance for errors
    filter_main_initial_leakage_converged_(0.01f), // Initial main filter leakage converged (default 0.005f)
    filter_main_initial_leakage_diverged_(0.2f),  // Initial main filter leakage diverged (default 0.5f)
    
    // FILTER TIMING CONFIGURATION 
    config_change_duration_blocks_(125),          // Config change duration in blocks (default 250) - Lower = faster config changes
    initial_state_seconds_(2.5f),                 // Initial state duration in seconds (default 2.5f) - Higher = longer initial learning
    conservative_initial_phase_(false),           // Conservative initial phase (default false) - true = more cautious initial adaptation
    
    // SUPPRESSOR NORMAL TUNING
    max_dec_factor_lf_(0.55f),                     // Normal max decrease factor low-freq (default 0.25f) - Higher = stronger echo suppression
    max_inc_factor_(2.0f),                        // Normal max increase factor (default 2.0f) - Higher = faster voice recovery
    
    // SUPPRESSOR NEAREND TUNING
    nearend_max_dec_factor_lf_(.15f),             // Nearend max decrease factor low-freq (default 0.25f) - Higher = stronger nearend suppression
    nearend_max_inc_factor_(2.0f),                // Nearend max increase factor (default 2.0f) - Higher = faster nearend voice recovery
    
    // DOMINANT NEAREND DETECTION
    enr_threshold_(0.1f),                         // ENR threshold (default 0.25f) - Higher = less sensitive nearend detection
    snr_threshold_(5.0f),                        // SNR threshold (default 30.0f) - Lower = more sensitive to noise
    hold_duration_(20),                            // Hold duration in blocks (default 50) - Lower = faster switching
    trigger_threshold_(1),                        // Trigger threshold (default 12) - Lower = easier to trigger nearend detection
    
    // DELAY ESTIMATION CONFIGURATION
    delay_down_sampling_factor_(4),               // Delay down sampling factor (default 4) - Lower = higher precision, more CPU
    delay_num_filters_(16),                       // Number of delay filters (default 5) - Higher = more accurate delay estimation
    delay_estimate_smoothing_(0.98f) {            // Delay estimate smoothing (default 0.7f) - Higher = more stable delay estimates
}

WqAec3Processor::~WqAec3Processor() {
    std::lock_guard<std::mutex> lock(mutex_);
    echo_controller_.reset();
    aec_factory_.reset();
    audio_render_buffer_.reset();
    audio_capture_buffer_.reset();
    high_pass_filter_.reset();
    render_buffer_.clear();
}

bool WqAec3Processor::Initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try {
        LOGI("Initializing Enhanced WebRTC AEC3 for TTS: %dHz, %d channels (ERLE Optimization 2025-01-31)", kSampleRate, kChannels);
        
        // 🔧 CRITICAL FIX: Destroy existing AEC3 components for fresh session
        echo_controller_.reset();
        aec_factory_.reset();
        audio_render_buffer_.reset();
        audio_capture_buffer_.reset();
        high_pass_filter_.reset();
        render_buffer_.clear();
        
        // 🚀 PRODUCTION-GRADE AEC3 CONFIGURATION WITH NEWER ANDROID COMPATIBILITY 
        webrtc::EchoCanceller3Config config;
        
        // // 🎯 ERLE CONFIGURATION FROM CONSTRUCTOR PARAMETERS 
        // config.erle.max_l = erle_max_l_;  // Low-freq ERLE limit from constructor
        // config.erle.max_h = erle_max_h_;  // High-freq ERLE limit from constructor
        // config.erle.min = erle_min_;      // Minimum ERLE from constructor
        // LOGI("🎯 ERLE limits configured: max_l=%.1fdB, max_h=%.1fdB (from constructor)", 
        //      config.erle.max_l, config.erle.max_h);
        
        // // 🚀 ENHANCED FILTER CONFIGURATION FROM CONSTRUCTOR PARAMETERS 
        // config.filter.main.length_blocks = filter_length_blocks_;
        // config.filter.main.leakage_converged = filter_leakage_converged_;
        // config.filter.main.leakage_diverged = filter_leakage_diverged_;
        
        // // 🔧 FILTER ERROR AND INITIAL CONFIGURATION FROM CONSTRUCTOR PARAMETERS
        // config.filter.main.error_floor = filter_error_floor_;
        // config.filter.main.error_ceil = filter_error_ceil_;
        // config.filter.main_initial.leakage_converged = filter_main_initial_leakage_converged_;
        // config.filter.main_initial.leakage_diverged = filter_main_initial_leakage_diverged_;
        
        // // 🎯 SUPPRESSOR TUNING FROM CONSTRUCTOR PARAMETERS 
        // config.suppressor.normal_tuning.max_dec_factor_lf = max_dec_factor_lf_;
        // config.suppressor.normal_tuning.max_inc_factor = max_inc_factor_;
        // config.suppressor.nearend_tuning.max_dec_factor_lf = nearend_max_dec_factor_lf_;
        // config.suppressor.nearend_tuning.max_inc_factor = nearend_max_inc_factor_;
        
        // // 🚀 DELAY ESTIMATION FROM CONSTRUCTOR PARAMETERS 
        // config.delay.down_sampling_factor = delay_down_sampling_factor_;
        // config.delay.num_filters = delay_num_filters_;
        // config.delay.delay_estimate_smoothing = delay_estimate_smoothing_;
        
        // LOGI("🚀 AEC3 configured from constructor parameters: filter_length=%zu, max_dec_lf=%.1f", 
        //      config.filter.main.length_blocks, config.suppressor.normal_tuning.max_dec_factor_lf);
        
        // // 🎯 FILTER TIMING CONFIGURATION FROM CONSTRUCTOR PARAMETERS 
        // config.filter.config_change_duration_blocks = config_change_duration_blocks_;
        // config.filter.initial_state_seconds = initial_state_seconds_;
        // config.filter.conservative_initial_phase = conservative_initial_phase_;
        
        // // 🎯 DOMINANT NEAREND DETECTION FROM CONSTRUCTOR PARAMETERS 
        // config.suppressor.dominant_nearend_detection.enr_threshold = enr_threshold_;
        // config.suppressor.dominant_nearend_detection.snr_threshold = snr_threshold_;
        // config.suppressor.dominant_nearend_detection.hold_duration = hold_duration_;
        // config.suppressor.dominant_nearend_detection.trigger_threshold = trigger_threshold_;
        
        // Create AEC3 factory and controller
        aec_factory_ = std::make_unique<webrtc::EchoCanceller3Factory>(config);
        if (!aec_factory_) {
            LOGE("Failed to create AEC3 factory");
            return false;
        }

        echo_controller_ = aec_factory_->Create(kSampleRate, kChannels, kChannels);
        if (!echo_controller_) {
            LOGE("Failed to create AEC3 controller");
            return false;
        }

        // Create AudioBuffers with IDENTICAL parameters
        audio_render_buffer_ = std::make_unique<webrtc::AudioBuffer>(
            kSampleRate, kChannels,
            kSampleRate, kChannels,
            kSampleRate, kChannels);

        audio_capture_buffer_ = std::make_unique<webrtc::AudioBuffer>(
            kSampleRate, kChannels,
            kSampleRate, kChannels,
            kSampleRate, kChannels);

        if (!audio_render_buffer_ || !audio_capture_buffer_) {
            LOGE("Failed to create audio buffers");
            return false;
        }

        // Create high-pass filter
        high_pass_filter_ = std::make_unique<webrtc::HighPassFilter>(kSampleRate, kChannels);
        if (!high_pass_filter_) {
            LOGE("Failed to create high-pass filter");
            return false;
        }

        // Reset all statistical counters for fresh session
        frame_counter_ = 0;
        last_delay_estimation_ = 0;
        delay_estimation_counter_ = 0;
        total_render_frames_ = 0;
        total_capture_frames_ = 0;
        current_optimal_delay_ms_ = kStreamDelay;
        timing_sync_enabled_ = true;
        initialization_frames_ = 0;
        is_initialization_complete_ = false;
        
        LOGI("WebRTC AEC3 initialized successfully: %dHz, %d channels, %dms delay (complete state reset)", 
             kSampleRate, kChannels, kStreamDelay);
        return true;
    } catch (const std::exception& e) {
        LOGE("Exception during AEC3 initialization: %s", e.what());
        return false;
    }
}

bool WqAec3Processor::ProcessTtsAudio(const int16_t* tts_data, size_t length) {
    if (length != kFrameSize) {
        LOGE("Invalid TTS data: length=%zu, expected=%d", length, kFrameSize);
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!echo_controller_ || !audio_render_buffer_) {
        LOGE("AEC3 not initialized");
        return false;
    }

    try {
        // Enhanced reference signal processing for device compatibility
        if (timing_sync_enabled_) {
            double frame_energy = CalculateFrameEnergy(tts_data, length);
            
            // Silent frame enhancement: boost weak TTS signals
            std::vector<int16_t> enhanced_tts_data(tts_data, tts_data + length);
            if (frame_energy < 1000.0) {
                for (size_t i = 0; i < enhanced_tts_data.size(); ++i) {
                    enhanced_tts_data[i] = static_cast<int16_t>(
                        std::min(static_cast<int>(enhanced_tts_data[i] * 2.0f), 
                               static_cast<int>(INT16_MAX)));
                }
                LOGV("🔧 Enhanced weak TTS signal: energy %.1f -> %.1f", 
                     frame_energy, CalculateFrameEnergy(enhanced_tts_data.data(), length));
            }
            
            TimedFrame timed_frame(enhanced_tts_data.data(), length, frame_counter_++);
            render_buffer_.push_back(std::move(timed_frame));
            
            // Maintain buffer size for optimal delay range
            if (render_buffer_.size() > kDelayBufferSize) {
                render_buffer_.pop_front();
            }
        }
        
        // Create AudioFrame from input data (following demo.cc exactly)
        webrtc::AudioFrame render_frame;
        render_frame.UpdateFrame(0, tts_data, kFrameSize, kSampleRate, 
                               webrtc::AudioFrame::kNormalSpeech, 
                               webrtc::AudioFrame::kVadActive, kChannels);

        // Follow demo.cc pipeline exactly
        audio_render_buffer_->CopyFrom(&render_frame);
        audio_render_buffer_->SplitIntoFrequencyBands();
        echo_controller_->AnalyzeRender(audio_render_buffer_.get());
        audio_render_buffer_->MergeFrequencyBands();

        total_render_frames_++;
        
        double render_energy = CalculateFrameEnergy(tts_data, length);
        
        LOGV("Processed TTS reference signal: frame=%llu, energy=%.2f, buffer_size=%zu", 
             (unsigned long long)frame_counter_ - 1, render_energy, render_buffer_.size());
        
        return true;
    } catch (const std::exception& e) {
        LOGE("Exception in ProcessTtsAudio: %s", e.what());
        return false;
    }
}

bool WqAec3Processor::ProcessMicrophoneAudio(const int16_t* mic_data, int16_t* output_data, size_t length) {
    if (length != kFrameSize) {
        LOGE("Invalid mic data: length=%zu, expected=%d", length, kFrameSize);
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!echo_controller_ || !audio_capture_buffer_ || !high_pass_filter_) {
        LOGE("AEC3 not initialized");
        return false;
    }

    try {
        auto capture_timestamp = std::chrono::high_resolution_clock::now();
        
        // Initialization stabilization: warm-up period
        initialization_frames_++;
        if (initialization_frames_ >= kInitializationFrames && !is_initialization_complete_) {
            is_initialization_complete_ = true;
            LOGI("🔧 Initialization complete after %d frames - AEC3 ready for optimal performance", 
                 initialization_frames_);
        }
        
        // Enhanced device-adaptive timing synchronization
        if (timing_sync_enabled_) {
            webrtc::EchoControl::Metrics current_metrics = echo_controller_->GetMetrics();
            int aec3_detected_delay = current_metrics.delay_ms;
            
            if (!is_initialization_complete_) {
                // During initialization: use stable delay, avoid aggressive changes
                if (initialization_frames_ % 50 == 0) {
                    echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
                    LOGV("🔧 Gentle init delay: %dms (frame %d/%d)", 
                         current_optimal_delay_ms_, initialization_frames_, kInitializationFrames);
                }
            } else {
                // After initialization: normal delay management with cross-device auto-adjustment
                if (aec3_detected_delay <= 0 || aec3_detected_delay > 500) {
                    const TimedFrame* best_reference = FindOptimalReferenceFrame(capture_timestamp);
                    if (best_reference) {
                        int timing_based_delay = EstimateOptimalDelay(capture_timestamp, best_reference->timestamp);
                        if (timing_based_delay >= kMinDelayMs && timing_based_delay <= kMaxDelayMs) {
                            current_optimal_delay_ms_ = timing_based_delay;
                            echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
                            LOGI("🔧 Device delay fix: Forced timing-based delay %dms (AEC3 detection failed: %dms)", 
                                 current_optimal_delay_ms_, aec3_detected_delay);
                        }
                    }
                }
            }
            
            // Log delay mismatch for debugging
            if (std::abs(aec3_detected_delay - current_optimal_delay_ms_) > 20) {
                LOGW("⚠️ Delay mismatch: AEC3=%dms vs Set=%dms (diff=%dms)", 
                     aec3_detected_delay, current_optimal_delay_ms_, 
                     std::abs(aec3_detected_delay - current_optimal_delay_ms_));
            }
            
            // Gradual delay adjustment for cross-device compatibility
            if (is_initialization_complete_) {
                if (aec3_detected_delay > 10 && aec3_detected_delay < 200 && 
                    std::abs(aec3_detected_delay - current_optimal_delay_ms_) > 30) {
                    
                    int target_delay = aec3_detected_delay;
                    int adjustment = (target_delay > current_optimal_delay_ms_) ? 5 : -5;
                    current_optimal_delay_ms_ = current_optimal_delay_ms_ + adjustment;
                    current_optimal_delay_ms_ = std::max(kMinDelayMs, std::min(kMaxDelayMs, current_optimal_delay_ms_));
                    
                    LOGI("🔧 Gradual delay adjustment: %dms -> %dms (target: %dms)", 
                         current_optimal_delay_ms_ - adjustment, current_optimal_delay_ms_, target_delay);
                } else {
                    // Normal delay detection working - use hybrid approach for best cross-device performance
                    const TimedFrame* best_reference = FindOptimalReferenceFrame(capture_timestamp);
                    if (best_reference) {
                        int timing_based_delay = EstimateOptimalDelay(capture_timestamp, best_reference->timestamp);
                        int weighted_delay = static_cast<int>(aec3_detected_delay * 0.7f + timing_based_delay * 0.3f);
                        
                        if (std::abs(weighted_delay - current_optimal_delay_ms_) > 5) {
                            int old_delay = current_optimal_delay_ms_;
                            current_optimal_delay_ms_ = weighted_delay;
                            LOGI("🎯 Cross-device delay optimization: %dms -> %dms (AEC3=%dms, Timing=%dms)", 
                                 old_delay, current_optimal_delay_ms_, aec3_detected_delay, timing_based_delay);
                        }
                    }
                }
            }
            
            // Aggressive delay enforcement for problematic devices
            static int delay_set_counter = 0;
            if (++delay_set_counter % 10 == 0) {
                echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
                LOGV("🔧 Delay enforcement: %dms (frame %d)", current_optimal_delay_ms_, delay_set_counter);
            }
        }
        
        // Create AudioFrame from input data (following demo.cc exactly)
        webrtc::AudioFrame capture_frame;
        capture_frame.UpdateFrame(0, mic_data, kFrameSize, kSampleRate,
                                webrtc::AudioFrame::kNormalSpeech,
                                webrtc::AudioFrame::kVadActive, kChannels);

        // Follow demo.cc pipeline exactly for maximum ERLE
        audio_capture_buffer_->CopyFrom(&capture_frame);
        echo_controller_->AnalyzeCapture(audio_capture_buffer_.get());
        audio_capture_buffer_->SplitIntoFrequencyBands();
        high_pass_filter_->Process(audio_capture_buffer_.get(), true);
        echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
        echo_controller_->ProcessCapture(audio_capture_buffer_.get(), false);
        audio_capture_buffer_->MergeFrequencyBands();
        
        // Copy processed data back to output
        audio_capture_buffer_->CopyTo(&capture_frame);
        memcpy(output_data, capture_frame.data(), length * sizeof(int16_t));

        total_capture_frames_++;
        
        // Calculate energy for quality assessment
        double capture_energy = CalculateFrameEnergy(mic_data, length);
        double output_energy = CalculateFrameEnergy(output_data, length);
        double suppression_ratio = capture_energy > 0 ? output_energy / capture_energy : 1.0;
        
        // 🎯 CONTINUOUS AEC3 PROCESSING 
        // Keep AEC3 running continuously for consistent echo removal and clear voice
        // output_data already contains AEC3 processed result - use as is
        LOGV("🎯 Continuous AEC3: Using processed output for optimal echo removal and voice clarity");
        
        // Periodic delay estimation and ERLE optimization
        if (++delay_estimation_counter_ >= kDelayEstimationFrames) {
            PerformDelayEstimationOptimization();
            delay_estimation_counter_ = 0;
        }
        
        LOGV("🎯 Enhanced AEC3 processing: frame=%llu, delay=%dms, suppression=%.3f, in_energy=%.2f, out_energy=%.2f", 
             (unsigned long long)total_capture_frames_, current_optimal_delay_ms_, suppression_ratio, capture_energy, output_energy);
        
        // 🎯 Real-time clean audio buffering 
        // Store processed clean audio frame for immediate availability
        {
            std::lock_guard<std::mutex> buffer_lock(clean_audio_buffer_mutex_);
            std::vector<float> cleanFrame(kFrameSize);
            for (size_t i = 0; i < kFrameSize; ++i) {
                cleanFrame[i] = output_data[i] / 32768.0f; // Convert int16 to float [-1.0, 1.0]
            }
            
            // Debug: Log first few samples to verify data BEFORE moving
            if (clean_audio_buffer_.size() % 50 == 0) { // Log every 50th frame
                LOGI("🎯 Clean audio frame buffered: frame %zu, input_samples [%d, %d, %d, %d], float_samples [%.6f, %.6f, %.6f, %.6f]", 
                     clean_audio_buffer_.size() + 1, output_data[0], output_data[1], output_data[2], output_data[3],
                     cleanFrame[0], cleanFrame[1], cleanFrame[2], cleanFrame[3]);
            }
            
            // CRITICAL DEBUG: Always log first frame to see if AEC3 produces any output
            if (clean_audio_buffer_.size() == 0) {
                LOGI("🔍 FIRST FRAME DEBUG: output_data [%d, %d, %d, %d], capture_energy=%.2f, output_energy=%.2f", 
                     output_data[0], output_data[1], output_data[2], output_data[3], capture_energy, output_energy);
            }
            
            clean_audio_buffer_.push_back(std::move(cleanFrame));
            LOGV("🎯 Clean audio frame buffered: %zu total frames", clean_audio_buffer_.size());
        }
        
        return true;
    } catch (const std::exception& e) {
        LOGE("Exception in ProcessMicrophoneAudio: %s", e.what());
        return false;
    }
}

size_t WqAec3Processor::GetAndClearCleanAudioBuffer(std::vector<std::vector<float>>& outputFrames) {
    std::lock_guard<std::mutex> buffer_lock(clean_audio_buffer_mutex_);
    
    outputFrames = std::move(clean_audio_buffer_);
    clean_audio_buffer_.clear();
    
    size_t frameCount = outputFrames.size();
    LOGI("🎯 Retrieved %zu clean audio frames from buffer and cleared", frameCount);
    
    return frameCount;
}

size_t WqAec3Processor::GetCleanAudioBuffer(std::vector<std::vector<float>>& outputFrames) {
    std::lock_guard<std::mutex> buffer_lock(clean_audio_buffer_mutex_);
    
    outputFrames = clean_audio_buffer_; // Copy without moving
    
    size_t frameCount = outputFrames.size();
    LOGI("🎯 Retrieved %zu clean audio frames from buffer (no clear)", frameCount);
    
    return frameCount;
}

void WqAec3Processor::ClearCleanAudioBuffer() {
    std::lock_guard<std::mutex> buffer_lock(clean_audio_buffer_mutex_);
    clean_audio_buffer_.clear();
    LOGI("🎯 Clean audio buffer cleared");
}



bool WqAec3Processor::GetMetrics(double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!echo_controller_) {
        return false;
    }

    try {
        webrtc::EchoControl::Metrics metrics = echo_controller_->GetMetrics();
        *echo_return_loss = metrics.echo_return_loss;
        *echo_return_loss_enhancement = metrics.echo_return_loss_enhancement;
        *delay_ms = metrics.delay_ms;
        return true;
    } catch (const std::exception& e) {
        LOGE("Exception in GetMetrics: %s", e.what());
        return false;
    }
}

bool WqAec3Processor::GetEnhancedMetrics(double* echo_return_loss, double* echo_return_loss_enhancement, 
                       int* delay_ms, uint64_t* render_frames, uint64_t* capture_frames, 
                       int* optimal_delay) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!echo_controller_) return false;
    
    try {
        webrtc::EchoControl::Metrics metrics = echo_controller_->GetMetrics();
        *echo_return_loss = metrics.echo_return_loss;
        *echo_return_loss_enhancement = metrics.echo_return_loss_enhancement;
        *delay_ms = metrics.delay_ms;
        *render_frames = total_render_frames_;
        *capture_frames = total_capture_frames_;
        *optimal_delay = current_optimal_delay_ms_;
        return true;
    } catch (const std::exception& e) {
        LOGE("Exception in GetEnhancedMetrics: %s", e.what());
        return false;
    }
}

void WqAec3Processor::SetStreamDelay(int delay_ms) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (echo_controller_) {
        manual_delay_ms_ = delay_ms;
        // 🎯 INITIAL SETTING: Only set if auto-adjustment hasn't started yet
        if (current_optimal_delay_ms_ == kStreamDelay) {
            current_optimal_delay_ms_ = delay_ms;
            LOGI("🎯 Initial stream delay set: %dms (will be auto-optimized for device)", delay_ms);
        } else {
            LOGI("🎯 Stream delay received: %dms (auto-adjustment active, using optimized: %dms)", 
                 delay_ms, current_optimal_delay_ms_);
        }
        echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
    }
}

bool WqAec3Processor::EnableTimingSync(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    timing_sync_enabled_ = enable;
    LOGI("🎯 Timing synchronization %s", enable ? "enabled" : "disabled");
    
    if (!enable) {
        render_buffer_.clear();
    }
    
    return true;
}

bool WqAec3Processor::AutoOptimizeDelay() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!echo_controller_) return false;
    
    try {
        PerformDelayEstimationOptimization();
        LOGI("🎯 Auto delay optimization completed: %dms", current_optimal_delay_ms_);
        return true;
    } catch (const std::exception& e) {
        LOGE("Exception in auto delay optimization: %s", e.what());
        return false;
    }
}

// ========== Configuration Methods Implementation ==========

void WqAec3Processor::SetConfigChangeDuration(int blocks) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_change_duration_blocks_ = std::max(0, std::min(1000, blocks));
    LOGI("📝 AEC3 config change duration: %d blocks", config_change_duration_blocks_);
}

void WqAec3Processor::SetInitialStateSeconds(float seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    initial_state_seconds_ = std::max(0.0f, std::min(100.0f, seconds));
    LOGI("📝 AEC3 initial state duration: %.2f seconds", initial_state_seconds_);
}

void WqAec3Processor::SetConservativeInitialPhase(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    conservative_initial_phase_ = enable;
    LOGI("📝 AEC3 conservative initial phase: %s", enable ? "enabled" : "disabled");
}

void WqAec3Processor::SetMaxDecFactorLF(float factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_dec_factor_lf_ = std::max(0.0f, std::min(100.0f, factor));
    LOGI("🎛️ AEC3 max decrease factor LF: %.2f", max_dec_factor_lf_);
}

void WqAec3Processor::SetMaxIncFactor(float factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_inc_factor_ = std::max(0.0f, std::min(100.0f, factor));
    LOGI("🎛️ AEC3 max increase factor: %.2f", max_inc_factor_);
}

void WqAec3Processor::SetNearendMaxDecFactorLF(float factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    nearend_max_dec_factor_lf_ = std::max(0.0f, std::min(100.0f, factor));
    LOGI("🎙️ AEC3 nearend max decrease factor LF: %.2f", nearend_max_dec_factor_lf_);
}

void WqAec3Processor::SetNearendMaxIncFactor(float factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    nearend_max_inc_factor_ = std::max(0.0f, std::min(100.0f, factor));
    LOGI("🎙️ AEC3 nearend max increase factor: %.2f", nearend_max_inc_factor_);
}

void WqAec3Processor::SetEnrThreshold(float threshold) {
    std::lock_guard<std::mutex> lock(mutex_);
    enr_threshold_ = std::max(0.0f, std::min(1000000.0f, threshold));
    LOGI("🔍 AEC3 ENR threshold: %.2f", enr_threshold_);
}

void WqAec3Processor::SetSnrThreshold(float threshold) {
    std::lock_guard<std::mutex> lock(mutex_);
    snr_threshold_ = std::max(0.0f, std::min(1000000.0f, threshold));
    LOGI("🔍 AEC3 SNR threshold: %.2f", snr_threshold_);
}

void WqAec3Processor::SetHoldDuration(int duration) {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_duration_ = std::max(0, std::min(10000, duration));
    LOGI("🔍 AEC3 hold duration: %d", hold_duration_);
}

void WqAec3Processor::SetTriggerThreshold(int threshold) {
    std::lock_guard<std::mutex> lock(mutex_);
    trigger_threshold_ = std::max(0, std::min(10000, threshold));
    LOGI("🔍 AEC3 trigger threshold: %d", trigger_threshold_);
}

// ========== ERLE Adjustment Parameter Methods ==========

void WqAec3Processor::SetFilterLengthBlocks(int blocks) {
    std::lock_guard<std::mutex> lock(mutex_);
    filter_length_blocks_ = std::max(1, std::min(100, blocks));
    LOGI("🎯 AEC3 filter length blocks: %d", filter_length_blocks_);
}

void WqAec3Processor::SetFilterLeakageConverged(float leakage) {
    std::lock_guard<std::mutex> lock(mutex_);
    filter_leakage_converged_ = std::max(0.000001f, std::min(1.0f, leakage));
    LOGI("🎯 AEC3 filter leakage converged: %.6f", filter_leakage_converged_);
}

void WqAec3Processor::SetFilterLeakageDiverged(float leakage) {
    std::lock_guard<std::mutex> lock(mutex_);
    filter_leakage_diverged_ = std::max(0.001f, std::min(1.0f, leakage));
    LOGI("🎯 AEC3 filter leakage diverged: %.6f", filter_leakage_diverged_);
}

void WqAec3Processor::SetDelayDownSamplingFactor(int factor) {
    std::lock_guard<std::mutex> lock(mutex_);
    delay_down_sampling_factor_ = std::max(1, std::min(8, factor));
    LOGI("🎯 AEC3 delay down sampling factor: %d", delay_down_sampling_factor_);
}

void WqAec3Processor::SetDelayNumFilters(int filters) {
    std::lock_guard<std::mutex> lock(mutex_);
    delay_num_filters_ = std::max(1, std::min(32, filters));
    LOGI("🎯 AEC3 delay num filters: %d", delay_num_filters_);
}

void WqAec3Processor::SetDelayEstimateSmoothing(float smoothing) {
    std::lock_guard<std::mutex> lock(mutex_);
    delay_estimate_smoothing_ = std::max(0.1f, std::min(0.99f, smoothing));
    LOGI("🎯 AEC3 delay estimate smoothing: %.3f", delay_estimate_smoothing_);
}

// ========== Private Methods Implementation ==========

double WqAec3Processor::CalculateFrameEnergy(const int16_t* samples, size_t length) const {
    double energy = 0.0;
    for (size_t i = 0; i < length; ++i) {
        energy += samples[i] * samples[i];
    }
    return energy / length;
}

const TimedFrame* WqAec3Processor::FindOptimalReferenceFrame(const std::chrono::high_resolution_clock::time_point& capture_time) {
    if (render_buffer_.empty()) return nullptr;
    
    auto target_time = capture_time - std::chrono::milliseconds(current_optimal_delay_ms_);
    
    const TimedFrame* best_match = nullptr;
    auto min_time_diff = std::chrono::milliseconds(static_cast<long>(kTimingToleranceMs * 2));
    
    for (const auto& frame : render_buffer_) {
        auto time_diff = std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(
            frame.timestamp - target_time).count());
        
        if (time_diff < min_time_diff.count()) {
            min_time_diff = std::chrono::milliseconds(time_diff);
            best_match = &frame;
        }
    }
    
    return best_match;
}

int WqAec3Processor::EstimateOptimalDelay(const std::chrono::high_resolution_clock::time_point& capture_time,
                       const std::chrono::high_resolution_clock::time_point& render_time) {
    auto measured_delay = std::chrono::duration_cast<std::chrono::milliseconds>(
        capture_time - render_time).count();
    
    measured_delay = std::max(static_cast<long long>(kMinDelayMs), 
                             std::min(static_cast<long long>(kMaxDelayMs), measured_delay));
    
    return static_cast<int>(measured_delay);
}

void WqAec3Processor::PerformDelayEstimationOptimization() {
    if (!echo_controller_) return;
    
    try {
        webrtc::EchoControl::Metrics current_metrics = echo_controller_->GetMetrics();
        int aec3_delay = current_metrics.delay_ms;
        double current_erle = current_metrics.echo_return_loss_enhancement;
        
        static double last_erle = 0.0;
        static int stable_delay_counter = 0;
        static int best_delay_so_far = current_optimal_delay_ms_;
        static double best_erle_so_far = 0.0;
        
        bool aec3_delay_reliable = (aec3_delay > 0 && aec3_delay <= 500);
        
        if (!aec3_delay_reliable) {
            LOGW("🔧 Device delay detection unreliable: %dms, using adaptive search", aec3_delay);
            
            static int search_step = 0;
            static bool search_direction_up = true;
            
            if (current_erle < 1.0) {
                int new_delay = current_optimal_delay_ms_;
                
                if (search_direction_up) {
                    new_delay += 10;
                    if (new_delay > 200) {
                        search_direction_up = false;
                        new_delay = current_optimal_delay_ms_ - 10;
                    }
                } else {
                    new_delay -= 10;
                    if (new_delay < 20) {
                        search_direction_up = true;
                        new_delay = current_optimal_delay_ms_ + 10;
                    }
                }
                
                current_optimal_delay_ms_ = std::max(kMinDelayMs, std::min(kMaxDelayMs, new_delay));
                echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
                
                LOGI("🔍 Device delay search: trying %dms (step %d, ERLE=%.2fdB)", 
                     current_optimal_delay_ms_, ++search_step, current_erle);
            }
        } else {
            if (current_erle > last_erle + 0.5) {
                stable_delay_counter++;
                if (current_erle > best_erle_so_far) {
                    best_erle_so_far = current_erle;
                    best_delay_so_far = current_optimal_delay_ms_;
                }
                LOGI("🎯 ERLE improved: %.2fdB -> %.2fdB (delay=%dms, stable=%d)", 
                     last_erle, current_erle, current_optimal_delay_ms_, stable_delay_counter);
            } else if (current_erle < last_erle - 1.0) {
                stable_delay_counter = 0;
                
                if (best_erle_so_far > current_erle + 1.0) {
                    current_optimal_delay_ms_ = best_delay_so_far;
                    LOGI("🔧 Reverting to best delay: %dms (ERLE %.2fdB -> %.2fdB)", 
                         best_delay_so_far, current_erle, best_erle_so_far);
                } else {
                    int timing_estimate = GetTimingBasedDelayEstimate();
                    if (timing_estimate > 0) {
                        int adjustment = (timing_estimate > current_optimal_delay_ms_) ? 5 : -5;
                        current_optimal_delay_ms_ = std::max(kMinDelayMs, 
                            std::min(kMaxDelayMs, current_optimal_delay_ms_ + adjustment));
                    }
                }
                
                echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
                LOGW("🎯 ERLE degraded: %.2fdB -> %.2fdB, adjusting delay to %dms", 
                     last_erle, current_erle, current_optimal_delay_ms_);
            }
        }
        
        last_erle = current_erle;
        last_delay_estimation_ = aec3_delay;
        
        if (total_capture_frames_ % 50 == 0) {
            echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
            LOGV("🔧 Periodic delay enforcement: %dms (frame %llu)", 
                 current_optimal_delay_ms_, (unsigned long long)total_capture_frames_);
        }
        
    } catch (const std::exception& e) {
        LOGE("Exception in enhanced delay optimization: %s", e.what());
    }
}

int WqAec3Processor::GetTimingBasedDelayEstimate() {
    if (render_buffer_.empty()) return 0;
    
    auto now = std::chrono::high_resolution_clock::now();
    auto latest_render = render_buffer_.back().timestamp;
    
    auto estimated_delay_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - latest_render).count();
    
    long delay_long = static_cast<long>(estimated_delay_ms);
    return static_cast<int>(std::max(static_cast<long>(kMinDelayMs), 
                                    std::min(static_cast<long>(kMaxDelayMs), delay_long)));
}

} // namespace webrtc_aec3_tts
