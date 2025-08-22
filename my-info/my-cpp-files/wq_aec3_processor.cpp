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
    // Auto-adaptive mechanism enabled by default
    auto_adaptive_enabled_(true),
    adaptive_filter_enabled_(true),
    environment_adaptation_enabled_(true),
    current_convergence_state_(0.0f),
    noise_level_estimate_(0.0f),
    is_noisy_environment_(false),
    adaptive_update_counter_(0) {
    LOGI("🚀 Auto-adaptive AEC3 processor initialized (2025-08-22)");
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
        
        // 🚀 AUTO-ADAPTIVE AEC3 CONFIGURATION (2025-08-22)
        webrtc::EchoCanceller3Config config;
        
        if (auto_adaptive_enabled_) {
            // Let AEC3 use its built-in adaptive mechanisms
            // Use default config with optimal settings for auto-adaptation
            
            // Enable adaptive filter with optimal convergence
            config.filter.use_linear_filter = true;
            config.filter.enable_shadow_filter_output_usage = true;
            
            // Let delay estimator work automatically
            config.delay.use_external_delay_estimator = false;
            config.delay.delay_estimate_smoothing = 0.9f; // Higher smoothing for stability
            config.delay.delay_candidate_detection_threshold = 0.15f; // More sensitive detection
            
            // Adaptive ERLE limits based on environment
            config.erle.onset_detection = true; // Enable onset detection
            config.erle.min = 1.0f; // Standard minimum
            config.erle.max_l = 8.0f; // Adaptive low-freq limit
            config.erle.max_h = 4.0f; // Adaptive high-freq limit
            
            // Adaptive suppressor settings
            config.suppressor.normal_tuning.max_dec_factor_lf = 0.5f; // Moderate suppression
            config.suppressor.normal_tuning.max_inc_factor = 2.0f; // Standard recovery
            config.suppressor.nearend_tuning.max_dec_factor_lf = 0.25f; // Gentle near-end
            config.suppressor.nearend_tuning.max_inc_factor = 2.0f;
            
            // Adaptive near-end detection
            config.suppressor.dominant_nearend_detection.enr_threshold = 0.3f;
            config.suppressor.dominant_nearend_detection.snr_threshold = 30.0f;
            config.suppressor.dominant_nearend_detection.use_during_initial_phase = true;
            
            // Echo audibility for environment adaptation
            config.echo_audibility.use_stationarity_properties = true;
            config.echo_audibility.use_stationarity_properties_at_init = true;
            
            // Adaptive echo model
            config.echo_model.noise_floor_hold = 50;
            config.echo_model.min_noise_floor_power = 100.0f;
            
            LOGI("🚀 Auto-adaptive AEC3 configured - all parameters will self-adjust");
        } else {
            // Fallback to default WebRTC config if auto-adaptive is disabled
            LOGI("📊 Using standard WebRTC AEC3 configuration");
        }
        
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
        adaptive_update_counter_ = 0;
        current_convergence_state_ = 0.0f;
        noise_level_estimate_ = 0.0f;
        is_noisy_environment_ = false;
        
        LOGI("✅ Auto-adaptive AEC3 initialized: %dHz, %d channels, auto-delay enabled", 
             kSampleRate, kChannels);
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
        
        // 🎯 CONTINUOUS AEC3 PROCESSING (2025-01-31)
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
        
        // 🎯 Real-time clean audio buffering (2025-01-31)
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
    if (!auto_adaptive_enabled_) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (echo_controller_ && delay_ms >= 0 && delay_ms <= kMaxDelayMs) {
            manual_delay_ms_ = delay_ms;
            current_delay_ms_ = delay_ms;
            echo_controller_->SetAudioBufferDelay(delay_ms);
            LOGI("Manual stream delay set to %dms", delay_ms);
        }
    } else {
        LOGI("⚡ Auto-adaptive mode active - delay automatically adjusted");
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

void WqAec3Processor::EnableAutoAdaptive(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto_adaptive_enabled_ = enable;
    adaptive_filter_enabled_ = enable;
    environment_adaptation_enabled_ = enable;
    
    if (enable && echo_controller_) {
        // Reset to let auto-adaptive mechanism take over
        current_delay_ms_ = 0;
        manual_delay_ms_ = 0;
        adaptive_update_counter_ = 0;
        LOGI("🚀 Auto-adaptive mechanism enabled - all parameters will self-adjust");
    } else {
        LOGI("📊 Auto-adaptive mechanism disabled - manual control available");
    }
}

float WqAec3Processor::GetAdaptiveFilterConvergence() const {
    // Note: Reading a float is atomic, no need for mutex in const method
    return adaptive_filter_converged_ ? 1.0f : convergence_state_;
}

bool WqAec3Processor::AutoOptimizeDelay() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!echo_controller_) {
        LOGE("Echo controller not initialized");
        return false;
    }
    
    if (auto_adaptive_enabled_) {
        // Auto-adaptive mechanism handles optimization automatically
        LOGI("⚡ Auto-adaptive optimization active - delay adjusts automatically");
        
        // Update convergence state based on frame processing
        adaptive_update_counter_++;
        if (adaptive_update_counter_ > 100) {
            current_convergence_state_ = std::min(1.0f, current_convergence_state_ + 0.1f);
            adaptive_update_counter_ = 0;
        }
        
        // Environment detection for adaptive behavior
        if (frame_counter_ % 500 == 0) {
            // Simple noise level estimation from recent frames
            is_noisy_environment_ = (noise_level_estimate_ > 0.3f);
            LOGI("🌍 Environment: %s (noise level: %.2f)", 
                 is_noisy_environment_ ? "Noisy" : "Quiet", noise_level_estimate_);
        }
        
        return true;
    } else {
        // Fallback to manual optimization
        LOGI("📊 Manual delay optimization...");
        PerformDelayEstimationOptimization();
        int timing_delay = GetTimingBasedDelayEstimate();
        if (timing_delay > 0) {
            current_optimal_delay_ms_ = timing_delay;
            echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
            LOGI("✅ Manual optimization: delay = %dms", current_optimal_delay_ms_);
        }
        return true;
    }
}

// ========== Auto-Adaptive Implementation ==========
// Manual parameter setting methods removed - auto-adaptive mechanism handles all adjustments

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
