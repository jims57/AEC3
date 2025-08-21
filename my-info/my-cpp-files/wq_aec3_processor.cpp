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
    // 🎯 PRODUCTION-GRADE DEFAULTS FOR OPTIMAL ERLE PERFORMANCE
    config_change_duration_blocks_(100),
    initial_state_seconds_(2.0f),
    conservative_initial_phase_(false),
    max_dec_factor_lf_(6.0f),          // Increased for better echo suppression
    max_inc_factor_(4.0f),             // Balanced voice recovery
    nearend_max_dec_factor_lf_(3.0f),  // Better voice preservation
    nearend_max_inc_factor_(5.0f),     // Clear voice when user speaks
    enr_threshold_(0.15f),             // More sensitive voice detection
    snr_threshold_(12.0f),             // Optimized SNR threshold
    hold_duration_(8),                 // Stable voice detection
    trigger_threshold_(3),             // Responsive triggering
    // 🎯 ERLE ADJUSTMENT PARAMETERS FOR PRODUCTION PERFORMANCE
    filter_length_blocks_(30),         // Increased for better echo learning
    filter_leakage_converged_(0.000003f), // Tighter convergence
    filter_leakage_diverged_(0.003f),  // Better divergence recovery
    delay_down_sampling_factor_(2),    // Maintain precision
    delay_num_filters_(20),            // More filters for better detection
    delay_estimate_smoothing_(0.99f),  // Higher smoothing for stability
    // 🎯 Production-Grade ERLE Monitoring Initialization
    production_timing_sync_enabled_(true),
    clockdrift_detection_enabled_(true),
    last_erle_fullband_log2_(0.0),
    adaptive_filter_convergence_counter_(0),
    is_adaptive_filter_converged_(false) {
    
    // Initialize production-grade metrics
    current_production_metrics_ = {};
    erle_history_.reserve(1000); // Reserve space for ERLE history tracking
}

WqAec3Processor::~WqAec3Processor() {
    std::lock_guard<std::mutex> lock(mutex_);
    echo_controller_.reset();
    aec_factory_.reset();
    audio_render_buffer_.reset();
    audio_capture_buffer_.reset();
    high_pass_filter_.reset();
    render_buffer_.clear();
    erle_history_.clear();
}

bool WqAec3Processor::Initialize() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    try {
        LOGI("Initializing Production-Grade WebRTC AEC3 for TTS: %dHz, %d channels (ERLE Target: %.1fdB)", 
             kSampleRate, kChannels, kTargetErleDb);
        
        // 🔧 CRITICAL FIX: Destroy existing AEC3 components for fresh session
        echo_controller_.reset();
        aec_factory_.reset();
        audio_render_buffer_.reset();
        audio_capture_buffer_.reset();
        high_pass_filter_.reset();
        render_buffer_.clear();
        erle_history_.clear();
        
        // 🎯 PRODUCTION-GRADE AEC3 CONFIGURATION WITH BUILT-IN ESTIMATOR OPTIMIZATION (2025-01-31)
        webrtc::EchoCanceller3Config config;
        
        // 🎯 CRITICAL: Production-Grade ERLE Configuration
        config.erle.max_l = 30.0f;  // High-freq ERLE limit: 30dB (production-grade)
        config.erle.max_h = 20.0f;  // Low-freq ERLE limit: 20dB (production-grade)
        config.erle.min = 0.05f;    // Minimum ERLE: 0.05dB (tighter minimum)
        config.erle.onset_detection = true;  // Enable onset detection for better tracking
        config.erle.num_sections = 3;        // Multiple sections for signal-dependent estimation
        LOGI("🎯 Production-Grade ERLE limits: max_l=%.1fdB, max_h=%.1fdB, sections=%d", 
             config.erle.max_l, config.erle.max_h, config.erle.num_sections);
        
        // 🎯 ENHANCED FILTER CONFIGURATION FOR CONVERGENCE (2025-01-31)
        config.filter.main.length_blocks = (filter_length_blocks_ > 0) ? filter_length_blocks_ : 25;  // Balanced for convergence
        config.filter.main.leakage_converged = (filter_leakage_converged_ > 0.0f) ? filter_leakage_converged_ : 0.000010f;  // Higher for convergence
        config.filter.main.leakage_diverged = (filter_leakage_diverged_ > 0.0f) ? filter_leakage_diverged_ : 0.005f;  // Standard recovery
        
        // 🎯 PRODUCTION-GRADE FILTER ROBUSTNESS - Conservative for convergence
        config.filter.main.error_floor = 0.001f;     // Higher error floor for stability
        config.filter.main.error_ceil = 2.0f;        // Lower error ceiling for convergence
        config.filter.main_initial.leakage_converged = 0.01f;    // Higher initial leakage for faster convergence
        config.filter.main_initial.leakage_diverged = 0.05f;     // Conservative initial divergence
        
        // 🎯 OPTIMIZED SUPPRESSOR TUNING FOR CONVERGENCE (2025-01-31)
        // More conservative settings to help filter convergence
        config.suppressor.normal_tuning.max_dec_factor_lf = std::min(max_dec_factor_lf_, 2.0f);    // Limit suppression
        config.suppressor.normal_tuning.max_inc_factor = std::min(max_inc_factor_, 2.0f);          // Gentle recovery
        config.suppressor.nearend_tuning.max_dec_factor_lf = std::min(nearend_max_dec_factor_lf_, 1.0f);  // Preserve voice
        config.suppressor.nearend_tuning.max_inc_factor = std::min(nearend_max_inc_factor_, 2.5f);        // Clear voice recovery
        
        // 🎯 PRODUCTION-GRADE DELAY ESTIMATION FOR CONVERGENCE (2025-01-31)
        config.delay.down_sampling_factor = (delay_down_sampling_factor_ > 0) ? delay_down_sampling_factor_ : 2;
        config.delay.num_filters = (delay_num_filters_ > 0) ? delay_num_filters_ : 16;  // Standard count for stability
        config.delay.delay_estimate_smoothing = (delay_estimate_smoothing_ > 0.0f) ? delay_estimate_smoothing_ : 0.95f;  // Balanced smoothing
        config.delay.delay_candidate_detection_threshold = 0.3f;  // Less sensitive for stability
        
        // 🎯 ENABLE INTERNAL DELAY ESTIMATOR FOR PRECISE TIMING SYNC
        config.delay.use_external_delay_estimator = false;  // Use WebRTC's built-in estimator
        config.delay.log_warning_on_delay_changes = true;   // Enable delay change logging
        
        // 🎯 DOMINANT NEAREND DETECTION OPTIMIZATION
        config.suppressor.dominant_nearend_detection.enr_threshold = enr_threshold_;
        config.suppressor.dominant_nearend_detection.snr_threshold = snr_threshold_;
        config.suppressor.dominant_nearend_detection.hold_duration = hold_duration_;
        config.suppressor.dominant_nearend_detection.trigger_threshold = trigger_threshold_;
        
        LOGI("🎯 Production-Grade AEC3 configured: filter_length=%zu, max_dec_lf=%.1f, delay_filters=%d", 
             config.filter.main.length_blocks, config.suppressor.normal_tuning.max_dec_factor_lf, config.delay.num_filters);
        
        // Apply runtime adjustable parameters
        if (config_change_duration_blocks_ > 0) {
            config.filter.config_change_duration_blocks = config_change_duration_blocks_;
        }
        if (initial_state_seconds_ > 0.0f) {
            config.filter.initial_state_seconds = initial_state_seconds_;
        }
        config.filter.conservative_initial_phase = conservative_initial_phase_;
        
        // Create AEC3 factory and controller
        aec_factory_ = std::make_unique<webrtc::EchoCanceller3Factory>(config);
        if (!aec_factory_) {
            LOGE("Failed to create Production-Grade AEC3 factory");
            return false;
        }

        echo_controller_ = aec_factory_->Create(kSampleRate, kChannels, kChannels);
        if (!echo_controller_) {
            LOGE("Failed to create Production-Grade AEC3 controller");
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
        
        // 🎯 Initialize Production-Grade ERLE Monitoring (2025-01-31)
        erle_monitoring_counter_ = 0;
        production_timing_sync_enabled_ = true;
        clockdrift_detection_enabled_ = true;
        last_erle_fullband_log2_ = 0.0;
        adaptive_filter_convergence_counter_ = 0;
        is_adaptive_filter_converged_ = false;
        erle_history_.clear();
        
        // Initialize production metrics
        current_production_metrics_ = {};
        current_production_metrics_.timing_sync_accuracy_ms = kTimingToleranceMs;
        
        LOGI("Production-Grade WebRTC AEC3 initialized: %dHz, %d channels, %dms delay (ERLE target: %.1fdB)", 
             kSampleRate, kChannels, kStreamDelay, kTargetErleDb);
        return true;
    } catch (const std::exception& e) {
        LOGE("Exception during Production-Grade AEC3 initialization: %s", e.what());
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
        // 🎯 Enhanced reference signal processing for production-grade performance
        if (timing_sync_enabled_) {
            double frame_energy = CalculateFrameEnergy(tts_data, length);
            
            // Production-grade signal normalization for consistent ERLE
            std::vector<int16_t> normalized_tts_data(tts_data, tts_data + length);
            
            // Apply smooth energy normalization instead of aggressive enhancement
            if (frame_energy > 0.0) {
                // Target consistent energy level for filter convergence
                constexpr double kTargetEnergyLevel = 50000.0;  // Consistent target for AEC3
                
                if (frame_energy < kTargetEnergyLevel * 0.1) {  // Very weak signals
                    double gain = std::sqrt(kTargetEnergyLevel * 0.3 / frame_energy);
                    gain = std::min(gain, 2.0);  // Limit gain to prevent distortion
                    
                    for (size_t i = 0; i < normalized_tts_data.size(); ++i) {
                        int32_t amplified = static_cast<int32_t>(normalized_tts_data[i] * gain);
                        normalized_tts_data[i] = static_cast<int16_t>(
                            std::max(static_cast<int32_t>(INT16_MIN),
                                   std::min(static_cast<int32_t>(INT16_MAX), amplified)));
                    }
                    
                    double new_energy = CalculateFrameEnergy(normalized_tts_data.data(), length);
                    LOGV("🎯 Normalized TTS signal: energy %.1f -> %.1f (gain=%.2f)", 
                         frame_energy, new_energy, gain);
                }
            }
            
            TimedFrame timed_frame(normalized_tts_data.data(), length, frame_counter_++);
            render_buffer_.push_back(std::move(timed_frame));
            
            // Maintain buffer size for optimal delay range
            if (render_buffer_.size() > kDelayBufferSize) {
                render_buffer_.pop_front();
            }
        }
        
        // Create AudioFrame from input data (following WebRTC production pipeline)
        webrtc::AudioFrame render_frame;
        render_frame.UpdateFrame(0, tts_data, kFrameSize, kSampleRate, 
                               webrtc::AudioFrame::kNormalSpeech, 
                               webrtc::AudioFrame::kVadActive, kChannels);

        // Follow WebRTC production pipeline exactly for maximum ERLE
        audio_render_buffer_->CopyFrom(&render_frame);
        audio_render_buffer_->SplitIntoFrequencyBands();
        echo_controller_->AnalyzeRender(audio_render_buffer_.get());
        audio_render_buffer_->MergeFrequencyBands();

        total_render_frames_++;
        
        double render_energy = CalculateFrameEnergy(tts_data, length);
        
        LOGV("Production TTS reference processed: frame=%llu, energy=%.2f, buffer_size=%zu", 
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
        
        // Extended initialization stabilization for production
        initialization_frames_++;
        if (initialization_frames_ >= kInitializationFrames && !is_initialization_complete_) {
            is_initialization_complete_ = true;
            LOGI("🎯 Production initialization complete after %d frames - AEC3 ready for optimal ERLE", 
                 initialization_frames_);
        }
        
        // 🎯 Production-Grade Timing Synchronization with Built-in Estimators (2025-01-31)
        if (production_timing_sync_enabled_) {
            webrtc::EchoControl::Metrics current_metrics = echo_controller_->GetMetrics();
            int aec3_detected_delay = current_metrics.delay_ms;
            
            // Use WebRTC's built-in delay estimator for production-grade accuracy
            if (is_initialization_complete_) {
                if (aec3_detected_delay > 0 && aec3_detected_delay <= 400) {
                    // WebRTC's internal delay estimator is working - use it
                    int new_optimal_delay = aec3_detected_delay;
                    if (std::abs(new_optimal_delay - current_optimal_delay_ms_) > 10) {
                        current_optimal_delay_ms_ = new_optimal_delay;
                        echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
                        LOGI("🎯 Production delay update from WebRTC estimator: %dms", current_optimal_delay_ms_);
                    }
                } else {
                    // Fallback to timing-based estimation for problematic devices
                    const TimedFrame* best_reference = FindOptimalReferenceFrame(capture_timestamp);
                    if (best_reference) {
                        int timing_based_delay = EstimateOptimalDelay(capture_timestamp, best_reference->timestamp);
                        if (timing_based_delay >= kMinDelayMs && timing_based_delay <= kMaxDelayMs) {
                            current_optimal_delay_ms_ = timing_based_delay;
                            echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
                            LOGI("🎯 Production fallback to timing-based delay: %dms", current_optimal_delay_ms_);
                        }
                    }
                }
            }
        }
        
        // Create AudioFrame from input data (WebRTC production pipeline)
        webrtc::AudioFrame capture_frame;
        capture_frame.UpdateFrame(0, mic_data, kFrameSize, kSampleRate,
                                webrtc::AudioFrame::kNormalSpeech,
                                webrtc::AudioFrame::kVadActive, kChannels);

        // Follow WebRTC production pipeline exactly for maximum ERLE
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
        
        // 🎯 Production-Grade ERLE Monitoring and Optimization
        if (++erle_monitoring_counter_ >= kErleMonitoringFrames) {
            UpdateProductionErleEstimates();
            MonitorAdaptiveFilterConvergence();
            
            // Optimize performance based on ERLE estimates
            if (is_initialization_complete_) {
                OptimizeBasedOnErleEstimates();
            }
            
            erle_monitoring_counter_ = 0;
        }
        
        // 🎯 Production-Grade Delay Estimation Optimization
        if (++delay_estimation_counter_ >= kDelayEstimationFrames) {
            AdaptDelayBasedOnPathEstimator();
            
            // Handle clock drift detection if enabled
            if (clockdrift_detection_enabled_) {
                HandleClockdriftDetection();
            }
            
            delay_estimation_counter_ = 0;
        }
        
        // Calculate energy for quality assessment
        double capture_energy = CalculateFrameEnergy(mic_data, length);
        double output_energy = CalculateFrameEnergy(output_data, length);
        double suppression_ratio = capture_energy > 0 ? output_energy / capture_energy : 1.0;
        
        LOGV("🎯 Production AEC3: frame=%llu, delay=%dms, suppression=%.3f, ERLE=%.1fdB", 
             (unsigned long long)total_capture_frames_, current_optimal_delay_ms_, 
             suppression_ratio, current_production_metrics_.erle_fullband_log2 * 3.01);  // Convert log2 to dB
        
        // 🎯 Real-time clean audio buffering
        {
            std::lock_guard<std::mutex> buffer_lock(clean_audio_buffer_mutex_);
            std::vector<float> cleanFrame(kFrameSize);
            for (size_t i = 0; i < kFrameSize; ++i) {
                cleanFrame[i] = output_data[i] / 32768.0f; // Convert int16 to float [-1.0, 1.0]
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

// 🎯 Production-Grade ERLE Performance Methods Implementation

ProductionErleMetrics WqAec3Processor::GetProductionErleMetrics() {
    std::lock_guard<std::mutex> lock(production_metrics_mutex_);
    return current_production_metrics_;
}

bool WqAec3Processor::OptimizeProductionErlePerformance() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!echo_controller_) return false;
    
    try {
        // Force update of production ERLE estimates
        UpdateProductionErleEstimates();
        
        // Perform optimization based on current estimates
        OptimizeBasedOnErleEstimates();
        
        // Recalibrate delay estimation
        RecalibrateDelayEstimation();
        
        LOGI("🎯 Production ERLE optimization completed: ERLE=%.1fdB, Filter converged=%s", 
             current_production_metrics_.erle_fullband_log2 * 3.01,
             current_production_metrics_.filter_converged ? "Yes" : "No");
        
        return true;
    } catch (const std::exception& e) {
        LOGE("Exception in OptimizeProductionErlePerformance: %s", e.what());
        return false;
    }
}

bool WqAec3Processor::EnableProductionTimingSync(bool enable_precise_sync, bool enable_clockdrift_detection) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    production_timing_sync_enabled_ = enable_precise_sync;
    clockdrift_detection_enabled_ = enable_clockdrift_detection;
    
    if (!enable_precise_sync) {
        render_buffer_.clear();
    }
    
    LOGI("🎯 Production timing sync: precise=%s, clockdrift=%s", 
         enable_precise_sync ? "enabled" : "disabled",
         enable_clockdrift_detection ? "enabled" : "disabled");
    
    return true;
}

bool WqAec3Processor::RecalibrateDelayEstimation() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!echo_controller_) return false;
    
    try {
        // Reset delay estimation counters
        delay_estimation_counter_ = 0;
        last_delay_estimation_ = 0;
        
        // Clear render buffer for fresh estimation
        render_buffer_.clear();
        
        // Force delay recalibration by setting a temporary delay
        int temp_delay = current_optimal_delay_ms_;
        echo_controller_->SetAudioBufferDelay(temp_delay + 10);
        echo_controller_->SetAudioBufferDelay(temp_delay);
        
        LOGI("🎯 Production delay estimation recalibrated: %dms", temp_delay);
        return true;
    } catch (const std::exception& e) {
        LOGE("Exception in RecalibrateDelayEstimation: %s", e.what());
        return false;
    }
}

// 🎯 WebRTC Built-in Estimator Integration Methods Implementation

void WqAec3Processor::UpdateProductionErleEstimates() {
    if (!echo_controller_) return;
    
    try {
        webrtc::EchoControl::Metrics metrics = echo_controller_->GetMetrics();
        
        std::lock_guard<std::mutex> metrics_lock(production_metrics_mutex_);
        
        // Update production metrics with WebRTC's built-in estimator data
        current_production_metrics_.erl_estimate = metrics.echo_return_loss;
        current_production_metrics_.erle_fullband_log2 = metrics.echo_return_loss_enhancement / 3.01; // Convert dB to log2
        current_production_metrics_.matched_filter_delay_samples = metrics.delay_ms * kSampleRate / 1000;
        current_production_metrics_.delay_estimate_reliable = (metrics.delay_ms > 0 && metrics.delay_ms <= 400);
        
        // Track ERLE history for convergence analysis
        erle_history_.push_back(current_production_metrics_.erle_fullband_log2);
        if (erle_history_.size() > 100) {
            erle_history_.erase(erle_history_.begin());
        }
        
        // Calculate subband average (simplified for production use)
        current_production_metrics_.erle_subband_average = current_production_metrics_.erle_fullband_log2;
        
        // Update timing sync accuracy
        current_production_metrics_.timing_sync_accuracy_ms = kTimingToleranceMs;
        
        last_erle_fullband_log2_ = current_production_metrics_.erle_fullband_log2;
        
    } catch (const std::exception& e) {
        LOGE("Exception in UpdateProductionErleEstimates: %s", e.what());
    }
}

void WqAec3Processor::MonitorAdaptiveFilterConvergence() {
    if (erle_history_.size() < 10) return;
    
    try {
        // Check for filter convergence based on ERLE stability
        double recent_erle_avg = 0.0;
        double older_erle_avg = 0.0;
        
        size_t recent_start = erle_history_.size() - 5;
        for (size_t i = recent_start; i < erle_history_.size(); ++i) {
            recent_erle_avg += erle_history_[i];
        }
        recent_erle_avg /= 5.0;
        
        size_t older_start = erle_history_.size() - 10;
        for (size_t i = older_start; i < recent_start; ++i) {
            older_erle_avg += erle_history_[i];
        }
        older_erle_avg /= 5.0;
        
        // Filter is considered converged if ERLE is stable and above minimum threshold
        bool erle_stable = std::abs(recent_erle_avg - older_erle_avg) < 0.5; // 1.5dB stability threshold
        bool erle_sufficient = recent_erle_avg * 3.01 >= kMinAcceptableErleDb; // Convert to dB
        
        if (erle_stable && erle_sufficient) {
            adaptive_filter_convergence_counter_++;
            if (adaptive_filter_convergence_counter_ >= 3) {
                is_adaptive_filter_converged_ = true;
                current_production_metrics_.filter_converged = true;
                current_production_metrics_.linear_filter_quality = std::min(1.0, recent_erle_avg * 3.01 / kTargetErleDb);
            }
        } else {
            adaptive_filter_convergence_counter_ = 0;
            is_adaptive_filter_converged_ = false;
            current_production_metrics_.filter_converged = false;
            current_production_metrics_.linear_filter_quality = recent_erle_avg * 3.01 / kTargetErleDb;
        }
        
    } catch (const std::exception& e) {
        LOGE("Exception in MonitorAdaptiveFilterConvergence: %s", e.what());
    }
}

void WqAec3Processor::OptimizeBasedOnErleEstimates() {
    if (!echo_controller_) return;
    
    try {
        double current_erle_db = current_production_metrics_.erle_fullband_log2 * 3.01;
        
        // If ERLE is below target, try optimization strategies
        if (current_erle_db < kTargetErleDb) {
            // Strategy 1: Adjust delay if it seems unreliable
            if (!current_production_metrics_.delay_estimate_reliable) {
                RecalibrateDelayEstimation();
                LOGI("🎯 ERLE optimization: Recalibrated delay (ERLE=%.1fdB < target=%.1fdB)", 
                     current_erle_db, kTargetErleDb);
            }
            
            // Strategy 2: If ERLE is very low, reset filter convergence
            if (current_erle_db < kMinAcceptableErleDb) {
                is_adaptive_filter_converged_ = false;
                adaptive_filter_convergence_counter_ = 0;
                LOGW("🎯 ERLE critically low: Reset filter convergence (ERLE=%.1fdB)", current_erle_db);
            }
        }
        
    } catch (const std::exception& e) {
        LOGE("Exception in OptimizeBasedOnErleEstimates: %s", e.what());
    }
}

void WqAec3Processor::HandleClockdriftDetection() {
    if (!echo_controller_) return;
    
    try {
        webrtc::EchoControl::Metrics metrics = echo_controller_->GetMetrics();
        
        // WebRTC's internal clockdrift detection (simplified interpretation)
        // In a full implementation, you would access the actual clockdrift detector
        bool has_clockdrift = (metrics.delay_ms > 0 && 
                              std::abs(metrics.delay_ms - current_optimal_delay_ms_) > 50);
        
        if (has_clockdrift) {
            current_production_metrics_.clockdrift_level = 1.0;  // Detected
            LOGW("🎯 Clock drift detected: AEC3_delay=%dms, optimal=%dms", 
                 metrics.delay_ms, current_optimal_delay_ms_);
        } else {
            current_production_metrics_.clockdrift_level = 0.0;  // Not detected
        }
        
    } catch (const std::exception& e) {
        LOGE("Exception in HandleClockdriftDetection: %s", e.what());
    }
}

void WqAec3Processor::AdaptDelayBasedOnPathEstimator() {
    if (!echo_controller_) return;
    
    try {
        webrtc::EchoControl::Metrics current_metrics = echo_controller_->GetMetrics();
        int estimated_delay = current_metrics.delay_ms;
        
        // Use WebRTC's built-in delay estimator if reliable
        if (estimated_delay > 0 && estimated_delay <= 400 && 
            std::abs(estimated_delay - current_optimal_delay_ms_) > 15) {
            
            int old_delay = current_optimal_delay_ms_;
            current_optimal_delay_ms_ = estimated_delay;
            echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
            
            LOGI("🎯 Production delay adaptation: %dms -> %dms (WebRTC estimator)", 
                 old_delay, current_optimal_delay_ms_);
        }
        
    } catch (const std::exception& e) {
        LOGE("Exception in AdaptDelayBasedOnPathEstimator: %s", e.what());
    }
}

// Continue with existing methods...
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
        current_optimal_delay_ms_ = delay_ms;
        LOGI("🎯 Production stream delay set: %dms", delay_ms);
        echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
    }
}

bool WqAec3Processor::EnableTimingSync(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    timing_sync_enabled_ = enable;
    production_timing_sync_enabled_ = enable;
    LOGI("🎯 Production timing synchronization %s", enable ? "enabled" : "disabled");
    
    if (!enable) {
        render_buffer_.clear();
    }
    
    return true;
}

bool WqAec3Processor::AutoOptimizeDelay() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!echo_controller_) return false;
    
    try {
        // Use production-grade optimization
        OptimizeProductionErlePerformance();
        LOGI("🎯 Production auto delay optimization completed: %dms", current_optimal_delay_ms_);
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
        
        // Use WebRTC's built-in delay estimator for production optimization
        if (aec3_delay > 0 && aec3_delay <= 400) {
            if (std::abs(aec3_delay - current_optimal_delay_ms_) > 20) {
                current_optimal_delay_ms_ = aec3_delay;
                echo_controller_->SetAudioBufferDelay(current_optimal_delay_ms_);
                LOGI("🎯 Production delay optimization: %dms (ERLE: %.1fdB)", current_optimal_delay_ms_, current_erle);
            }
        }
        
        last_delay_estimation_ = aec3_delay;
        
    } catch (const std::exception& e) {
        LOGE("Exception in production delay optimization: %s", e.what());
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
