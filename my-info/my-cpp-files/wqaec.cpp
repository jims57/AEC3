#include "wqaec.h"
#include "wqaec_config.h"

// WebRTC includes
#include "api/echo_canceller3_factory.h"
#include "api/echo_canceller3_config.h"
#include "audio_processing/audio_buffer.h"
#include "audio_processing/audio_frame.h"
#include "audio_processing/high_pass_filter.h"

#include <memory>
#include <mutex>
#include <vector>
#include <chrono>
#include <atomic>
#include <cstring>

#ifdef __ANDROID__
#include <android/log.h>
#define LOG_TAG "WQAEC"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGV(...) if(debug_logging_enabled) __android_log_print(ANDROID_LOG_VERBOSE, LOG_TAG, __VA_ARGS__)
#elif defined(__APPLE__)
#include <os/log.h>
#define LOGI(...) os_log(OS_LOG_DEFAULT, __VA_ARGS__)
#define LOGE(...) os_log_error(OS_LOG_DEFAULT, __VA_ARGS__)
#define LOGD(...) os_log_debug(OS_LOG_DEFAULT, __VA_ARGS__)
#define LOGV(...) if(debug_logging_enabled) os_log_debug(OS_LOG_DEFAULT, __VA_ARGS__)
#else
#include <iostream>
#define LOGI(...) printf(__VA_ARGS__); printf("\n")
#define LOGE(...) printf(__VA_ARGS__); printf("\n")
#define LOGD(...) printf(__VA_ARGS__); printf("\n")
#define LOGV(...) if(debug_logging_enabled) { printf(__VA_ARGS__); printf("\n"); }
#endif

namespace wqaec {

// Global debug logging flag
static std::atomic<bool> debug_logging_enabled{false};

/**
 * Production implementation of WQAEC with automatic timing synchronization
 * 
 * This implementation provides:
 * 1. Automatic delay detection using 5-filter NLMS approach (from WebRTC research)
 * 2. Real-time adaptive parameter optimization
 * 3. Cross-platform compatibility (Android/iOS)
 * 4. Production-grade performance and stability
 * 
 * Based on research papers:
 * - WebRTC AEC3 timing alignment (5 NLMS filters)
 * - Adaptive filter convergence detection
 * - Mobile audio processing optimization
 */
class WQAECImpl : public WQAEC {
private:
    // Core WebRTC AEC3 components
    std::unique_ptr<webrtc::EchoCanceller3Factory> aec3_factory_;
    std::unique_ptr<webrtc::EchoControl> echo_controller_;
    std::unique_ptr<webrtc::AudioBuffer> render_buffer_;
    std::unique_ptr<webrtc::AudioBuffer> capture_buffer_;
    std::unique_ptr<webrtc::HighPassFilter> high_pass_filter_;
    
    // WQAEC components for automatic optimization
    std::unique_ptr<TimingSyncManager> timing_sync_;
    AEC3Config config_;
    
    // Thread safety
    mutable std::mutex processing_mutex_;
    mutable std::mutex config_mutex_;
    
    // State management
    std::atomic<bool> initialized_{false};
    std::atomic<bool> destroyed_{false};
    
    // Performance monitoring
    std::atomic<int> frames_processed_{0};
    std::atomic<int> adaptations_count_{0};
    std::atomic<int64_t> total_processing_time_us_{0};
    std::atomic<float> current_erle_{0.0f};
    std::atomic<float> current_convergence_{0.0f};
    
    // Timing management
    std::chrono::steady_clock::time_point last_adaptation_time_;
    std::chrono::steady_clock::time_point last_metrics_update_;
    
    // Callbacks
    DelayDetectionCallback delay_callback_;
    PerformanceCallback performance_callback_;
    AdaptationCallback adaptation_callback_;
    
    // Adaptive state
    int last_detected_delay_ms_ = 0;
    float adaptation_confidence_ = 0.0f;
    bool timing_synced_ = false;
    
public:
    explicit WQAECImpl(const AEC3Config& config) : config_(config) {
        last_adaptation_time_ = std::chrono::steady_clock::now();
        last_metrics_update_ = last_adaptation_time_;
        
        LOGI("WQAEC created with environment=%d, auto_delay=%s", 
             static_cast<int>(config_.environment),
             config_.adaptation.enable_auto_delay ? "enabled" : "disabled");
    }
    
    ~WQAECImpl() override {
        Destroy();
    }
    
    // =================================================================
    // Core API Implementation
    // =================================================================
    
    bool Initialize() override {
        std::lock_guard<std::mutex> lock(processing_mutex_);
        
        if (initialized_.load()) {
            LOGW("WQAEC already initialized");
            return true;
        }
        
        if (destroyed_.load()) {
            LOGE("Cannot initialize destroyed WQAEC instance");
            return false;
        }
        
        try {
            LOGI("Initializing WQAEC with WebRTC AEC3...");
            
            // Validate configuration
            if (!config_.Validate()) {
                LOGE("Invalid AEC3 configuration");
                return false;
            }
            
            // Create WebRTC AEC3 configuration with production optimizations
            webrtc::EchoCanceller3Config webrtc_config = CreateOptimizedWebRTCConfig();
            
            // Create AEC3 factory and controller
            aec3_factory_ = std::make_unique<webrtc::EchoCanceller3Factory>(webrtc_config);
            if (!aec3_factory_) {
                LOGE("Failed to create WebRTC AEC3 factory");
                return false;
            }
            
            echo_controller_ = aec3_factory_->Create(
                AEC3Config::SAMPLE_RATE, 
                AEC3Config::CHANNELS, 
                AEC3Config::CHANNELS
            );
            if (!echo_controller_) {
                LOGE("Failed to create WebRTC AEC3 controller");
                return false;
            }
            
            // Create audio buffers (all with identical rates to prevent resampling)
            render_buffer_ = std::make_unique<webrtc::AudioBuffer>(
                AEC3Config::SAMPLE_RATE, AEC3Config::CHANNELS,
                AEC3Config::SAMPLE_RATE, AEC3Config::CHANNELS,
                AEC3Config::SAMPLE_RATE, AEC3Config::CHANNELS
            );
            
            capture_buffer_ = std::make_unique<webrtc::AudioBuffer>(
                AEC3Config::SAMPLE_RATE, AEC3Config::CHANNELS,
                AEC3Config::SAMPLE_RATE, AEC3Config::CHANNELS,
                AEC3Config::SAMPLE_RATE, AEC3Config::CHANNELS
            );
            
            if (!render_buffer_ || !capture_buffer_) {
                LOGE("Failed to create audio buffers");
                return false;
            }
            
            // Create high-pass filter
            high_pass_filter_ = std::make_unique<webrtc::HighPassFilter>(
                AEC3Config::SAMPLE_RATE, 
                AEC3Config::CHANNELS
            );
            if (!high_pass_filter_) {
                LOGE("Failed to create high-pass filter");
                return false;
            }
            
            // Initialize timing synchronization manager
            timing_sync_ = std::make_unique<TimingSyncManager>(config_);
            if (!timing_sync_) {
                LOGE("Failed to create timing sync manager");
                return false;
            }
            
            // Set initial stream delay
            echo_controller_->SetAudioBufferDelay(config_.current_delay_ms);
            
            // Reset performance counters
            frames_processed_.store(0);
            adaptations_count_.store(0);
            total_processing_time_us_.store(0);
            current_erle_.store(0.0f);
            current_convergence_.store(0.0f);
            
            initialized_.store(true);
            
            LOGI("WQAEC initialized successfully: %dHz, %d channels, %dms initial delay",
                 AEC3Config::SAMPLE_RATE, AEC3Config::CHANNELS, config_.current_delay_ms);
            
            return true;
            
        } catch (const std::exception& e) {
            LOGE("Exception during WQAEC initialization: %s", e.what());
            return false;
        }
    }
    
    bool ProcessReferenceAudio(const int16_t* reference_audio, size_t length) override {
        if (!ValidateProcessingState(length)) {
            return false;
        }
        
        auto start_time = std::chrono::steady_clock::now();
        
        std::lock_guard<std::mutex> lock(processing_mutex_);
        
        try {
            // Get current timestamp for timing analysis
            int64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                start_time.time_since_epoch()).count();
            
            // Analyze reference frame for timing synchronization
            if (timing_sync_) {
                timing_sync_->AnalyzeReferenceFrame(reference_audio, length, timestamp_us);
            }
            
            // Create AudioFrame from input data
            webrtc::AudioFrame render_frame;
            render_frame.UpdateFrame(
                0, reference_audio, length, AEC3Config::SAMPLE_RATE,
                webrtc::AudioFrame::kNormalSpeech,
                webrtc::AudioFrame::kVadActive, 
                AEC3Config::CHANNELS
            );
            
            // Process through WebRTC AEC3 pipeline
            render_buffer_->CopyFrom(&render_frame);
            render_buffer_->SplitIntoFrequencyBands();
            echo_controller_->AnalyzeRender(render_buffer_.get());
            render_buffer_->MergeFrequencyBands();
            
            // Update processing statistics
            auto end_time = std::chrono::steady_clock::now();
            auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - start_time).count();
            
            total_processing_time_us_.fetch_add(processing_time);
            
            LOGV("Processed reference audio: %zu samples, %lld us", length, processing_time);
            
            return true;
            
        } catch (const std::exception& e) {
            LOGE("Exception in ProcessReferenceAudio: %s", e.what());
            return false;
        }
    }
    
    bool ProcessCaptureAudio(const int16_t* capture_audio, int16_t* clean_output, size_t length) override {
        if (!ValidateProcessingState(length)) {
            return false;
        }
        
        auto start_time = std::chrono::steady_clock::now();
        
        std::lock_guard<std::mutex> lock(processing_mutex_);
        
        try {
            // Get current timestamp for timing analysis
            int64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                start_time.time_since_epoch()).count();
            
            // Analyze capture frame for timing synchronization
            if (timing_sync_) {
                timing_sync_->AnalyzeCaptureFrame(capture_audio, length, timestamp_us);
                
                // Check if we should perform automatic adaptation
                if (ShouldPerformAdaptation()) {
                    PerformAutomaticAdaptation();
                }
            }
            
            // Create AudioFrame from input data
            webrtc::AudioFrame capture_frame;
            capture_frame.UpdateFrame(
                0, capture_audio, length, AEC3Config::SAMPLE_RATE,
                webrtc::AudioFrame::kNormalSpeech,
                webrtc::AudioFrame::kVadActive,
                AEC3Config::CHANNELS
            );
            
            // Process through WebRTC AEC3 pipeline
            capture_buffer_->CopyFrom(&capture_frame);
            capture_buffer_->SplitIntoFrequencyBands();
            
            // Apply high-pass filter
            high_pass_filter_->Process(capture_buffer_.get(), true);
            
            // Set current delay (may have been updated by adaptation)
            echo_controller_->SetAudioBufferDelay(config_.current_delay_ms);
            
            // Core AEC3 processing
            echo_controller_->AnalyzeCapture(capture_buffer_.get());
            echo_controller_->ProcessCapture(capture_buffer_.get(), false);
            
            // Merge frequency bands and copy output
            capture_buffer_->MergeFrequencyBands();
            capture_buffer_->CopyTo(&capture_frame);
            
            // Copy processed data to output
            std::memcpy(clean_output, capture_frame.data(), length * sizeof(int16_t));
            
            // Update performance statistics
            int frame_count = frames_processed_.fetch_add(1) + 1;
            
            auto end_time = std::chrono::steady_clock::now();
            auto processing_time = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - start_time).count();
            
            total_processing_time_us_.fetch_add(processing_time);
            
            // Update metrics periodically
            if (frame_count % 100 == 0) {
                UpdatePerformanceMetrics();
            }
            
            LOGV("Processed capture audio: %zu samples, %lld us, frame=%d", 
                 length, processing_time, frame_count);
            
            return true;
            
        } catch (const std::exception& e) {
            LOGE("Exception in ProcessCaptureAudio: %s", e.what());
            return false;
        }
    }
    
    AEC3Metrics GetMetrics() const override {
        if (!initialized_.load() || destroyed_.load()) {
            return AEC3Metrics{};
        }
        
        std::lock_guard<std::mutex> lock(processing_mutex_);
        
        AEC3Metrics metrics;
        
        try {
            // Get WebRTC AEC3 metrics
            if (echo_controller_) {
                webrtc::EchoControl::Metrics webrtc_metrics = echo_controller_->GetMetrics();
                metrics.echo_return_loss = webrtc_metrics.echo_return_loss;
                metrics.echo_return_loss_enhancement = webrtc_metrics.echo_return_loss_enhancement;
                metrics.detected_delay_ms = webrtc_metrics.delay_ms;
            }
            
            // Add WQAEC-specific metrics
            metrics.set_delay_ms = config_.current_delay_ms;
            metrics.adaptation_count = adaptations_count_.load();
            
            if (timing_sync_) {
                metrics.signal_energy = timing_sync_->GetReferenceEnergy();
                metrics.filter_convergence = timing_sync_->GetFilterConvergence();
                metrics.is_echo_detected = (metrics.echo_return_loss_enhancement > 1.0);
                metrics.is_voice_detected = (timing_sync_->GetCaptureEnergy() > config_.MIN_ENERGY_THRESHOLD);
            }
            
            // Update cached values
            current_erle_.store(static_cast<float>(metrics.echo_return_loss_enhancement));
            current_convergence_.store(metrics.filter_convergence);
            
        } catch (const std::exception& e) {
            LOGE("Exception in GetMetrics: %s", e.what());
        }
        
        return metrics;
    }
    
    void Destroy() override {
        if (destroyed_.exchange(true)) {
            return;  // Already destroyed
        }
        
        LOGI("Destroying WQAEC instance...");
        
        std::lock_guard<std::mutex> lock(processing_mutex_);
        
        // Clear callbacks to prevent use after destruction
        delay_callback_ = nullptr;
        performance_callback_ = nullptr;
        adaptation_callback_ = nullptr;
        
        // Destroy WebRTC components
        echo_controller_.reset();
        aec3_factory_.reset();
        render_buffer_.reset();
        capture_buffer_.reset();
        high_pass_filter_.reset();
        
        // Destroy WQAEC components
        timing_sync_.reset();
        
        initialized_.store(false);
        
        LOGI("WQAEC destroyed successfully");
    }
    
    // =================================================================
    // Configuration API Implementation
    // =================================================================
    
    void SetStreamDelay(int delay_ms) override {
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        delay_ms = std::max(AEC3Config::MIN_DELAY_MS, 
                           std::min(AEC3Config::MAX_DELAY_MS, delay_ms));
        
        int old_delay = config_.current_delay_ms;
        config_.current_delay_ms = delay_ms;
        
        // Disable auto-delay when manual delay is set
        if (config_.adaptation.enable_auto_delay) {
            config_.adaptation.enable_auto_delay = false;
            LOGI("Auto-delay detection disabled due to manual delay setting");
        }
        
        LOGI("Stream delay updated: %dms -> %dms (manual)", old_delay, delay_ms);
        
        // Trigger callback if set
        if (delay_callback_) {
            try {
                delay_callback_(old_delay, delay_ms, 1.0f);  // Manual setting = 100% confidence
            } catch (...) {
                LOGE("Exception in delay detection callback");
            }
        }
    }
    
    void SetEchoSuppression(float strength) override {
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        strength = std::max(8.0f, std::min(20.0f, strength));
        config_.suppression.echo_suppression = strength;
        config_.environment = AEC3Config::Environment::CUSTOM;
        
        LOGI("Echo suppression updated to %.1f", strength);
        
        // Note: WebRTC AEC3 requires reinitialization for some parameter changes
        // For production apps, consider marking for reinitialization on next opportunity
    }
    
    void SetVoiceRecovery(float speed) override {
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        speed = std::max(1.0f, std::min(5.0f, speed));
        config_.suppression.voice_recovery = speed;
        config_.environment = AEC3Config::Environment::CUSTOM;
        
        LOGI("Voice recovery updated to %.1f", speed);
    }
    
    void SetVoiceProtection(float level) override {
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        level = std::max(1.0f, std::min(8.0f, level));
        config_.suppression.voice_protection = level;
        config_.environment = AEC3Config::Environment::CUSTOM;
        
        LOGI("Voice protection updated to %.1f", level);
    }
    
    void SetEnvironment(AEC3Config::Environment env) override {
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        config_.SetEnvironmentPreset(env);
        
        LOGI("Environment preset applied: %d", static_cast<int>(env));
    }
    
    // =================================================================
    // Advanced API Implementation
    // =================================================================
    
    void EnableAutoDelayDetection(bool enable) override {
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        config_.adaptation.enable_auto_delay = enable;
        
        if (enable && timing_sync_) {
            timing_sync_->ResetAdaptation();
        }
        
        LOGI("Auto-delay detection %s", enable ? "enabled" : "disabled");
    }
    
    void EnableEnergyAdaptation(bool enable) override {
        std::lock_guard<std::mutex> lock(config_mutex_);
        
        config_.adaptation.enable_energy_adaptation = enable;
        
        LOGI("Energy adaptation %s", enable ? "enabled" : "disabled");
    }
    
    void TriggerAdaptation() override {
        if (!initialized_.load() || destroyed_.load()) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(processing_mutex_);
        
        if (timing_sync_) {
            timing_sync_->ResetAdaptation();
            adaptations_count_.fetch_add(1);
            
            LOGI("Manual adaptation triggered");
            
            if (adaptation_callback_) {
                try {
                    adaptation_callback_("manual_trigger", 1.0f);
                } catch (...) {
                    LOGE("Exception in adaptation callback");
                }
            }
        }
    }
    
    void ResetAdaptiveState() override {
        if (!initialized_.load() || destroyed_.load()) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(processing_mutex_);
        
        if (timing_sync_) {
            timing_sync_->ResetAdaptation();
            
            // Reset performance counters
            frames_processed_.store(0);
            adaptations_count_.store(0);
            total_processing_time_us_.store(0);
            current_erle_.store(0.0f);
            current_convergence_.store(0.0f);
            
            timing_synced_ = false;
            adaptation_confidence_ = 0.0f;
            
            LOGI("Adaptive state reset");
        }
    }
    
    void SetDelayDetectionCallback(DelayDetectionCallback callback) override {
        delay_callback_ = callback;
    }
    
    void SetPerformanceCallback(PerformanceCallback callback) override {
        performance_callback_ = callback;
    }
    
    void SetAdaptationCallback(AdaptationCallback callback) override {
        adaptation_callback_ = callback;
    }
    
    // =================================================================
    // Utility API Implementation
    // =================================================================
    
    AEC3Config GetCurrentConfig() const override {
        std::lock_guard<std::mutex> lock(config_mutex_);
        return config_;
    }
    
    bool UpdateConfig(const AEC3Config& config) override {
        if (!config.Validate()) {
            LOGE("Invalid configuration provided");
            return false;
        }
        
        std::lock_guard<std::mutex> lock(config_mutex_);
        config_ = config;
        
        LOGI("Configuration updated");
        return true;
    }
    
    bool IsReady() const override {
        return initialized_.load() && !destroyed_.load() && 
               echo_controller_ != nullptr && timing_sync_ != nullptr;
    }
    
    bool IsTimingSynced() const override {
        return timing_synced_ && timing_sync_ && timing_sync_->IsTimingSynced();
    }
    
    float GetConvergenceQuality() const override {
        return current_convergence_.load();
    }
    
    void GetProcessingStats(int& frames_processed, int& adaptations_count, int& avg_processing_time_us) const override {
        frames_processed = frames_processed_.load();
        adaptations_count = adaptations_count_.load();
        
        int total_frames = frames_processed;
        if (total_frames > 0) {
            avg_processing_time_us = static_cast<int>(total_processing_time_us_.load() / total_frames);
        } else {
            avg_processing_time_us = 0;
        }
    }
    
private:
    // =================================================================
    // Internal Helper Methods
    // =================================================================
    
    webrtc::EchoCanceller3Config CreateOptimizedWebRTCConfig() const {
        webrtc::EchoCanceller3Config webrtc_config;
        
        // Apply WQAEC configuration to WebRTC config
        webrtc_config.filter.main.length_blocks = config_.suppression.filter_length_blocks;
        webrtc_config.filter.main.leakage_converged = 0.001f;
        webrtc_config.filter.main.leakage_diverged = 0.1f;
        webrtc_config.filter.main.error_floor = 0.001f;
        webrtc_config.filter.main.noise_gate = config_.suppression.noise_gate;
        
        // Suppression configuration based on WQAEC parameters
        webrtc_config.suppressor.normal_tuning.max_dec_factor_lf = config_.suppression.echo_suppression;
        webrtc_config.suppressor.normal_tuning.max_inc_factor = config_.suppression.voice_recovery;
        webrtc_config.suppressor.nearend_tuning.max_inc_factor = config_.suppression.voice_recovery;
        webrtc_config.suppressor.nearend_tuning.max_dec_factor_lf = config_.suppression.voice_protection;
        
        // Delay configuration
        webrtc_config.delay.down_sampling_factor = 4;
        webrtc_config.delay.num_filters = 6;
        webrtc_config.delay.delay_headroom_samples = 32;
        webrtc_config.delay.hysteresis_limit_blocks = 1;
        webrtc_config.delay.fixed_capture_delay_samples = 0;
        webrtc_config.delay.delay_estimate_smoothing = 0.7f;
        webrtc_config.delay.delay_candidate_detection_threshold = 0.2f;
        
        LOGD("Created optimized WebRTC config: filter_length=%d, echo_suppression=%.1f",
             config_.suppression.filter_length_blocks, config_.suppression.echo_suppression);
        
        return webrtc_config;
    }
    
    bool ValidateProcessingState(size_t length) const {
        if (!initialized_.load()) {
            LOGE("WQAEC not initialized");
            return false;
        }
        
        if (destroyed_.load()) {
            LOGE("WQAEC has been destroyed");
            return false;
        }
        
        if (length != AEC3Config::FRAME_SIZE) {
            LOGE("Invalid frame size: %zu, expected %d", length, AEC3Config::FRAME_SIZE);
            return false;
        }
        
        return true;
    }
    
    bool ShouldPerformAdaptation() const {
        if (!config_.adaptation.enable_auto_delay || !timing_sync_) {
            return false;
        }
        
        // Check if enough time has passed since last adaptation
        auto now = std::chrono::steady_clock::now();
        auto time_since_adaptation = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_adaptation_time_).count();
        
        // Perform adaptation every few seconds if conditions are met
        if (time_since_adaptation < 3000) {  // 3 seconds minimum
            return false;
        }
        
        // Check if timing sync indicates adaptation is needed
        return timing_sync_->ShouldTriggerAdaptation();
    }
    
    void PerformAutomaticAdaptation() {
        if (!timing_sync_) {
            return;
        }
        
        try {
            // Detect optimal delay using NLMS filter analysis
            int new_delay = timing_sync_->DetectOptimalDelay();
            
            // Only adapt if the change is significant
            int current_delay = config_.current_delay_ms;
            int delay_difference = std::abs(new_delay - current_delay);
            
            if (delay_difference >= 5) {  // 5ms minimum change threshold
                // Calculate adaptation confidence based on filter convergence
                float convergence = timing_sync_->GetFilterConvergence();
                adaptation_confidence_ = convergence;
                
                if (convergence > 0.6f) {  // Require good convergence for adaptation
                    config_.current_delay_ms = new_delay;
                    adaptations_count_.fetch_add(1);
                    last_adaptation_time_ = std::chrono::steady_clock::now();
                    
                    LOGI("Automatic delay adaptation: %dms -> %dms (confidence=%.2f)",
                         current_delay, new_delay, convergence);
                    
                    // Trigger callbacks
                    if (delay_callback_) {
                        try {
                            delay_callback_(current_delay, new_delay, convergence);
                        } catch (...) {
                            LOGE("Exception in delay detection callback");
                        }
                    }
                    
                    if (adaptation_callback_) {
                        try {
                            adaptation_callback_("auto_delay_adaptation", static_cast<float>(new_delay));
                        } catch (...) {
                            LOGE("Exception in adaptation callback");
                        }
                    }
                    
                    // Update timing sync state
                    timing_sync_->UpdateDelayEstimate(new_delay);
                    timing_synced_ = true;
                }
            }
            
        } catch (const std::exception& e) {
            LOGE("Exception during automatic adaptation: %s", e.what());
        }
    }
    
    void UpdatePerformanceMetrics() {
        try {
            AEC3Metrics metrics = GetMetrics();
            
            // Update timing sync status
            timing_synced_ = timing_sync_ && timing_sync_->IsTimingSynced();
            
            // Trigger performance callback
            if (performance_callback_) {
                auto now = std::chrono::steady_clock::now();
                auto time_since_last = std::chrono::duration_cast<std::chrono::seconds>(
                    now - last_metrics_update_).count();
                
                if (time_since_last >= 1) {  // Call every second
                    last_metrics_update_ = now;
                    
                    try {
                        performance_callback_(metrics);
                    } catch (...) {
                        LOGE("Exception in performance callback");
                    }
                }
            }
            
            // Log performance status periodically
            LOGI("Performance: ERLE=%.1fdB, delay=%dms, convergence=%.2f, adaptations=%d",
                 metrics.echo_return_loss_enhancement, 
                 metrics.detected_delay_ms,
                 metrics.filter_convergence,
                 adaptations_count_.load());
            
        } catch (const std::exception& e) {
            LOGE("Exception in UpdatePerformanceMetrics: %s", e.what());
        }
    }
};

// =================================================================
// Factory Methods Implementation
// =================================================================

std::unique_ptr<WQAEC> WQAEC::Create() {
    AEC3Config config = AEC3Config::CreateDefaultConfig();
    return std::make_unique<WQAECImpl>(config);
}

std::unique_ptr<WQAEC> WQAEC::Create(const AEC3Config& config) {
    if (!config.Validate()) {
        LOGE("Invalid configuration provided to WQAEC::Create");
        return nullptr;
    }
    return std::make_unique<WQAECImpl>(config);
}

std::unique_ptr<WQAEC> WQAEC::CreateForEnvironment(AEC3Config::Environment env) {
    AEC3Config config = AEC3Config::CreateEnvironmentConfig(env);
    return std::make_unique<WQAECImpl>(config);
}

std::unique_ptr<WQAEC> WQAEC::CreateOptimized(bool low_latency, bool high_quality) {
    AEC3Config config = AEC3Config::CreateOptimizedConfig(low_latency, high_quality);
    return std::make_unique<WQAECImpl>(config);
}

// =================================================================
// Static Utility Methods Implementation
// =================================================================

bool WQAEC::ValidateAudioFormat(int sample_rate, int channels, int frame_size) {
    return (sample_rate == AEC3Config::SAMPLE_RATE) &&
           (channels == AEC3Config::CHANNELS) &&
           (frame_size == AEC3Config::FRAME_SIZE);
}

int WQAEC::GetRecommendedDelayForPlatform(const char* platform) {
    if (!platform) {
        return 80;  // Default
    }
    
    std::string platform_str(platform);
    std::transform(platform_str.begin(), platform_str.end(), platform_str.begin(), ::tolower);
    
    if (platform_str.find("android") != std::string::npos) {
        return 80;   // Android typical
    } else if (platform_str.find("ios") != std::string::npos) {
        return 60;   // iOS typically lower
    } else {
        return 80;   // Default
    }
}

const char* WQAEC::GetVersion() {
    return "WQAEC 1.0.0 - Production WebRTC AEC3 with Automatic Timing Sync (2025-01-30)";
}

void WQAEC::SetDebugLogging(bool enable) {
    debug_logging_enabled.store(enable);
    LOGI("Debug logging %s", enable ? "enabled" : "disabled");
}

} // namespace wqaec

// =================================================================
// C API Implementation for JNI/Swift Integration
// =================================================================

extern "C" {

using namespace wqaec;

WQAECHandle wqaec_create() {
    try {
        auto instance = WQAEC::Create();
        return instance.release();  // Transfer ownership to C API
    } catch (...) {
        return nullptr;
    }
}

WQAECHandle wqaec_create_for_environment(int environment) {
    try {
        AEC3Config::Environment env = static_cast<AEC3Config::Environment>(environment);
        auto instance = WQAEC::CreateForEnvironment(env);
        return instance.release();
    } catch (...) {
        return nullptr;
    }
}

WQAECHandle wqaec_create_optimized(int low_latency, int high_quality) {
    try {
        auto instance = WQAEC::CreateOptimized(low_latency != 0, high_quality != 0);
        return instance.release();
    } catch (...) {
        return nullptr;
    }
}

int wqaec_initialize(WQAECHandle handle) {
    if (!handle) return 0;
    try {
        WQAEC* instance = static_cast<WQAEC*>(handle);
        return instance->Initialize() ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

void wqaec_destroy(WQAECHandle handle) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            delete instance;
        } catch (...) {
            // Ignore exceptions during destruction
        }
    }
}

int wqaec_process_reference_audio(WQAECHandle handle, const int16_t* audio, size_t length) {
    if (!handle || !audio) return 0;
    try {
        WQAEC* instance = static_cast<WQAEC*>(handle);
        return instance->ProcessReferenceAudio(audio, length) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int wqaec_process_capture_audio(WQAECHandle handle, const int16_t* input, int16_t* output, size_t length) {
    if (!handle || !input || !output) return 0;
    try {
        WQAEC* instance = static_cast<WQAEC*>(handle);
        return instance->ProcessCaptureAudio(input, output, length) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int wqaec_get_metrics(WQAECHandle handle, double* echo_return_loss, double* echo_return_loss_enhancement, 
                     int* detected_delay_ms, float* filter_convergence) {
    if (!handle) return 0;
    try {
        WQAEC* instance = static_cast<WQAEC*>(handle);
        AEC3Metrics metrics = instance->GetMetrics();
        
        if (echo_return_loss) *echo_return_loss = metrics.echo_return_loss;
        if (echo_return_loss_enhancement) *echo_return_loss_enhancement = metrics.echo_return_loss_enhancement;
        if (detected_delay_ms) *detected_delay_ms = metrics.detected_delay_ms;
        if (filter_convergence) *filter_convergence = metrics.filter_convergence;
        
        return 1;
    } catch (...) {
        return 0;
    }
}

void wqaec_set_stream_delay(WQAECHandle handle, int delay_ms) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            instance->SetStreamDelay(delay_ms);
        } catch (...) {}
    }
}

void wqaec_set_echo_suppression(WQAECHandle handle, float strength) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            instance->SetEchoSuppression(strength);
        } catch (...) {}
    }
}

void wqaec_set_voice_recovery(WQAECHandle handle, float speed) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            instance->SetVoiceRecovery(speed);
        } catch (...) {}
    }
}

void wqaec_set_voice_protection(WQAECHandle handle, float level) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            instance->SetVoiceProtection(level);
        } catch (...) {}
    }
}

void wqaec_set_environment(WQAECHandle handle, int environment) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            AEC3Config::Environment env = static_cast<AEC3Config::Environment>(environment);
            instance->SetEnvironment(env);
        } catch (...) {}
    }
}

void wqaec_enable_auto_delay_detection(WQAECHandle handle, int enable) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            instance->EnableAutoDelayDetection(enable != 0);
        } catch (...) {}
    }
}

void wqaec_enable_energy_adaptation(WQAECHandle handle, int enable) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            instance->EnableEnergyAdaptation(enable != 0);
        } catch (...) {}
    }
}

void wqaec_trigger_adaptation(WQAECHandle handle) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            instance->TriggerAdaptation();
        } catch (...) {}
    }
}

void wqaec_reset_adaptive_state(WQAECHandle handle) {
    if (handle) {
        try {
            WQAEC* instance = static_cast<WQAEC*>(handle);
            instance->ResetAdaptiveState();
        } catch (...) {}
    }
}

int wqaec_is_ready(WQAECHandle handle) {
    if (!handle) return 0;
    try {
        WQAEC* instance = static_cast<WQAEC*>(handle);
        return instance->IsReady() ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

int wqaec_is_timing_synced(WQAECHandle handle) {
    if (!handle) return 0;
    try {
        WQAEC* instance = static_cast<WQAEC*>(handle);
        return instance->IsTimingSynced() ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

float wqaec_get_convergence_quality(WQAECHandle handle) {
    if (!handle) return 0.0f;
    try {
        WQAEC* instance = static_cast<WQAEC*>(handle);
        return instance->GetConvergenceQuality();
    } catch (...) {
        return 0.0f;
    }
}

int wqaec_validate_audio_format(int sample_rate, int channels, int frame_size) {
    return WQAEC::ValidateAudioFormat(sample_rate, channels, frame_size) ? 1 : 0;
}

int wqaec_get_recommended_delay_for_platform(const char* platform) {
    return WQAEC::GetRecommendedDelayForPlatform(platform);
}

const char* wqaec_get_version() {
    return WQAEC::GetVersion();
}

void wqaec_set_debug_logging(int enable) {
    WQAEC::SetDebugLogging(enable != 0);
}

} // extern "C"
