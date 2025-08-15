#include "wqaec_config.h"
#include <memory>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cstring>

#ifdef __ANDROID__
#include <android/log.h>
#define LOG_TAG "WQAEC_Config"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#else
#include <iostream>
#define LOGI(...) printf(__VA_ARGS__); printf("\n")
#define LOGE(...) printf(__VA_ARGS__); printf("\n")
#define LOGD(...) printf(__VA_ARGS__); printf("\n")
#endif

namespace wqaec {

// =============================================================================
// AEC3Config Implementation
// =============================================================================

AEC3Config AEC3Config::CreateDefaultConfig() {
    AEC3Config config;
    
    // Set WebRTC AEC3 optimal defaults based on demo.cc and research papers
    config.suppression.echo_suppression = 12.0f;     // Conservative start (demo.cc style)
    config.suppression.voice_recovery = 2.8f;        // Moderate recovery speed
    config.suppression.voice_protection = 2.0f;      // Balanced voice protection
    config.suppression.noise_gate = 0.05f;           // Lower noise gate for cleaner output
    config.suppression.filter_length_blocks = 20;    // Longer filter for better modeling (demo.cc uses more blocks)
    
    config.adaptation.enable_auto_delay = true;
    config.adaptation.enable_energy_adaptation = true;
    config.adaptation.enable_erle_monitoring = true;
    config.adaptation.min_frames_for_adaptation = 100;  // More frames for stable adaptation
    config.adaptation.target_erle_db = 8.0f;           // Realistic target ERLE
    config.adaptation.erle_improvement_threshold = 1.5f;
    
    config.mobile.enable_cpu_optimization = false;     // Disable for better quality initially
    config.mobile.enable_memory_optimization = false;  // Disable for better quality initially
    config.mobile.enable_battery_optimization = false;
    config.mobile.max_processing_time_us = 8000;       // Allow more processing time
    
    config.environment = Environment::OFFICE;
    config.current_delay_ms = 0;  // Start with 0, let AEC3 detect automatically
    
    LOGI("Created default AEC3 config: echo_suppression=%.1f, voice_recovery=%.1f, filter_blocks=%d", 
         config.suppression.echo_suppression, config.suppression.voice_recovery, config.suppression.filter_length_blocks);
    
    return config;
}

AEC3Config AEC3Config::CreateEnvironmentConfig(Environment env) {
    AEC3Config config = CreateDefaultConfig();
    config.SetEnvironmentPreset(env);
    return config;
}

AEC3Config AEC3Config::CreateOptimizedConfig(bool low_latency, bool high_quality) {
    AEC3Config config = CreateDefaultConfig();
    
    if (low_latency) {
        config.suppression.filter_length_blocks = 10;  // Shorter filter for low latency
        config.mobile.max_processing_time_us = 3000;   // Stricter timing
        config.adaptation.min_frames_for_adaptation = 25; // Faster adaptation
        LOGI("Applied low-latency optimizations");
    }
    
    if (high_quality) {
        config.suppression.echo_suppression = 18.0f;   // More aggressive echo removal
        config.suppression.filter_length_blocks = 20;  // Longer filter for better modeling
        config.adaptation.target_erle_db = 12.0f;      // Higher quality target
        LOGI("Applied high-quality optimizations");
    }
    
    return config;
}

bool AEC3Config::Validate() const {
    // Validate suppression parameters
    if (suppression.echo_suppression < 8.0f || suppression.echo_suppression > 20.0f) {
        LOGE("Invalid echo_suppression: %.1f (must be 8.0-20.0)", suppression.echo_suppression);
        return false;
    }
    
    if (suppression.voice_recovery < 1.0f || suppression.voice_recovery > 5.0f) {
        LOGE("Invalid voice_recovery: %.1f (must be 1.0-5.0)", suppression.voice_recovery);
        return false;
    }
    
    if (suppression.voice_protection < 1.0f || suppression.voice_protection > 8.0f) {
        LOGE("Invalid voice_protection: %.1f (must be 1.0-8.0)", suppression.voice_protection);
        return false;
    }
    
    if (suppression.filter_length_blocks < 10 || suppression.filter_length_blocks > 25) {
        LOGE("Invalid filter_length_blocks: %d (must be 10-25)", suppression.filter_length_blocks);
        return false;
    }
    
    // Validate delay parameters
    if (current_delay_ms < MIN_DELAY_MS || current_delay_ms > MAX_DELAY_MS) {
        LOGE("Invalid current_delay_ms: %d (must be %d-%d)", 
             current_delay_ms, MIN_DELAY_MS, MAX_DELAY_MS);
        return false;
    }
    
    LOGD("AEC3Config validation passed");
    return true;
}

void AEC3Config::ApplyMobileOptimizations() {
    if (mobile.enable_cpu_optimization) {
        // Reduce CPU usage on mobile devices
        suppression.filter_length_blocks = std::min(suppression.filter_length_blocks, 15);
        adaptation.min_frames_for_adaptation = std::max(adaptation.min_frames_for_adaptation, 75);
        LOGD("Applied CPU optimizations");
    }
    
    if (mobile.enable_memory_optimization) {
        // Optimize memory usage
        mobile.max_processing_time_us = std::min(mobile.max_processing_time_us, 8000);
        LOGD("Applied memory optimizations");
    }
    
    if (mobile.enable_battery_optimization) {
        // Reduce processing for battery saving
        suppression.echo_suppression = std::min(suppression.echo_suppression, 12.0f);
        adaptation.enable_energy_adaptation = false;
        LOGD("Applied battery optimizations");
    }
}

void AEC3Config::SetEnvironmentPreset(Environment env) {
    environment = env;
    
    switch (env) {
        case Environment::OFFICE:
            suppression.echo_suppression = 15.0f;
            suppression.voice_recovery = 3.5f;
            suppression.voice_protection = 1.5f;
            suppression.noise_gate = 0.1f;
            LOGI("Applied OFFICE environment preset");
            break;
            
        case Environment::OUTDOOR:
            suppression.echo_suppression = 18.0f;    // More aggressive for noisy environment
            suppression.voice_recovery = 2.5f;      // Slower recovery due to noise
            suppression.voice_protection = 2.5f;    // More protection needed
            suppression.noise_gate = 0.2f;          // Higher noise gate
            LOGI("Applied OUTDOOR environment preset");
            break;
            
        case Environment::QUIET:
            suppression.echo_suppression = 12.0f;    // Less aggressive in quiet environment
            suppression.voice_recovery = 4.0f;      // Faster recovery possible
            suppression.voice_protection = 1.0f;    // Minimal protection needed
            suppression.noise_gate = 0.05f;         // Lower noise gate
            LOGI("Applied QUIET environment preset");
            break;
            
        case Environment::CUSTOM:
            LOGI("Using CUSTOM environment - no preset applied");
            break;
    }
}

// =============================================================================
// AudioBuffer Implementation
// =============================================================================

AudioBuffer::AudioBuffer(size_t cap) : size(0), capacity(cap), timestamp_us(0) {
    data = new int16_t[capacity];
    std::memset(data, 0, capacity * sizeof(int16_t));
}

AudioBuffer::~AudioBuffer() {
    delete[] data;
}

void AudioBuffer::Reset() {
    size = 0;
    timestamp_us = 0;
    std::memset(data, 0, capacity * sizeof(int16_t));
}

bool AudioBuffer::IsValid() const {
    return data != nullptr && size <= capacity && size > 0;
}

void AudioBuffer::SetTimestamp(int64_t timestamp) {
    timestamp_us = timestamp;
}

// =============================================================================
// TimingSyncManager Implementation
// =============================================================================

/**
 * Internal filter state for NLMS delay estimation
 * Implements the 5-filter approach described in WebRTC AEC3 research
 */
struct TimingSyncManager::FilterState {
    // 5 NLMS filters for delay estimation (as per WebRTC research)
    struct NLMSFilter {
        std::vector<float> coefficients;
        std::vector<float> input_buffer;
        float energy_sum;
        float convergence_measure;
        int delay_offset;
        bool is_converged;
        
        NLMSFilter(int length, int offset) 
            : coefficients(length, 0.0f)
            , input_buffer(length, 0.0f)
            , energy_sum(0.0f)
            , convergence_measure(0.0f)
            , delay_offset(offset)
            , is_converged(false) {}
    };
    
    std::vector<NLMSFilter> delay_filters;
    
    // Cross-correlation analysis
    std::vector<float> reference_history;
    std::vector<float> capture_history;
    int history_size;
    
    // Energy analysis
    float ref_energy_window[64];
    float cap_energy_window[64];
    int energy_index;
    
    FilterState(const AEC3Config& config) : energy_index(0) {
        // Initialize 5 NLMS filters with overlapping delay ranges
        int filter_length = config.FILTER_BLOCKS * config.SAMPLES_PER_BLOCK;
        int overlap = config.FILTER_OVERLAP_BLOCKS * config.SAMPLES_PER_BLOCK;
        
        for (int i = 0; i < config.NUM_DELAY_FILTERS; i++) {
            int delay_offset = i * (filter_length - overlap);
            delay_filters.emplace_back(filter_length, delay_offset);
        }
        
        // Initialize history buffers for cross-correlation
        history_size = config.MAX_DELAY_MS * config.SAMPLE_RATE / 1000;
        reference_history.resize(history_size, 0.0f);
        capture_history.resize(history_size, 0.0f);
        
        // Initialize energy windows
        std::memset(ref_energy_window, 0, sizeof(ref_energy_window));
        std::memset(cap_energy_window, 0, sizeof(cap_energy_window));
        
        LOGI("Initialized TimingSyncManager with %d NLMS filters, history_size=%d", 
             config.NUM_DELAY_FILTERS, history_size);
    }
};

/**
 * Advanced delay estimator using energy-based filter selection
 * Based on WebRTC AEC3 delay estimation research
 */
class TimingSyncManager::DelayEstimator {
private:
    const AEC3Config& config_;
    FilterState& state_;
    
    // Statistics for adaptation decision
    float best_erle_so_far_;
    int best_delay_so_far_;
    int stable_frames_count_;
    bool is_adaptation_active_;
    
public:
    DelayEstimator(const AEC3Config& config, FilterState& state) 
        : config_(config)
        , state_(state)
        , best_erle_so_far_(0.0f)
        , best_delay_so_far_(config.current_delay_ms)
        , stable_frames_count_(0)
        , is_adaptation_active_(true) {}
    
    int EstimateDelay(float ref_energy, float cap_energy) {
        // Only estimate if we have sufficient signal energy
        if (ref_energy < config_.MIN_ENERGY_THRESHOLD || 
            cap_energy < config_.MIN_ENERGY_THRESHOLD) {
            return best_delay_so_far_;
        }
        
        // Find the filter with maximum energy (convergence indicator)
        int best_filter_idx = 0;
        float max_energy = 0.0f;
        
        for (size_t i = 0; i < state_.delay_filters.size(); i++) {
            auto& filter = state_.delay_filters[i];
            
            // Calculate filter energy as convergence measure
            float filter_energy = 0.0f;
            for (float coeff : filter.coefficients) {
                filter_energy += coeff * coeff;
            }
            
            if (filter_energy > max_energy) {
                max_energy = filter_energy;
                best_filter_idx = i;
            }
        }
        
        // Calculate delay from the best filter
        int estimated_delay_samples = state_.delay_filters[best_filter_idx].delay_offset;
        
        // Find peak within the filter coefficients
        auto& best_filter = state_.delay_filters[best_filter_idx];
        int peak_index = 0;
        float peak_value = 0.0f;
        
        for (size_t j = 0; j < best_filter.coefficients.size(); j++) {
            if (std::abs(best_filter.coefficients[j]) > peak_value) {
                peak_value = std::abs(best_filter.coefficients[j]);
                peak_index = j;
            }
        }
        
        // Convert to milliseconds
        int total_delay_samples = estimated_delay_samples + peak_index;
        int delay_ms = (total_delay_samples * 1000) / config_.SAMPLE_RATE;
        
        // Validate and constrain the delay
        delay_ms = std::max(config_.MIN_DELAY_MS, 
                           std::min(config_.MAX_DELAY_MS, delay_ms));
        
        // Update convergence measure
        best_filter.convergence_measure = max_energy;
        best_filter.is_converged = (max_energy > config_.CONVERGENCE_THRESHOLD);
        
        LOGD("Delay estimation: filter_idx=%d, delay=%dms, energy=%.3f, converged=%s",
             best_filter_idx, delay_ms, max_energy, 
             best_filter.is_converged ? "yes" : "no");
        
        return delay_ms;
    }
    
    void UpdateNLMSFilters(const float* reference, const float* capture, size_t length) {
        // Update each NLMS filter with new audio data
        for (auto& filter : state_.delay_filters) {
            UpdateSingleNLMSFilter(filter, reference, capture, length);
        }
    }
    
    float GetBestConvergence() const {
        float best_convergence = 0.0f;
        for (const auto& filter : state_.delay_filters) {
            best_convergence = std::max(best_convergence, filter.convergence_measure);
        }
        return best_convergence;
    }
    
    bool IsConverged() const {
        // Check if at least one filter has converged
        for (const auto& filter : state_.delay_filters) {
            if (filter.is_converged) {
                return true;
            }
        }
        return false;
    }
    
private:
    void UpdateSingleNLMSFilter(FilterState::NLMSFilter& filter, 
                               const float* reference, const float* capture, size_t length) {
        // Simplified NLMS update (production version would be more sophisticated)
        float step_size = config_.NLMS_STEP_SIZE;
        
        // Shift input buffer
        std::memmove(filter.input_buffer.data(), 
                    filter.input_buffer.data() + length,
                    (filter.input_buffer.size() - length) * sizeof(float));
        
        // Add new reference data to buffer
        for (size_t i = 0; i < length && i < filter.input_buffer.size(); i++) {
            filter.input_buffer[filter.input_buffer.size() - length + i] = reference[i];
        }
        
        // Calculate prediction error and update coefficients
        for (size_t i = 0; i < length; i++) {
            float prediction = 0.0f;
            
            // Compute filter output
            for (size_t j = 0; j < filter.coefficients.size() && j + i < filter.input_buffer.size(); j++) {
                prediction += filter.coefficients[j] * filter.input_buffer[filter.input_buffer.size() - length + i - j];
            }
            
            // Error signal
            float error = capture[i] - prediction;
            
            // Normalize step size by input energy
            float input_energy = 0.0001f;  // Regularization
            for (size_t j = 0; j < filter.coefficients.size() && j + i < filter.input_buffer.size(); j++) {
                float input_val = filter.input_buffer[filter.input_buffer.size() - length + i - j];
                input_energy += input_val * input_val;
            }
            
            float normalized_step = step_size / input_energy;
            
            // Update filter coefficients
            for (size_t j = 0; j < filter.coefficients.size() && j + i < filter.input_buffer.size(); j++) {
                float input_val = filter.input_buffer[filter.input_buffer.size() - length + i - j];
                filter.coefficients[j] += normalized_step * error * input_val;
            }
        }
        
        // Update energy sum for convergence detection
        filter.energy_sum = 0.0f;
        for (float coeff : filter.coefficients) {
            filter.energy_sum += coeff * coeff;
        }
    }
};

TimingSyncManager::TimingSyncManager(const AEC3Config& config) 
    : config_(config), frame_count_(0) {
    filter_state_ = std::make_unique<FilterState>(config);
    delay_estimator_ = std::make_unique<DelayEstimator>(config, *filter_state_);
    
    LOGI("TimingSyncManager initialized with auto-delay detection");
}

TimingSyncManager::~TimingSyncManager() = default;

void TimingSyncManager::AnalyzeReferenceFrame(const int16_t* reference_audio, 
                                            size_t length, int64_t timestamp) {
    last_reference_timestamp_ = timestamp;
    
    // Convert to float and calculate energy
    std::vector<float> float_ref(length);
    float energy = 0.0f;
    for (size_t i = 0; i < length; i++) {
        float_ref[i] = reference_audio[i] / 32768.0f;
        energy += float_ref[i] * float_ref[i];
    }
    
    // Update energy window
    filter_state_->ref_energy_window[filter_state_->energy_index % 64] = energy;
    
    // Update reference history for cross-correlation
    size_t history_offset = filter_state_->reference_history.size() - length;
    std::memmove(filter_state_->reference_history.data(),
                filter_state_->reference_history.data() + length,
                history_offset * sizeof(float));
    
    std::memcpy(filter_state_->reference_history.data() + history_offset,
               float_ref.data(), length * sizeof(float));
    
    reference_energy_sum_ = (reference_energy_sum_ + energy) / 2.0f;  // Running average
    
    LOGD("Reference frame analyzed: energy=%.1f, timestamp=%lld", energy, timestamp);
}

void TimingSyncManager::AnalyzeCaptureFrame(const int16_t* capture_audio, 
                                          size_t length, int64_t timestamp) {
    last_capture_timestamp_ = timestamp;
    
    // Convert to float and calculate energy
    std::vector<float> float_cap(length);
    float energy = 0.0f;
    for (size_t i = 0; i < length; i++) {
        float_cap[i] = capture_audio[i] / 32768.0f;
        energy += float_cap[i] * float_cap[i];
    }
    
    // Update energy window
    filter_state_->cap_energy_window[filter_state_->energy_index % 64] = energy;
    filter_state_->energy_index++;
    
    // Update capture history
    size_t history_offset = filter_state_->capture_history.size() - length;
    std::memmove(filter_state_->capture_history.data(),
                filter_state_->capture_history.data() + length,
                history_offset * sizeof(float));
    
    std::memcpy(filter_state_->capture_history.data() + history_offset,
               float_cap.data(), length * sizeof(float));
    
    capture_energy_sum_ = (capture_energy_sum_ + energy) / 2.0f;  // Running average
    
    // Update NLMS filters for delay estimation
    if (filter_state_->reference_history.size() >= length && 
        filter_state_->capture_history.size() >= length) {
        
        const float* ref_data = filter_state_->reference_history.data() + 
                               filter_state_->reference_history.size() - length;
        const float* cap_data = float_cap.data();
        
        delay_estimator_->UpdateNLMSFilters(ref_data, cap_data, length);
    }
    
    frame_count_++;
    
    LOGD("Capture frame analyzed: energy=%.1f, timestamp=%lld, frame=%d", 
         energy, timestamp, frame_count_);
}

int TimingSyncManager::DetectOptimalDelay() {
    if (!delay_estimator_) {
        return config_.current_delay_ms;
    }
    
    // Use NLMS filter energy analysis for delay detection
    int estimated_delay = delay_estimator_->EstimateDelay(reference_energy_sum_, capture_energy_sum_);
    
    LOGI("Optimal delay detected: %dms (was %dms), convergence=%.3f", 
         estimated_delay, config_.current_delay_ms, GetFilterConvergence());
    
    return estimated_delay;
}

bool TimingSyncManager::IsTimingSynced() const {
    return delay_estimator_ && delay_estimator_->IsConverged() && 
           frame_count_ > config_.adaptation.min_frames_for_adaptation;
}

float TimingSyncManager::GetFilterConvergence() const {
    return delay_estimator_ ? delay_estimator_->GetBestConvergence() : 0.0f;
}

void TimingSyncManager::UpdateDelayEstimate(int new_delay_ms) {
    config_.current_delay_ms = std::max(config_.MIN_DELAY_MS, 
                                       std::min(config_.MAX_DELAY_MS, new_delay_ms));
    config_.frames_since_adaptation = 0;
    
    LOGI("Delay estimate updated to %dms", config_.current_delay_ms);
}

void TimingSyncManager::ResetAdaptation() {
    config_.frames_since_adaptation = 0;
    config_.is_converged = false;
    frame_count_ = 0;
    
    // Reset filter states
    if (filter_state_) {
        for (auto& filter : filter_state_->delay_filters) {
            std::fill(filter.coefficients.begin(), filter.coefficients.end(), 0.0f);
            std::fill(filter.input_buffer.begin(), filter.input_buffer.end(), 0.0f);
            filter.energy_sum = 0.0f;
            filter.convergence_measure = 0.0f;
            filter.is_converged = false;
        }
    }
    
    LOGI("Adaptation reset - filters reinitialized");
}

bool TimingSyncManager::ShouldTriggerAdaptation() const {
    return config_.adaptation.enable_auto_delay && 
           frame_count_ > config_.adaptation.min_frames_for_adaptation &&
           config_.frames_since_adaptation > config_.ADAPTATION_FRAMES;
}

float TimingSyncManager::GetReferenceEnergy() const {
    return reference_energy_sum_;
}

float TimingSyncManager::GetCaptureEnergy() const {
    return capture_energy_sum_;
}

float TimingSyncManager::GetCrossCorrelationPeak() const {
    // Simplified cross-correlation peak calculation
    if (filter_state_->reference_history.empty() || filter_state_->capture_history.empty()) {
        return 0.0f;
    }
    
    float max_correlation = 0.0f;
    size_t search_range = std::min(static_cast<size_t>(100), 
                                  filter_state_->reference_history.size() / 2);
    
    for (size_t lag = 0; lag < search_range; lag++) {
        float correlation = 0.0f;
        size_t samples = std::min(static_cast<size_t>(480), 
                                 filter_state_->reference_history.size() - lag);
        
        for (size_t i = 0; i < samples; i++) {
            correlation += filter_state_->reference_history[i + lag] * 
                          filter_state_->capture_history[i];
        }
        
        max_correlation = std::max(max_correlation, std::abs(correlation));
    }
    
    return max_correlation;
}

} // namespace wqaec
