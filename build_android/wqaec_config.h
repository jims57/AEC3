#pragma once

#include <cstdint>
#include <memory>

namespace wqaec {

/**
 * WebRTC AEC3 Configuration for Production TTS Applications
 * 
 * Based on WebRTC AEC3 research:
 * - Uses 5 NLMS filters for delay estimation (as described in timing alignment paper)
 * - Implements adaptive delay detection with energy-based convergence
 * - Optimized for mobile devices with automatic parameter adjustment
 * 
 * Date: 2025-01-30
 */
struct AEC3Config {
    // Core audio parameters (fixed for WebRTC AEC3)
    static constexpr int SAMPLE_RATE = 48000;
    static constexpr int FRAME_SIZE = 480;  // 10ms at 48kHz
    static constexpr int CHANNELS = 1;
    
    // Adaptive delay estimation parameters (based on WebRTC AEC3 paper)
    static constexpr int MAX_DELAY_MS = 512;  // Maximum detectable delay as per research
    static constexpr int MIN_DELAY_MS = 0;    // Allow 0 for auto-detection
    static constexpr int DELAY_SEARCH_RANGE = 5;  // Search range around current estimate
    
    // Filter convergence parameters (from NLMS algorithm research)
    static constexpr float NLMS_STEP_SIZE = 0.7f;    // Step size as mentioned in paper
    static constexpr int NUM_DELAY_FILTERS = 5;      // 5 NLMS filters as per WebRTC design
    static constexpr int FILTER_BLOCKS = 32;         // 32 blocks per filter
    static constexpr int SAMPLES_PER_BLOCK = 16;     // 16 samples per block
    static constexpr int FILTER_OVERLAP_BLOCKS = 8;  // 8 blocks overlap between filters
    
    // Energy analysis thresholds for automatic adaptation
    static constexpr float MIN_ENERGY_THRESHOLD = 1000.0f;   // Minimum signal energy
    static constexpr float CONVERGENCE_THRESHOLD = 0.85f;    // Filter convergence threshold
    static constexpr int ADAPTATION_FRAMES = 100;            // Frames to wait before re-adaptation
    
    // Production-tuned AEC3 parameters with optimal defaults
    struct SuppressionLevels {
        float echo_suppression = 15.0f;      // 8.0-20.0 range, optimal for TTS
        float voice_recovery = 3.5f;         // 1.0-5.0 range, balanced recovery
        float voice_protection = 1.5f;       // 1.0-8.0 range, protect human voice
        float noise_gate = 0.1f;             // 0.05-0.5 range, moderate noise gate
        int filter_length_blocks = 13;       // 10-25 range, proven optimal for 48kHz
    };
    
    // Environment-specific presets
    enum class Environment {
        OFFICE,     // Typical office environment (default)
        OUTDOOR,    // Noisy outdoor environment
        QUIET,      // Quiet indoor environment
        CUSTOM      // User-defined parameters
    };
    
    // Automatic adaptation settings
    struct AdaptationConfig {
        bool enable_auto_delay = true;           // Enable automatic delay detection
        bool enable_energy_adaptation = true;    // Adapt based on signal energy
        bool enable_erle_monitoring = true;      // Monitor ERLE for quality assessment
        int min_frames_for_adaptation = 50;      // Minimum frames before adaptation
        float target_erle_db = 10.0f;            // Target ERLE for good performance
        float erle_improvement_threshold = 2.0f; // Minimum ERLE improvement to keep changes
    };
    
    // Mobile device optimization
    struct MobileOptimization {
        bool enable_cpu_optimization = true;     // Enable CPU-specific optimizations
        bool enable_memory_optimization = true;  // Enable memory usage optimization
        bool enable_battery_optimization = false; // Reduce CPU usage for battery
        int max_processing_time_us = 5000;       // Maximum processing time per frame (5ms)
    };
    
    // Default configuration factory methods
    static AEC3Config CreateDefaultConfig();
    static AEC3Config CreateEnvironmentConfig(Environment env);
    static AEC3Config CreateOptimizedConfig(bool low_latency = false, bool high_quality = true);
    
    // Configuration validation
    bool Validate() const;
    void ApplyMobileOptimizations();
    void SetEnvironmentPreset(Environment env);
    
    // Member variables with production-optimized defaults
    SuppressionLevels suppression;
    AdaptationConfig adaptation;
    MobileOptimization mobile;
    Environment environment = Environment::OFFICE;
    
    // Current adaptive state (runtime values)
    mutable int current_delay_ms = 80;        // Current detected delay
    mutable float current_erle_db = 0.0f;     // Current ERLE performance
    mutable bool is_converged = false;        // Whether filters have converged
    mutable int frames_since_adaptation = 0;  // Frames since last adaptation
};

/**
 * Audio buffer structure for efficient processing
 */
struct AudioBuffer {
    int16_t* data;
    size_t size;
    size_t capacity;
    int64_t timestamp_us;  // Microsecond timestamp for precise timing
    
    AudioBuffer(size_t cap = AEC3Config::FRAME_SIZE);
    ~AudioBuffer();
    
    void Reset();
    bool IsValid() const;
    void SetTimestamp(int64_t timestamp);
};

/**
 * Performance metrics for monitoring AEC3 effectiveness
 */
struct AEC3Metrics {
    double echo_return_loss = 0.0;           // ERL in dB
    double echo_return_loss_enhancement = 0.0; // ERLE in dB
    int detected_delay_ms = 0;               // Automatically detected delay
    int set_delay_ms = 0;                    // User-set delay
    float signal_energy = 0.0f;              // Current signal energy
    float filter_convergence = 0.0f;         // Filter convergence quality (0-1)
    bool is_echo_detected = false;           // Whether echo is actively detected
    bool is_voice_detected = false;          // Whether human voice is detected
    int adaptation_count = 0;                // Number of adaptations performed
    
    // Quality indicators
    bool IsGoodPerformance() const {
        return echo_return_loss_enhancement > 5.0 && filter_convergence > 0.7;
    }
    
    bool IsExcellentPerformance() const {
        return echo_return_loss_enhancement > 10.0 && filter_convergence > 0.9;
    }
};

/**
 * Timing synchronization manager for automatic delay detection
 * Implements the 5-filter NLMS approach described in WebRTC AEC3 research
 */
class TimingSyncManager {
public:
    TimingSyncManager(const AEC3Config& config);
    ~TimingSyncManager();
    
    // Core timing analysis methods
    void AnalyzeReferenceFrame(const int16_t* reference_audio, size_t length, int64_t timestamp);
    void AnalyzeCaptureFrame(const int16_t* capture_audio, size_t length, int64_t timestamp);
    
    // Automatic delay detection (implements 5-filter NLMS approach)
    int DetectOptimalDelay();
    bool IsTimingSynced() const;
    float GetFilterConvergence() const;
    
    // Adaptive adjustment methods
    void UpdateDelayEstimate(int new_delay_ms);
    void ResetAdaptation();
    bool ShouldTriggerAdaptation() const;
    
    // Energy and quality analysis
    float GetReferenceEnergy() const;
    float GetCaptureEnergy() const;
    float GetCrossCorrelationPeak() const;
    
private:
    struct FilterState;
    class DelayEstimator;
    
    std::unique_ptr<FilterState> filter_state_;
    std::unique_ptr<DelayEstimator> delay_estimator_;
    mutable AEC3Config config_;
    
    // Timing analysis state
    int64_t last_reference_timestamp_ = 0;
    int64_t last_capture_timestamp_ = 0;
    float reference_energy_sum_ = 0.0f;
    float capture_energy_sum_ = 0.0f;
    int frame_count_ = 0;
    
    // Internal helper methods
    void InitializeFilters();
    void UpdateEnergyStatistics(float ref_energy, float cap_energy);
    bool ValidateTimingConsistency() const;
};

} // namespace wqaec
