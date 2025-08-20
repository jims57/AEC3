#ifndef TTS_AEC3_PROCESSOR_H
#define TTS_AEC3_PROCESSOR_H

#include <memory>
#include <vector>
#include <mutex>
#include <chrono>
#include <queue>
#include <deque>
#include <algorithm>
#include <cmath>

#include "api/echo_canceller3_factory.h"
#include "api/echo_canceller3_config.h"
#include "audio_processing/audio_buffer.h"
#include "audio_processing/audio_frame.h"
#include "audio_processing/high_pass_filter.h"

namespace webrtc_aec3_tts {

// Enhanced timing synchronization buffer for optimal ERLE
struct TimedFrame {
    std::vector<int16_t> data;
    std::chrono::high_resolution_clock::time_point timestamp;
    uint64_t frame_id;
    
    TimedFrame(const int16_t* samples, size_t size, uint64_t id);
};

/**
 * WebRTC AEC3 Processor for TTS Echo Cancellation (2025-01-31)
 * 
 * This class provides production-grade acoustic echo cancellation specifically
 * optimized for TTS (Text-to-Speech) applications using WebRTC AEC3 algorithm.
 * 
 * Key Features:
 * - Enhanced ERLE performance (>12dB target vs standard 6.2dB)
 * - Universal Android device compatibility
 * - Precise timing synchronization
 * - Mobile developer parameter control
 * - Production-ready stability
 */
class WqAec3Processor {
public:
    // Audio configuration constants
    static constexpr int kSampleRate = 48000;
    static constexpr int kFrameSize = 480;  // 10ms at 48kHz
    static constexpr int kChannels = 1;     // Mono
    static constexpr int kStreamDelay = 100; // Android typical delay

    // Enhanced ERLE optimization constants
    static constexpr int kMaxDelayMs = 500;
    static constexpr int kMinDelayMs = 20;
    static constexpr int kDelayBufferSize = kMaxDelayMs * kSampleRate / 1000 / kFrameSize;
    static constexpr double kTimingToleranceMs = 2.0;
    static constexpr int kDelayEstimationFrames = 50;
    static constexpr int kInitializationFrames = 100;

    /**
     * Constructor with optimized default parameters
     * Based on adjust-ERLE-result.md for enhanced performance
     */
    WqAec3Processor();
    
    /**
     * Destructor - cleans up all resources
     */
    ~WqAec3Processor();

    // ========== CORE AEC3 METHODS ==========
    
    /**
     * Initialize the AEC3 processor with enhanced configuration
     * @return true if initialization successful
     */
    bool Initialize();

    /**
     * Process TTS audio (reference signal)
     * Call this BEFORE playing the TTS audio through speakers
     * @param tts_data TTS audio samples (must be kFrameSize length)
     * @param length Number of samples (must equal kFrameSize)
     * @return true if processing successful
     */
    bool ProcessTtsAudio(const int16_t* tts_data, size_t length);

    /**
     * Get clean microphone audio with echo cancellation and sample rate conversion
     * This is the main method for mobile developers - handles input/output conversion automatically
     * @param inputSamples Input audio samples (int16_t array)
     * @param numInputSamples Number of input samples
     * @param inputSampleRate Input sample rate (16000, 24000, 44100, 48000, etc.)
     * @param outputSamples Output audio samples (will be allocated, caller must free)
     * @param numOutputSamples Output sample count (will be set)
     * @param outputSampleRate Output sample rate (16000, 24000, 44100, 48000)
     * @param erle Output ERLE value (will be set)
     * @param delayMs Output delay in milliseconds (will be set)
     * @return 0=success, negative=error code
     */
    int GetCleanMicrophoneAudio(const int16_t* inputSamples,
                               uint32_t numInputSamples,
                               uint32_t inputSampleRate,
                               int16_t** outputSamples,
                               uint32_t* numOutputSamples,
                               uint32_t outputSampleRate,
                               double* erle,
                               int* delayMs);
                               
    /**
     * Clear internal input buffer - call when stopping recording or switching streams
     */
    void ClearInputBuffer();

    /**
     * Get current AEC performance metrics
     * @param echo_return_loss Output: ERL value
     * @param echo_return_loss_enhancement Output: ERLE value
     * @param delay_ms Output: Detected delay in milliseconds
     * @return true if metrics retrieved successfully
     */
    bool GetMetrics(double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms);

    /**
     * Get enhanced AEC performance metrics with detailed information
     * @param echo_return_loss Output: ERL value
     * @param echo_return_loss_enhancement Output: ERLE value
     * @param delay_ms Output: Detected delay in milliseconds
     * @param render_frames Output: Total processed TTS frames
     * @param capture_frames Output: Total processed microphone frames
     * @param optimal_delay Output: Current optimal delay setting
     * @return true if metrics retrieved successfully
     */
    bool GetEnhancedMetrics(double* echo_return_loss, double* echo_return_loss_enhancement, 
                           int* delay_ms, uint64_t* render_frames, uint64_t* capture_frames, 
                           int* optimal_delay);

    // ========== CONFIGURATION METHODS ==========
    
    /**
     * Update stream delay compensation
     * @param delay_ms Delay in milliseconds (typically 80-150ms for Android)
     */
    void SetStreamDelay(int delay_ms);
    
    /**
     * Enable or disable precise timing synchronization
     * @param enable true to enable timing sync, false to disable
     * @return true if setting applied successfully
     */
    bool EnableTimingSync(bool enable);
    
    /**
     * Automatically optimize delay for maximum ERLE performance
     * @return true if optimization completed successfully
     */
    bool AutoOptimizeDelay();

    // ========== OFFICIAL AEC3 PARAMETER CONTROL ==========
    
    // Filter Configuration Methods
    void SetConfigChangeDuration(int blocks);
    void SetInitialStateSeconds(float seconds);
    void SetConservativeInitialPhase(bool enable);
    
    // Suppressor Normal Tuning Methods
    void SetMaxDecFactorLF(float factor);
    void SetMaxIncFactor(float factor);
    
    // Suppressor Nearend Tuning Methods
    void SetNearendMaxDecFactorLF(float factor);
    void SetNearendMaxIncFactor(float factor);
    
    // Dominant Nearend Detection Methods
    void SetEnrThreshold(float threshold);
    void SetSnrThreshold(float threshold);
    void SetHoldDuration(int duration);
    void SetTriggerThreshold(int threshold);

    // ========== ERLE ADJUSTMENT PARAMETERS FOR MOBILE DEVELOPERS ==========
    
    /**
     * Set filter length in blocks for echo learning
     * @param blocks 1-100 range, default=25 (from adjust-ERLE-result.md)
     */
    void SetFilterLengthBlocks(int blocks);
    
    /**
     * Set filter leakage when converged for stability
     * @param leakage 0.000001-1.0 range, default=0.000005 (from adjust-ERLE-result.md)
     */
    void SetFilterLeakageConverged(float leakage);
    
    /**
     * Set filter leakage when diverged for recovery
     * @param leakage 0.001-1.0 range, default=0.005 (from adjust-ERLE-result.md)
     */
    void SetFilterLeakageDiverged(float leakage);
    
    /**
     * Set delay estimation down sampling factor for precision
     * @param factor 1-8 range, default=2 (from adjust-ERLE-result.md)
     */
    void SetDelayDownSamplingFactor(int factor);
    
    /**
     * Set number of delay estimation filters
     * @param filters 1-32 range, default=16 (from adjust-ERLE-result.md)
     */
    void SetDelayNumFilters(int filters);
    
    /**
     * Set delay estimate smoothing factor for stability
     * @param smoothing 0.1-0.99 range, default=0.98 (from adjust-ERLE-result.md)
     */
    void SetDelayEstimateSmoothing(float smoothing);

private:
    // Internal implementation methods
    double CalculateFrameEnergy(const int16_t* samples, size_t length) const;
    const TimedFrame* FindOptimalReferenceFrame(const std::chrono::high_resolution_clock::time_point& capture_time);
    int EstimateOptimalDelay(const std::chrono::high_resolution_clock::time_point& capture_time,
                           const std::chrono::high_resolution_clock::time_point& render_time);
    void PerformDelayEstimationOptimization();
    int GetTimingBasedDelayEstimate();
    
    /**
     * Internal AEC3 processing method (hidden from public API)
     * @param mic_data Microphone input samples (must be kFrameSize length)
     * @param output_data Output buffer for processed audio (must be kFrameSize length)
     * @param length Number of samples (must equal kFrameSize)
     * @return true if processing successful
     */
    bool ProcessMicrophoneAudio(const int16_t* mic_data, int16_t* output_data, size_t length);

    // Core WebRTC components
    std::mutex mutex_;
    std::unique_ptr<webrtc::EchoCanceller3Factory> aec_factory_;
    std::unique_ptr<webrtc::EchoControl> echo_controller_;
    
    // 🎯 Input buffering for GetCleanMicrophoneAudio (2025-01-31)
    // Buffer to accumulate partial frames until we have exactly 480 samples for AEC3
    std::vector<int16_t> input_buffer_;
    mutable std::mutex input_buffer_mutex_;
    std::unique_ptr<webrtc::AudioBuffer> audio_render_buffer_;
    std::unique_ptr<webrtc::AudioBuffer> audio_capture_buffer_;
    std::unique_ptr<webrtc::HighPassFilter> high_pass_filter_;
    
    // Enhanced timing synchronization
    std::deque<TimedFrame> render_buffer_;
    uint64_t frame_counter_;
    int last_delay_estimation_;
    int current_optimal_delay_ms_;
    int delay_estimation_counter_;
    uint64_t total_render_frames_;
    uint64_t total_capture_frames_;
    bool timing_sync_enabled_;
    
    // Initialization stabilization
    int initialization_frames_;
    bool is_initialization_complete_;
    
    // Adaptive delay management
    int current_delay_ms_;
    int manual_delay_ms_;
    
    // Configuration parameters (runtime adjustable)
    int config_change_duration_blocks_;
    float initial_state_seconds_;
    bool conservative_initial_phase_;
    float max_dec_factor_lf_;
    float max_inc_factor_;
    float nearend_max_dec_factor_lf_;
    float nearend_max_inc_factor_;
    float enr_threshold_;
    float snr_threshold_;
    int hold_duration_;
    int trigger_threshold_;
    
    // ERLE adjustment parameters for mobile developers
    int filter_length_blocks_;
    float filter_leakage_converged_;
    float filter_leakage_diverged_;
    int delay_down_sampling_factor_;
    int delay_num_filters_;
    float delay_estimate_smoothing_;
};

} // namespace webrtc_aec3_tts

#endif // TTS_AEC3_PROCESSOR_H
