#pragma once

#include "wqaec_config.h"
#include <memory>
#include <functional>

/**
 * WQAEC - Production-Ready WebRTC AEC3 Wrapper for Mobile Applications
 * 
 * This is the main interface for mobile developers (iOS and Android).
 * All timing synchronization, adaptive delay detection, and optimization
 * is handled automatically at the C++ level.
 * 
 * Key Features:
 * - Automatic timing synchronization using 5-filter NLMS approach
 * - Real-time adaptive delay detection and optimization
 * - Cross-platform compatibility (iOS and Android)
 * - Zero-configuration for basic usage
 * - Production-optimized for TTS applications
 * 
 * Usage:
 * 1. Create instance: auto aec = wqaec::WQAEC::Create()
 * 2. Process reference: aec->ProcessReferenceAudio(tts_data, length)
 * 3. Process microphone: aec->ProcessCaptureAudio(mic_data, clean_output, length)
 * 4. Monitor performance: aec->GetMetrics()
 * 
 * Date: 2025-01-30
 * Author: WebRTC AEC3 Optimization Team
 */

namespace wqaec {

/**
 * Callback function types for advanced monitoring
 */
using DelayDetectionCallback = std::function<void(int old_delay_ms, int new_delay_ms, float confidence)>;
using PerformanceCallback = std::function<void(const AEC3Metrics& metrics)>;
using AdaptationCallback = std::function<void(const char* event, float value)>;

/**
 * Main WQAEC class - Single interface for all AEC3 functionality
 */
class WQAEC {
public:
    /**
     * Factory methods for easy creation
     */
    static std::unique_ptr<WQAEC> Create();
    static std::unique_ptr<WQAEC> Create(const AEC3Config& config);
    static std::unique_ptr<WQAEC> CreateForEnvironment(AEC3Config::Environment env);
    static std::unique_ptr<WQAEC> CreateOptimized(bool low_latency = false, bool high_quality = true);
    
    /**
     * Destructor - automatically cleans up all resources
     */
    virtual ~WQAEC() = default;
    
    // =================================================================
    // CORE API - Essential methods for mobile developers
    // =================================================================
    
    /**
     * Initialize the AEC3 processor
     * 
     * @return true if initialization successful
     * 
     * Note: This must be called before any audio processing
     */
    virtual bool Initialize() = 0;
    
    /**
     * Process reference audio (TTS/speaker output)
     * 
     * Call this BEFORE playing the audio through speakers.
     * This allows AEC3 to learn the reference signal for echo cancellation.
     * 
     * @param reference_audio Pointer to 16-bit PCM audio data
     * @param length Number of samples (must be exactly 480 for 10ms at 48kHz)
     * @return true if processing successful
     * 
     * Threading: Thread-safe, can be called from any thread
     * Timing: Must be called in real-time, BEFORE speaker playback
     */
    virtual bool ProcessReferenceAudio(const int16_t* reference_audio, size_t length) = 0;
    
    /**
     * Process capture audio (microphone input) and remove echo
     * 
     * This is the main echo cancellation function. Input microphone audio
     * and get back clean audio with echo removed.
     * 
     * @param capture_audio Pointer to 16-bit PCM microphone audio
     * @param clean_output Pointer to output buffer (same size as input)
     * @param length Number of samples (must be exactly 480 for 10ms at 48kHz)
     * @return true if processing successful
     * 
     * Threading: Thread-safe, can be called from any thread
     * Timing: Process in real-time, ideally synchronized with reference
     * 
     * Note: The automatic timing sync system will optimize performance over time
     */
    virtual bool ProcessCaptureAudio(const int16_t* capture_audio, 
                                   int16_t* clean_output, 
                                   size_t length) = 0;
    
    /**
     * Get current performance metrics
     * 
     * @return AEC3Metrics structure with current performance data
     * 
     * Use this to monitor AEC3 effectiveness:
     * - ERLE > 5dB = Good performance
     * - ERLE > 10dB = Excellent performance
     * - filter_convergence > 0.7 = Well converged
     */
    virtual AEC3Metrics GetMetrics() const = 0;
    
    /**
     * Clean up and release all resources
     * 
     * Safe to call multiple times. All processing will stop.
     */
    virtual void Destroy() = 0;
    
    // =================================================================
    // CONFIGURATION API - Runtime parameter adjustment
    // =================================================================
    
    /**
     * Set manual delay compensation
     * 
     * @param delay_ms Delay in milliseconds (5-512ms range)
     * 
     * Note: Auto-delay detection will override this unless disabled.
     * For production apps, it's better to rely on automatic detection.
     */
    virtual void SetStreamDelay(int delay_ms) = 0;
    
    /**
     * Set echo suppression strength
     * 
     * @param strength Echo suppression level (8.0-20.0 range)
     *                Higher values = more aggressive echo removal
     *                Default: 15.0 (balanced)
     * 
     * Recommended values:
     * - Quiet environments: 12.0-15.0
     * - Normal environments: 15.0-18.0
     * - Noisy environments: 18.0-20.0
     */
    virtual void SetEchoSuppression(float strength) = 0;
    
    /**
     * Set voice recovery speed
     * 
     * @param speed Voice recovery speed (1.0-5.0 range)
     *              Higher values = faster voice recovery after echo
     *              Default: 3.5 (balanced)
     * 
     * Recommended values:
     * - Conservative: 2.0-3.0 (slower but safer)
     * - Balanced: 3.0-4.0 (good compromise)
     * - Aggressive: 4.0-5.0 (fast but may affect voice quality)
     */
    virtual void SetVoiceRecovery(float speed) = 0;
    
    /**
     * Set voice protection level
     * 
     * @param level Voice protection level (1.0-8.0 range)
     *              Lower values = better voice preservation
     *              Default: 1.5 (high voice protection)
     * 
     * Recommended values:
     * - High protection: 1.0-2.0 (preserve voice quality)
     * - Balanced: 2.0-4.0 (good compromise)
     * - Low protection: 4.0-8.0 (more aggressive echo removal)
     */
    virtual void SetVoiceProtection(float level) = 0;
    
    /**
     * Set environment preset for optimal performance
     * 
     * @param env Environment type
     * 
     * This automatically configures all parameters for the specified environment:
     * - OFFICE: Balanced settings for typical office environment
     * - OUTDOOR: Aggressive settings for noisy outdoor use
     * - QUIET: Gentle settings for quiet indoor use
     * - CUSTOM: Preserve current manual settings
     */
    virtual void SetEnvironment(AEC3Config::Environment env) = 0;
    
    // =================================================================
    // ADVANCED API - Optional features for power users
    // =================================================================
    
    /**
     * Enable/disable automatic delay detection
     * 
     * @param enable true to enable automatic delay detection (recommended)
     * 
     * When enabled, the system continuously monitors timing and adjusts
     * delay compensation automatically for optimal ERLE performance.
     */
    virtual void EnableAutoDelayDetection(bool enable) = 0;
    
    /**
     * Enable/disable energy-based adaptation
     * 
     * @param enable true to enable energy-based parameter adaptation
     * 
     * When enabled, the system adjusts parameters based on signal energy
     * levels to optimize performance in different acoustic conditions.
     */
    virtual void EnableEnergyAdaptation(bool enable) = 0;
    
    /**
     * Force immediate adaptation/re-convergence
     * 
     * Use this when acoustic conditions change significantly
     * (e.g., user moves to different location, changes device orientation)
     */
    virtual void TriggerAdaptation() = 0;
    
    /**
     * Reset all adaptive state
     * 
     * Resets filters and adaptation state. Use when starting a new session
     * or when audio routing changes significantly.
     */
    virtual void ResetAdaptiveState() = 0;
    
    /**
     * Set callback for delay detection events
     * 
     * @param callback Function to call when delay is automatically detected/changed
     * 
     * Useful for debugging and monitoring automatic delay adaptation.
     */
    virtual void SetDelayDetectionCallback(DelayDetectionCallback callback) = 0;
    
    /**
     * Set callback for performance monitoring
     * 
     * @param callback Function to call periodically with performance metrics
     * 
     * Called every ~1 second with current AEC3 performance data.
     */
    virtual void SetPerformanceCallback(PerformanceCallback callback) = 0;
    
    /**
     * Set callback for adaptation events
     * 
     * @param callback Function to call when adaptation events occur
     * 
     * Useful for detailed monitoring of the adaptation system.
     */
    virtual void SetAdaptationCallback(AdaptationCallback callback) = 0;
    
    // =================================================================
    // UTILITY API - Helper methods
    // =================================================================
    
    /**
     * Get current configuration
     * 
     * @return Copy of current AEC3Config
     */
    virtual AEC3Config GetCurrentConfig() const = 0;
    
    /**
     * Update configuration
     * 
     * @param config New configuration to apply
     * @return true if configuration was valid and applied
     * 
     * Note: Some parameters require reinitialization to take effect.
     */
    virtual bool UpdateConfig(const AEC3Config& config) = 0;
    
    /**
     * Check if the system is properly initialized and ready
     * 
     * @return true if ready for audio processing
     */
    virtual bool IsReady() const = 0;
    
    /**
     * Get timing synchronization status
     * 
     * @return true if timing is well synchronized
     */
    virtual bool IsTimingSynced() const = 0;
    
    /**
     * Get filter convergence quality
     * 
     * @return Convergence quality (0.0-1.0, higher is better)
     * 
     * Values > 0.7 indicate good convergence
     * Values > 0.9 indicate excellent convergence
     */
    virtual float GetConvergenceQuality() const = 0;
    
    /**
     * Get processing statistics
     * 
     * @param[out] frames_processed Total frames processed since initialization
     * @param[out] adaptations_count Number of automatic adaptations performed
     * @param[out] avg_processing_time_us Average processing time per frame in microseconds
     */
    virtual void GetProcessingStats(int& frames_processed, 
                                  int& adaptations_count,
                                  int& avg_processing_time_us) const = 0;
    
    /**
     * Validate audio format
     * 
     * @param sample_rate Audio sample rate
     * @param channels Number of channels
     * @param frame_size Frame size in samples
     * @return true if format is supported
     * 
     * Currently only supports: 48kHz, mono, 480 samples (10ms)
     */
    static bool ValidateAudioFormat(int sample_rate, int channels, int frame_size);
    
    /**
     * Get recommended delay for platform
     * 
     * @param platform Platform name ("android", "ios", or "unknown")
     * @return Recommended initial delay in milliseconds
     */
    static int GetRecommendedDelayForPlatform(const char* platform);
    
    /**
     * Get version information
     * 
     * @return Version string
     */
    static const char* GetVersion();
    
    /**
     * Enable/disable debug logging
     * 
     * @param enable true to enable verbose debug logging
     */
    static void SetDebugLogging(bool enable);
};

// =================================================================
// C API - For easier integration with JNI/Swift
// =================================================================

extern "C" {

/**
 * C API for JNI integration (Android) and Swift integration (iOS)
 */

// Handle type for C API
typedef void* WQAECHandle;

/**
 * C API functions - mirror the main C++ API
 */
WQAECHandle wqaec_create();
WQAECHandle wqaec_create_for_environment(int environment);  // 0=OFFICE, 1=OUTDOOR, 2=QUIET
WQAECHandle wqaec_create_optimized(int low_latency, int high_quality);

int wqaec_initialize(WQAECHandle handle);
void wqaec_destroy(WQAECHandle handle);

int wqaec_process_reference_audio(WQAECHandle handle, const int16_t* audio, size_t length);
int wqaec_process_capture_audio(WQAECHandle handle, const int16_t* input, int16_t* output, size_t length);

// Get metrics as separate values for easier JNI handling
int wqaec_get_metrics(WQAECHandle handle, 
                     double* echo_return_loss,
                     double* echo_return_loss_enhancement, 
                     int* detected_delay_ms,
                     float* filter_convergence);

// Parameter setters
void wqaec_set_stream_delay(WQAECHandle handle, int delay_ms);
void wqaec_set_echo_suppression(WQAECHandle handle, float strength);
void wqaec_set_voice_recovery(WQAECHandle handle, float speed);
void wqaec_set_voice_protection(WQAECHandle handle, float level);
void wqaec_set_environment(WQAECHandle handle, int environment);

// Advanced features
void wqaec_enable_auto_delay_detection(WQAECHandle handle, int enable);
void wqaec_enable_energy_adaptation(WQAECHandle handle, int enable);
void wqaec_trigger_adaptation(WQAECHandle handle);
void wqaec_reset_adaptive_state(WQAECHandle handle);

// Status queries
int wqaec_is_ready(WQAECHandle handle);
int wqaec_is_timing_synced(WQAECHandle handle);
float wqaec_get_convergence_quality(WQAECHandle handle);

// Utility functions
int wqaec_validate_audio_format(int sample_rate, int channels, int frame_size);
int wqaec_get_recommended_delay_for_platform(const char* platform);
const char* wqaec_get_version();
void wqaec_set_debug_logging(int enable);

} // extern "C"

} // namespace wqaec
