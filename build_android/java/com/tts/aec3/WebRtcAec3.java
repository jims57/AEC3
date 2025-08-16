package com.tts.aec3;

/**
 * WQAEC - Production WebRTC AEC3 wrapper with automatic timing synchronization
 * 
 * This class provides a minimal, zero-configuration interface for mobile developers.
 * All timing synchronization, adaptive delay detection, and optimization is handled
 * automatically at the C++ level for maximum stability and cross-platform performance.
 * 
 * Key Features:
 * - Automatic timing synchronization (5-filter NLMS approach)
 * - Real-time adaptive delay detection and optimization  
 * - Zero-configuration for basic usage
 * - Production-optimized for TTS applications
 * - Cross-platform C++ core (works on iOS and Android)
 * 
 * Basic Usage:
 * 1. Create and initialize: aec3 = new WebRtcAec3(); aec3.initialize();
 * 2. Process TTS audio: aec3.processTtsAudio(tts_data) BEFORE playing
 * 3. Process microphone: clean_audio = aec3.processMicrophoneAudio(mic_data)
 * 4. Monitor performance: metrics = aec3.getMetrics()
 * 
 * All audio must be: 48kHz, 16-bit PCM, mono, 480 samples (10ms chunks)
 * Date: 2025-01-30
 */
public class WebRtcAec3 {
    static {
        System.loadLibrary("webrtc_aec3_tts");
    }

    // Audio configuration constants
    public static final int SAMPLE_RATE = 48000;
    public static final int FRAME_SIZE = 480;  // 10ms at 48kHz
    public static final int CHANNELS = 1;      // Mono
    public static final int BITS_PER_SAMPLE = 16;
    
    // Environment presets
    public static final int ENVIRONMENT_OFFICE = 0;   // Balanced for office use (default)
    public static final int ENVIRONMENT_OUTDOOR = 1;  // Aggressive for noisy outdoor
    public static final int ENVIRONMENT_QUIET = 2;    // Gentle for quiet indoor

    // Native instance handle
    private long nativeHandle = 0;

    /**
     * Create native instance (factory method support)
     * @return native handle
     */
    private native long nativeCreate();
    
    /**
     * Create native instance for specific environment
     * @param environment Environment preset (ENVIRONMENT_* constants)
     * @return native handle
     */
    private native long nativeCreateForEnvironment(int environment);

    /**
     * Initialize the AEC3 processor
     * @param handle Native handle
     * @return true if initialization successful
     */
    private native boolean nativeInitialize(long handle);

    /**
     * Clean up resources
     * @param handle Native handle
     */
    private native void nativeDestroy(long handle);

    /**
     * Process TTS audio (reference signal) - Call BEFORE playing audio
     * @param handle Native handle
     * @param ttsData TTS audio data (must be exactly 480 samples, 16-bit PCM)
     * @return true if processing successful
     */
    private native boolean nativeProcessTtsAudio(long handle, short[] ttsData);

    /**
     * Process microphone audio and remove echo
     * @param handle Native handle  
     * @param micData Microphone audio data (must be exactly 480 samples, 16-bit PCM)
     * @return Clean audio with echo removed, or null if error
     */
    private native short[] nativeProcessMicrophoneAudio(long handle, short[] micData);

    /**
     * Get current AEC metrics for monitoring performance
     * @param handle Native handle
     * @return double array: [echo_return_loss, echo_return_loss_enhancement, delay_ms, convergence]
     */
    private native double[] nativeGetMetrics(long handle);

    /**
     * Set manual stream delay compensation
     * @param handle Native handle
     * @param delayMs Delay in milliseconds (5-512ms range)
     */
    private native void nativeSetStreamDelay(long handle, int delayMs);
    
    /**
     * Set echo suppression strength (8.0-20.0 range, default 15.0)
     * @param handle Native handle
     * @param strength Echo suppression level
     */
    private native void nativeSetEchoSuppression(long handle, float strength);
    
    /**
     * Set voice recovery speed (1.0-5.0 range, default 3.5)
     * @param handle Native handle  
     * @param speed Voice recovery speed
     */
    private native void nativeSetVoiceRecovery(long handle, float speed);
    
    /**
     * Set voice protection level (1.0-8.0 range, default 1.5)
     * @param handle Native handle
     * @param level Voice protection level
     */
    private native void nativeSetVoiceProtection(long handle, float level);
    
    /**
     * Set environment preset for optimal performance
     * @param handle Native handle
     * @param environment Environment type (ENVIRONMENT_* constants)
     */
    private native void nativeSetEnvironment(long handle, int environment);
    
    /**
     * Enable/disable automatic delay detection (default enabled)
     * @param handle Native handle
     * @param enable true to enable automatic delay detection
     */
    private native void nativeEnableAutoDelayDetection(long handle, boolean enable);
    
    /**
     * Force immediate adaptation/re-convergence
     * @param handle Native handle
     */
    private native void nativeTriggerAdaptation(long handle);
    
    /**
     * Check if timing is well synchronized
     * @param handle Native handle
     * @return true if timing is well synchronized
     */
    private native boolean nativeIsTimingSynced(long handle);
    
    /**
     * Get filter convergence quality (0.0-1.0, higher is better)
     * @param handle Native handle
     * @return Convergence quality
     */
    private native float nativeGetConvergenceQuality(long handle);

    // High-level Java API - Simplified for mobile developers
    private boolean initialized = false;

    /**
     * Default constructor - creates with optimal office environment settings
     */
    public WebRtcAec3() {
        nativeHandle = nativeCreate();
    }
    
    /**
     * Constructor with environment preset
     * @param environment Environment preset (ENVIRONMENT_OFFICE, ENVIRONMENT_OUTDOOR, ENVIRONMENT_QUIET)
     */
    public WebRtcAec3(int environment) {
        nativeHandle = nativeCreateForEnvironment(environment);
    }

    /**
     * Initialize the AEC processor with automatic optimization
     * @return true if successful
     */
    public boolean initialize() {
        if (nativeHandle == 0) {
            return false;
        }
        if (!initialized) {
            initialized = nativeInitialize(nativeHandle);
        }
        return initialized;
    }

    /**
     * Clean up and release all resources
     */
    public void destroy() {
        if (nativeHandle != 0) {
            nativeDestroy(nativeHandle);
            nativeHandle = 0;
            initialized = false;
        }
    }

    /**
     * Process TTS audio chunk - Call BEFORE playing the audio through speakers
     * 
     * The C++ layer automatically handles timing synchronization and adaptive optimization.
     * 
     * @param ttsData TTS audio data (must be exactly 480 samples, 16-bit PCM, 48kHz)
     * @return true if processing successful
     */
    public boolean processTtsAudio(short[] ttsData) {
        if (!initialized || nativeHandle == 0 || ttsData == null || ttsData.length != FRAME_SIZE) {
            return false;
        }
        return nativeProcessTtsAudio(nativeHandle, ttsData);
    }

    /**
     * Process microphone audio and get echo-cancelled output
     * 
     * The C++ layer automatically performs timing synchronization, delay detection,
     * and adaptive optimization for optimal ERLE performance.
     * 
     * @param micData Microphone input (must be exactly 480 samples, 16-bit PCM, 48kHz)
     * @return Clean audio with echo removed, or null if error
     */
    public short[] processMicrophoneAudio(short[] micData) {
        if (!initialized || nativeHandle == 0 || micData == null || micData.length != FRAME_SIZE) {
            return null;
        }
        return nativeProcessMicrophoneAudio(nativeHandle, micData);
    }

    /**
     * Get current AEC performance metrics
     * @return AecMetrics object with current performance data
     */
    public AecMetrics getMetrics() {
        if (!initialized || nativeHandle == 0) return null;
        
        double[] metrics = nativeGetMetrics(nativeHandle);
        if (metrics != null && metrics.length >= 4) {
            return new AecMetrics(metrics[0], metrics[1], (int)metrics[2], (float)metrics[3]);
        }
        return null;
    }

    /**
     * Set manual stream delay (disables automatic delay detection)
     * 
     * Note: It's recommended to rely on automatic delay detection unless you have
     * specific timing requirements.
     * 
     * @param delayMs Delay in milliseconds (5-512ms range)
     */
    public void setStreamDelay(int delayMs) {
        if (initialized && nativeHandle != 0) {
            nativeSetStreamDelay(nativeHandle, delayMs);
        }
    }
    
    /**
     * Set echo suppression strength
     * @param strength Echo suppression level (8.0-20.0 range, default 15.0)
     */
    public void setEchoSuppression(float strength) {
        if (initialized && nativeHandle != 0) {
            nativeSetEchoSuppression(nativeHandle, strength);
        }
    }
    
    /**
     * Set voice recovery speed  
     * @param speed Voice recovery speed (1.0-5.0 range, default 3.5)
     */
    public void setVoiceRecovery(float speed) {
        if (initialized && nativeHandle != 0) {
            nativeSetVoiceRecovery(nativeHandle, speed);
        }
    }
    
    /**
     * Set voice protection level
     * @param level Voice protection level (1.0-8.0 range, default 1.5)
     */
    public void setVoiceProtection(float level) {
        if (initialized && nativeHandle != 0) {
            nativeSetVoiceProtection(nativeHandle, level);
        }
    }
    
    /**
     * Set environment preset for automatic optimization
     * @param environment Environment type (ENVIRONMENT_* constants)
     */
    public void setEnvironment(int environment) {
        if (initialized && nativeHandle != 0) {
            nativeSetEnvironment(nativeHandle, environment);
        }
    }
    
    /**
     * Enable/disable automatic delay detection (enabled by default)
     * @param enable true to enable automatic delay detection and optimization
     */
    public void enableAutoDelayDetection(boolean enable) {
        if (initialized && nativeHandle != 0) {
            nativeEnableAutoDelayDetection(nativeHandle, enable);
        }
    }

    /**
     * Force immediate adaptation when acoustic conditions change
     * 
     * Use this when user moves location, changes device orientation, etc.
     */
    public void triggerAdaptation() {
        if (initialized && nativeHandle != 0) {
            nativeTriggerAdaptation(nativeHandle);
        }
    }
    
    /**
     * Check if timing is well synchronized
     * @return true if automatic timing synchronization is working well
     */
    public boolean isTimingSynced() {
        if (!initialized || nativeHandle == 0) return false;
        return nativeIsTimingSynced(nativeHandle);
    }
    
    /**
     * Get filter convergence quality
     * @return Convergence quality (0.0-1.0, higher is better, >0.7 is good)
     */
    public float getConvergenceQuality() {
        if (!initialized || nativeHandle == 0) return 0.0f;
        return nativeGetConvergenceQuality(nativeHandle);
    }

    /**
     * Class to hold AEC performance metrics with convergence information
     */
    public static class AecMetrics {
        public final double echoReturnLoss;           // ERL in dB  
        public final double echoReturnLossEnhancement; // ERLE in dB (main quality indicator)
        public final int delayMs;                     // Detected delay in milliseconds
        public final float convergence;               // Filter convergence quality (0.0-1.0)

        public AecMetrics(double erl, double erle, int delay, float conv) {
            this.echoReturnLoss = erl;
            this.echoReturnLossEnhancement = erle;
            this.delayMs = delay;
            this.convergence = conv;
        }

        /**
         * Check if AEC performance is good
         * @return true if ERLE > 5dB and convergence > 0.7
         */
        public boolean isGoodPerformance() {
            return echoReturnLossEnhancement > 5.0 && convergence > 0.7;
        }
        
        /**
         * Check if AEC performance is excellent  
         * @return true if ERLE > 10dB and convergence > 0.9
         */
        public boolean isExcellentPerformance() {
            return echoReturnLossEnhancement > 10.0 && convergence > 0.9;
        }

        @Override
        public String toString() {
            return String.format("AEC Metrics: ERL=%.1fdB, ERLE=%.1fdB, Delay=%dms, Convergence=%.2f", 
                               echoReturnLoss, echoReturnLossEnhancement, delayMs, convergence);
        }
    }
    
    /**
     * Finalizer to ensure native resources are cleaned up
     */
    @Override
    protected void finalize() throws Throwable {
        try {
            destroy();
        } finally {
            super.finalize();
        }
    }
}
