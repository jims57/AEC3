package com.tts.aec3;

/**
 * WebRTC AEC3 wrapper for TTS echo cancellation
 * 
 * This class provides a simple interface to WebRTC's Acoustic Echo Cancellation (AEC3)
 * specifically optimized for TTS (Text-to-Speech) applications.
 * 
 * Usage:
 * 1. Initialize the AEC processor
 * 2. For each TTS audio chunk: call processTtsAudio() BEFORE playing it
 * 3. For each microphone chunk: call processMicrophoneAudio() to get clean audio
 * 4. Monitor performance with getMetrics()
 * 
 * Important: All audio must be 48kHz, 16-bit PCM, mono, 480 samples (10ms chunks)
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

    /**
     * Initialize the AEC processor
     * @return true if successful
     */
    public native boolean nativeInitialize();

    /**
     * Clean up resources
     */
    public native void nativeDestroy();

    /**
     * Process TTS audio (reference signal)
     * Call this BEFORE playing the TTS audio through speakers
     * 
     * @param ttsData TTS audio data (480 samples, 16-bit PCM)
     * @return true if processing successful
     */
    public native boolean nativeProcessTtsAudio(short[] ttsData);

    /**
     * Process microphone audio and remove echo
     * 
     * @param micData Microphone audio data (480 samples, 16-bit PCM)
     * @param outputData Output buffer for processed audio (480 samples)
     * @return true if processing successful
     */
    public native boolean nativeProcessMicrophoneAudio(short[] micData, short[] outputData);

    /**
     * Get current AEC metrics for monitoring performance
     * @return double array: [echo_return_loss, echo_return_loss_enhancement, delay_ms]
     */
    public native double[] nativeGetMetrics();

    /**
     * Update stream delay compensation
     * @param delayMs Delay in milliseconds (typically 80-150ms for Android)
     */
    public native void nativeSetStreamDelay(int delayMs);
    
    // 🎛️ OFFICIAL AEC3 PARAMETER CONTROL (2025-01-31)
    // These native methods directly correspond to official WebRTC AEC3 configuration parameters
    
    // Filter Configuration Native Methods
    public native void nativeSetConfigChangeDuration(int blocks);          // 0-1000 range, 0=default
    public native void nativeSetInitialStateSeconds(float seconds);        // 0.0-3.0 range, 0=default  
    public native void nativeSetConservativeInitialPhase(boolean enable);  // true/false
    
    // Suppressor Normal Tuning Native Methods
    public native void nativeSetMaxDecFactorLF(float factor);             // 0.0-100.0 range, 0=default
    public native void nativeSetMaxIncFactor(float factor);               // 0.0-100.0 range, 0=default
    
    // Suppressor Nearend Tuning Native Methods  
    public native void nativeSetNearendMaxDecFactorLF(float factor);      // 0.0-100.0 range, 0=default
    public native void nativeSetNearendMaxIncFactor(float factor);        // 0.0-100.0 range, 0=default
    
    // Dominant Nearend Detection Native Methods
    public native void nativeSetEnrThreshold(float threshold);            // 0.0-1000.0 range, 0=default
    public native void nativeSetSnrThreshold(float threshold);            // 0.0-1000.0 range, 0=default
    public native void nativeSetHoldDuration(int duration);               // 0-10000 range, 0=default
    public native void nativeSetTriggerThreshold(int threshold);          // 0-10000 range, 0=default
    
    // 🎯 ENHANCED ERLE OPTIMIZATION METHODS (2025-01-31)
    public native boolean nativeAutoOptimizeDelay();               // Automatic delay optimization
    public native double[] nativeGetEnhancedMetrics();             // [ERL, ERLE, delay, render_frames, capture_frames, optimal_delay]
    public native boolean nativeEnableTimingSync(boolean enable);   // Enable/disable precise timing sync
    
    // 🎯 ERLE ADJUSTMENT PARAMETER NATIVE METHODS FOR MOBILE DEVELOPERS (2025-01-31)
    public native void nativeSetFilterLengthBlocks(int blocks);           // Filter length blocks (1-100)
    public native void nativeSetFilterLeakageConverged(float leakage);    // Filter leakage converged (0.000001-1.0)
    public native void nativeSetFilterLeakageDiverged(float leakage);     // Filter leakage diverged (0.001-1.0)
    public native void nativeSetDelayDownSamplingFactor(int factor);      // Delay down sampling factor (1-8)
    public native void nativeSetDelayNumFilters(int filters);             // Delay number of filters (1-32)
    public native void nativeSetDelayEstimateSmoothing(float smoothing);  // Delay estimate smoothing (0.1-0.99)
    
    // 🎯 CLEAN AUDIO CONVERSION NATIVE METHODS (2025-01-31)
    public native byte[] nativeGetCleanAudioAsWAV(int outputSampleRate);  // Get buffered clean audio as WAV
    public native byte[] nativeGetCleanAudioAsPCM(int outputSampleRate);  // Get buffered clean audio as PCM
    public native void nativeClearCleanAudioBuffer();                     // Clear clean audio buffer

    // High-level Java API
    private boolean initialized = false;

    /**
     * Initialize the AEC processor
     * @return true if successful
     */
    public boolean initialize() {
        if (!initialized) {
            initialized = nativeInitialize();
        }
        return initialized;
    }

    /**
     * Clean up and release resources
     */
    public void destroy() {
        if (initialized) {
            nativeDestroy();
            initialized = false;
        }
    }

    /**
     * Process TTS audio chunk
     * @param ttsData Audio data (must be exactly 480 samples)
     * @return true if successful
     */
    public boolean processTtsAudio(short[] ttsData) {
        if (!initialized || ttsData.length != FRAME_SIZE) {
            return false;
        }
        return nativeProcessTtsAudio(ttsData);
    }

    /**
     * Process microphone audio and get echo-cancelled output
     * @param micData Microphone input (must be exactly 480 samples)
     * @return Echo-cancelled audio, or null if error
     */
    public short[] processMicrophoneAudio(short[] micData) {
        if (!initialized || micData.length != FRAME_SIZE) {
            return null;
        }
        
        short[] output = new short[FRAME_SIZE];
        if (nativeProcessMicrophoneAudio(micData, output)) {
            return output;
        }
        return null;
    }

    /**
     * Get AEC performance metrics
     * @return AecMetrics object with performance data
     */
    public AecMetrics getMetrics() {
        if (!initialized) return null;
        
        double[] metrics = nativeGetMetrics();
        if (metrics != null && metrics.length == 3) {
            return new AecMetrics(metrics[0], metrics[1], (int)metrics[2]);
        }
        return null;
    }

    /**
     * Adjust stream delay for optimal performance
     * @param delayMs Delay in milliseconds
     */
    public void setStreamDelay(int delayMs) {
        if (initialized) {
            nativeSetStreamDelay(delayMs);
        }
    }
    
    // 🎛️ OFFICIAL AEC3 PARAMETER CONTROL METHODS (2025-01-31)
    // These methods directly control the official WebRTC AEC3 configuration parameters
    // Use 0 values to apply AEC3 defaults, or set specific values for custom tuning
    
    // ======= FILTER CONFIGURATION METHODS =======
    
    /**
     * Set AEC3 configuration change duration in blocks
     * Controls how smoothly AEC3 transitions between different configurations
     * @param blocks 0-1000 range, 0=use AEC3 default, typical values: 50-250 blocks
     */
    public void setConfigChangeDuration(int blocks) {
        if (initialized) {
            nativeSetConfigChangeDuration(blocks);
        }
    }
    
    /**
     * Set AEC3 initial state duration in seconds  
     * Time AEC3 spends in initial learning phase before full operation
     * @param seconds 0.0-3.0 range, 0=use AEC3 default, typical values: 0.5-2.5 seconds
     */
    public void setInitialStateSeconds(float seconds) {
        if (initialized) {
            nativeSetInitialStateSeconds(seconds);
        }
    }
    
    /**
     * Enable/disable conservative initial phase
     * Conservative mode = slower initial convergence but more stable
     * @param enable true=conservative (safer), false=aggressive (faster convergence)
     */
    public void setConservativeInitialPhase(boolean enable) {
        if (initialized) {
            nativeSetConservativeInitialPhase(enable);
        }
    }
    
    // ======= SUPPRESSOR NORMAL TUNING METHODS =======
    
    /**
     * Set maximum decrease factor for low frequencies (echo suppression strength)
     * Higher values = more aggressive echo suppression but may affect voice quality
     * @param factor 0.0-100.0 range, 0=use AEC3 default, typical values: 2.0-25.0
     */
    public void setMaxDecFactorLF(float factor) {
        if (initialized) {
            nativeSetMaxDecFactorLF(factor);
        }
    }
    
    /**
     * Set maximum increase factor (voice recovery speed)
     * Higher values = faster voice recovery after echo suppression
     * @param factor 0.0-100.0 range, 0=use AEC3 default, typical values: 1.5-5.0
     */
    public void setMaxIncFactor(float factor) {
        if (initialized) {
            nativeSetMaxIncFactor(factor);
        }
    }
    
    // ======= SUPPRESSOR NEAREND TUNING METHODS =======
    
    /**
     * Set nearend maximum decrease factor for low frequencies (voice protection)
     * Lower values = better voice preservation when user is speaking
     * @param factor 0.0-100.0 range, 0=use AEC3 default, typical values: 1.0-8.0
     */
    public void setNearendMaxDecFactorLF(float factor) {
        if (initialized) {
            nativeSetNearendMaxDecFactorLF(factor);
        }
    }
    
    /**
     * Set nearend maximum increase factor (nearend voice recovery)
     * Higher values = clearer voice when user is speaking
     * @param factor 0.0-100.0 range, 0=use AEC3 default, typical values: 2.0-8.0
     */
    public void setNearendMaxIncFactor(float factor) {
        if (initialized) {
            nativeSetNearendMaxIncFactor(factor);
        }
    }
    
    // ======= DOMINANT NEAREND DETECTION METHODS =======
    
    /**
     * Set Energy-to-Noise Ratio threshold for voice detection
     * Lower values = more sensitive voice detection = better voice preservation
     * @param threshold 0.0-1000.0 range, 0=use AEC3 default, typical values: 0.1-1.0
     */
    public void setEnrThreshold(float threshold) {
        if (initialized) {
            nativeSetEnrThreshold(threshold);
        }
    }
    
    /**
     * Set Signal-to-Noise Ratio threshold for voice detection
     * Lower values = voice detection at lower signal levels
     * @param threshold 0.0-1000.0 range, 0=use AEC3 default, typical values: 10.0-30.0
     */
    public void setSnrThreshold(float threshold) {
        if (initialized) {
            nativeSetSnrThreshold(threshold);
        }
    }
    
    /**
     * Set hold duration for voice detection (in processing blocks)
     * Longer duration = more stable voice detection but slower response
     * @param duration 0-10000 range, 0=use AEC3 default, typical values: 5-20 blocks
     */
    public void setHoldDuration(int duration) {
        if (initialized) {
            nativeSetHoldDuration(duration);
        }
    }
    
    /**
     * Set trigger threshold for voice detection activation
     * Lower values = voice detection triggers more easily
     * @param threshold 0-10000 range, 0=use AEC3 default, typical values: 1-5
     */
    public void setTriggerThreshold(int threshold) {
        if (initialized) {
            nativeSetTriggerThreshold(threshold);
        }
    }
    
    // 🎯 ENHANCED ERLE OPTIMIZATION METHODS FOR MOBILE DEVELOPERS (2025-01-31)
    
    /**
     * Automatically optimize delay for maximum ERLE performance
     * Call this when you notice poor echo cancellation performance
     * @return true if optimization was successful
     */
    public boolean autoOptimizeDelay() {
        if (!initialized) return false;
        return nativeAutoOptimizeDelay();
    }
    
    /**
     * Get enhanced AEC performance metrics with detailed information
     * @return EnhancedAecMetrics object with comprehensive performance data
     */
    public EnhancedAecMetrics getEnhancedMetrics() {
        if (!initialized) return null;
        
        double[] metrics = nativeGetEnhancedMetrics();
        if (metrics != null && metrics.length == 6) {
            return new EnhancedAecMetrics(metrics[0], metrics[1], (int)metrics[2], 
                                        (long)metrics[3], (long)metrics[4], (int)metrics[5]);
        }
        return null;
    }
    
    /**
     * Enable or disable precise timing synchronization
     * Disable for lower CPU usage if timing sync is not critical
     * @param enable true to enable timing sync, false to disable
     * @return true if setting was applied successfully
     */
    public boolean enableTimingSync(boolean enable) {
        if (!initialized) return false;
        return nativeEnableTimingSync(enable);
    }
    
    // 🎯 ERLE ADJUSTMENT PARAMETER METHODS FOR MOBILE DEVELOPERS (2025-01-31)
    // Based on adjust-ERLE-result.md - fine-tune ERLE performance and convergence speed
    
    /**
     * Set filter length in blocks for echo learning
     * Higher values = better echo learning but slower convergence
     * @param blocks 1-100 range, default=25 (from adjust-ERLE-result.md)
     */
    public void setFilterLengthBlocks(int blocks) {
        if (initialized) {
            nativeSetFilterLengthBlocks(blocks);
        }
    }
    
    /**
     * Set filter leakage when converged for stability
     * Lower values = faster convergence but less stability
     * @param leakage 0.000001-1.0 range, default=0.000005 (from adjust-ERLE-result.md)
     */
    public void setFilterLeakageConverged(float leakage) {
        if (initialized) {
            nativeSetFilterLeakageConverged(leakage);
        }
    }
    
    /**
     * Set filter leakage when diverged for recovery
     * Lower values = tighter divergence recovery
     * @param leakage 0.001-1.0 range, default=0.005 (from adjust-ERLE-result.md)
     */
    public void setFilterLeakageDiverged(float leakage) {
        if (initialized) {
            nativeSetFilterLeakageDiverged(leakage);
        }
    }
    
    /**
     * Set delay estimation down sampling factor for precision
     * Lower values = higher precision but more CPU usage
     * @param factor 1-8 range, default=2 (from adjust-ERLE-result.md)
     */
    public void setDelayDownSamplingFactor(int factor) {
        if (initialized) {
            nativeSetDelayDownSamplingFactor(factor);
        }
    }
    
    /**
     * Set number of delay estimation filters
     * Higher values = better delay detection across devices
     * @param filters 1-32 range, default=16 (from adjust-ERLE-result.md)
     */
    public void setDelayNumFilters(int filters) {
        if (initialized) {
            nativeSetDelayNumFilters(filters);
        }
    }
    
    /**
     * Set delay estimate smoothing factor for stability
     * Higher values = more stable delay estimation
     * @param smoothing 0.1-0.99 range, default=0.98 (from adjust-ERLE-result.md)
     */
    public void setDelayEstimateSmoothing(float smoothing) {
        if (initialized) {
            nativeSetDelayEstimateSmoothing(smoothing);
        }
    }
    
    // 🎯 CLEAN AUDIO CONVERSION METHODS (2025-01-31)
    
    /**
     * Get accumulated clean audio as WAV format and clear buffer
     * This method retrieves all processed clean audio frames since recording started
     * @param outputSampleRate Desired output sample rate (default: 44100)
     * @return WAV file data as byte array, or null if no audio available
     */
    public byte[] getCleanAudioAsWAV(int outputSampleRate) {
        if (!initialized) return null;
        return nativeGetCleanAudioAsWAV(outputSampleRate);
    }
    
    /**
     * Get accumulated clean audio as WAV format with default sample rate
     * @return WAV file data as byte array, or null if no audio available
     */
    public byte[] getCleanAudioAsWAV() {
        return getCleanAudioAsWAV(44100);
    }
    
    /**
     * Get accumulated clean audio as PCM format and clear buffer
     * This method retrieves all processed clean audio frames since recording started
     * @param outputSampleRate Desired output sample rate (default: 44100)
     * @return PCM audio data as byte array (16-bit little-endian), or null if no audio available
     */
    public byte[] getCleanAudioAsPCM(int outputSampleRate) {
        if (!initialized) return null;
        return nativeGetCleanAudioAsPCM(outputSampleRate);
    }
    
    /**
     * Get accumulated clean audio as PCM format with default sample rate
     * @return PCM audio data as byte array (16-bit little-endian), or null if no audio available
     */
    public byte[] getCleanAudioAsPCM() {
        return getCleanAudioAsPCM(44100);
    }
    
    /**
     * Clear the accumulated clean audio buffer without retrieving data
     * Use this to discard accumulated audio when starting a new recording session
     */
    public void clearCleanAudioBuffer() {
        if (initialized) {
            nativeClearCleanAudioBuffer();
        }
    }

    /**
     * Class to hold AEC performance metrics
     */
    public static class AecMetrics {
        public final double echoReturnLoss;
        public final double echoReturnLossEnhancement;
        public final int delayMs;

        public AecMetrics(double erl, double erle, int delay) {
            this.echoReturnLoss = erl;
            this.echoReturnLossEnhancement = erle;
            this.delayMs = delay;
        }

        @Override
        public String toString() {
            return String.format("AEC Metrics: ERL=%.2fdB, ERLE=%.2fdB, Delay=%dms", 
                               echoReturnLoss, echoReturnLossEnhancement, delayMs);
        }
    }
    
    /**
     * Enhanced AEC performance metrics with detailed information (2025-01-31)
     */
    public static class EnhancedAecMetrics {
        public final double echoReturnLoss;
        public final double echoReturnLossEnhancement;
        public final int delayMs;
        public final long renderFrames;
        public final long captureFrames;
        public final int optimalDelayMs;

        public EnhancedAecMetrics(double erl, double erle, int delay, long renderFrames, long captureFrames, int optimalDelay) {
            this.echoReturnLoss = erl;
            this.echoReturnLossEnhancement = erle;
            this.delayMs = delay;
            this.renderFrames = renderFrames;
            this.captureFrames = captureFrames;
            this.optimalDelayMs = optimalDelay;
        }

        @Override
        public String toString() {
            return String.format("Enhanced AEC Metrics: ERL=%.2fdB, ERLE=%.2fdB, Delay=%dms, " +
                               "RenderFrames=%d, CaptureFrames=%d, OptimalDelay=%dms", 
                               echoReturnLoss, echoReturnLossEnhancement, delayMs, 
                               renderFrames, captureFrames, optimalDelayMs);
        }
        
        /**
         * Get ERLE quality assessment
         * @return Quality level: "Excellent" (>15dB), "Good" (>10dB), "Fair" (>5dB), "Poor" (<5dB)
         */
        public String getErleQuality() {
            if (echoReturnLossEnhancement >= 15.0) return "Excellent";
            else if (echoReturnLossEnhancement >= 10.0) return "Good";
            else if (echoReturnLossEnhancement >= 5.0) return "Fair";
            else return "Poor";
        }
        
        /**
         * Check if frames are synchronized (equal render and capture frame counts)
         * @return true if frames are well synchronized
         */
        public boolean isFrameSynchronized() {
            if (renderFrames == 0 || captureFrames == 0) return false;
            double ratio = (double) Math.min(renderFrames, captureFrames) / Math.max(renderFrames, captureFrames);
            return ratio > 0.95; // Within 5% is considered synchronized
        }
    }
}
