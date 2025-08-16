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
     * Initialize the AEC3 processor
     * @return true if initialization successful
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
    
    // 🎛️ RUNTIME PARAMETER CONTROL FOR PRODUCTION TUNING
    public native void nativeSetEchoSuppression(float strength);    // 8.0-20.0 range
    public native void nativeSetVoiceRecovery(float speed);         // 1.0-5.0 range
    public native void nativeSetVoiceProtection(float level);       // 1.0-8.0 range  
    public native void nativeSetFilterLength(int blocks);          // 10-25 range
    public native void nativeSetNoiseGate(float threshold);        // 0.05-0.5 range
    
    // 🎙️ VOICE CLARITY RUNTIME CONTROLS (2025-01-30)
    public native void nativeSetEchoSuppressionStrength(float strength);  // 3.0-10.0 range
    public native void nativeSetVoiceRecoverySpeed(float speed);           // 1.5-5.0 range
    public native void nativeSetVoiceDetectionSensitivity(float sensitivity); // 0.2-0.8 range
    public native void nativeSetVoiceTriggerSpeed(int speed);              // 1-5 range
    
    // 🎯 ENHANCED ERLE OPTIMIZATION METHODS (2025-01-30)
    public native boolean nativeAutoOptimizeDelay();               // Automatic delay optimization
    public native double[] nativeGetEnhancedMetrics();             // [ERL, ERLE, delay, render_frames, capture_frames, optimal_delay]
    public native boolean nativeEnableTimingSync(boolean enable);   // Enable/disable precise timing sync

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
    
    // 🎛️ PRODUCTION TUNING METHODS - REAL-TIME PARAMETER ADJUSTMENT
    
    /**
     * Set echo suppression strength (higher = more aggressive echo removal)
     * @param strength 8.0-20.0 range, default 12.0
     */
    public void setEchoSuppression(float strength) {
        if (initialized) {
            nativeSetEchoSuppression(strength);
        }
    }
    
    /**
     * Set voice recovery speed (higher = faster voice recovery after echo)
     * @param speed 1.0-5.0 range, default 3.0
     */
    public void setVoiceRecovery(float speed) {
        if (initialized) {
            nativeSetVoiceRecovery(speed);
        }
    }
    
    /**
     * Set voice protection level (lower = better voice preservation)
     * @param level 1.0-8.0 range, default 2.0
     */
    public void setVoiceProtection(float level) {
        if (initialized) {
            nativeSetVoiceProtection(level);
        }
    }
    
    /**
     * Set filter length (longer = better echo modeling, higher CPU usage)
     * @param blocks 10-25 range, default 20
     */
    public void setFilterLength(int blocks) {
        if (initialized) {
            nativeSetFilterLength(blocks);
        }
    }
    
    /**
     * Set noise gate threshold (lower = more sensitive)
     * @param threshold 0.05-0.5 range, default 0.1
     */
    public void setNoiseGate(float threshold) {
        if (initialized) {
            nativeSetNoiseGate(threshold);
        }
    }
    
    // 🎙️ VOICE CLARITY RUNTIME CONTROLS FOR UI ADJUSTMENT (2025-01-30)
    
    /**
     * Set echo suppression strength for voice clarity tuning
     * Lower values = less aggressive suppression = clearer voice but potentially more echo
     * @param strength 3.0-10.0 range, default 6.0, recommended 4.0-8.0 for voice clarity
     */
    public void setEchoSuppressionStrength(float strength) {
        if (initialized) {
            nativeSetEchoSuppressionStrength(strength);
        }
    }
    
    /**
     * Set voice recovery speed for faster voice restoration
     * Higher values = faster voice recovery after echo suppression
     * @param speed 1.5-5.0 range, default 2.5, recommended 2.0-4.0 for balance
     */
    public void setVoiceRecoverySpeed(float speed) {
        if (initialized) {
            nativeSetVoiceRecoverySpeed(speed);
        }
    }
    
    /**
     * Set voice detection sensitivity 
     * Lower values = more sensitive to voice = better voice preservation
     * @param sensitivity 0.2-0.8 range, default 0.4, recommended 0.3-0.5 for clarity
     */
    public void setVoiceDetectionSensitivity(float sensitivity) {
        if (initialized) {
            nativeSetVoiceDetectionSensitivity(sensitivity);
        }
    }
    
    /**
     * Set voice trigger speed for faster voice detection
     * Lower values = faster voice trigger = quicker voice preservation
     * @param speed 1-5 range, default 2, recommended 1-3 for responsive voice
     */
    public void setVoiceTriggerSpeed(int speed) {
        if (initialized) {
            nativeSetVoiceTriggerSpeed(speed);
        }
    }
    
    // 🎯 ENHANCED ERLE OPTIMIZATION METHODS FOR MOBILE DEVELOPERS (2025-01-30)
    
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
     * Enhanced AEC performance metrics with detailed information (2025-01-30)
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
