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
}
