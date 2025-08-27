package cn.watchfun.aec3;

public class WqAecProcessor {
    static {
        System.loadLibrary("wq_aec3_tts");
    }

    public static final int SAMPLE_RATE = 48000;
    public static final int FRAME_SIZE = 480;
    public static final int CHANNELS = 1;
    public static final int BITS_PER_SAMPLE = 16;

    public native boolean nativeInitialize();
    public native void nativeDestroy();
    public native boolean nativeProcessRenderAudio(short[] renderData);
    public native boolean nativeProcessCaptureAudio(short[] captureData, short[] outputData);
    public native double[] nativeGetMetrics();
    public native void nativeSetAudioBufferDelay(int delayMs);
    public native byte[] nativeGetCleanAudioBuffer();
    public native void nativeClearCleanAudioBuffer();

    private boolean initialized = false;

    public boolean initialize() {
        if (!initialized) {
            initialized = nativeInitialize();
        }
        return initialized;
    }

    public void destroy() {
        if (initialized) {
            nativeDestroy();
            initialized = false;
        }
    }

    public boolean processTtsAudio(short[] ttsData) {
        if (!initialized || ttsData.length != FRAME_SIZE) {
            return false;
        }
        return nativeProcessRenderAudio(ttsData);
    }

    public short[] processMicrophoneAudio(short[] micData) {
        if (!initialized || micData.length != FRAME_SIZE) {
            return null;
        }
        
        short[] output = new short[FRAME_SIZE];
        if (nativeProcessCaptureAudio(micData, output)) {
            return output;
        }
        return null;
    }

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

    public AecMetrics getMetrics() {
        if (!initialized) return null;
        
        double[] metrics = nativeGetMetrics();
        if (metrics != null && metrics.length == 3) {
            return new AecMetrics(metrics[0], metrics[1], (int)metrics[2]);
        }
        return null;
    }
}
