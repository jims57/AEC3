#include <jni.h>
#include <memory>
#include "wq_aec3_processor.h"
#include "wq_aec3_convertor.h"

// JNI Implementation for WebRTC AEC3 TTS Echo Cancellation (2025-01-31)
// This file provides the JNI bridge between Java and C++ for the TTS AEC3 processor

// Global processor instance
static std::unique_ptr<webrtc_aec3_tts::WqAec3Processor> g_processor;

extern "C" {

// ========== CORE AEC3 JNI METHODS ==========

/**
 * Initialize the AEC3 processor
 * @return true if initialization successful
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeInitialize(JNIEnv *env, jobject thiz) {
    g_processor = std::make_unique<webrtc_aec3_tts::WqAec3Processor>();
    return g_processor->Initialize() ? JNI_TRUE : JNI_FALSE;
}

/**
 * Clean up and destroy the AEC3 processor
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeDestroy(JNIEnv *env, jobject thiz) {
    g_processor.reset();
}

/**
 * Process TTS audio (reference signal)
 * @param tts_data TTS audio samples (must be 480 samples)
 * @return true if processing successful
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeProcessTtsAudio(JNIEnv *env, jobject thiz, jshortArray tts_data) {
    if (!g_processor) return JNI_FALSE;
    
    jsize length = env->GetArrayLength(tts_data);
    if (length != webrtc_aec3_tts::WqAec3Processor::kFrameSize) return JNI_FALSE;
    
    jshort* data = env->GetShortArrayElements(tts_data, nullptr);
    bool result = g_processor->ProcessTtsAudio(
        reinterpret_cast<const int16_t*>(data), length);
    env->ReleaseShortArrayElements(tts_data, data, JNI_ABORT);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

/**
 * Process microphone audio and remove echo
 * @param mic_data Microphone input samples (must be 480 samples)
 * @param output_data Output buffer for processed audio (must be 480 samples)
 * @return true if processing successful
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeProcessMicrophoneAudio(JNIEnv *env, jobject thiz, 
                                                          jshortArray mic_data, jshortArray output_data) {
    if (!g_processor) return JNI_FALSE;
    
    jsize length = env->GetArrayLength(mic_data);
    if (length != webrtc_aec3_tts::WqAec3Processor::kFrameSize) return JNI_FALSE;
    
    jshort* input = env->GetShortArrayElements(mic_data, nullptr);
    jshort* output = env->GetShortArrayElements(output_data, nullptr);
    
    bool result = g_processor->ProcessMicrophoneAudio(
        reinterpret_cast<const int16_t*>(input), 
        reinterpret_cast<int16_t*>(output), length);
    
    env->ReleaseShortArrayElements(mic_data, input, JNI_ABORT);
    env->ReleaseShortArrayElements(output_data, output, 0);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

/**
 * Get current AEC performance metrics
 * @return double array: [echo_return_loss, echo_return_loss_enhancement, delay_ms]
 */
JNIEXPORT jdoubleArray JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeGetMetrics(JNIEnv *env, jobject thiz) {
    if (!g_processor) return nullptr;
    
    double erl, erle;
    int delay_ms;
    if (!g_processor->GetMetrics(&erl, &erle, &delay_ms)) {
        return nullptr;
    }
    
    jdoubleArray result = env->NewDoubleArray(3);
    double metrics[] = {erl, erle, static_cast<double>(delay_ms)};
    env->SetDoubleArrayRegion(result, 0, 3, metrics);
    return result;
}

/**
 * Update stream delay compensation
 * @param delay_ms Delay in milliseconds
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetStreamDelay(JNIEnv *env, jobject thiz, jint delay_ms) {
    if (g_processor) {
        g_processor->SetStreamDelay(delay_ms);
    }
}

// ========== OFFICIAL AEC3 PARAMETER CONTROL JNI METHODS ==========

// Filter Configuration JNI Methods
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetConfigChangeDuration(JNIEnv *env, jobject thiz, jint blocks) {
    if (g_processor) {
        g_processor->SetConfigChangeDuration(blocks);
    }
}

JNIEXPORT void JNICALL  
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetInitialStateSeconds(JNIEnv *env, jobject thiz, jfloat seconds) {
    if (g_processor) {
        g_processor->SetInitialStateSeconds(seconds);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetConservativeInitialPhase(JNIEnv *env, jobject thiz, jboolean enable) {
    if (g_processor) {
        g_processor->SetConservativeInitialPhase(enable == JNI_TRUE);
    }
}

// Suppressor Normal Tuning JNI Methods
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetMaxDecFactorLF(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetMaxDecFactorLF(factor);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetMaxIncFactor(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetMaxIncFactor(factor);
    }
}

// Suppressor Nearend Tuning JNI Methods
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetNearendMaxDecFactorLF(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetNearendMaxDecFactorLF(factor);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetNearendMaxIncFactor(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetNearendMaxIncFactor(factor);
    }
}

// Dominant Nearend Detection JNI Methods
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetEnrThreshold(JNIEnv *env, jobject thiz, jfloat threshold) {
    if (g_processor) {
        g_processor->SetEnrThreshold(threshold);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetSnrThreshold(JNIEnv *env, jobject thiz, jfloat threshold) {
    if (g_processor) {
        g_processor->SetSnrThreshold(threshold);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetHoldDuration(JNIEnv *env, jobject thiz, jint duration) {
    if (g_processor) {
        g_processor->SetHoldDuration(duration);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetTriggerThreshold(JNIEnv *env, jobject thiz, jint threshold) {
    if (g_processor) {
        g_processor->SetTriggerThreshold(threshold);
    }
}

// ========== ENHANCED ERLE OPTIMIZATION JNI METHODS ==========

/**
 * Automatically optimize delay for maximum ERLE performance
 * @return true if optimization completed successfully
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeAutoOptimizeDelay(JNIEnv *env, jobject thiz) {
    if (g_processor) {
        return g_processor->AutoOptimizeDelay() ? JNI_TRUE : JNI_FALSE;
    }
    return JNI_FALSE;
}

/**
 * Get enhanced AEC performance metrics with detailed information
 * @return double array: [ERL, ERLE, delay, render_frames, capture_frames, optimal_delay]
 */
JNIEXPORT jdoubleArray JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeGetEnhancedMetrics(JNIEnv *env, jobject thiz) {
    if (!g_processor) return nullptr;
    
    double erl, erle;
    int delay_ms;
    uint64_t render_frames, capture_frames;
    int optimal_delay;
    
    if (!g_processor->GetEnhancedMetrics(&erl, &erle, &delay_ms, &render_frames, &capture_frames, &optimal_delay)) {
        return nullptr;
    }
    
    jdoubleArray result = env->NewDoubleArray(6);
    double metrics[] = {erl, erle, static_cast<double>(delay_ms), 
                       static_cast<double>(render_frames), static_cast<double>(capture_frames),
                       static_cast<double>(optimal_delay)};
    env->SetDoubleArrayRegion(result, 0, 6, metrics);
    return result;
}

/**
 * Enable or disable precise timing synchronization
 * @param enable true to enable timing sync, false to disable
 * @return true if setting applied successfully
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeEnableTimingSync(JNIEnv *env, jobject thiz, jboolean enable) {
    if (g_processor) {
        return g_processor->EnableTimingSync(enable == JNI_TRUE) ? JNI_TRUE : JNI_FALSE;
    }
    return JNI_FALSE;
}

// ========== ERLE ADJUSTMENT PARAMETER JNI METHODS FOR MOBILE DEVELOPERS ==========

/**
 * Set filter length in blocks for echo learning
 * @param blocks 1-100 range, default=25 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetFilterLengthBlocks(JNIEnv *env, jobject thiz, jint blocks) {
    if (g_processor) {
        g_processor->SetFilterLengthBlocks(blocks);
    }
}

/**
 * Set filter leakage when converged for stability
 * @param leakage 0.000001-1.0 range, default=0.000005 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetFilterLeakageConverged(JNIEnv *env, jobject thiz, jfloat leakage) {
    if (g_processor) {
        g_processor->SetFilterLeakageConverged(leakage);
    }
}

/**
 * Set filter leakage when diverged for recovery
 * @param leakage 0.001-1.0 range, default=0.005 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetFilterLeakageDiverged(JNIEnv *env, jobject thiz, jfloat leakage) {
    if (g_processor) {
        g_processor->SetFilterLeakageDiverged(leakage);
    }
}

/**
 * Set delay estimation down sampling factor for precision
 * @param factor 1-8 range, default=2 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetDelayDownSamplingFactor(JNIEnv *env, jobject thiz, jint factor) {
    if (g_processor) {
        g_processor->SetDelayDownSamplingFactor(factor);
    }
}

/**
 * Set number of delay estimation filters
 * @param filters 1-32 range, default=16 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetDelayNumFilters(JNIEnv *env, jobject thiz, jint filters) {
    if (g_processor) {
        g_processor->SetDelayNumFilters(filters);
    }
}

/**
 * Set delay estimate smoothing factor for stability
 * @param smoothing 0.1-0.99 range, default=0.98 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeSetDelayEstimateSmoothing(JNIEnv *env, jobject thiz, jfloat smoothing) {
    if (g_processor) {
        g_processor->SetDelayEstimateSmoothing(smoothing);
    }
}

// ========== CLEAN AUDIO CONVERSION JNI METHODS ==========

/**
 * Get clean audio buffer and convert to WAV format
 * @param outputSampleRate Output sample rate (default: 44100)
 * @return byte array containing WAV data, or null on error
 */
JNIEXPORT jbyteArray JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeGetCleanAudioAsWAV(JNIEnv *env, jobject thiz, jint outputSampleRate) {
    if (!g_processor) return nullptr;
    
    // Get clean audio frames from processor buffer (no clear - PCM method will clear)
    std::vector<std::vector<float>> audioFrames;
    size_t frameCount = g_processor->GetCleanAudioBuffer(audioFrames);
    
    if (frameCount == 0) {
        return nullptr; // No audio frames available
    }
    
    // Convert to WAV format
    uint8_t* wavData = nullptr;
    size_t wavSize = 0;
    int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToWAV(
        audioFrames, 48000, &wavData, &wavSize, outputSampleRate);
    
    if (result != 0 || !wavData || wavSize == 0) {
        if (wavData) free(wavData);
        return nullptr;
    }
    
    // Create Java byte array
    jbyteArray wavArray = env->NewByteArray(static_cast<jsize>(wavSize));
    if (!wavArray) {
        free(wavData);
        return nullptr;
    }
    
    env->SetByteArrayRegion(wavArray, 0, static_cast<jsize>(wavSize), 
                           reinterpret_cast<const jbyte*>(wavData));
    
    free(wavData);
    return wavArray;
}

/**
 * Get clean audio buffer and convert to PCM format
 * @param outputSampleRate Output sample rate (default: 44100)
 * @return byte array containing PCM data, or null on error
 */
JNIEXPORT jbyteArray JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeGetCleanAudioAsPCM(JNIEnv *env, jobject thiz, jint outputSampleRate) {
    if (!g_processor) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3_TTS", "PCM: g_processor is null");
        return nullptr;
    }
    
    // Get clean audio frames from processor buffer
    std::vector<std::vector<float>> audioFrames;
    size_t frameCount = g_processor->GetCleanAudioBuffer(audioFrames);
    
    __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: Retrieved %zu frames from buffer", frameCount);
    
    if (frameCount == 0) {
        __android_log_print(ANDROID_LOG_WARN, "WebRTC_AEC3_TTS", "PCM: No audio frames available");
        return nullptr; // No audio frames available
    }
    
    // Debug: Check first frame content
    if (!audioFrames.empty() && !audioFrames[0].empty()) {
        float firstSample = audioFrames[0][0];
        float lastSample = audioFrames[0][audioFrames[0].size()-1];
        __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: First frame samples: first=%.6f, last=%.6f, size=%zu", 
                           firstSample, lastSample, audioFrames[0].size());
    }
    
    // Convert to PCM format
    uint8_t* pcmData = nullptr;
    size_t pcmSize = 0;
    int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToPCM(
        audioFrames, 48000, &pcmData, &pcmSize, outputSampleRate);
    
    __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: Conversion result=%d, pcmData=%p, pcmSize=%zu", 
                       result, pcmData, pcmSize);
    
    if (result != 0 || !pcmData || pcmSize == 0) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3_TTS", "PCM: Conversion failed - result=%d, data=%p, size=%zu", 
                           result, pcmData, pcmSize);
        if (pcmData) free(pcmData);
        return nullptr;
    }
    
    // Debug: Check first few PCM bytes
    int16_t* pcmSamples = reinterpret_cast<int16_t*>(pcmData);
    __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: First samples: [%d, %d, %d, %d]", 
                       pcmSamples[0], pcmSamples[1], pcmSamples[2], pcmSamples[3]);
    
    // Create Java byte array
    jbyteArray pcmArray = env->NewByteArray(static_cast<jsize>(pcmSize));
    if (!pcmArray) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3_TTS", "PCM: Failed to create Java byte array");
        free(pcmData);
        return nullptr;
    }
    
    env->SetByteArrayRegion(pcmArray, 0, static_cast<jsize>(pcmSize), 
                           reinterpret_cast<const jbyte*>(pcmData));
    
    __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: Successfully created byte array of size %zu", pcmSize);
    
    free(pcmData);
    return pcmArray;
}

/**
 * Clear the clean audio buffer without retrieving data
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WebRtcAec3_nativeClearCleanAudioBuffer(JNIEnv *env, jobject thiz) {
    if (g_processor) {
        g_processor->ClearCleanAudioBuffer();
    }
}



} // extern "C"
