#include <jni.h>
#include <memory>
#include "wq_aec3_processor.h"

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
Java_com_tts_aec3_WebRtcAec3_nativeInitialize(JNIEnv *env, jobject thiz) {
    g_processor = std::make_unique<webrtc_aec3_tts::WqAec3Processor>();
    return g_processor->Initialize() ? JNI_TRUE : JNI_FALSE;
}

/**
 * Clean up and destroy the AEC3 processor
 */
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeDestroy(JNIEnv *env, jobject thiz) {
    g_processor.reset();
}

/**
 * Process TTS audio (reference signal)
 * @param tts_data TTS audio samples (must be 480 samples)
 * @return true if processing successful
 */
JNIEXPORT jboolean JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeProcessTtsAudio(JNIEnv *env, jobject thiz, jshortArray tts_data) {
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
 * Get clean microphone audio with echo cancellation and sample rate conversion
 * @param inputSamples Input audio samples (int16_t array)
 * @param inputSampleRate Input sample rate
 * @param outputSampleRate Output sample rate  
 * @return jintArray: [error_code, num_output_samples, output_samples..., erle_int, delay_ms]
 *         error_code: 0=success, negative=error
 *         If success: [0, num_samples, sample1, sample2, ..., erle_int, delay_ms]
 *         erle_int: ERLE * 1000 (to pass as integer)
 */
JNIEXPORT jintArray JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeGetCleanMicrophoneAudio(JNIEnv *env, jobject thiz, 
                                                          jshortArray inputSamples, 
                                                          jint inputSampleRate, 
                                                          jint outputSampleRate) {
    if (!g_processor) {
        jintArray result = env->NewIntArray(1);
        jint error = -3;
        env->SetIntArrayRegion(result, 0, 1, &error);
        return result;
    }
    
    jsize inputLength = env->GetArrayLength(inputSamples);
    if (inputLength <= 0) {
        jintArray result = env->NewIntArray(1);
        jint error = -2;
        env->SetIntArrayRegion(result, 0, 1, &error);
        return result;
    }
    
    jshort* inputData = env->GetShortArrayElements(inputSamples, nullptr);
    
    int16_t* outputSamples = nullptr;
    uint32_t numOutputSamples = 0;
    double erle = 0.0;
    int delayMs = 0;
    
    int result = g_processor->GetCleanMicrophoneAudio(
        reinterpret_cast<const int16_t*>(inputData),
        static_cast<uint32_t>(inputLength),
        static_cast<uint32_t>(inputSampleRate),
        &outputSamples,
        &numOutputSamples,
        static_cast<uint32_t>(outputSampleRate),
        &erle,
        &delayMs);
    
    env->ReleaseShortArrayElements(inputSamples, inputData, JNI_ABORT);
    
    if (result != 0) {
        // Error occurred
        jintArray errorResult = env->NewIntArray(1);
        env->SetIntArrayRegion(errorResult, 0, 1, &result);
        return errorResult;
    }
    
    // Success - pack result: [0, num_samples, samples..., erle_int, delay_ms]
    jintArray successResult = env->NewIntArray(3 + numOutputSamples);
    if (!successResult) {
        if (outputSamples) free(outputSamples);
        jintArray errorResult = env->NewIntArray(1);
        jint error = -4;
        env->SetIntArrayRegion(errorResult, 0, 1, &error);
        return errorResult;
    }
    
    // Pack the result
    std::vector<jint> resultData;
    resultData.push_back(0); // Success code
    resultData.push_back(static_cast<jint>(numOutputSamples));
    
    // Add audio samples
    for (uint32_t i = 0; i < numOutputSamples; ++i) {
        resultData.push_back(static_cast<jint>(outputSamples[i]));
    }
    
    // Add ERLE (multiply by 1000 to preserve decimal places)
    resultData.push_back(static_cast<jint>(erle * 1000.0));
    // Add delay
    resultData.push_back(static_cast<jint>(delayMs));
    
    env->SetIntArrayRegion(successResult, 0, resultData.size(), resultData.data());
    
    // Free allocated memory
    if (outputSamples) {
        free(outputSamples);
    }
    
    return successResult;
}

/**
 * Get current AEC performance metrics
 * @return double array: [echo_return_loss, echo_return_loss_enhancement, delay_ms]
 */
JNIEXPORT jdoubleArray JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeGetMetrics(JNIEnv *env, jobject thiz) {
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
Java_com_tts_aec3_WebRtcAec3_nativeSetStreamDelay(JNIEnv *env, jobject thiz, jint delay_ms) {
    if (g_processor) {
        g_processor->SetStreamDelay(delay_ms);
    }
}

// ========== OFFICIAL AEC3 PARAMETER CONTROL JNI METHODS ==========

// Filter Configuration JNI Methods
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetConfigChangeDuration(JNIEnv *env, jobject thiz, jint blocks) {
    if (g_processor) {
        g_processor->SetConfigChangeDuration(blocks);
    }
}

JNIEXPORT void JNICALL  
Java_com_tts_aec3_WebRtcAec3_nativeSetInitialStateSeconds(JNIEnv *env, jobject thiz, jfloat seconds) {
    if (g_processor) {
        g_processor->SetInitialStateSeconds(seconds);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetConservativeInitialPhase(JNIEnv *env, jobject thiz, jboolean enable) {
    if (g_processor) {
        g_processor->SetConservativeInitialPhase(enable == JNI_TRUE);
    }
}

// Suppressor Normal Tuning JNI Methods
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetMaxDecFactorLF(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetMaxDecFactorLF(factor);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetMaxIncFactor(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetMaxIncFactor(factor);
    }
}

// Suppressor Nearend Tuning JNI Methods
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetNearendMaxDecFactorLF(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetNearendMaxDecFactorLF(factor);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetNearendMaxIncFactor(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetNearendMaxIncFactor(factor);
    }
}

// Dominant Nearend Detection JNI Methods
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetEnrThreshold(JNIEnv *env, jobject thiz, jfloat threshold) {
    if (g_processor) {
        g_processor->SetEnrThreshold(threshold);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetSnrThreshold(JNIEnv *env, jobject thiz, jfloat threshold) {
    if (g_processor) {
        g_processor->SetSnrThreshold(threshold);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetHoldDuration(JNIEnv *env, jobject thiz, jint duration) {
    if (g_processor) {
        g_processor->SetHoldDuration(duration);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetTriggerThreshold(JNIEnv *env, jobject thiz, jint threshold) {
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
Java_com_tts_aec3_WebRtcAec3_nativeAutoOptimizeDelay(JNIEnv *env, jobject thiz) {
    if (g_processor) {
        return g_processor->AutoOptimizeDelay() ? JNI_TRUE : JNI_FALSE;
    }
    return JNI_FALSE;
}

/**
 * Clear internal input buffer - call when stopping recording or switching streams (2025-01-31)
 */
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeClearInputBuffer(JNIEnv *env, jobject thiz) {
    if (g_processor) {
        g_processor->ClearInputBuffer();
    }
}

/**
 * Get enhanced AEC performance metrics with detailed information
 * @return double array: [ERL, ERLE, delay, render_frames, capture_frames, optimal_delay]
 */
JNIEXPORT jdoubleArray JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeGetEnhancedMetrics(JNIEnv *env, jobject thiz) {
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
Java_com_tts_aec3_WebRtcAec3_nativeEnableTimingSync(JNIEnv *env, jobject thiz, jboolean enable) {
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
Java_com_tts_aec3_WebRtcAec3_nativeSetFilterLengthBlocks(JNIEnv *env, jobject thiz, jint blocks) {
    if (g_processor) {
        g_processor->SetFilterLengthBlocks(blocks);
    }
}

/**
 * Set filter leakage when converged for stability
 * @param leakage 0.000001-1.0 range, default=0.000005 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetFilterLeakageConverged(JNIEnv *env, jobject thiz, jfloat leakage) {
    if (g_processor) {
        g_processor->SetFilterLeakageConverged(leakage);
    }
}

/**
 * Set filter leakage when diverged for recovery
 * @param leakage 0.001-1.0 range, default=0.005 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetFilterLeakageDiverged(JNIEnv *env, jobject thiz, jfloat leakage) {
    if (g_processor) {
        g_processor->SetFilterLeakageDiverged(leakage);
    }
}

/**
 * Set delay estimation down sampling factor for precision
 * @param factor 1-8 range, default=2 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetDelayDownSamplingFactor(JNIEnv *env, jobject thiz, jint factor) {
    if (g_processor) {
        g_processor->SetDelayDownSamplingFactor(factor);
    }
}

/**
 * Set number of delay estimation filters
 * @param filters 1-32 range, default=16 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetDelayNumFilters(JNIEnv *env, jobject thiz, jint filters) {
    if (g_processor) {
        g_processor->SetDelayNumFilters(filters);
    }
}

/**
 * Set delay estimate smoothing factor for stability
 * @param smoothing 0.1-0.99 range, default=0.98 (from adjust-ERLE-result.md)
 */
JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetDelayEstimateSmoothing(JNIEnv *env, jobject thiz, jfloat smoothing) {
    if (g_processor) {
        g_processor->SetDelayEstimateSmoothing(smoothing);
    }
}

} // extern "C"
