#include <jni.h>
#include <memory>
#include <android/log.h>
#include "wq_aec3_processor.h"

// Clean WebRTC AEC3 JNI Implementation using Official API
// This file provides the JNI bridge between Java and the clean C++ AEC3 processor

// Global processor instance
static std::unique_ptr<webrtc_aec3_tts::WqAec3Processor> g_processor;

extern "C" {

// ========== Core AEC3 JNI Methods ==========

/**
 * Initialize AEC3 processor
 * @return true if initialization successful
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeInitialize(JNIEnv *env, jobject thiz) {
    try {
        g_processor = std::make_unique<webrtc_aec3_tts::WqAec3Processor>();
        bool result = g_processor->Initialize();
        
        __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3", 
                           "AEC3 processor initialization: %s", result ? "SUCCESS" : "FAILED");
        
        return result ? JNI_TRUE : JNI_FALSE;
    } catch (const std::exception& e) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3", 
                           "AEC3 initialization exception: %s", e.what());
        return JNI_FALSE;
    }
}

/**
 * Destroy AEC3 processor
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeDestroy(JNIEnv *env, jobject thiz) {
    if (g_processor) {
        g_processor->Destroy();
        g_processor.reset();
        __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3", "AEC3 processor destroyed");
    }
}

/**
 * Process render audio (reference signal for TTS)
 * @param render_data TTS audio samples (must be 480 samples for 48kHz)
 * @return true if processing successful
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeProcessRenderAudio(JNIEnv *env, jobject thiz, jshortArray render_data) {
    if (!g_processor) return JNI_FALSE;
    
    jsize length = env->GetArrayLength(render_data);
    if (length != webrtc_aec3_tts::WqAec3Processor::kFrameSize) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3", 
                           "Invalid render frame size: %d, expected: %d", 
                           length, webrtc_aec3_tts::WqAec3Processor::kFrameSize);
        return JNI_FALSE;
    }
    
    jshort* data = env->GetShortArrayElements(render_data, nullptr);
    bool result = g_processor->ProcessRenderAudio(
        reinterpret_cast<const int16_t*>(data), length);
    env->ReleaseShortArrayElements(render_data, data, JNI_ABORT);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

/**
 * Process capture audio and remove echo
 * @param capture_data Microphone input samples (must be 480 samples for 48kHz)
 * @param output_data Output buffer for processed audio (must be 480 samples)
 * @return true if processing successful
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeProcessCaptureAudio(JNIEnv *env, jobject thiz, 
                                                               jshortArray capture_data, jshortArray output_data) {
    if (!g_processor) return JNI_FALSE;
    
    jsize length = env->GetArrayLength(capture_data);
    if (length != webrtc_aec3_tts::WqAec3Processor::kFrameSize) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3", 
                           "Invalid capture frame size: %d, expected: %d", 
                           length, webrtc_aec3_tts::WqAec3Processor::kFrameSize);
        return JNI_FALSE;
    }
    
    jshort* input = env->GetShortArrayElements(capture_data, nullptr);
    jshort* output = env->GetShortArrayElements(output_data, nullptr);
    
    bool result = g_processor->ProcessCaptureAudio(
        reinterpret_cast<const int16_t*>(input), 
        reinterpret_cast<int16_t*>(output), length);
    
    env->ReleaseShortArrayElements(capture_data, input, JNI_ABORT);
    env->ReleaseShortArrayElements(output_data, output, 0);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

/**
 * Get current AEC performance metrics
 * @return double array: [echo_return_loss, echo_return_loss_enhancement, delay_ms]
 */
JNIEXPORT jdoubleArray JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeGetMetrics(JNIEnv *env, jobject thiz) {
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
 * Set audio buffer delay compensation
 * @param delay_ms Delay in milliseconds
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetAudioBufferDelay(JNIEnv *env, jobject thiz, jint delay_ms) {
    if (g_processor) {
        g_processor->SetAudioBufferDelay(delay_ms);
    }
}

// ========== Clean Audio Buffer Management ==========

/**
 * Get clean audio buffer as byte array for WAV file generation
 * @return byte array containing processed audio data as int16 PCM
 */
JNIEXPORT jbyteArray JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeGetCleanAudioBuffer(JNIEnv *env, jobject thiz) {
    if (!g_processor) return nullptr;
    
    try {
        std::vector<uint8_t> audio_bytes = g_processor->GetCleanAudioAsBytes();
        
        if (audio_bytes.empty()) {
            return nullptr;
        }
        
        jbyteArray result = env->NewByteArray(audio_bytes.size());
        env->SetByteArrayRegion(result, 0, audio_bytes.size(), 
                               reinterpret_cast<const jbyte*>(audio_bytes.data()));
        
        __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3", 
                           "Retrieved %zu frames, %zu total samples", 
                           audio_bytes.size() / (2 * webrtc_aec3_tts::WqAec3Processor::kFrameSize),
                           audio_bytes.size() / 2);
        
        return result;
    } catch (const std::exception& e) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3", 
                           "Error getting clean audio buffer: %s", e.what());
        return nullptr;
    }
}

/**
 * Clear clean audio buffer without retrieving data
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeClearCleanAudioBuffer(JNIEnv *env, jobject thiz) {
    if (g_processor) {
        g_processor->ClearCleanAudioBuffer();
        __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3", "Clean audio buffer cleared");
    }
}



} // extern "C"
