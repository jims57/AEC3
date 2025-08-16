#include <jni.h>
#include <android/log.h>
#include "wqaec.h"

#define LOG_TAG "WQAEC_Android"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)

using namespace wqaec;

extern "C" {

// JNI function implementations using WQAEC C API

JNIEXPORT jlong JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeCreate(JNIEnv *env, jobject thiz) {
    WQAECHandle handle = wqaec_create();
    LOGI("WQAEC instance created: %p", handle);
    return reinterpret_cast<jlong>(handle);
}

JNIEXPORT jlong JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeCreateForEnvironment(JNIEnv *env, jobject thiz, jint environment) {
    WQAECHandle handle = wqaec_create_for_environment(environment);
    LOGI("WQAEC instance created for environment %d: %p", environment, handle);
    return reinterpret_cast<jlong>(handle);
}

JNIEXPORT jboolean JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeInitialize(JNIEnv *env, jobject thiz, jlong handle) {
    if (handle == 0) return JNI_FALSE;
    
    WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
    int result = wqaec_initialize(wqaec_handle);
    
    LOGI("WQAEC initialization result: %s", result ? "success" : "failed");
    return result ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeDestroy(JNIEnv *env, jobject thiz, jlong handle) {
    if (handle != 0) {
        WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
        wqaec_destroy(wqaec_handle);
        LOGI("WQAEC instance destroyed: %p", wqaec_handle);
    }
}

JNIEXPORT jboolean JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeProcessTtsAudio(JNIEnv *env, jobject thiz, jlong handle, jshortArray tts_data) {
    if (handle == 0 || !tts_data) return JNI_FALSE;
    
    WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
    
    jsize length = env->GetArrayLength(tts_data);
    if (length != 480) {  // Must be exactly 480 samples (10ms at 48kHz)
        LOGE("Invalid TTS frame size: %d, expected 480", length);
        return JNI_FALSE;
    }
    
    jshort* data = env->GetShortArrayElements(tts_data, nullptr);
    if (!data) return JNI_FALSE;
    
    int result = wqaec_process_reference_audio(wqaec_handle, 
                                              reinterpret_cast<const int16_t*>(data), 
                                              length);
    
    env->ReleaseShortArrayElements(tts_data, data, JNI_ABORT);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jshortArray JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeProcessMicrophoneAudio(JNIEnv *env, jobject thiz, jlong handle, jshortArray mic_data) {
    if (handle == 0 || !mic_data) return nullptr;
    
    WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
    
    jsize length = env->GetArrayLength(mic_data);
    if (length != 480) {  // Must be exactly 480 samples (10ms at 48kHz)
        LOGE("Invalid microphone frame size: %d, expected 480", length);
        return nullptr;
    }
    
    jshort* input = env->GetShortArrayElements(mic_data, nullptr);
    if (!input) return nullptr;
    
    // Create output array
    jshortArray output_array = env->NewShortArray(length);
    if (!output_array) {
        env->ReleaseShortArrayElements(mic_data, input, JNI_ABORT);
        return nullptr;
    }
    
    jshort* output = env->GetShortArrayElements(output_array, nullptr);
    if (!output) {
        env->ReleaseShortArrayElements(mic_data, input, JNI_ABORT);
        return nullptr;
    }
    
    int result = wqaec_process_capture_audio(wqaec_handle,
                                           reinterpret_cast<const int16_t*>(input),
                                           reinterpret_cast<int16_t*>(output),
                                           length);
    
    env->ReleaseShortArrayElements(mic_data, input, JNI_ABORT);
    
    if (result) {
        env->ReleaseShortArrayElements(output_array, output, 0);  // Copy back
        return output_array;
    } else {
        env->ReleaseShortArrayElements(output_array, output, JNI_ABORT);  // Don't copy back
        return nullptr;
    }
}

JNIEXPORT jdoubleArray JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeGetMetrics(JNIEnv *env, jobject thiz, jlong handle) {
    if (handle == 0) return nullptr;
    
    WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
    
    double erl, erle;
    int delay_ms;
    float convergence;
    
    int result = wqaec_get_metrics(wqaec_handle, &erl, &erle, &delay_ms, &convergence);
    
    if (!result) return nullptr;
    
    jdoubleArray metrics_array = env->NewDoubleArray(4);
    if (!metrics_array) return nullptr;
    
    double metrics[] = {erl, erle, static_cast<double>(delay_ms), static_cast<double>(convergence)};
    env->SetDoubleArrayRegion(metrics_array, 0, 4, metrics);
    
    return metrics_array;
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetStreamDelay(JNIEnv *env, jobject thiz, jlong handle, jint delay_ms) {
    if (handle != 0) {
        WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
        wqaec_set_stream_delay(wqaec_handle, delay_ms);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetEchoSuppression(JNIEnv *env, jobject thiz, jlong handle, jfloat strength) {
    if (handle != 0) {
        WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
        wqaec_set_echo_suppression(wqaec_handle, strength);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetVoiceRecovery(JNIEnv *env, jobject thiz, jlong handle, jfloat speed) {
    if (handle != 0) {
        WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
        wqaec_set_voice_recovery(wqaec_handle, speed);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetVoiceProtection(JNIEnv *env, jobject thiz, jlong handle, jfloat level) {
    if (handle != 0) {
        WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
        wqaec_set_voice_protection(wqaec_handle, level);
    }
}

JNIEXPORT void JNICALL  
Java_com_tts_aec3_WebRtcAec3_nativeSetEnvironment(JNIEnv *env, jobject thiz, jlong handle, jint environment) {
    if (handle != 0) {
        WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
        wqaec_set_environment(wqaec_handle, environment);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeEnableAutoDelayDetection(JNIEnv *env, jobject thiz, jlong handle, jboolean enable) {
    if (handle != 0) {
        WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
        wqaec_enable_auto_delay_detection(wqaec_handle, enable ? 1 : 0);
    }
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeTriggerAdaptation(JNIEnv *env, jobject thiz, jlong handle) {
    if (handle != 0) {
        WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
        wqaec_trigger_adaptation(wqaec_handle);
    }
}

JNIEXPORT jboolean JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeIsTimingSynced(JNIEnv *env, jobject thiz, jlong handle) {
    if (handle == 0) return JNI_FALSE;
    
    WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
    int result = wqaec_is_timing_synced(wqaec_handle);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jfloat JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeGetConvergenceQuality(JNIEnv *env, jobject thiz, jlong handle) {
    if (handle == 0) return 0.0f;
    
    WQAECHandle wqaec_handle = reinterpret_cast<WQAECHandle>(handle);
    return wqaec_get_convergence_quality(wqaec_handle);
}

} // extern "C"
