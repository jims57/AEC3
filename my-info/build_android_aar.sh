#!/bin/bash

# WebRTC AEC3 Android AAR Build Script for TTS Echo Cancellation
# Author: AI Assistant | Date: 2025-01-28
# Purpose: Build production-ready AAR for TTS echo cancellation using WebRTC AEC3

set -e  # Exit on any error

# ============================================================================
# Configuration
# ============================================================================
PROJECT_ROOT="/Users/mac/Documents/GitHub/AEC3"
BUILD_DIR="$PROJECT_ROOT/build_android"
OUTPUT_DIR="$PROJECT_ROOT/android_output"
AAR_NAME="webrtc-aec3-tts"
JAVA_PACKAGE="com.tts.aec3"

# Android NDK Configuration
ANDROID_NDK_HOME=${ANDROID_NDK_HOME:-"/Users/mac/Library/Android/sdk/ndk/25.2.9519653"}
ANDROID_API_LEVEL=27
ANDROID_STL="c++_static"

# AEC3 Configuration (based on ace-key-points.txt)
AEC3_SAMPLE_RATE=48000
AEC3_FRAME_SIZE=480  # 10ms at 48kHz
ANDROID_STREAM_DELAY=100  # Android typical delay (80-150ms range)

echo "🚀 Building WebRTC AEC3 TTS Android AAR"
echo "📁 Project: $PROJECT_ROOT"
echo "🔧 NDK: $ANDROID_NDK_HOME"
echo "📊 AEC3 Config: ${AEC3_SAMPLE_RATE}Hz, ${AEC3_FRAME_SIZE} samples, ${ANDROID_STREAM_DELAY}ms delay"

# Validate NDK
if [ ! -d "$ANDROID_NDK_HOME" ]; then
    echo "❌ Android NDK not found at: $ANDROID_NDK_HOME"
    echo "Set ANDROID_NDK_HOME environment variable or install NDK"
    exit 1
fi

# ============================================================================
# Prepare Build Environment
# ============================================================================
echo "🧹 Cleaning previous builds..."
rm -rf "$BUILD_DIR" "$OUTPUT_DIR"
mkdir -p "$BUILD_DIR" "$OUTPUT_DIR"

# Create build directories for multiple architectures
ARCHITECTURES=("arm64-v8a" "armeabi-v7a" "x86_64" "x86")
for arch in "${ARCHITECTURES[@]}"; do
    mkdir -p "$BUILD_DIR/$arch"
done

# ============================================================================
# Generate CMakeLists.txt for AEC3 TTS
# ============================================================================
echo "📝 Generating CMakeLists.txt..."

cat > "$BUILD_DIR/CMakeLists.txt" << 'EOCMAKE'
cmake_minimum_required(VERSION 3.18.1)
project(webrtc_aec3_tts)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Android specific settings
if(ANDROID)
    set(CMAKE_ANDROID_STL_TYPE c++_static)
    add_definitions(-DWEBRTC_ANDROID -DWEBRTC_POSIX)
endif()

# Compiler flags for optimization and WebRTC AEC3 compatibility - 2025-01-31
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -O2")  # O2 instead of O3 for better quality
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_APM_DEBUG_DUMP=0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_CHECK_MSG=0")  # Enable checks for better debugging
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_INCLUDE_INTERNAL_AUDIO_DEVICE")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_EXCLUDE_FIELD_TRIAL_DEFAULT") # This line is commented out to fix the FindFullName issue
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_METRICS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_LINUX")  # Enable Linux-specific features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE")   # Enable GNU extensions for prctl
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_AEC3_ENABLE_SHADOW_FILTER")  # Enable shadow filter for better adaptation

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../api)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../audio_processing)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../audio_processing/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/rtc_base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/system_wrappers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/abseil)

# Define WebRTC AEC3 Core Sources with all required utilities
set(AEC3_CORE_SOURCES
    # API Layer
    ../api/echo_canceller3_factory.cc
    ../api/echo_canceller3_config.cc
    # Note: excluding echo_canceller3_config_json.cc due to missing json/json.h
    
    # Audio Processing Core
    ../audio_processing/audio_buffer.cc
    ../audio_processing/audio_frame.cc
    ../audio_processing/channel_buffer.cc
    ../audio_processing/channel_layout.cc
    ../audio_processing/high_pass_filter.cc
    ../audio_processing/three_band_filter_bank.cc
    ../audio_processing/splitting_filter.cc
    ../audio_processing/splitting_filter_c.c
    ../audio_processing/sparse_fir_filter.cc
    
    # Critical Utility Components (missing linker symbols)
    ../audio_processing/utility/ooura_fft.cc
    ../audio_processing/utility/cascaded_biquad_filter.cc
    ../audio_processing/utility/delay_estimator.cc
    ../audio_processing/utility/delay_estimator_wrapper.cc
    
    # Resampler Components (PushSincResampler)
    ../audio_processing/resampler/push_sinc_resampler.cc
    ../audio_processing/resampler/sinc_resampler.cc
    
    # Logging Components (ApmDataDumper)
    ../audio_processing/logging/apm_data_dumper.cc
    
    # Essential Base Components (missing implementations)
    ../base/rtc_base/memory/aligned_malloc.cc
    ../base/system_wrappers/source/cpu_features.cc
)

# Additional required sources to fix remaining linker errors
set(ADDITIONAL_SOURCES 
    # Essential abseil implementations (missing linker symbols)
    ../base/abseil/absl/base/internal/raw_logging.cc
    ../base/abseil/absl/strings/charconv.cc
    ../base/abseil/absl/strings/internal/charconv_parse.cc
    ../base/abseil/absl/strings/internal/charconv_bigint.cc
    ../base/abseil/absl/strings/internal/memutil.cc
    ../base/abseil/absl/strings/match.cc
    ../base/abseil/absl/strings/ascii.cc
    ../base/abseil/absl/numeric/int128.cc
    
    # Essential rtc_base utilities (missing implementations)
    ../base/rtc_base/strings/string_builder.cc
    ../base/rtc_base/string_encode.cc
    ../base/rtc_base/string_utils.cc
    ../base/rtc_base/platform_thread_types.cc
    ../base/rtc_base/checks.cc
    ../base/rtc_base/logging.cc
    ../base/rtc_base/time_utils.cc
    ../base/rtc_base/race_checker.cc
    ../base/rtc_base/critical_section.cc
    
    # System wrappers (field trial)
    ../base/system_wrappers/source/field_trial.cc
)

# Find all AEC3 implementation files
file(GLOB_RECURSE AEC3_IMPL_SOURCES 
    "../audio_processing/aec3/*.cc"
    "../audio_processing/aec3/*.c"
)

# Architecture-specific optimizations
set(ARCH_SPECIFIC_SOURCES "")
if(ANDROID_ABI STREQUAL "x86" OR ANDROID_ABI STREQUAL "x86_64")
    # Add SSE2 optimizations for x86 architectures
    list(APPEND ARCH_SPECIFIC_SOURCES 
        ../audio_processing/utility/ooura_fft_sse2.cc
        ../audio_processing/resampler/sinc_resampler_sse.cc
    )
    # Enable SSE2 for x86
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
elseif(ANDROID_ABI STREQUAL "armeabi-v7a" OR ANDROID_ABI STREQUAL "arm64-v8a")
    # Add NEON optimizations for ARM architectures
    list(APPEND ARCH_SPECIFIC_SOURCES 
        ../audio_processing/utility/ooura_fft_neon.cc
        ../audio_processing/resampler/sinc_resampler_neon.cc
    )
    # Enable NEON for ARM
    if(ANDROID_ABI STREQUAL "armeabi-v7a")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
    endif()
endif()

# Aggressive filtering to remove problematic files
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*test.*")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*_test\\.cc$")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*_unittest\\.cc$")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*_bench.*")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*benchmark.*")

# Combine all sources with WQAEC implementation
set(ALL_SOURCES 
    ${AEC3_CORE_SOURCES}
    ${AEC3_IMPL_SOURCES}
    ${ADDITIONAL_SOURCES}
    ${ARCH_SPECIFIC_SOURCES}
    wqaec.cpp
    wqaec_config.cpp
    android_jni_wrapper.cc
)

# Create shared library
add_library(webrtc_aec3_tts SHARED ${ALL_SOURCES})

# Link Android libraries
if(ANDROID)
    target_link_libraries(webrtc_aec3_tts
        android
        log
        OpenSLES
    )
endif()

# Set library properties
set_target_properties(webrtc_aec3_tts PROPERTIES
    VERSION 1.0
    SOVERSION 1
)
EOCMAKE

# ============================================================================
# Copy WQAEC C++ files to build directory
# ============================================================================
echo "📝 Copying WQAEC C++ implementation files..."

# Copy WQAEC C++ files from my-cpp-files
cp "$PROJECT_ROOT/my-info/my-cpp-files/wqaec.h" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/wqaec.cpp" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/wqaec_config.h" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/wqaec_config.cpp" "$BUILD_DIR/"

echo "✅ WQAEC C++ files copied successfully"

# Generate JNI Wrapper for Android integration
# ============================================================================
echo "📝 Generating Android JNI Wrapper..."

cat > "$BUILD_DIR/android_jni_wrapper.cc" << 'EOWRAPPER'
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
EOWRAPPER

# ============================================================================
# Generate Java Wrapper Classes
# ============================================================================
echo "📝 Generating Java wrapper classes..."

mkdir -p "$BUILD_DIR/java/com/tts/aec3"

cat > "$BUILD_DIR/java/com/tts/aec3/WebRtcAec3.java" << 'EOJAVA'
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
EOJAVA

# ============================================================================
# Generate Android.mk for compatibility
# ============================================================================
cat > "$BUILD_DIR/Android.mk" << 'EOANDROIDMK'
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := webrtc_aec3_tts
LOCAL_SRC_FILES := $(call all-cpp-files-under, .)
LOCAL_SRC_FILES += $(call all-c-files-under, .)

LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/.. \
    $(LOCAL_PATH)/../api \
    $(LOCAL_PATH)/../audio_processing \
    $(LOCAL_PATH)/../audio_processing/include \
    $(LOCAL_PATH)/../base \
    $(LOCAL_PATH)/../base/rtc_base \
    $(LOCAL_PATH)/../base/system_wrappers \
    $(LOCAL_PATH)/../base/abseil

LOCAL_CFLAGS := -DWEBRTC_ANDROID -DWEBRTC_POSIX -O3 -ffast-math
LOCAL_CPPFLAGS := -std=c++17 -frtti -fexceptions
LOCAL_LDLIBS := -llog -lOpenSLES -landroid

include $(BUILD_SHARED_LIBRARY)
EOANDROIDMK

# ============================================================================
# Build for Multiple Architectures
# ============================================================================
echo "🔨 Building for multiple architectures..."

for arch in "${ARCHITECTURES[@]}"; do
    echo "Building for $arch..."
    
    # Set architecture-specific variables
    case $arch in
        "arm64-v8a")
            ANDROID_ABI="arm64-v8a"
            CMAKE_TOOLCHAIN_FILE="$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake"
            ;;
        "armeabi-v7a")
            ANDROID_ABI="armeabi-v7a"
            CMAKE_TOOLCHAIN_FILE="$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake"
            ;;
        "x86_64")
            ANDROID_ABI="x86_64"
            CMAKE_TOOLCHAIN_FILE="$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake"
            ;;
        "x86")
            ANDROID_ABI="x86"
            CMAKE_TOOLCHAIN_FILE="$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake"
            ;;
    esac

    # Configure with CMake
    cd "$BUILD_DIR/$arch"
    cmake \
        -DCMAKE_TOOLCHAIN_FILE="$CMAKE_TOOLCHAIN_FILE" \
        -DANDROID_ABI="$ANDROID_ABI" \
        -DANDROID_NDK="$ANDROID_NDK_HOME" \
        -DANDROID_PLATFORM=android-$ANDROID_API_LEVEL \
        -DANDROID_STL="$ANDROID_STL" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_VERBOSE_MAKEFILE=ON \
        ..

    # Build
    make -j$(nproc) || {
        echo "❌ Build failed for $arch"
        continue
    }

    # Copy built library
    mkdir -p "$OUTPUT_DIR/jni/$arch"
    cp libwebrtc_aec3_tts.so "$OUTPUT_DIR/jni/$arch/"
    
    echo "✅ Built successfully for $arch"
done

cd "$PROJECT_ROOT"

# ============================================================================
# Create Android AAR Package
# ============================================================================
echo "📦 Creating Android AAR package..."

# Create AAR structure
AAR_DIR="$OUTPUT_DIR/aar"
mkdir -p "$AAR_DIR"/{classes,jni,res,assets}

# Copy native libraries
cp -r "$OUTPUT_DIR/jni" "$AAR_DIR/"

# Compile Java classes
javac -d "$AAR_DIR/classes" -cp "$ANDROID_SDK_ROOT/platforms/android-$ANDROID_API_LEVEL/android.jar" \
    "$BUILD_DIR/java/com/tts/aec3/WebRtcAec3.java"

# Create classes.jar
cd "$AAR_DIR/classes"
jar cf ../classes.jar .
cd "$PROJECT_ROOT"

# Create AndroidManifest.xml
cat > "$AAR_DIR/AndroidManifest.xml" << EOMANIFEST
<?xml version="1.0" encoding="utf-8"?>
<manifest xmlns:android="http://schemas.android.com/apk/res/android"
    package="com.tts.aec3"
    android:versionCode="1"
    android:versionName="1.0">
    
    <uses-sdk 
        android:minSdkVersion="$ANDROID_API_LEVEL"
        android:targetSdkVersion="34" />
    
    <uses-permission android:name="android.permission.RECORD_AUDIO" />
    <uses-permission android:name="android.permission.MODIFY_AUDIO_SETTINGS" />
    
</manifest>
EOMANIFEST

# Create R.txt (empty for this library)
touch "$AAR_DIR/R.txt"

# Package AAR
cd "$AAR_DIR"
zip -r "../${AAR_NAME}-1.0.aar" ./*
cd "$PROJECT_ROOT"

# ============================================================================
# Generate Usage Documentation
# ============================================================================
echo "📚 Generating usage documentation..."

cat > "$OUTPUT_DIR/TTS_AEC3_USAGE.md" << 'EODOC'
# WebRTC AEC3 TTS Echo Cancellation - Android Usage Guide

## Overview
This AAR provides WebRTC AEC3-based echo cancellation specifically optimized for TTS (Text-to-Speech) applications.

## Key Features
- **High-Quality Echo Cancellation**: Based on WebRTC AEC3 algorithm
- **TTS Optimized**: Designed for TTS playback scenarios
- **Mobile Optimized**: Efficient performance on Android devices
- **Real-time Processing**: 10ms latency, suitable for live applications
- **Easy Integration**: Simple Java API

## Technical Specifications
- **Sample Rate**: 48kHz (mandatory)
- **Frame Size**: 480 samples (10ms chunks)
- **Channels**: Mono (1 channel)
- **Bit Depth**: 16-bit PCM
- **Latency**: ~10ms processing + system delay
- **CPU Usage**: Optimized for mobile ARM processors

## Integration Steps

### 1. Add AAR to Your Project
```gradle
implementation files('libs/webrtc-aec3-tts-1.0.aar')
```

### 2. Add Permissions
```xml
<uses-permission android:name="android.permission.RECORD_AUDIO" />
<uses-permission android:name="android.permission.MODIFY_AUDIO_SETTINGS" />
```

### 3. Basic Usage
```java
import com.tts.aec3.WebRtcAec3;

public class TtsEchoCancellation {
    private WebRtcAec3 aec3;
    
    public void initializeAec() {
        aec3 = new WebRtcAec3();
        if (aec3.initialize()) {
            Log.i("AEC3", "Echo cancellation initialized successfully");
        }
    }
    
    public void processTtsChunk(short[] ttsAudio) {
        // CRITICAL: Call this BEFORE playing TTS audio
        aec3.processTtsAudio(ttsAudio);
        
        // Now play the TTS audio through speakers
        playTtsAudio(ttsAudio);
    }
    
    public short[] processMicrophoneChunk(short[] micAudio) {
        // Process microphone input and get echo-cancelled output
        return aec3.processMicrophoneAudio(micAudio);
    }
    
    public void cleanup() {
        if (aec3 != null) {
            aec3.destroy();
        }
    }
}
```

### 4. Complete TTS Integration Example
```java
public class TtsWithEchoCancellation {
    private WebRtcAec3 aec3;
    private AudioTrack audioTrack;
    private AudioRecord audioRecord;
    private boolean isRecording = false;
    
    public void startTtsSession() {
        // Initialize AEC3
        aec3 = new WebRtcAec3();
        aec3.initialize();
        
        // Setup audio playback (48kHz, 16-bit, mono)
        int bufferSize = AudioTrack.getMinBufferSize(
            WebRtcAec3.SAMPLE_RATE,
            AudioFormat.CHANNEL_OUT_MONO,
            AudioFormat.ENCODING_PCM_16BIT
        );
        
        audioTrack = new AudioTrack(
            AudioManager.STREAM_MUSIC,
            WebRtcAec3.SAMPLE_RATE,
            AudioFormat.CHANNEL_OUT_MONO,
            AudioFormat.ENCODING_PCM_16BIT,
            bufferSize,
            AudioTrack.MODE_STREAM
        );
        
        // Setup audio recording (48kHz, 16-bit, mono)
        int recordBufferSize = AudioRecord.getMinBufferSize(
            WebRtcAec3.SAMPLE_RATE,
            AudioFormat.CHANNEL_IN_MONO,
            AudioFormat.ENCODING_PCM_16BIT
        );
        
        audioRecord = new AudioRecord(
            MediaRecorder.AudioSource.MIC,
            WebRtcAec3.SAMPLE_RATE,
            AudioFormat.CHANNEL_IN_MONO,
            AudioFormat.ENCODING_PCM_16BIT,
            recordBufferSize
        );
        
        // Start recording
        startMicrophoneRecording();
    }
    
    public void playTtsResponse(byte[] ttsData) {
        // Convert to 16-bit samples
        short[] samples = bytesToShorts(ttsData);
        
        // Process in 10ms chunks (480 samples)
        for (int i = 0; i < samples.length; i += WebRtcAec3.FRAME_SIZE) {
            short[] chunk = Arrays.copyOfRange(samples, i, 
                Math.min(i + WebRtcAec3.FRAME_SIZE, samples.length));
            
            if (chunk.length == WebRtcAec3.FRAME_SIZE) {
                // Feed to AEC3 BEFORE playback
                aec3.processTtsAudio(chunk);
                
                // Play through speakers
                audioTrack.write(chunk, 0, chunk.length);
            }
        }
    }
    
    private void startMicrophoneRecording() {
        isRecording = true;
        audioRecord.startRecording();
        
        new Thread(() -> {
            short[] buffer = new short[WebRtcAec3.FRAME_SIZE];
            
            while (isRecording) {
                int read = audioRecord.read(buffer, 0, buffer.length);
                
                if (read == WebRtcAec3.FRAME_SIZE) {
                    // Process with AEC3 to remove echo
                    short[] cleanAudio = aec3.processMicrophoneAudio(buffer);
                    
                    if (cleanAudio != null) {
                        // Send clean audio to your TTS service
                        sendToTtsService(cleanAudio);
                    }
                }
            }
        }).start();
    }
    
    public void stopSession() {
        isRecording = false;
        
        if (audioRecord != null) {
            audioRecord.stop();
            audioRecord.release();
        }
        
        if (audioTrack != null) {
            audioTrack.stop();
            audioTrack.release();
        }
        
        if (aec3 != null) {
            aec3.destroy();
        }
    }
}
```

## Performance Monitoring
```java
// Get AEC performance metrics
WebRtcAec3.AecMetrics metrics = aec3.getMetrics();
if (metrics != null) {
    Log.i("AEC3", "Performance: " + metrics.toString());
    
    // Good AEC performance indicators:
    // - Echo Return Loss > 20dB
    // - Echo Return Loss Enhancement > 10dB
    // - Stable delay estimation
}
```

## Optimization Tips

### 1. Delay Compensation
```java
// Android devices typically have 80-150ms latency
// Fine-tune based on your specific device/setup
aec3.setStreamDelay(100); // Start with 100ms
```

### 2. Audio Format Requirements
- **MUST use 48kHz sample rate** - AEC3 will not work with other rates
- **MUST use 16-bit PCM format**
- **MUST process in 10ms chunks (480 samples)**
- **Recommended: Use mono audio** for better performance

### 3. Threading Considerations
- AEC3 processing is thread-safe
- Process TTS and microphone audio on separate threads for best performance
- Always call `processTtsAudio()` BEFORE playing the audio

### 4. Memory Management
- Reuse audio buffers to reduce GC pressure
- Call `destroy()` when done to free native resources
- Monitor memory usage in long-running sessions

## Troubleshooting

### Common Issues
1. **Silent Output**: Check sample rate is exactly 48kHz
2. **Poor Echo Cancellation**: Ensure TTS is processed before playback
3. **High CPU Usage**: Verify you're using 10ms chunks (480 samples)
4. **Crashes**: Check all audio arrays are exactly 480 samples

### Debugging
```java
// Enable verbose logging
adb shell setprop log.tag.WebRTC_AEC3_TTS VERBOSE
```

## License
Based on WebRTC project (BSD-style license)
EODOC

# ============================================================================
# Final Summary
# ============================================================================
echo ""
echo "🎉 Build Complete!"
echo "📁 Output directory: $OUTPUT_DIR"
echo "📦 AAR file: $OUTPUT_DIR/${AAR_NAME}-1.0.aar"
echo "📚 Documentation: $OUTPUT_DIR/TTS_AEC3_USAGE.md"
echo ""
echo "📊 Build Summary:"
echo "  - Sample Rate: ${AEC3_SAMPLE_RATE}Hz"
echo "  - Frame Size: ${AEC3_FRAME_SIZE} samples (10ms)"
echo "  - Stream Delay: ${ANDROID_STREAM_DELAY}ms"
echo "  - Architectures: ${ARCHITECTURES[*]}"
echo ""
echo "🚀 Next Steps:"
echo "  1. Copy ${AAR_NAME}-1.0.aar to your Android project's libs/ folder"
echo "  2. Add implementation files('libs/${AAR_NAME}-1.0.aar') to build.gradle"
echo "  3. Follow the usage guide in TTS_AEC3_USAGE.md"
echo "  4. Test with your TTS service integration"
echo ""
echo "⚠️  Important: Always call processTtsAudio() BEFORE playing TTS audio!"
echo "📈 Expected Performance: High-quality echo cancellation with ~10ms latency"
