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

# Compiler flags for optimization and WebRTC compatibility
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -ffast-math -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_APM_DEBUG_DUMP=0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_CHECK_MSG=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_INCLUDE_INTERNAL_AUDIO_DEVICE")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_EXCLUDE_FIELD_TRIAL_DEFAULT") # This line is commented out to fix the FindFullName issue
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_METRICS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_LINUX")  # Enable Linux-specific features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE")   # Enable GNU extensions for prctl

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

# Combine all sources with additional required implementations
set(ALL_SOURCES 
    ${AEC3_CORE_SOURCES}
    ${AEC3_IMPL_SOURCES}
    ${ADDITIONAL_SOURCES}
    ${ARCH_SPECIFIC_SOURCES}
    tts_aec3_wrapper.cc
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
# Copy C++ Source Files to Build Directory
# ============================================================================
echo "📝 Copying C++ source files to build directory..."

# Copy C++ implementation files from my-cpp-files/
cp "$PROJECT_ROOT/my-info/my-cpp-files/wq_aec3_processor.h" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/wq_aec3_processor.cpp" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/webrtc_compat.h" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/webrtc_compat.cpp" "$BUILD_DIR/"

# Copy JNI implementation file
cp "$PROJECT_ROOT/my-info/wq_aec3_jni.cpp" "$BUILD_DIR/"

# Copy usage documentation
cp "$PROJECT_ROOT/my-info/my-cpp-files/TTS_AEC3_USAGE.md" "$OUTPUT_DIR/"

echo "✅ C++ source files and documentation copied successfully"

# Create main wrapper file that includes all components
cat > "$BUILD_DIR/tts_aec3_wrapper.cc" << 'EOWRAPPER'
// TTS AEC3 Wrapper - Main entry point (2025-01-31)
// This file combines all C++ components for the WebRTC AEC3 TTS library

// Include the main processor implementation
#include "wq_aec3_processor.cpp"

// Include WebRTC compatibility layer
#include "webrtc_compat.cpp"

// Include JNI implementation  
#include "wq_aec3_jni.cpp"
EOWRAPPER

# ============================================================================
# Generate Java Wrapper Classes
# ============================================================================
echo "📝 Generating Java wrapper classes..."

mkdir -p "$BUILD_DIR/java/com/tts/aec3"

cat > "$BUILD_DIR/java/com/tts/aec3/WebRtcAec3.java" << 'EOJAVA'
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
echo "📚 Usage documentation already copied from my-cpp-files/TTS_AEC3_USAGE.md"

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
echo "📈 Expected Performance: Enhanced ERLE (>15dB target vs previous 6.2dB) with precise timing synchronization"
echo "🎯 ERLE Optimization Features: Auto delay optimization, enhanced timing sync, demo.cc pipeline compliance"

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
echo "📈 Expected Performance: Enhanced ERLE (>15dB target vs previous 6.2dB) with precise timing synchronization"
echo "🎯 ERLE Optimization Features: Auto delay optimization, enhanced timing sync, demo.cc pipeline compliance"

