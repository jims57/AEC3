#!/bin/bash

# WebRTC AEC3 Android AAR构建脚本 - 用于TTS回声消除
# 作者: Jimmy Gan | 日期: 2025-08-27
# 目的: 使用WebRTC AEC3构建生产就绪的TTS回声消除AAR库

set -e  # 任何错误时退出

# ============================================================================
# 配置
# ============================================================================
PROJECT_ROOT="/Users/mac/Documents/GitHub/AEC3"
BUILD_DIR="$PROJECT_ROOT/build_android"
OUTPUT_DIR="$PROJECT_ROOT/android_output"
AAR_NAME="wq-aec3"
JAVA_PACKAGE="cn.watchfun.aec3"

# Android NDK配置
ANDROID_NDK_HOME=${ANDROID_NDK_HOME:-"/Users/mac/Library/Android/sdk/ndk/25.2.9519653"}
ANDROID_API_LEVEL=27
ANDROID_STL="c++_static"

# AEC3配置（基于ace-key-points.txt）
AEC3_SAMPLE_RATE=48000
AEC3_FRAME_SIZE=480  # 10ms at 48kHz
ANDROID_STREAM_DELAY=100  # Android流延迟（毫秒）

echo "🚀 正在构建WebRTC AEC3 TTS Android AAR库"
echo "📁 项目目录: $PROJECT_ROOT"
echo "🔧 NDK路径: $ANDROID_NDK_HOME"
echo "📊 AEC3配置: ${AEC3_SAMPLE_RATE}Hz, ${AEC3_FRAME_SIZE}个样本, ${ANDROID_STREAM_DELAY}毫秒延迟"

# 验证NDK
if [ ! -d "$ANDROID_NDK_HOME" ]; then
    echo "❌ 在以下路径找不到Android NDK: $ANDROID_NDK_HOME"
    echo "请设置ANDROID_NDK_HOME环境变量或安装NDK"
    exit 1
fi

# ============================================================================
# 准备构建环境
# ============================================================================
echo "🧹 正在清理之前的构建..."
rm -rf "$BUILD_DIR" "$OUTPUT_DIR"
mkdir -p "$BUILD_DIR" "$OUTPUT_DIR"

# 为多种架构创建构建目录
ARCHITECTURES=("arm64-v8a" "armeabi-v7a" "x86_64" "x86")
for arch in "${ARCHITECTURES[@]}"; do
    mkdir -p "$BUILD_DIR/$arch"
done

# ============================================================================
# 为AEC3 TTS生成CMakeLists.txt
# ============================================================================
echo "📝 正在生成CMakeLists.txt..."

cat > "$BUILD_DIR/CMakeLists.txt" << 'EOCMAKE'
cmake_minimum_required(VERSION 3.18.1)
project(webrtc_aec3_tts)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Android特定设置
if(ANDROID)
    set(CMAKE_ANDROID_STL_TYPE c++_static)
    add_definitions(-DWEBRTC_ANDROID -DWEBRTC_POSIX)
endif()

# 编译标志，用于优化和WebRTC兼容性
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -ffast-math -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_APM_DEBUG_DUMP=0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_CHECK_MSG=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_INCLUDE_INTERNAL_AUDIO_DEVICE")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_EXCLUDE_FIELD_TRIAL_DEFAULT") # This line is commented out to fix the FindFullName issue
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_METRICS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_LINUX")  # Enable Linux-specific features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE")   # Enable GNU extensions for prctl

# 包含目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../api)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../audio_processing)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../audio_processing/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/rtc_base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/system_wrappers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/abseil)

# 定义WebRTC AEC3核心源文件及所需工具
set(AEC3_CORE_SOURCES
    # API Layer
    ../api/echo_canceller3_factory.cc
    ../api/echo_canceller3_config.cc
    # 注意：由于缺少json/json.h，排除了echo_canceller3_config_json.cc
    
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
    
    # 关键工具组件（缺少链接符号）
    ../audio_processing/utility/ooura_fft.cc
    ../audio_processing/utility/cascaded_biquad_filter.cc
    ../audio_processing/utility/delay_estimator.cc
    ../audio_processing/utility/delay_estimator_wrapper.cc
    
    # 重采样器组件 (PushSincResampler)
    ../audio_processing/resampler/push_sinc_resampler.cc
    ../audio_processing/resampler/sinc_resampler.cc
    
    # 日志记录组件 (ApmDataDumper)
    ../audio_processing/logging/apm_data_dumper.cc
    
    # 基础必要组件（缺少实现）
    ../base/rtc_base/memory/aligned_malloc.cc
    ../base/system_wrappers/source/cpu_features.cc
)

# 修复剩余链接错误的额外必要源文件
set(ADDITIONAL_SOURCES 
    # 必要的abseil实现（缺少链接符号）
    ../base/abseil/absl/base/internal/raw_logging.cc
    ../base/abseil/absl/strings/charconv.cc
    ../base/abseil/absl/strings/internal/charconv_parse.cc
    ../base/abseil/absl/strings/internal/charconv_bigint.cc
    ../base/abseil/absl/strings/internal/memutil.cc
    ../base/abseil/absl/strings/match.cc
    ../base/abseil/absl/strings/ascii.cc
    ../base/abseil/absl/numeric/int128.cc
    
    # 必要的rtc_base工具（缺少实现）
    ../base/rtc_base/strings/string_builder.cc
    ../base/rtc_base/string_encode.cc
    ../base/rtc_base/string_utils.cc
    ../base/rtc_base/platform_thread_types.cc
    ../base/rtc_base/checks.cc
    ../base/rtc_base/logging.cc
    ../base/rtc_base/time_utils.cc
    ../base/rtc_base/race_checker.cc
    ../base/rtc_base/critical_section.cc
    
    # 系统包装器（现场试验）
    ../base/system_wrappers/source/field_trial.cc
)

# 查找所有AEC3实现文件
file(GLOB_RECURSE AEC3_IMPL_SOURCES 
    "../audio_processing/aec3/*.cc"
    "../audio_processing/aec3/*.c"
)

# 架构特定优化
set(ARCH_SPECIFIC_SOURCES "")
if(ANDROID_ABI STREQUAL "x86" OR ANDROID_ABI STREQUAL "x86_64")
    # 为x86架构添加SSE2优化
    list(APPEND ARCH_SPECIFIC_SOURCES 
        ../audio_processing/utility/ooura_fft_sse2.cc
        ../audio_processing/resampler/sinc_resampler_sse.cc
    )
    # 为x86启用SSE2
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
elseif(ANDROID_ABI STREQUAL "armeabi-v7a" OR ANDROID_ABI STREQUAL "arm64-v8a")
    # 为ARM架构添加NEON优化
    list(APPEND ARCH_SPECIFIC_SOURCES 
        ../audio_processing/utility/ooura_fft_neon.cc
        ../audio_processing/resampler/sinc_resampler_neon.cc
    )
    # 为ARM启用NEON
    if(ANDROID_ABI STREQUAL "armeabi-v7a")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
    endif()
endif()

# 积极过滤以移除有问题的文件
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*test.*")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*_test\\.cc$")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*_unittest\\.cc$")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*_bench.*")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*benchmark.*")

# 将所有源文件与额外需要的实现合并
set(ALL_SOURCES 
    ${AEC3_CORE_SOURCES}
    ${AEC3_IMPL_SOURCES}
    ${ADDITIONAL_SOURCES}
    ${ARCH_SPECIFIC_SOURCES}
    tts_aec3_wrapper.cc
)

# 创建共享库
add_library(wq_aec3_tts SHARED ${ALL_SOURCES})

# 链接Android库
if(ANDROID)
    target_link_libraries(wq_aec3_tts
        android
        log
        OpenSLES
    )
endif()

# 设置库属性
set_target_properties(wq_aec3_tts PROPERTIES
    VERSION 1.0
    SOVERSION 1
)
EOCMAKE

# ============================================================================
# 复制C++源文件到构建目录
# ============================================================================
echo "📝 正在复制C++源文件到构建目录..."

# 从my-cpp-files/复制C++实现文件
cp "$PROJECT_ROOT/my-info/my-cpp-files/wq_aec3_processor.h" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/wq_aec3_processor.cpp" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/webrtc_compat.h" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/webrtc_compat.cpp" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/wq_aec3_convertor.h" "$BUILD_DIR/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/wq_aec3_convertor.cpp" "$BUILD_DIR/"

# 复制JNI实现文件
cp "$PROJECT_ROOT/my-info/wq_aec3_jni.cpp" "$BUILD_DIR/"

# 复制使用文档
cp "$PROJECT_ROOT/my-info/my-cpp-files/TTS_AEC3_USAGE.md" "$OUTPUT_DIR/"

echo "✅ C++源文件和文档复制成功"

# 创建包含所有组件的主包装文件
cat > "$BUILD_DIR/tts_aec3_wrapper.cc" << 'EOWRAPPER'
// TTS AEC3包装器 - 主入口点
// 该文件组合了WebRTC AEC3 TTS库的所有C++组件

// 包含主处理器实现
#include "wq_aec3_processor.cpp"

// 包含WebRTC兼容层
#include "webrtc_compat.cpp"

// 包含音频转换器实现
#include "wq_aec3_convertor.cpp"

// 包含JNI实现
#include "wq_aec3_jni.cpp"
EOWRAPPER

# ============================================================================
# 生成Java包装类
# ============================================================================
echo "📝 正在生成Java包装类..."

mkdir -p "$BUILD_DIR/java/cn/watchfun/aec3"

cat > "$BUILD_DIR/java/cn/watchfun/aec3/WqAecProcessor.java" << 'EOJAVA'
package cn.watchfun.aec3;

/**
 * 用于TTS回声消除的WebRTC AEC3包装器
 * 
 * 该类为WebRTC的声学回声消除(AEC3)提供了一个简单接口，
 * 专门为TTS(文本转语音)应用优化。
 * 
 * 使用方法：
 * 1. 初始化AEC处理器
 * 2. 对于每个TTS音频块：在播放前调用processTtsAudio()
 * 3. 对于每个麦克风音频块：调用processMicrophoneAudio()获取干净音频
 * 4. 使用getMetrics()监控性能
 * 
 * 重要提示：所有音频必须是48kHz、16位PCM、单声道、480个样本(10ms块)
 */
public class WqAecProcessor {
    static {
        System.loadLibrary("wq_aec3_tts");
    }

    // 音频配置常量
    public static final int SAMPLE_RATE = 48000;
    public static final int FRAME_SIZE = 480;  // 10ms at 48kHz
    public static final int CHANNELS = 1;      // Mono
    public static final int BITS_PER_SAMPLE = 16;

    /**
     * 初始化AEC处理器
     * @return 如果成功则返回true
     */
    public native boolean nativeInitialize();

    /**
     * 清理资源
     */
    public native void nativeDestroy();

    /**
     * 处理TTS音频（参考信号）
     * 在通过扬声器播放TTS音频之前调用此方法
     * 
     * @param ttsData TTS音频数据（480个样本，16位PCM）
     * @return 处理成功返回true
     */
    public native boolean nativeProcessTtsAudio(short[] ttsData);

    /**
     * 处理麦克风音频并移除回声
     * 
     * @param micData 麦克风输入数据（480个样本，16位PCM）
     * @param outputData 处理后音频的输出缓冲区（480个样本，16位PCM）
     * @return 处理成功返回true
     */
    public native boolean nativeProcessMicrophoneAudio(short[] micData, short[] outputData);

    /**
     * 获取当前AEC性能指标
     * 
     * @return 返回[ERL, ERLE, delay_ms]数组，如果不可用则返回null
     */
    public native double[] nativeGetMetrics();

    /**
     * 更新流延迟补偿
     * 
     * @param delayMs 延迟毫秒数（Android通常为80-150ms）
     */
    public native void nativeSetStreamDelay(int delayMs);
    
    // 官方AEC3参数控制
    // 这些原生方法直接对应官方WebRTC AEC3配置参数
    
    // 滤波器配置原生方法
    public native void nativeSetConfigChangeDuration(int blocks);          // 0-1000 range, 0=default
    public native void nativeSetInitialStateSeconds(float seconds);        // 0.0-3.0 range, 0=default  
    public native void nativeSetConservativeInitialPhase(boolean enable);  // true/false
    
    // 抑制器常规调校原生方法
    public native void nativeSetMaxDecFactorLF(float factor);             // 0.0-100.0 range, 0=default
    public native void nativeSetMaxIncFactor(float factor);               // 0.0-100.0 range, 0=default
    
    // 近端抑制器调校原生方法
    public native void nativeSetNearendMaxDecFactorLF(float factor);      // 0.0-100.0 range, 0=default
    public native void nativeSetNearendMaxIncFactor(float factor);        // 0.0-100.0 range, 0=default
    
    // 主导近端检测原生方法
    public native void nativeSetEnrThreshold(float threshold);            // 0.0-1000.0 range, 0=default
    public native void nativeSetSnrThreshold(float threshold);            // 0.0-1000.0 range, 0=default
    public native void nativeSetHoldDuration(int duration);               // 0-10000 range, 0=default
    public native void nativeSetTriggerThreshold(int threshold);          // 0-10000 range, 0=default
    
    // 增强的ERLE优化方法
    public native boolean nativeAutoOptimizeDelay();               // Automatic delay optimization
    public native double[] nativeGetEnhancedMetrics();             // [ERL, ERLE, delay, render_frames, capture_frames, optimal_delay]
    public native boolean nativeEnableTimingSync(boolean enable);   // Enable/disable precise timing sync
    
    // 面向移动开发者的ERLE调整参数原生方法
    public native void nativeSetFilterLengthBlocks(int blocks);           // Filter length blocks (1-100)
    public native void nativeSetFilterLeakageConverged(float leakage);    // Filter leakage converged (0.000001-1.0)
    public native void nativeSetFilterLeakageDiverged(float leakage);     // Filter leakage diverged (0.001-1.0)
    public native void nativeSetDelayDownSamplingFactor(int factor);      // Delay down sampling factor (1-8)
    public native void nativeSetDelayNumFilters(int filters);             // Delay number of filters (1-32)
    public native void nativeSetDelayEstimateSmoothing(float smoothing);  // Delay estimate smoothing (0.1-0.99)
    
    // 干净音频转换原生方法
    public native byte[] nativeGetCleanAudioAsWAV(int outputSampleRate);  // Get buffered clean audio as WAV
    public native byte[] nativeGetCleanAudioAsPCM(int outputSampleRate);  // Get buffered clean audio as PCM
    public native void nativeClearCleanAudioBuffer();                     // Clear clean audio buffer

    // 高级Java API
    private boolean initialized = false;

    /**
     * 初始化AEC处理器
     * @return true if successful
     */
    public boolean initialize() {
        if (!initialized) {
            initialized = nativeInitialize();
        }
        return initialized;
    }

    /**
     * 清理资源
     */
    public void destroy() {
        if (initialized) {
            nativeDestroy();
            initialized = false;
        }
    }

    /**
     * 处理TTS音频块
     * @param ttsData 音频数据（必须是480个样本）
     * @return 处理成功返回true
     */
    public boolean processTtsAudio(short[] ttsData) {
        if (!initialized || ttsData.length != FRAME_SIZE) {
            return false;
        }
        return nativeProcessTtsAudio(ttsData);
    }

    /**
     * 处理麦克风音频并获取回声消除后的音频
     * @param micData 麦克风输入（必须是480个样本）
     * @return 回声消除后的音频，如果出错则返回null
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
     * 获取AEC性能指标
     * 
     * @return AecMetrics对象，包含性能数据
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
     * 调整流延迟以获得最佳性能
     * @param delayMs 延迟毫秒数
     */
    public void setStreamDelay(int delayMs) {
        if (initialized) {
            nativeSetStreamDelay(delayMs);
        }
    }
    
    // 官方AEC3参数控制方法
    // 这些方法直接控制WebRTC AEC3的配置参数
    // 使用0值应用AEC3默认值，或设置特定值进行自定义调优
    
    // ======= 滤波器配置方法 =======
    
    /**
     * 设置AEC3配置更改持续时间（以块为单位）
     * 控制AEC3在不同配置之间转换的平滑度
     * @param blocks 0-1000范围，0=使用AEC3默认值，典型值：50-250块
     */
    public void setConfigChangeDuration(int blocks) {
        if (initialized) {
            nativeSetConfigChangeDuration(blocks);
        }
    }
    
    /**
     * 设置AEC3初始状态持续时间（秒）
     * AEC3在完全运行前花费在初始学习阶段的时间
     * @param seconds 0.0-3.0范围，0=使用AEC3默认值，典型值：0.5-2.5秒
     */
    public void setInitialStateSeconds(float seconds) {
        if (initialized) {
            nativeSetInitialStateSeconds(seconds);
        }
    }
    
    /**
     * 启用/禁用保守初始阶段
     * 保守模式 = 初始收敛较慢但更稳定
     * @param enable true=保守（更安全），false=激进（收敛更快）
     */
    public void setConservativeInitialPhase(boolean enable) {
        if (initialized) {
            nativeSetConservativeInitialPhase(enable);
        }
    }
    
    // ======= 抑制器常规调优方法 =======
    
    /**
     * 设置低频最大衰减因子（回声抑制强度）
     * 值越高 = 回声抑制越强，但可能影响语音质量
     * @param factor 0.0-100.0范围，0=使用AEC3默认值，典型值：2.0-25.0
     */
    public void setMaxDecFactorLF(float factor) {
        if (initialized) {
            nativeSetMaxDecFactorLF(factor);
        }
    }
    
    /**
     * 设置最大增加因子（语音恢复速度）
     * 值越高 = 回声抑制后语音恢复越快
     * @param factor 0.0-100.0范围，0=使用AEC3默认值，典型值：1.5-5.0
     */
    public void setMaxIncFactor(float factor) {
        if (initialized) {
            nativeSetMaxIncFactor(factor);
        }
    }
    
    // ======= 抑制器近端调优方法 =======
    
    /**
     * 设置近端低频最大衰减因子（语音保护）
     * 值越低 = 用户说话时语音保留越好
     * @param factor 0.0-100.0范围，0=使用AEC3默认值，典型值：1.0-8.0
     */
    public void setNearendMaxDecFactorLF(float factor) {
        if (initialized) {
            nativeSetNearendMaxDecFactorLF(factor);
        }
    }
    
    /**
     * 设置近端最大增加因子（近端语音恢复）
     * 值越高 = 用户说话时语音越清晰
     * @param factor 0.0-100.0范围，0=使用AEC3默认值，典型值：2.0-8.0
     */
    public void setNearendMaxIncFactor(float factor) {
        if (initialized) {
            nativeSetNearendMaxIncFactor(factor);
        }
    }
    
    // ======= 主导近端检测方法 =======
    
    /**
     * 设置语音检测的能量噪声比(ENR)阈值
     * 值越低 = 语音检测越敏感 = 语音保留越好
     * @param threshold 0.0-1000.0范围，0=使用AEC3默认值，典型值：0.1-1.0
     */
    public void setEnrThreshold(float threshold) {
        if (initialized) {
            nativeSetEnrThreshold(threshold);
        }
    }
    
    /**
     * 设置语音检测的信噪比(SNR)阈值
     * 值越低 = 语音检测越敏感 = 语音保留越好
     * @param threshold 0.0-1000.0范围，0=使用AEC3默认值，典型值：10.0-30.0
     */
    public void setSnrThreshold(float threshold) {
        if (initialized) {
            nativeSetSnrThreshold(threshold);
        }
    }
    
    /**
     * 设置主导近端状态的保持时间（以块为单位）
     * 检测到主导近端后保持该状态的时长
     * @param duration 0-10000范围，0=使用AEC3默认值，典型值：100-500块
     */
    public void setHoldDuration(int duration) {
        if (initialized) {
            nativeSetHoldDuration(duration);
        }
    }
    
    /**
     * 设置主导近端检测的触发阈值
     * 值越低 = 越容易检测到主导近端
     * @param threshold 0-10000范围，0=使用AEC3默认值，典型值：10-100
     */
    public void setTriggerThreshold(int threshold) {
        if (initialized) {
            nativeSetTriggerThreshold(threshold);
        }
    }
    
    // ======= 增强型ERLE优化方法 =======
    
    /**
     * 自动优化延迟以获得最佳ERLE性能
     * 将分析当前音频流并调整延迟
     * @return 如果优化成功返回true
     */
    public boolean autoOptimizeDelay() {
        if (!initialized) return false;
        return nativeAutoOptimizeDelay();
    }
    
    /**
     * 获取包括时序信息在内的增强型指标
     * @return 返回包含[ERL, ERLE, current_delay_ms, render_frames, capture_frames, optimal_delay_ms]的数组
     *         如果不可用则返回null
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
     * 启用或禁用精确时间同步
     * 启用后，使用系统时钟同步TTS和麦克风流
     * @param enable 是否启用精确时间同步
     * @return 如果操作成功返回true
     */
    public boolean enableTimingSync(boolean enable) {
        if (!initialized) return false;
        return nativeEnableTimingSync(enable);
    }
    
    // ======= ERLE调整参数 =======
    // Based on adjust-ERLE-result.md - fine-tune ERLE performance and convergence speed
    
    /**
     * 设置滤波器长度（以块为单位）
     * 值越高 = 回声学习越好，但收敛速度越慢
     * @param blocks 1-100范围，0=使用AEC3默认值，典型值：25块
     */
    public void setFilterLengthBlocks(int blocks) {
        if (initialized) {
            nativeSetFilterLengthBlocks(blocks);
        }
    }
    
    /**
     * 设置收敛时的滤波器泄漏值以保持稳定性
     * 值越低 = 越稳定，但对回声路径变化的适应越慢
     * @param leakage 0.000001-1.0范围，默认值=0.0005
     */
    public void setFilterLeakageConverged(float leakage) {
        if (initialized) {
            nativeSetFilterLeakageConverged(leakage);
        }
    }
    
    /**
     * 设置发散时的滤波器泄漏值以实现更快的重新收敛
     * 值越高 = 回声路径变化时恢复越快
     * @param leakage 0.001-1.0范围，默认值=0.1
     */
    public void setFilterLeakageDiverged(float leakage) {
        if (initialized) {
            nativeSetFilterLeakageDiverged(leakage);
        }
    }
    
    /**
     * 设置延迟估计的下采样因子
     * 值越高 = CPU使用率越低，但延迟估计越粗糙
     * @param factor 1-8范围，默认值=4
     */
    public void setDelayDownSamplingFactor(int factor) {
        if (initialized) {
            nativeSetDelayDownSamplingFactor(factor);
        }
    }
    
    /**
     * 设置用于延迟估计的滤波器数量
     * 滤波器越多 = 延迟估计越好，但CPU使用率越高
     * @param filters 1-32范围，默认值=12
     */
    public void setDelayNumFilters(int filters) {
        if (initialized) {
            nativeSetDelayNumFilters(filters);
        }
    }
    
    /**
     * 设置延迟估计的平滑因子以保持稳定性
     * 值越高 = 延迟估计越稳定
     * @param smoothing 0.1-0.99范围，默认值=0.98
     */
    public void setDelayEstimateSmoothing(float smoothing) {
        if (initialized) {
            nativeSetDelayEstimateSmoothing(smoothing);
        }
    }
    
    // ======= 干净音频转换方法 =======
    
    /**
     * 获取缓冲的干净音频为WAV文件
     * @param outputSampleRate 目标采样率（例如：16000、24000、44100、48000）
     * @return WAV文件的字节数组，如果出错则返回null
     */
    public byte[] getCleanAudioAsWAV(int outputSampleRate) {
        if (!initialized) return null;
        return nativeGetCleanAudioAsWAV(outputSampleRate);
    }
    
    /**
     * 获取缓冲的干净音频为WAV格式（默认采样率）
     * @return WAV文件的字节数组，如果没有可用音频则返回null
     */
    public byte[] getCleanAudioAsWAV() {
        return getCleanAudioAsWAV(44100);
    }
    public byte[] getCleanAudioAsPCM(int outputSampleRate) {
        if (!initialized) return null;
        return nativeGetCleanAudioAsPCM(outputSampleRate);
    }
    
    /**
     * 获取缓冲的干净音频为PCM格式（默认采样率）
     * @return PCM音频数据的字节数组（16位小端序），如果没有可用音频则返回null
     */
    public byte[] getCleanAudioAsPCM() {
        return getCleanAudioAsPCM(44100);
    }
    
    /**
     * 清除已累积的干净音频缓冲区而不获取数据
     * 在开始新的录音会话时使用此方法丢弃累积的音频
     */
    public void clearCleanAudioBuffer() {
        if (initialized) {
            nativeClearCleanAudioBuffer();
        }
    }
    


    /**
     * 用于保存AEC性能指标的类
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
     * 包含详细信息的增强型AEC性能指标
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
         * 获取ERLE质量评估
         * @return 质量等级: "优秀" (>15dB), "良好" (>10dB), "一般" (>5dB), "差" (<5dB)
         */
        public String getErleQuality() {
            if (echoReturnLossEnhancement >= 15.0) return "Excellent";
            else if (echoReturnLossEnhancement >= 10.0) return "Good";
            else if (echoReturnLossEnhancement >= 5.0) return "Fair";
            else return "Poor";
        }
        
        /**
         * 检查帧是否同步（渲染和捕获帧数相等）
         * @return 如果帧同步良好则返回true
         */
        public boolean isFrameSynchronized() {
            if (renderFrames == 0 || captureFrames == 0) return false;
            double ratio = (double) Math.min(renderFrames, captureFrames) / Math.max(renderFrames, captureFrames);
            return ratio > 0.95; // 在5%以内视为同步
        }
    }
}
EOJAVA

# ============================================================================
# 生成兼容性的Android.mk文件
# ============================================================================
cat > "$BUILD_DIR/Android.mk" << 'EOANDROIDMK'
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := wq_aec3_tts
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
        echo "❌ $arch 架构构建失败"
        continue
    }

    # Copy built library
    mkdir -p "$OUTPUT_DIR/jni/$arch"
    cp libwq_aec3_tts.so "$OUTPUT_DIR/jni/$arch/"
    
    echo "✅ 为 $arch 架构构建成功"
done

cd "$PROJECT_ROOT"

# ============================================================================
# 创建Android AAR包
# ============================================================================
echo "📦 Creating Android AAR package..."

# 创建AAR目录结构
AAR_DIR="$OUTPUT_DIR/aar"
mkdir -p "$AAR_DIR"/{classes,jni,res,assets}

# 复制原生库
cp -r "$OUTPUT_DIR/jni" "$AAR_DIR/"

# 编译Java类
javac -d "$AAR_DIR/classes" -cp "$ANDROID_SDK_ROOT/platforms/android-$ANDROID_API_LEVEL/android.jar" \
    "$BUILD_DIR/java/cn/watchfun/aec3/WqAecProcessor.java"

# 创建classes.jar
cd "$AAR_DIR/classes"
jar cf ../classes.jar .
cd "$PROJECT_ROOT"

# 创建AndroidManifest.xml
cat > "$AAR_DIR/AndroidManifest.xml" << EOMANIFEST
<?xml version="1.0" encoding="utf-8"?>
<manifest xmlns:android="http://schemas.android.com/apk/res/android"
    package="cn.watchfun.aec3"
    android:versionCode="1"
    android:versionName="1.0">
    
    <uses-sdk 
        android:minSdkVersion="$ANDROID_API_LEVEL"
        android:targetSdkVersion="34" />
    
    <uses-permission android:name="android.permission.RECORD_AUDIO" />
    <uses-permission android:name="android.permission.MODIFY_AUDIO_SETTINGS" />
    
</manifest>
EOMANIFEST

# 创建R.txt（本库为空）
touch "$AAR_DIR/R.txt"

# 打包AAR
cd "$AAR_DIR"
zip -r "../${AAR_NAME}-1.0.aar" ./*
cd "$PROJECT_ROOT"

# ============================================================================
# 生成使用文档
# ============================================================================
echo "📚 Usage documentation already copied from my-cpp-files/TTS_AEC3_USAGE.md"

# 最终摘要
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
# 最终摘要
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

