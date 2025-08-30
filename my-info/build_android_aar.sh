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

# Oboe集成配置
OBOE_AAR_PATH="/Users/mac/Documents/GitHub/oboe/my-info/aar/oboe-1.9.3.aar"
OBOE_EXTRACT_DIR="$BUILD_DIR/oboe_extracted"

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
# 提取并集成Oboe AAR
# ============================================================================
echo "📦 正在提取Oboe AAR并集成到构建中..."
mkdir -p "$OBOE_EXTRACT_DIR"
cd "$OBOE_EXTRACT_DIR"
unzip -q "$OBOE_AAR_PATH"

# 复制Oboe头文件到构建目录
mkdir -p "$BUILD_DIR/oboe"
cp -r "$OBOE_EXTRACT_DIR/prefab/modules/oboe/include/oboe" "$BUILD_DIR/oboe/"

# 为每个架构复制Oboe库文件
for arch in "${ARCHITECTURES[@]}"; do
    case $arch in
        "arm64-v8a")
            OBOE_ARCH_DIR="android.arm64-v8a"
            ;;
        "armeabi-v7a")
            OBOE_ARCH_DIR="android.armeabi-v7a"
            ;;
        "x86_64")
            OBOE_ARCH_DIR="android.x86_64"
            ;;
        "x86")
            OBOE_ARCH_DIR="android.x86"
            ;;
    esac
    
    mkdir -p "$BUILD_DIR/$arch/oboe_libs"
    cp "$OBOE_EXTRACT_DIR/prefab/modules/oboe/libs/$OBOE_ARCH_DIR/liboboe.so" "$BUILD_DIR/$arch/oboe_libs/"
done

cd "$PROJECT_ROOT"
echo "✅ Oboe AAR集成完成"

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
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/oboe)

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

# 添加Oboe预构建库
add_library(oboe SHARED IMPORTED)
set_target_properties(oboe PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/${ANDROID_ABI}/oboe_libs/liboboe.so
)

# 链接Android库
if(ANDROID)
    target_link_libraries(wq_aec3_tts
        android
        log
        OpenSLES
        oboe
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
# 复制Java包装类
# ============================================================================
echo "📝 正在复制Java包装类..."

# 创建目标目录
mkdir -p "$BUILD_DIR/java/cn/watchfun/aec3"

# 从源目录复制WqAecProcessor.java到构建目录
cp "$PROJECT_ROOT/my-info/my-cpp-files/WqAecProcessor.java" "$BUILD_DIR/java/cn/watchfun/aec3/"

# 检查文件是否成功复制
if [ $? -ne 0 ]; then
    echo "❌ 复制WqAecProcessor.java失败"
    exit 1
fi

echo "✅ Java包装类复制成功"

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

# 复制Oboe库到AAR
for arch in "${ARCHITECTURES[@]}"; do
    # 复制oboe库文件到目标目录
    OBOE_LIB_PATH="$BUILD_DIR/$arch/oboe_libs/liboboe.so"
    if [ -f "$OBOE_LIB_PATH" ]; then
        cp "$OBOE_LIB_PATH" "$AAR_DIR/jni/$arch/"
        echo "✅ 复制oboe库到AAR: $arch/liboboe.so"
    else
        echo "⚠️  未找到oboe库: $OBOE_LIB_PATH"
    fi
    
    # 复制libc++_shared.so (oboe依赖)
    NDK_LIBCXX_DIR="$ANDROID_NDK/toolchains/llvm/prebuilt/darwin-x86_64/sysroot/usr/lib"
    if [ "$arch" = "arm64-v8a" ]; then
        LIBCXX_PATH="$NDK_LIBCXX_DIR/aarch64-linux-android/libc++_shared.so"
    elif [ "$arch" = "armeabi-v7a" ]; then
        LIBCXX_PATH="$NDK_LIBCXX_DIR/arm-linux-androideabi/libc++_shared.so"
    elif [ "$arch" = "x86_64" ]; then
        LIBCXX_PATH="$NDK_LIBCXX_DIR/x86_64-linux-android/libc++_shared.so"
    elif [ "$arch" = "x86" ]; then
        LIBCXX_PATH="$NDK_LIBCXX_DIR/i686-linux-android/libc++_shared.so"
    fi
    
    if [ -f "$LIBCXX_PATH" ]; then
        cp "$LIBCXX_PATH" "$AAR_DIR/jni/$arch/"
        echo "✅ 复制libc++_shared.so到AAR: $arch"
    else
        echo "⚠️  未找到libc++_shared.so: $LIBCXX_PATH"
    fi
done

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
# 确保输出目录存在
mkdir -p "$OUTPUT_DIR"
# 直接创建AAR文件到输出目录
zip -r "${OUTPUT_DIR}/${AAR_NAME}-1.0.aar" ./*
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

