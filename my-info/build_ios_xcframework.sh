#!/bin/bash

# WebRTC AEC3 iOS XCFramework构建脚本 - 用于TTS回声消除
# 作者: Jimmy Gan | 日期: 2025-09-03
# 目的: 使用WebRTC AEC3构建生产就绪的TTS回声消除iOS静态库XCFramework

set -e  # 任何错误时退出

# ============================================================================
# 配置
# ============================================================================
PROJECT_ROOT="/Users/mac/Documents/GitHub/AEC3"
BUILD_DIR="$PROJECT_ROOT/build_ios"
OUTPUT_DIR="$PROJECT_ROOT/ios_output"
FRAMEWORK_NAME="WqAec3"
LIBRARY_NAME="libwq_aec3_tts"

# iOS SDK配置
IOS_DEPLOYMENT_TARGET="11.0"
ARCHITECTURES=("arm64")  # 仅支持真机，不包含模拟器

# AEC3配置（基于ace-key-points.txt）
AEC3_SAMPLE_RATE=48000
AEC3_FRAME_SIZE=480  # 10ms at 48kHz
IOS_STREAM_DELAY=100  # iOS流延迟（毫秒）

echo "🚀 正在构建WebRTC AEC3 TTS iOS XCFramework静态库"
echo "📁 项目目录: $PROJECT_ROOT"
echo "🔧 iOS部署目标: $IOS_DEPLOYMENT_TARGET"
echo "📊 AEC3配置: ${AEC3_SAMPLE_RATE}Hz, ${AEC3_FRAME_SIZE}个样本, ${IOS_STREAM_DELAY}毫秒延迟"

# 验证Xcode
if ! command -v xcodebuild &> /dev/null; then
    echo "❌ 找不到xcodebuild，请安装Xcode"
    exit 1
fi

# ============================================================================
# 准备构建环境
# ============================================================================
echo "🧹 正在清理之前的构建..."
rm -rf "$BUILD_DIR" "$OUTPUT_DIR"
mkdir -p "$BUILD_DIR" "$OUTPUT_DIR"

# 为架构创建构建目录
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

# iOS特定设置
if(IOS)
    set(CMAKE_OSX_DEPLOYMENT_TARGET "11.0")
    add_definitions(-DWEBRTC_IOS -DWEBRTC_POSIX -DWEBRTC_MAC)
endif()

# 编译标志，用于优化和WebRTC兼容性
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -ffast-math -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_APM_DEBUG_DUMP=0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_CHECK_MSG=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_INCLUDE_INTERNAL_AUDIO_DEVICE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_METRICS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_LINUX")  # Enable Linux-specific features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE")   # Enable GNU extensions

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
    
    # 关键工具组件
    ../audio_processing/utility/ooura_fft.cc
    ../audio_processing/utility/cascaded_biquad_filter.cc
    ../audio_processing/utility/delay_estimator.cc
    ../audio_processing/utility/delay_estimator_wrapper.cc
    
    # 重采样器组件
    ../audio_processing/resampler/push_sinc_resampler.cc
    ../audio_processing/resampler/sinc_resampler.cc
    
    # 日志记录组件
    ../audio_processing/logging/apm_data_dumper.cc
    
    # 基础必要组件
    ../base/rtc_base/memory/aligned_malloc.cc
    ../base/system_wrappers/source/cpu_features.cc
)

# 修复剩余链接错误的额外必要源文件
set(ADDITIONAL_SOURCES 
    # 必要的abseil实现
    ../base/abseil/absl/base/internal/raw_logging.cc
    ../base/abseil/absl/strings/charconv.cc
    ../base/abseil/absl/strings/internal/charconv_parse.cc
    ../base/abseil/absl/strings/internal/charconv_bigint.cc
    ../base/abseil/absl/strings/internal/memutil.cc
    ../base/abseil/absl/strings/match.cc
    ../base/abseil/absl/strings/ascii.cc
    ../base/abseil/absl/numeric/int128.cc
    
    # 必要的rtc_base工具
    ../base/rtc_base/strings/string_builder.cc
    ../base/rtc_base/string_encode.cc
    ../base/rtc_base/string_utils.cc
    ../base/rtc_base/platform_thread_types.cc
    ../base/rtc_base/checks.cc
    ../base/rtc_base/logging.cc
    ../base/rtc_base/time_utils.cc
    ../base/rtc_base/race_checker.cc
    ../base/rtc_base/critical_section.cc
    
    # 系统包装器
    ../base/system_wrappers/source/field_trial.cc
)

# 查找所有AEC3实现文件
file(GLOB_RECURSE AEC3_IMPL_SOURCES 
    "../audio_processing/aec3/*.cc"
    "../audio_processing/aec3/*.c"
)

# ARM64架构优化
set(ARCH_SPECIFIC_SOURCES "")
if(CMAKE_OSX_ARCHITECTURES STREQUAL "arm64")
    # 为ARM64添加NEON优化
    list(APPEND ARCH_SPECIFIC_SOURCES 
        ../audio_processing/utility/ooura_fft_neon.cc
        ../audio_processing/resampler/sinc_resampler_neon.cc
    )
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
    ios_aec3_wrapper.cc
)

# 创建静态库
add_library(wq_aec3_tts STATIC ${ALL_SOURCES})

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

# 复制使用文档
cp "$PROJECT_ROOT/my-info/my-cpp-files/TTS_AEC3_USAGE.md" "$OUTPUT_DIR/"

echo "✅ C++源文件和文档复制成功"

# 创建iOS兼容的源文件
echo "📝 正在创建iOS兼容的源文件..."

# 创建iOS兼容的处理器实现
cat > "$BUILD_DIR/wq_aec3_processor_ios.cpp" << 'EOCPP'
#include "wq_aec3_processor.h"
#include "webrtc_compat.h"
#include <iostream>

// iOS兼容的日志宏
#define LOG_TAG "WebRTC_AEC3_TTS"
#define LOGI(...) printf("[INFO] " __VA_ARGS__); printf("\n")
#define LOGE(...) printf("[ERROR] " __VA_ARGS__); printf("\n")
#define LOGD(...) printf("[DEBUG] " __VA_ARGS__); printf("\n")
#define LOGV(...) printf("[VERBOSE] " __VA_ARGS__); printf("\n")
#define LOGW(...) printf("[WARN] " __VA_ARGS__); printf("\n")
EOCPP

# 将原始处理器代码附加到iOS版本（跳过Android特定的头文件）
tail -n +12 "$PROJECT_ROOT/my-info/my-cpp-files/wq_aec3_processor.cpp" >> "$BUILD_DIR/wq_aec3_processor_ios.cpp"

# 创建包含所有组件的主包装文件（不包含JNI）
cat > "$BUILD_DIR/ios_aec3_wrapper.cc" << 'EOWRAPPER'
// iOS AEC3包装器 - 主入口点
// 该文件组合了WebRTC AEC3 TTS库的所有C++组件（iOS版本）

// 包含主处理器实现（iOS兼容版本）
#include "wq_aec3_processor_ios.cpp"

// 包含WebRTC兼容层
#include "webrtc_compat.cpp"

// 包含音频转换器实现
#include "wq_aec3_convertor.cpp"

// 注意：iOS版本不包含JNI实现
EOWRAPPER

# ============================================================================
# 收集所有源文件
# ============================================================================
echo "📝 正在收集源文件..."

# 创建iOS兼容的平台适配文件
echo "📝 正在创建iOS平台适配文件..."

# 创建sys/prctl.h的iOS兼容实现
mkdir -p "$BUILD_DIR/sys"
cat > "$BUILD_DIR/sys/prctl.h" << 'EOPRCTL'
#ifndef SYS_PRCTL_H_IOS_COMPAT
#define SYS_PRCTL_H_IOS_COMPAT

// iOS兼容的prctl.h实现
#define PR_SET_NAME 15

#ifdef __cplusplus
extern "C" {
#endif

// iOS上的prctl实现，支持多种参数类型
static inline int prctl(int option, ...) {
    // iOS上的空实现，仅用于编译兼容性
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif // SYS_PRCTL_H_IOS_COMPAT
EOPRCTL

# 创建iOS兼容的系统头文件适配
cat > "$BUILD_DIR/ios_platform_compat.h" << 'EOCOMPAT'
#ifndef IOS_PLATFORM_COMPAT_H
#define IOS_PLATFORM_COMPAT_H

// iOS平台兼容性适配
#ifdef __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_IOS

// 为iOS提供缺失的Linux特定定义
#include <stddef.h>
#include <stdint.h>

// 为splitting_filter_c.c提供size_t定义
#ifndef _SIZE_T
#define _SIZE_T
typedef unsigned long size_t;
#endif

#endif // TARGET_OS_IOS
#endif // __APPLE__

#endif // IOS_PLATFORM_COMPAT_H
EOCOMPAT

# 收集完整的WebRTC AEC3源文件集（与Android AAR相同）
SOURCES=""

# 添加主要源文件
SOURCES="$SOURCES $BUILD_DIR/ios_aec3_wrapper.cc"

# 添加API层源文件
SOURCES="$SOURCES $PROJECT_ROOT/api/echo_canceller3_factory.cc"
SOURCES="$SOURCES $PROJECT_ROOT/api/echo_canceller3_config.cc"

# 添加音频处理核心文件（包含所有必需文件）
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/audio_buffer.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/audio_frame.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/channel_buffer.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/channel_layout.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/high_pass_filter.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/three_band_filter_bank.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/splitting_filter.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/splitting_filter_c.c"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/sparse_fir_filter.cc"

# 添加工具组件
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/utility/ooura_fft.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/utility/cascaded_biquad_filter.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/utility/delay_estimator.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/utility/delay_estimator_wrapper.cc"

# 添加重采样器组件
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/resampler/push_sinc_resampler.cc"
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/resampler/sinc_resampler.cc"

# 添加日志组件
SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/logging/apm_data_dumper.cc"

# 添加基础组件
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/memory/aligned_malloc.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/system_wrappers/source/cpu_features.cc"

# 添加abseil实现
SOURCES="$SOURCES $PROJECT_ROOT/base/abseil/absl/base/internal/raw_logging.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/abseil/absl/strings/charconv.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/abseil/absl/strings/internal/charconv_parse.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/abseil/absl/strings/internal/charconv_bigint.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/abseil/absl/strings/internal/memutil.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/abseil/absl/strings/match.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/abseil/absl/strings/ascii.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/abseil/absl/numeric/int128.cc"

# 添加rtc_base工具
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/strings/string_builder.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/string_encode.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/string_utils.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/platform_thread_types.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/checks.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/logging.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/time_utils.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/race_checker.cc"
SOURCES="$SOURCES $PROJECT_ROOT/base/rtc_base/critical_section.cc"

# 添加系统包装器
SOURCES="$SOURCES $PROJECT_ROOT/base/system_wrappers/source/field_trial.cc"

# 添加所有AEC3实现文件（与Android AAR相同）
for file in $(find "$PROJECT_ROOT/audio_processing/aec3" -name "*.cc" -o -name "*.c" | grep -v test | grep -v bench); do
    if [ -f "$file" ]; then
        SOURCES="$SOURCES $file"
    fi
done

# 添加ARM64 NEON优化（如果存在）
if [ -f "$PROJECT_ROOT/audio_processing/utility/ooura_fft_neon.cc" ]; then
    SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/utility/ooura_fft_neon.cc"
fi
if [ -f "$PROJECT_ROOT/audio_processing/resampler/sinc_resampler_neon.cc" ]; then
    SOURCES="$SOURCES $PROJECT_ROOT/audio_processing/resampler/sinc_resampler_neon.cc"
fi

# ============================================================================
# 为多种架构构建
# ============================================================================
echo "🔨 正在为多种架构构建..."

for arch in "${ARCHITECTURES[@]}"; do
    echo "正在为 $arch 构建..."
    
    # 获取SDK路径
    SDKPATH=$(xcrun -sdk iphoneos --show-sdk-path)
    
    # 设置编译器标志
    CFLAGS="-arch $arch -isysroot $SDKPATH -miphoneos-version-min=$IOS_DEPLOYMENT_TARGET"
    CFLAGS="$CFLAGS -Wno-deprecated-builtins -Wno-error -Wno-nan-infinity-disabled"
    CFLAGS="$CFLAGS -Wno-macro-redefined -fno-fast-math"
    CXXFLAGS="$CFLAGS -std=c++17 -fno-rtti -O3"
    CXXFLAGS="$CXXFLAGS -DWEBRTC_IOS -DWEBRTC_POSIX -DWEBRTC_MAC"
    CXXFLAGS="$CXXFLAGS -DWEBRTC_APM_DEBUG_DUMP=0 -DRTC_DISABLE_CHECK_MSG=1"
    CXXFLAGS="$CXXFLAGS -DWEBRTC_INCLUDE_INTERNAL_AUDIO_DEVICE -DRTC_DISABLE_METRICS"
    CXXFLAGS="$CXXFLAGS -DWEBRTC_LINUX -D_GNU_SOURCE"
    
    # 设置包含目录（包含iOS平台适配）
    INCLUDES="-I$BUILD_DIR -I$PROJECT_ROOT -I$PROJECT_ROOT/api -I$PROJECT_ROOT/audio_processing"
    INCLUDES="$INCLUDES -I$PROJECT_ROOT/audio_processing/include -I$PROJECT_ROOT/base"
    INCLUDES="$INCLUDES -I$PROJECT_ROOT/base/rtc_base -I$PROJECT_ROOT/base/system_wrappers"
    INCLUDES="$INCLUDES -I$PROJECT_ROOT/base/abseil"

    # 创建输出目录
    mkdir -p "$OUTPUT_DIR/lib/$arch"
    
    # 编译所有源文件为目标文件
    cd "$BUILD_DIR"
    OBJECTS=""
    for source in $SOURCES; do
        if [ -f "$source" ]; then
            obj_name=$(basename "$source" | sed 's/\.[^.]*$/.o/')
            echo "编译 $source -> $obj_name"
            
            if [[ "$source" == *.c ]]; then
                # 为C文件添加平台适配头文件
                xcrun clang $CFLAGS $INCLUDES -include ios_platform_compat.h -c "$source" -o "$obj_name"
            else
                # 为C++文件添加平台适配头文件
                xcrun clang++ $CXXFLAGS $INCLUDES -include ios_platform_compat.h -c "$source" -o "$obj_name"
            fi
            
            if [ $? -eq 0 ]; then
                OBJECTS="$OBJECTS $obj_name"
            else
                echo "⚠️  编译 $source 失败，跳过"
            fi
        else
            echo "⚠️  源文件不存在: $source"
        fi
    done

    # 创建静态库
    if [ -n "$OBJECTS" ]; then
        echo "创建静态库..."
        xcrun ar rcs "$OUTPUT_DIR/lib/$arch/libwq_aec3_tts.a" $OBJECTS
        
        if [ $? -eq 0 ]; then
            echo "✅ 为 $arch 架构构建成功"
        else
            echo "❌ 创建静态库失败"
            continue
        fi
    else
        echo "❌ 没有成功编译的目标文件"
        continue
    fi
done

cd "$PROJECT_ROOT"

# ============================================================================
# 创建iOS XCFramework包
# ============================================================================
echo "📦 正在创建iOS XCFramework包..."

# 创建XCFramework目录结构
XCFRAMEWORK_DIR="$OUTPUT_DIR/$FRAMEWORK_NAME.xcframework"
mkdir -p "$XCFRAMEWORK_DIR"

# 复制头文件
mkdir -p "$OUTPUT_DIR/include"
cp "$PROJECT_ROOT/my-info/my-cpp-files/wq_aec3_processor.h" "$OUTPUT_DIR/include/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/webrtc_compat.h" "$OUTPUT_DIR/include/"
cp "$PROJECT_ROOT/my-info/my-cpp-files/wq_aec3_convertor.h" "$OUTPUT_DIR/include/"

# 使用xcodebuild创建XCFramework
if [ -f "$OUTPUT_DIR/lib/arm64/libwq_aec3_tts.a" ]; then
    xcodebuild -create-xcframework \
        -library "$OUTPUT_DIR/lib/arm64/libwq_aec3_tts.a" \
        -headers "$OUTPUT_DIR/include" \
        -output "$XCFRAMEWORK_DIR"
    
    echo "✅ XCFramework创建成功"
else
    echo "❌ 找不到静态库文件，无法创建XCFramework"
    exit 1
fi

# ============================================================================
# 生成使用文档
# ============================================================================
echo "📚 正在生成iOS使用文档..."

cat > "$OUTPUT_DIR/iOS_USAGE.md" << 'EOUSAGE'
# WebRTC AEC3 iOS XCFramework 使用指南

## 集成步骤

### 1. 添加XCFramework到项目
1. 将 `WqAec3.xcframework` 拖拽到你的Xcode项目中
2. 在项目设置中，选择 "Embed & Sign" 为 "Do Not Embed"（静态库）
3. 确保在 "Link Binary With Libraries" 中添加了该框架

### 2. 导入头文件
```objc
#import "wq_aec3_processor.h"
```

### 3. 基本使用
```objc
// 初始化AEC处理器
webrtc_aec3_tts::WqAec3Processor* processor = new webrtc_aec3_tts::WqAec3Processor();
processor->Initialize();

// 处理TTS音频（参考信号）
processor->ProcessTtsAudio(tts_samples, frame_size);

// 处理麦克风音频并移除回声
processor->ProcessMicrophoneAudio(mic_samples, output_samples, frame_size);

// 获取性能指标
double erl, erle;
int delay_ms;
processor->GetMetrics(&erl, &erle, &delay_ms);
```

## 技术规格
- 采样率: 48kHz
- 帧大小: 480样本（10ms）
- 声道: 单声道
- 架构支持: arm64（仅真机）

## 注意事项
- 该XCFramework为静态库，无需设置"Embed & Sign"
- 仅支持iOS真机（arm64），不包含模拟器支持
- 确保音频数据格式为16位PCM
EOUSAGE

# ============================================================================
# 最终摘要
# ============================================================================
echo ""
echo "🎉 构建完成！"
echo "📁 输出目录: $OUTPUT_DIR"
echo "📦 XCFramework: $OUTPUT_DIR/$FRAMEWORK_NAME.xcframework"
echo "📚 文档: $OUTPUT_DIR/iOS_USAGE.md"
echo ""
echo "📊 构建摘要:"
echo "  - 采样率: ${AEC3_SAMPLE_RATE}Hz"
echo "  - 帧大小: ${AEC3_FRAME_SIZE} 样本 (10ms)"
echo "  - 流延迟: ${IOS_STREAM_DELAY}ms"
echo "  - 架构: ${ARCHITECTURES[*]}"
echo ""
echo "🚀 下一步:"
echo "  1. 将 $FRAMEWORK_NAME.xcframework 添加到你的iOS项目"
echo "  2. 设置为静态库（不要选择Embed & Sign）"
echo "  3. 按照 iOS_USAGE.md 中的指南进行集成"
echo "  4. 在真机上测试TTS回声消除功能"
echo ""
echo "⚠️  重要: 该XCFramework仅支持iOS真机（arm64架构）"
echo "📈 预期性能: 增强的ERLE (>15dB目标) 与精确时序同步"