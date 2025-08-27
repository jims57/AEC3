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
