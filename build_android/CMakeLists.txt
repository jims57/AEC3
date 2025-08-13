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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_CHECK_MSG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_INCLUDE_INTERNAL_AUDIO_DEVICE")

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../api)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../audio_processing)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../audio_processing/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/rtc_base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/system_wrappers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/abseil)

# Define WebRTC AEC3 Core Sources
set(AEC3_CORE_SOURCES
    # API Layer
    ../api/echo_canceller3_factory.cc
    ../api/echo_canceller3_config.cc
    ../api/echo_canceller3_config_json.cc
    
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
)

# Find all AEC3 implementation files
file(GLOB_RECURSE AEC3_IMPL_SOURCES 
    "../audio_processing/aec3/*.cc"
    "../audio_processing/aec3/*.c"
)

# Find base utility sources
file(GLOB_RECURSE BASE_SOURCES
    "../base/rtc_base/*.cc"
    "../base/system_wrappers/*.cc"
    "../base/abseil/absl/base/*.cc"
    "../base/abseil/absl/numeric/*.cc"
    "../base/abseil/absl/strings/*.cc"
    "../base/abseil/absl/types/*.cc"
)

# Filter out test files and unwanted sources
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*test.*")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*_test\\.cc$")
list(FILTER AEC3_IMPL_SOURCES EXCLUDE REGEX ".*_unittest\\.cc$")
list(FILTER BASE_SOURCES EXCLUDE REGEX ".*test.*")
list(FILTER BASE_SOURCES EXCLUDE REGEX ".*_test\\.cc$")

# Combine all sources
set(ALL_SOURCES 
    ${AEC3_CORE_SOURCES}
    ${AEC3_IMPL_SOURCES}
    ${BASE_SOURCES}
    tts_aec3_wrapper.cc
)

# Create static library
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
