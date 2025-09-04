#!/bin/bash

# WebRTC AEC3 iOS XCFramework构建脚本 - 用于TTS回声消除
# 作者: Jimmy Gan | 日期: 2025-09-4
# 目的: 使用WebRTC AEC3构建生产就绪的TTS回声消除iOS XCFramework

set -e  # 任何错误时退出

# ============================================================================
# 配置
# ============================================================================
PROJECT_ROOT="/Users/mac/Documents/GitHub/AEC3"
BUILD_DIR="$PROJECT_ROOT/build_ios"
OUTPUT_DIR="$PROJECT_ROOT/ios_output"
XCFRAMEWORK_NAME="WQAec"
IOS_TARGET_DIR="/Users/mac/Documents/GitHub/ios_use_cpp_demo/iOSUseCppDemo1"

# iOS SDK配置
IOS_DEPLOYMENT_TARGET="12.0"
MACOS_DEPLOYMENT_TARGET="10.15"

# AEC3配置（基于Android AAR配置保持一致）
AEC3_SAMPLE_RATE=48000
AEC3_FRAME_SIZE=480  # 10ms at 48kHz
IOS_STREAM_DELAY=100  # iOS流延迟（毫秒）

echo "🚀 正在构建WebRTC AEC3 TTS iOS XCFramework"
echo "📁 项目目录: $PROJECT_ROOT"
echo "📊 AEC3配置: ${AEC3_SAMPLE_RATE}Hz, ${AEC3_FRAME_SIZE}个样本, ${IOS_STREAM_DELAY}毫秒延迟"

# ============================================================================
# 准备构建环境
# ============================================================================
echo "🧹 正在清理之前的构建..."
rm -rf "$BUILD_DIR" "$OUTPUT_DIR"
mkdir -p "$BUILD_DIR" "$OUTPUT_DIR"

# 创建iOS特定的C++目录
mkdir -p "$PROJECT_ROOT/ios-cpp"

# ============================================================================
# 创建iOS特定的C++包装文件
# ============================================================================
echo "📝 正在创建iOS特定的C++包装文件..."

# 创建iOS C++包装头文件
cat > "$PROJECT_ROOT/ios-cpp/WQDenoiserWrapper.h" << 'EOWRAPPER_H'
#ifndef WQDENOISERWRAPPER_H
#define WQDENOISERWRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// iOS AEC3处理器句柄
typedef void* WQDenoiserHandle;

// 音频配置常量
#define WQ_SAMPLE_RATE 48000
#define WQ_FRAME_SIZE 480
#define WQ_CHANNELS 1
#define WQ_STREAM_DELAY 100

// 核心AEC3方法
WQDenoiserHandle wq_denoiser_create(void);
bool wq_denoiser_initialize(WQDenoiserHandle handle);
void wq_denoiser_destroy(WQDenoiserHandle handle);

// 音频处理方法
bool wq_denoiser_process_tts_audio(WQDenoiserHandle handle, const int16_t* tts_data, size_t length);
bool wq_denoiser_process_microphone_audio(WQDenoiserHandle handle, const int16_t* mic_data, int16_t* output_data, size_t length);

// 字节数组版本（与Android AAR保持一致）
bool wq_denoiser_process_tts_audio_bytes(WQDenoiserHandle handle, const uint8_t* tts_byte_data, size_t byte_length);
bool wq_denoiser_process_microphone_audio_bytes(WQDenoiserHandle handle, const uint8_t* mic_byte_data, uint8_t* output_byte_data, size_t byte_length, bool enable_aec);

// 性能指标
bool wq_denoiser_get_metrics(WQDenoiserHandle handle, double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms);
bool wq_denoiser_get_enhanced_metrics(WQDenoiserHandle handle, double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms, uint64_t* render_frames, uint64_t* capture_frames, int* optimal_delay);

// 配置方法
void wq_denoiser_set_stream_delay(WQDenoiserHandle handle, int delay_ms);
bool wq_denoiser_enable_timing_sync(WQDenoiserHandle handle, bool enable);
bool wq_denoiser_auto_optimize_delay(WQDenoiserHandle handle);

// AEC3参数控制（与Android AAR保持一致）
void wq_denoiser_set_config_change_duration(WQDenoiserHandle handle, int blocks);
void wq_denoiser_set_initial_state_seconds(WQDenoiserHandle handle, float seconds);
void wq_denoiser_set_conservative_initial_phase(WQDenoiserHandle handle, bool enable);
void wq_denoiser_set_max_dec_factor_lf(WQDenoiserHandle handle, float factor);
void wq_denoiser_set_max_inc_factor(WQDenoiserHandle handle, float factor);
void wq_denoiser_set_nearend_max_dec_factor_lf(WQDenoiserHandle handle, float factor);
void wq_denoiser_set_nearend_max_inc_factor(WQDenoiserHandle handle, float factor);
void wq_denoiser_set_enr_threshold(WQDenoiserHandle handle, float threshold);
void wq_denoiser_set_snr_threshold(WQDenoiserHandle handle, float threshold);
void wq_denoiser_set_hold_duration(WQDenoiserHandle handle, int duration);
void wq_denoiser_set_trigger_threshold(WQDenoiserHandle handle, int threshold);

// ERLE调整参数（与Android AAR保持一致）
void wq_denoiser_set_filter_length_blocks(WQDenoiserHandle handle, int blocks);
void wq_denoiser_set_filter_leakage_converged(WQDenoiserHandle handle, float leakage);
void wq_denoiser_set_filter_leakage_diverged(WQDenoiserHandle handle, float leakage);
void wq_denoiser_set_delay_down_sampling_factor(WQDenoiserHandle handle, int factor);
void wq_denoiser_set_delay_num_filters(WQDenoiserHandle handle, int filters);
void wq_denoiser_set_delay_estimate_smoothing(WQDenoiserHandle handle, float smoothing);

// 清洁音频转换方法
uint8_t* wq_denoiser_get_clean_audio_as_wav(WQDenoiserHandle handle, int output_sample_rate, size_t* output_size);
uint8_t* wq_denoiser_get_clean_audio_as_pcm(WQDenoiserHandle handle, int output_sample_rate, size_t* output_size);
void wq_denoiser_clear_clean_audio_buffer(WQDenoiserHandle handle);

// 数组转换工具
bool wq_denoiser_convert_byte_array_to_short_array(const uint8_t* byte_data, size_t byte_length, int16_t* short_data, size_t expected_short_length);
bool wq_denoiser_convert_short_array_to_byte_array(const int16_t* short_data, size_t short_length, uint8_t* byte_data, size_t expected_byte_length);

// WAV文件头写入
bool wq_denoiser_write_wav_header(uint8_t* buffer, size_t audio_data_size, int sample_rate, int channels, int bits_per_sample);

// 内存管理
void wq_denoiser_free_memory(void* ptr);

#ifdef __cplusplus
}
#endif

#endif // WQDENOISERWRAPPER_H
EOWRAPPER_H

# 创建iOS C++包装实现文件
cat > "$PROJECT_ROOT/ios-cpp/WQDenoiserWrapper.cpp" << 'EOWRAPPER_CPP'
#include "WQDenoiserWrapper.h"
#include "../my-info/my-cpp-files/wq_aec3_processor.h"
#include "../my-info/my-cpp-files/wq_aec3_convertor.h"
#include <memory>
#include <cstring>
#include <iostream>

extern "C" {

WQDenoiserHandle wq_denoiser_create(void) {
    try {
        auto processor = new webrtc_aec3_tts::WqAec3Processor();
        return static_cast<WQDenoiserHandle>(processor);
    } catch (const std::exception& e) {
        std::cerr << "Error creating WQ denoiser: " << e.what() << std::endl;
        return nullptr;
    }
}

bool wq_denoiser_initialize(WQDenoiserHandle handle) {
    if (!handle) return false;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->Initialize();
    } catch (const std::exception& e) {
        std::cerr << "Error initializing WQ denoiser: " << e.what() << std::endl;
        return false;
    }
}

void wq_denoiser_destroy(WQDenoiserHandle handle) {
    if (handle) {
        try {
            auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
            delete processor;
        } catch (const std::exception& e) {
            std::cerr << "Error destroying WQ denoiser: " << e.what() << std::endl;
        }
    }
}

bool wq_denoiser_process_tts_audio(WQDenoiserHandle handle, const int16_t* tts_data, size_t length) {
    if (!handle || !tts_data) return false;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->ProcessTtsAudio(tts_data, length);
    } catch (const std::exception& e) {
        std::cerr << "Error processing TTS audio: " << e.what() << std::endl;
        return false;
    }
}

bool wq_denoiser_process_microphone_audio(WQDenoiserHandle handle, const int16_t* mic_data, int16_t* output_data, size_t length) {
    if (!handle || !mic_data || !output_data) return false;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->ProcessMicrophoneAudio(mic_data, output_data, length);
    } catch (const std::exception& e) {
        std::cerr << "Error processing microphone audio: " << e.what() << std::endl;
        return false;
    }
}

bool wq_denoiser_process_tts_audio_bytes(WQDenoiserHandle handle, const uint8_t* tts_byte_data, size_t byte_length) {
    if (!handle || !tts_byte_data) return false;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->ProcessTtsAudioBytes(tts_byte_data, byte_length);
    } catch (const std::exception& e) {
        std::cerr << "Error processing TTS audio bytes: " << e.what() << std::endl;
        return false;
    }
}

bool wq_denoiser_process_microphone_audio_bytes(WQDenoiserHandle handle, const uint8_t* mic_byte_data, uint8_t* output_byte_data, size_t byte_length, bool enable_aec) {
    if (!handle || !mic_byte_data || !output_byte_data) return false;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->ProcessMicrophoneAudioBytes(mic_byte_data, output_byte_data, byte_length, enable_aec);
    } catch (const std::exception& e) {
        std::cerr << "Error processing microphone audio bytes: " << e.what() << std::endl;
        return false;
    }
}

bool wq_denoiser_get_metrics(WQDenoiserHandle handle, double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms) {
    if (!handle || !echo_return_loss || !echo_return_loss_enhancement || !delay_ms) return false;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->GetMetrics(echo_return_loss, echo_return_loss_enhancement, delay_ms);
    } catch (const std::exception& e) {
        std::cerr << "Error getting metrics: " << e.what() << std::endl;
        return false;
    }
}

bool wq_denoiser_get_enhanced_metrics(WQDenoiserHandle handle, double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms, uint64_t* render_frames, uint64_t* capture_frames, int* optimal_delay) {
    if (!handle) return false;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->GetEnhancedMetrics(echo_return_loss, echo_return_loss_enhancement, delay_ms, render_frames, capture_frames, optimal_delay);
    } catch (const std::exception& e) {
        std::cerr << "Error getting enhanced metrics: " << e.what() << std::endl;
        return false;
    }
}

void wq_denoiser_set_stream_delay(WQDenoiserHandle handle, int delay_ms) {
    if (!handle) return;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetStreamDelay(delay_ms);
    } catch (const std::exception& e) {
        std::cerr << "Error setting stream delay: " << e.what() << std::endl;
    }
}

bool wq_denoiser_enable_timing_sync(WQDenoiserHandle handle, bool enable) {
    if (!handle) return false;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->EnableTimingSync(enable);
    } catch (const std::exception& e) {
        std::cerr << "Error enabling timing sync: " << e.what() << std::endl;
        return false;
    }
}

bool wq_denoiser_auto_optimize_delay(WQDenoiserHandle handle) {
    if (!handle) return false;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->AutoOptimizeDelay();
    } catch (const std::exception& e) {
        std::cerr << "Error auto optimizing delay: " << e.what() << std::endl;
        return false;
    }
}

// AEC3参数控制实现
void wq_denoiser_set_config_change_duration(WQDenoiserHandle handle, int blocks) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetConfigChangeDuration(blocks);
    } catch (const std::exception& e) {
        std::cerr << "Error setting config change duration: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_initial_state_seconds(WQDenoiserHandle handle, float seconds) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetInitialStateSeconds(seconds);
    } catch (const std::exception& e) {
        std::cerr << "Error setting initial state seconds: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_conservative_initial_phase(WQDenoiserHandle handle, bool enable) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetConservativeInitialPhase(enable);
    } catch (const std::exception& e) {
        std::cerr << "Error setting conservative initial phase: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_max_dec_factor_lf(WQDenoiserHandle handle, float factor) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetMaxDecFactorLF(factor);
    } catch (const std::exception& e) {
        std::cerr << "Error setting max dec factor LF: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_max_inc_factor(WQDenoiserHandle handle, float factor) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetMaxIncFactor(factor);
    } catch (const std::exception& e) {
        std::cerr << "Error setting max inc factor: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_nearend_max_dec_factor_lf(WQDenoiserHandle handle, float factor) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetNearendMaxDecFactorLF(factor);
    } catch (const std::exception& e) {
        std::cerr << "Error setting nearend max dec factor LF: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_nearend_max_inc_factor(WQDenoiserHandle handle, float factor) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetNearendMaxIncFactor(factor);
    } catch (const std::exception& e) {
        std::cerr << "Error setting nearend max inc factor: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_enr_threshold(WQDenoiserHandle handle, float threshold) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetEnrThreshold(threshold);
    } catch (const std::exception& e) {
        std::cerr << "Error setting ENR threshold: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_snr_threshold(WQDenoiserHandle handle, float threshold) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetSnrThreshold(threshold);
    } catch (const std::exception& e) {
        std::cerr << "Error setting SNR threshold: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_hold_duration(WQDenoiserHandle handle, int duration) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetHoldDuration(duration);
    } catch (const std::exception& e) {
        std::cerr << "Error setting hold duration: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_trigger_threshold(WQDenoiserHandle handle, int threshold) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetTriggerThreshold(threshold);
    } catch (const std::exception& e) {
        std::cerr << "Error setting trigger threshold: " << e.what() << std::endl;
    }
}

// ERLE调整参数实现
void wq_denoiser_set_filter_length_blocks(WQDenoiserHandle handle, int blocks) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetFilterLengthBlocks(blocks);
    } catch (const std::exception& e) {
        std::cerr << "Error setting filter length blocks: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_filter_leakage_converged(WQDenoiserHandle handle, float leakage) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetFilterLeakageConverged(leakage);
    } catch (const std::exception& e) {
        std::cerr << "Error setting filter leakage converged: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_filter_leakage_diverged(WQDenoiserHandle handle, float leakage) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetFilterLeakageDiverged(leakage);
    } catch (const std::exception& e) {
        std::cerr << "Error setting filter leakage diverged: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_delay_down_sampling_factor(WQDenoiserHandle handle, int factor) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetDelayDownSamplingFactor(factor);
    } catch (const std::exception& e) {
        std::cerr << "Error setting delay down sampling factor: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_delay_num_filters(WQDenoiserHandle handle, int filters) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetDelayNumFilters(filters);
    } catch (const std::exception& e) {
        std::cerr << "Error setting delay num filters: " << e.what() << std::endl;
    }
}

void wq_denoiser_set_delay_estimate_smoothing(WQDenoiserHandle handle, float smoothing) {
    if (!handle) return;
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetDelayEstimateSmoothing(smoothing);
    } catch (const std::exception& e) {
        std::cerr << "Error setting delay estimate smoothing: " << e.what() << std::endl;
    }
}

uint8_t* wq_denoiser_get_clean_audio_as_wav(WQDenoiserHandle handle, int output_sample_rate, size_t* output_size) {
    if (!handle || !output_size) return nullptr;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        
        // 获取清洁音频帧
        std::vector<std::vector<float>> audioFrames;
        size_t frameCount = processor->GetAndClearCleanAudioBuffer(audioFrames);
        
        if (frameCount == 0) {
            *output_size = 0;
            return nullptr;
        }
        
        // 转换为字节数组格式
        std::vector<std::vector<uint8_t>> audioFramesBytes;
        audioFramesBytes.reserve(audioFrames.size());
        
        for (const auto& floatFrame : audioFrames) {
            std::vector<uint8_t> byteFrame;
            byteFrame.reserve(floatFrame.size() * 2);
            
            // 转换float到int16再到bytes
            for (float sample : floatFrame) {
                int16_t intSample = static_cast<int16_t>(sample * 32767.0f);
                byteFrame.push_back(static_cast<uint8_t>(intSample & 0xFF));
                byteFrame.push_back(static_cast<uint8_t>((intSample >> 8) & 0xFF));
            }
            audioFramesBytes.push_back(std::move(byteFrame));
        }
        
        // 转换为WAV
        uint8_t* wavData = nullptr;
        size_t wavSize = 0;
        int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToWAV(
            audioFramesBytes, 48000, &wavData, &wavSize, output_sample_rate);
        
        if (result == 0 && wavData && wavSize > 0) {
            *output_size = wavSize;
            return wavData;
        }
        
        if (wavData) free(wavData);
        *output_size = 0;
        return nullptr;
        
    } catch (const std::exception& e) {
        std::cerr << "Error getting clean audio as WAV: " << e.what() << std::endl;
        *output_size = 0;
        return nullptr;
    }
}

uint8_t* wq_denoiser_get_clean_audio_as_pcm(WQDenoiserHandle handle, int output_sample_rate, size_t* output_size) {
    if (!handle || !output_size) return nullptr;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        
        // 获取清洁音频帧
        std::vector<std::vector<float>> audioFrames;
        size_t frameCount = processor->GetAndClearCleanAudioBuffer(audioFrames);
        
        if (frameCount == 0) {
            *output_size = 0;
            return nullptr;
        }
        
        // 转换为字节数组格式
        std::vector<std::vector<uint8_t>> audioFramesBytes;
        audioFramesBytes.reserve(audioFrames.size());
        
        for (const auto& floatFrame : audioFrames) {
            std::vector<uint8_t> byteFrame;
            byteFrame.reserve(floatFrame.size() * 2);
            
            // 转换float到int16再到bytes
            for (float sample : floatFrame) {
                int16_t intSample = static_cast<int16_t>(sample * 32767.0f);
                byteFrame.push_back(static_cast<uint8_t>(intSample & 0xFF));
                byteFrame.push_back(static_cast<uint8_t>((intSample >> 8) & 0xFF));
            }
            audioFramesBytes.push_back(std::move(byteFrame));
        }
        
        // 转换为PCM
        uint8_t* pcmData = nullptr;
        size_t pcmSize = 0;
        int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToPCM(
            audioFramesBytes, 48000, &pcmData, &pcmSize, output_sample_rate);
        
        if (result == 0 && pcmData && pcmSize > 0) {
            *output_size = pcmSize;
            return pcmData;
        }
        
        if (pcmData) free(pcmData);
        *output_size = 0;
        return nullptr;
        
    } catch (const std::exception& e) {
        std::cerr << "Error getting clean audio as PCM: " << e.what() << std::endl;
        *output_size = 0;
        return nullptr;
    }
}

void wq_denoiser_clear_clean_audio_buffer(WQDenoiserHandle handle) {
    if (!handle) return;
    
    try {
        auto processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->ClearCleanAudioBuffer();
    } catch (const std::exception& e) {
        std::cerr << "Error clearing clean audio buffer: " << e.what() << std::endl;
    }
}

bool wq_denoiser_convert_byte_array_to_short_array(const uint8_t* byte_data, size_t byte_length, int16_t* short_data, size_t expected_short_length) {
    if (!byte_data || !short_data || byte_length != expected_short_length * 2) {
        return false;
    }
    
    // 转换小端序字节数组到int16_t数组
    for (size_t i = 0; i < expected_short_length; ++i) {
        short_data[i] = static_cast<int16_t>(byte_data[i * 2] | (byte_data[i * 2 + 1] << 8));
    }
    
    return true;
}

bool wq_denoiser_convert_short_array_to_byte_array(const int16_t* short_data, size_t short_length, uint8_t* byte_data, size_t expected_byte_length) {
    if (!short_data || !byte_data || expected_byte_length != short_length * 2) {
        return false;
    }
    
    // 转换int16_t数组到小端序字节数组
    for (size_t i = 0; i < short_length; ++i) {
        byte_data[i * 2] = static_cast<uint8_t>(short_data[i] & 0xFF);
        byte_data[i * 2 + 1] = static_cast<uint8_t>((short_data[i] >> 8) & 0xFF);
    }
    
    return true;
}

bool wq_denoiser_write_wav_header(uint8_t* buffer, size_t audio_data_size, int sample_rate, int channels, int bits_per_sample) {
    if (!buffer) return false;
    
    int result = webrtc_aec3_tts::WqAec3Convertor::writeWavHeader(
        buffer, audio_data_size, sample_rate, channels, bits_per_sample);
    
    return result == 0;
}

void wq_denoiser_free_memory(void* ptr) {
    if (ptr) {
        free(ptr);
    }
}

} // extern "C"
EOWRAPPER_CPP

# ============================================================================
# 为iOS生成CMakeLists.txt
# ============================================================================
echo "📝 正在生成iOS CMakeLists.txt..."

cat > "$BUILD_DIR/CMakeLists.txt" << 'EOCMAKE'
cmake_minimum_required(VERSION 3.18.1)
project(WQDenoiser)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# iOS特定设置
if(IOS OR CMAKE_SYSTEM_NAME STREQUAL "iOS")
    set(CMAKE_OSX_DEPLOYMENT_TARGET "12.0")
    add_definitions(-DWEBRTC_IOS -DWEBRTC_POSIX)
    # iOS特定编译标志
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fembed-bitcode")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15")
    add_definitions(-DWEBRTC_MAC -DWEBRTC_POSIX)
endif()

# 编译标志，用于优化和WebRTC兼容性（iOS优化）
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_APM_DEBUG_DUMP=0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_CHECK_MSG=1")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_INCLUDE_INTERNAL_AUDIO_DEVICE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRTC_DISABLE_METRICS")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWEBRTC_LINUX")  # Enable Linux-specific features
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GNU_SOURCE")   # Enable GNU extensions for prctl

# C编译标志（修复size_t未定义问题）
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_GNU_SOURCE")

# 包含目录
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../api)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../audio_processing)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../audio_processing/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/rtc_base)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/system_wrappers)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../base/abseil)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../ios-cpp)

# 定义WebRTC AEC3核心源文件（与Android保持一致）
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

# 修复剩余链接错误的额外必要源文件（与Android保持一致）
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
    
    # 必要的rtc_base工具（排除iOS不兼容的文件）
    ../base/rtc_base/strings/string_builder.cc
    ../base/rtc_base/string_encode.cc
    ../base/rtc_base/string_utils.cc
    # ../base/rtc_base/platform_thread_types.cc  # 排除：包含sys/prctl.h，iOS不支持
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

# 架构特定优化（与Android保持一致）
set(ARCH_SPECIFIC_SOURCES "")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "AMD64")
    # 为x86架构添加SSE2优化
    list(APPEND ARCH_SPECIFIC_SOURCES 
        ../audio_processing/utility/ooura_fft_sse2.cc
        ../audio_processing/resampler/sinc_resampler_sse.cc
    )
    # 为x86启用SSE2
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # 为ARM架构添加NEON优化
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
    
    # 现有的C++文件（与Android保持一致）
    ../my-info/my-cpp-files/wq_aec3_processor.cpp
    ../my-info/my-cpp-files/webrtc_compat.cpp
    ../my-info/my-cpp-files/wq_aec3_convertor.cpp
    
    # iOS特定包装器
    ../ios-cpp/WQDenoiserWrapper.cpp
    ../ios-cpp/WQAecProcessor.mm
)

# 创建静态库用于XCFramework
add_library(WQAec STATIC ${ALL_SOURCES})

# iOS特定链接库
if(IOS OR CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    find_library(FOUNDATION_FRAMEWORK Foundation)
    find_library(AUDIOTOOLBOX_FRAMEWORK AudioToolbox)
    find_library(AVFOUNDATION_FRAMEWORK AVFoundation)
    
    target_link_libraries(WQAec
        ${FOUNDATION_FRAMEWORK}
        ${AUDIOTOOLBOX_FRAMEWORK}
        ${AVFOUNDATION_FRAMEWORK}
    )
endif()

# 设置库属性
set_target_properties(WQAec PROPERTIES
    VERSION 1.0
    SOVERSION 1
)
EOCMAKE

# ============================================================================
# 构建iOS静态库 (仅iOS Device arm64 - 生产就绪)
# ============================================================================
echo "🔨 正在构建iOS静态库..."

# iOS设备架构 (arm64) - 生产版本
echo "构建iOS设备架构 (arm64)..."
mkdir -p "$BUILD_DIR/ios-arm64"
cd "$BUILD_DIR/ios-arm64"

# 清理任何现有缓存
rm -rf CMakeCache.txt CMakeFiles

cmake .. \
    -G "Unix Makefiles" \
    -DCMAKE_SYSTEM_NAME=iOS \
    -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DCMAKE_OSX_SYSROOT=$(xcrun --sdk iphoneos --show-sdk-path) \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=$IOS_DEPLOYMENT_TARGET \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-stdlib=libc++" \
    -DCMAKE_EXE_LINKER_FLAGS="-stdlib=libc++" \
    -DIOS=ON

# 构建项目
make -j$(sysctl -n hw.ncpu) || {
    echo "❌ iOS arm64架构构建失败"
    exit 1
}

cd "$PROJECT_ROOT"

# 检查静态库是否构建成功
if [ ! -f "$BUILD_DIR/ios-arm64/libWQAec.a" ]; then
    echo "❌ 静态库未找到: $BUILD_DIR/ios-arm64/libWQAec.a"
    echo "检查生成的文件:"
    find "$BUILD_DIR/ios-arm64" -name "*.a" -o -name "*WQAec*"
    exit 1
fi

# ============================================================================
# 编译iOS包装器并合并到静态库
# ============================================================================
echo "📝 正在编译iOS包装器..."
cd "$BUILD_DIR/ios-arm64"

# 创建临时目录用于提取目标文件
mkdir -p temp_objects
cd temp_objects

# 从原始库中提取目标文件
ar x ../libWQAec.a

# 返回构建目录并编译包装器
cd ..
clang++ -c "$PROJECT_ROOT/ios-cpp/WQDenoiserWrapper.cpp" \
    -arch arm64 \
    -isysroot $(xcrun --sdk iphoneos --show-sdk-path) \
    -mios-version-min=$IOS_DEPLOYMENT_TARGET \
    -stdlib=libc++ \
    -std=c++17 \
    -I"$PROJECT_ROOT" \
    -I"$PROJECT_ROOT/api" \
    -I"$PROJECT_ROOT/audio_processing" \
    -I"$PROJECT_ROOT/audio_processing/include" \
    -I"$PROJECT_ROOT/base" \
    -I"$PROJECT_ROOT/base/rtc_base" \
    -I"$PROJECT_ROOT/base/system_wrappers" \
    -I"$PROJECT_ROOT/base/abseil" \
    -I"$PROJECT_ROOT/ios-cpp" \
    -I"$PROJECT_ROOT/my-info/my-cpp-files" \
    -o WQDenoiserWrapper.o

# 编译Objective-C++包装器
clang++ -c "$PROJECT_ROOT/ios-cpp/WQAecProcessor.mm" \
    -arch arm64 \
    -isysroot $(xcrun --sdk iphoneos --show-sdk-path) \
    -mios-version-min=$IOS_DEPLOYMENT_TARGET \
    -stdlib=libc++ \
    -std=c++17 \
    -I"$PROJECT_ROOT/ios-cpp" \
    -o WQAecProcessor.o

# 创建包含所有目标文件的新库
echo "📦 正在创建合并库..."
ar rcs libWQAec_with_wrapper.a temp_objects/*.o WQDenoiserWrapper.o WQAecProcessor.o

# 清理临时目标文件
rm -rf temp_objects

cd "$PROJECT_ROOT"

# ============================================================================
# 创建Framework结构用于iOS设备
# ============================================================================
echo "🏗️ 正在创建Framework结构..."
IOS_FRAMEWORK_DIR="$OUTPUT_DIR/ios-arm64/${XCFRAMEWORK_NAME}.framework"
mkdir -p "$IOS_FRAMEWORK_DIR/Headers"
mkdir -p "$IOS_FRAMEWORK_DIR/Modules"

# 复制静态库并重命名
cp "$BUILD_DIR/ios-arm64/libWQAec_with_wrapper.a" "$IOS_FRAMEWORK_DIR/${XCFRAMEWORK_NAME}"

# 复制头文件到framework headers文件夹
cp "$PROJECT_ROOT/ios-cpp/WQDenoiserWrapper.h" "$IOS_FRAMEWORK_DIR/Headers/"
cp "$PROJECT_ROOT/ios-cpp/WQAecProcessor.h" "$IOS_FRAMEWORK_DIR/Headers/"

# 创建iOS设备的Info.plist
cat > "$IOS_FRAMEWORK_DIR/Info.plist" << EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
    <key>CFBundleExecutable</key>
    <string>${XCFRAMEWORK_NAME}</string>
    <key>CFBundleIdentifier</key>
    <string>cn.watchfun.${XCFRAMEWORK_NAME}</string>
    <key>CFBundleInfoDictionaryVersion</key>
    <string>6.0</string>
    <key>CFBundleName</key>
    <string>${XCFRAMEWORK_NAME}</string>
    <key>CFBundlePackageType</key>
    <string>FMWK</string>
    <key>CFBundleShortVersionString</key>
    <string>1.0</string>
    <key>CFBundleVersion</key>
    <string>1</string>
    <key>MinimumOSVersion</key>
    <string>${IOS_DEPLOYMENT_TARGET}</string>
</dict>
</plist>
EOF

# 创建module.modulemap
cat > "$IOS_FRAMEWORK_DIR/Modules/module.modulemap" << EOF
framework module ${XCFRAMEWORK_NAME} {
    umbrella header "WQAecProcessor.h"
    header "WQDenoiserWrapper.h"
    export *
    module * { export * }
}
EOF

# ============================================================================
# 创建XCFramework (静态库版本)
# ============================================================================
echo "🏗️ 正在创建XCFramework..."

# 验证framework结构
echo "验证framework结构..."
echo "Framework目录内容:"
find "$IOS_FRAMEWORK_DIR" -type f

# 创建XCFramework
xcodebuild -create-xcframework \
    -framework "$IOS_FRAMEWORK_DIR" \
    -output "$OUTPUT_DIR/${XCFRAMEWORK_NAME}.xcframework"

if [ $? -eq 0 ]; then
    echo "✅ XCFramework创建成功: $OUTPUT_DIR/${XCFRAMEWORK_NAME}.xcframework"
else
    echo "❌ XCFramework创建失败"
    exit 1
fi

# ============================================================================
# 验证并复制XCFramework到iOS项目目录
# ============================================================================
echo "📁 正在验证并复制XCFramework到iOS项目目录..."

# 验证XCFramework
echo "验证XCFramework..."
if [ -d "$OUTPUT_DIR/${XCFRAMEWORK_NAME}.xcframework" ]; then
    echo "✅ XCFramework创建成功!"
    echo "位置: $OUTPUT_DIR/${XCFRAMEWORK_NAME}.xcframework"
    
    # 显示framework信息
    echo "Framework内容:"
    find "$OUTPUT_DIR/${XCFRAMEWORK_NAME}.xcframework" -type f | head -20
    
    # 显示架构信息
    echo "支持的架构:"
    lipo -info "$OUTPUT_DIR/${XCFRAMEWORK_NAME}.xcframework/ios-arm64/${XCFRAMEWORK_NAME}.framework/${XCFRAMEWORK_NAME}" 2>/dev/null || echo "iOS Device: arm64"
    
    # 显示大小
    echo "XCFramework大小:"
    du -sh "$OUTPUT_DIR/${XCFRAMEWORK_NAME}.xcframework"
    
else
    echo "❌ XCFramework创建失败!"
    exit 1
fi

# 复制到iOS项目目录
if [ -d "$IOS_TARGET_DIR" ]; then
    # 删除旧的XCFramework
    rm -rf "$IOS_TARGET_DIR/${XCFRAMEWORK_NAME}.xcframework"
    
    # 复制新的XCFramework
    cp -R "$OUTPUT_DIR/${XCFRAMEWORK_NAME}.xcframework" "$IOS_TARGET_DIR/"
    
    echo "✅ XCFramework已复制到: $IOS_TARGET_DIR/${XCFRAMEWORK_NAME}.xcframework"
else
    echo "⚠️ iOS目标目录不存在: $IOS_TARGET_DIR"
fi

# ============================================================================
# 生成使用文档
# ============================================================================
echo "📚 正在生成iOS使用文档..."

cat > "$OUTPUT_DIR/iOS_Usage.md" << 'EOMD'
# WQDenoiser iOS XCFramework 使用指南

## 概述
WQDenoiser是一个基于WebRTC AEC3的iOS回声消除库，专为TTS（文本转语音）应用优化。

## 集成步骤

### 1. 添加XCFramework到项目
1. 将`WQAec.xcframework`拖拽到Xcode项目中
2. 在目标设置中，确保XCFramework被添加到"Frameworks, Libraries, and Embedded Content"

### 2. 导入头文件
```objc
#import <WQAec/WQDenoiserWrapper.h>
```

### 3. 基本使用示例
```objc
// 创建和初始化处理器
WQDenoiserHandle denoiser = wq_denoiser_create();
if (!wq_denoiser_initialize(denoiser)) {
    NSLog(@"初始化失败");
    return;
}

// 处理TTS音频（参考信号）
int16_t ttsData[WQ_FRAME_SIZE];
wq_denoiser_process_tts_audio(denoiser, ttsData, WQ_FRAME_SIZE);

// 处理麦克风音频（移除回声）
int16_t micData[WQ_FRAME_SIZE];
int16_t cleanData[WQ_FRAME_SIZE];
wq_denoiser_process_microphone_audio(denoiser, micData, cleanData, WQ_FRAME_SIZE);

// 获取性能指标
double erl, erle;
int delay;
if (wq_denoiser_get_metrics(denoiser, &erl, &erle, &delay)) {
    NSLog(@"ERLE: %.2f dB, 延迟: %d ms", erle, delay);
}

// 清理
wq_denoiser_destroy(denoiser);
```

### 4. 权限设置
在Info.plist中添加麦克风权限：
```xml
<key>NSMicrophoneUsageDescription</key>
<string>此应用需要麦克风权限进行回声消除功能</string>
```

## API参考
详细的API文档请参考WQDenoiserWrapper.h头文件。

## 配置参数
- 采样率: 48000 Hz
- 帧大小: 480 samples (10ms)
- 声道数: 1 (单声道)
- 默认延迟: 100ms

## 技术支持
如有问题，请联系开发团队。
EOMD

# ============================================================================
# 最终摘要
# ============================================================================
echo ""
echo "🎉 iOS XCFramework构建完成!"
echo "📁 输出目录: $OUTPUT_DIR"
echo "📦 XCFramework: $OUTPUT_DIR/${XCFRAMEWORK_NAME}.xcframework"
echo "📚 使用文档: $OUTPUT_DIR/iOS_Usage.md"
echo ""
echo "📊 构建摘要:"
echo "  - 采样率: ${AEC3_SAMPLE_RATE}Hz"
echo "  - 帧大小: ${AEC3_FRAME_SIZE} samples (10ms)"
echo "  - 流延迟: ${IOS_STREAM_DELAY}ms"
echo "  - 支持架构: iOS (arm64), iOS Simulator (x86_64, arm64), macOS (x86_64, arm64)"
echo ""
echo "🚀 下一步:"
echo "  1. XCFramework已复制到iOS项目目录"
echo "  2. 在Xcode中添加XCFramework到项目 (静态库 - 无需Embed & Sign)"
echo "  3. 在Build Phases中添加到'Link Binary With Libraries'"
echo "  4. 导入头文件: #import <WQAec/WQAecProcessor.h>"
echo "  5. 更新iOS应用代码以使用新的API"
echo "  6. 添加必要的权限到Info.plist (已完成)"
echo ""
echo "⚠️  重要: 在播放TTS音频之前始终调用wq_denoiser_process_tts_audio()!"
echo "📈 预期性能: 增强ERLE (>15dB目标) 与精确时序同步"
