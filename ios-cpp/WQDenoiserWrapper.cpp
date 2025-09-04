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
