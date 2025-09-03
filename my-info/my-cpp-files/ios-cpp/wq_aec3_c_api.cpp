#include "wq_aec3_c_api.h"
#include "../wq_aec3_processor.h"
#include "../wq_aec3_convertor.h"
#include <memory>
#include <vector>

// C API implementation that wraps the C++ classes
extern "C" {

WqAec3ProcessorHandle wq_aec3_processor_create(void) {
    try {
        auto processor = std::make_unique<webrtc_aec3_tts::WqAec3Processor>();
        return processor.release();
    } catch (...) {
        return nullptr;
    }
}

bool wq_aec3_processor_initialize(WqAec3ProcessorHandle handle) {
    if (!handle) return false;
    
    try {
        auto* processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        return processor->Initialize();
    } catch (...) {
        return false;
    }
}

void wq_aec3_processor_set_stream_delay(WqAec3ProcessorHandle handle, int delay_ms) {
    if (!handle) return;
    
    try {
        auto* processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        processor->SetStreamDelay(delay_ms);
    } catch (...) {
        // Ignore exceptions in void function
    }
}

bool wq_aec3_processor_process_tts_audio_bytes(WqAec3ProcessorHandle handle, 
                                               const uint8_t* audio_data, 
                                               int data_length) {
    if (!handle || !audio_data || data_length <= 0) return false;
    
    try {
        auto* processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        
        // Convert bytes to int16_t samples
        if (data_length % 2 != 0) return false; // Must be even for 16-bit samples
        
        const int16_t* samples = reinterpret_cast<const int16_t*>(audio_data);
        size_t sample_count = data_length / 2;
        
        return processor->ProcessTtsAudio(samples, sample_count);
    } catch (...) {
        return false;
    }
}

bool wq_aec3_processor_process_microphone_audio_bytes(WqAec3ProcessorHandle handle,
                                                      const uint8_t* mic_data,
                                                      int data_length,
                                                      bool enable_aec,
                                                      uint8_t* output_data) {
    if (!handle || !mic_data || !output_data || data_length <= 0) return false;
    
    try {
        auto* processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        
        // Convert bytes to int16_t samples
        if (data_length % 2 != 0) return false; // Must be even for 16-bit samples
        
        const int16_t* mic_samples = reinterpret_cast<const int16_t*>(mic_data);
        int16_t* output_samples = reinterpret_cast<int16_t*>(output_data);
        size_t sample_count = data_length / 2;
        
        return processor->ProcessMicrophoneAudio(mic_samples, output_samples, sample_count);
    } catch (...) {
        return false;
    }
}

bool wq_aec3_convertor_convert_clean_audio_to_wav_bytes(const uint8_t** audio_frames,
                                                        const int* frame_sizes,
                                                        int frame_count,
                                                        int input_sample_rate,
                                                        int output_sample_rate,
                                                        uint8_t** wav_data,
                                                        int* wav_size) {
    if (!audio_frames || !frame_sizes || !wav_data || !wav_size || frame_count <= 0) {
        return false;
    }
    
    try {
        // Convert audio frames to the format expected by the C++ converter
        std::vector<std::vector<uint8_t>> audioFramesBytes;
        audioFramesBytes.reserve(frame_count);
        
        for (int i = 0; i < frame_count; i++) {
            if (!audio_frames[i] || frame_sizes[i] <= 0) continue;
            
            std::vector<uint8_t> frame(audio_frames[i], audio_frames[i] + frame_sizes[i]);
            audioFramesBytes.push_back(std::move(frame));
        }
        
        if (audioFramesBytes.empty()) return false;
        
        // Use the C++ converter
        size_t output_size = 0;
        int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToWAVBytes(
            audioFramesBytes, input_sample_rate, wav_data, &output_size, output_sample_rate);
        
        if (result == 0 && *wav_data && output_size > 0) {
            *wav_size = static_cast<int>(output_size);
            return true;
        }
        
        return false;
    } catch (...) {
        return false;
    }
}

void wq_aec3_processor_destroy(WqAec3ProcessorHandle handle) {
    if (!handle) return;
    
    try {
        auto* processor = static_cast<webrtc_aec3_tts::WqAec3Processor*>(handle);
        delete processor;
    } catch (...) {
        // Ignore exceptions in destructor
    }
}

} // extern "C"
