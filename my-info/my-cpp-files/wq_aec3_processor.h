#ifndef WQ_AEC3_PROCESSOR_H_
#define WQ_AEC3_PROCESSOR_H_

#include <memory>
#include <vector>
#include <mutex>

// Official WebRTC AEC3 API includes
#include "api/echo_canceller3_factory.h"
#include "api/echo_canceller3_config.h"
#include "api/echo_control.h"
#include "audio_processing/audio_buffer.h"
#include "audio_processing/high_pass_filter.h"

namespace webrtc_aec3_tts {

/**
 * WebRTC AEC3 Processor - Clean implementation using official WebRTC API
 * 
 * This class provides a simplified interface to WebRTC's official AEC3 implementation
 * for TTS echo cancellation applications. It uses the official EchoControl API
 * and EchoCanceller3Factory as demonstrated in the WebRTC demo.
 */
class WqAec3Processor {
public:
    // Audio configuration constants
    static constexpr int kSampleRateHz = 48000;
    static constexpr int kChannels = 1;
    static constexpr int kFrameSize = 480;  // 10ms at 48kHz (samples_per_frame = sample_rate / 100)
    static constexpr int kBitsPerSample = 16;      // 16-bit PCM

    WqAec3Processor();
    ~WqAec3Processor();

    // Core AEC3 operations
    bool Initialize();
    void Destroy();
    
    // Audio processing methods (following official WebRTC demo pattern)
    bool ProcessRenderAudio(const int16_t* render_data, size_t length);
    bool ProcessCaptureAudio(const int16_t* capture_data, int16_t* output_data, size_t length);
    
    // Performance metrics
    bool GetMetrics(double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms);
    void SetAudioBufferDelay(int delay_ms);
    
    // Clean audio buffer management for real-time output
    void AddCleanAudioFrame(const float* audio_data, size_t length);
    size_t GetCleanAudioBuffer(std::vector<std::vector<float>>& audio_frames);
    void ClearCleanAudioBuffer();
    
    // Get clean audio as byte array for WAV file generation
    std::vector<uint8_t> GetCleanAudioAsBytes();

private:
    // Official WebRTC AEC3 components
    std::unique_ptr<webrtc::EchoCanceller3Factory> aec3_factory_;
    std::unique_ptr<webrtc::EchoControl> echo_controller_;
    std::unique_ptr<webrtc::HighPassFilter> high_pass_filter_;
    
    // Audio buffers (following demo.cc pattern)
    std::unique_ptr<webrtc::AudioBuffer> render_audio_buffer_;
    std::unique_ptr<webrtc::AudioBuffer> capture_audio_buffer_;
    std::unique_ptr<webrtc::AudioBuffer> linear_output_buffer_;
    
    // Configuration
    webrtc::EchoCanceller3Config aec3_config_;
    webrtc::StreamConfig stream_config_;
    
    // Clean audio buffer for real-time output
    std::vector<std::vector<float>> clean_audio_frames_;
    mutable std::mutex clean_audio_mutex_;
    
    // Internal state
    bool initialized_;
    
    // Helper methods
    bool InitializeAudioBuffers();
    void ProcessAudioFrame(webrtc::AudioBuffer* buffer);
};

} // namespace webrtc_aec3_tts

#endif // WQ_AEC3_PROCESSOR_H_