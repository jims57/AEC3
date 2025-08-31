#pragma once

#include <oboe/Oboe.h>
#include <memory>
#include <atomic>
#include <vector>
#include <mutex>
#include <thread>
#include <chrono>

namespace webrtc_aec3_tts {

/**
 * C++ Oboe录音器，用于高精度音频录制和AEC3处理
 * 提供与C++播放完美同步的录音功能
 */
class WqAec3Recorder : public oboe::AudioStreamDataCallback {
public:
    static constexpr int32_t kSampleRate = 48000;
    static constexpr int32_t kFrameSize = 480;  // 10ms at 48kHz
    static constexpr int32_t kChannels = 1;     // Mono
    
    WqAec3Recorder();
    ~WqAec3Recorder();
    
    // 初始化和清理
    bool Initialize();
    void Destroy();
    
    // 录音控制
    bool StartRecording();
    void StopRecording();
    bool IsRecording() const { return is_recording_.load(); }
    
    // 获取录制的清洁音频数据
    std::vector<std::vector<int16_t>> GetCleanAudioFrames();
    void ClearAudioFrames();
    
    // WAV文件保存
    std::vector<uint8_t> ConvertToWavBytes();
    int GetRecordedFrameCount() const;
    
    // 设置AEC3处理器引用（用于实时AEC处理）
    void SetAec3Processor(class WqAec3Processor* processor);
    
    // 自适应延迟优化
    void SetAdaptiveDelayEnabled(bool enabled) { adaptive_delay_enabled_ = enabled; }
    bool IsAdaptiveDelayEnabled() const { return adaptive_delay_enabled_; }
    
    // Oboe回调
    oboe::DataCallbackResult onAudioReady(oboe::AudioStream* audioStream,
                                         void* audioData,
                                         int32_t numFrames) override;
    
private:
    // Oboe录音流
    std::shared_ptr<oboe::AudioStream> recording_stream_;
    
    // 录音状态
    std::atomic<bool> is_recording_{false};
    std::atomic<bool> is_initialized_{false};
    
    // AEC3处理器引用
    class WqAec3Processor* aec3_processor_{nullptr};
    
    // 清洁音频缓冲
    std::vector<std::vector<int16_t>> clean_audio_frames_;
    mutable std::mutex clean_audio_mutex_;
    
    // 自适应延迟优化
    std::atomic<bool> adaptive_delay_enabled_{true};
    std::atomic<int> optimal_delay_ms_{100};
    std::atomic<double> best_erle_{-50.0};
    
    // 延迟优化线程
    std::thread delay_optimization_thread_;
    std::atomic<bool> optimization_active_{false};
    
    // 内部方法
    void OptimizeDelayThread();
    void ProcessAudioFrame(const int16_t* input_data, int32_t frame_count);
};

} // namespace webrtc_aec3_tts