#include "wq_aec3_recorder.h"
#include "wq_aec3_processor.h"
#include <android/log.h>

#define LOG_TAG "WebRTC_AEC3_Rec"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace webrtc_aec3_tts {

WqAec3Recorder::WqAec3Recorder() {
    LOGI("🎙️ WqAec3Recorder constructor");
}

WqAec3Recorder::~WqAec3Recorder() {
    Destroy();
    LOGI("🎙️ WqAec3Recorder destructor");
}

bool WqAec3Recorder::Initialize() {
    if (is_initialized_.load()) {
        LOGV("🎙️ Recorder already initialized");
        return true;
    }
    
    LOGI("🎙️ Initializing Oboe recording stream...");
    
    oboe::AudioStreamBuilder builder;
    builder.setDirection(oboe::Direction::Input)
           ->setPerformanceMode(oboe::PerformanceMode::LowLatency)
           ->setSharingMode(oboe::SharingMode::Exclusive)
           ->setFormat(oboe::AudioFormat::I16)
           ->setChannelCount(kChannels)
           ->setSampleRate(kSampleRate)
           ->setFramesPerDataCallback(kFrameSize)
           ->setDataCallback(this)
           ->setUsage(oboe::Usage::VoiceCommunication)
           ->setContentType(oboe::ContentType::Speech)
           ->setInputPreset(oboe::InputPreset::VoiceCommunication);
    
    oboe::Result result = builder.openStream(recording_stream_);
    if (result != oboe::Result::OK) {
        LOGE("🎙️ Failed to create recording stream: %s", oboe::convertToText(result));
        return false;
    }
    
    LOGI("🎙️ Recording stream created successfully");
    LOGI("🎙️ Sample Rate: %d, Buffer Size: %d, Channels: %d", 
         recording_stream_->getSampleRate(),
         recording_stream_->getBufferSizeInFrames(),
         recording_stream_->getChannelCount());
    
    is_initialized_.store(true);
    return true;
}

void WqAec3Recorder::Destroy() {
    StopRecording();
    
    if (recording_stream_) {
        recording_stream_->close();
        recording_stream_.reset();
    }
    
    is_initialized_.store(false);
    LOGI("🎙️ Recorder destroyed");
}

bool WqAec3Recorder::StartRecording() {
    if (!is_initialized_.load()) {
        LOGE("🎙️ Recorder not initialized");
        return false;
    }
    
    if (is_recording_.load()) {
        LOGV("🎙️ Recording already active");
        return true;
    }
    
    // 清空之前的音频数据
    ClearAudioFrames();
    
    oboe::Result result = recording_stream_->requestStart();
    if (result != oboe::Result::OK) {
        LOGE("🎙️ Failed to start recording: %s", oboe::convertToText(result));
        return false;
    }
    
    is_recording_.store(true);
    
    // 启动自适应延迟优化
    if (adaptive_delay_enabled_.load() && aec3_processor_) {
        optimization_active_.store(true);
        delay_optimization_thread_ = std::thread(&WqAec3Recorder::OptimizeDelayThread, this);
    }
    
    LOGI("🎙️ C++ recording started with Oboe");
    return true;
}

void WqAec3Recorder::StopRecording() {
    if (!is_recording_.load()) {
        return;
    }
    
    is_recording_.store(false);
    
    // 停止延迟优化
    optimization_active_.store(false);
    if (delay_optimization_thread_.joinable()) {
        delay_optimization_thread_.join();
    }
    
    if (recording_stream_) {
        recording_stream_->requestStop();
    }
    
    LOGI("🎙️ C++ recording stopped");
}

std::vector<std::vector<int16_t>> WqAec3Recorder::GetCleanAudioFrames() {
    std::lock_guard<std::mutex> lock(clean_audio_mutex_);
    return clean_audio_frames_;
}

void WqAec3Recorder::ClearAudioFrames() {
    std::lock_guard<std::mutex> lock(clean_audio_mutex_);
    clean_audio_frames_.clear();
}

void WqAec3Recorder::SetAec3Processor(WqAec3Processor* processor) {
    aec3_processor_ = processor;
    LOGI("🎙️ AEC3 processor reference set for C++ recording");
}

oboe::DataCallbackResult WqAec3Recorder::onAudioReady(oboe::AudioStream* audioStream,
                                                      void* audioData,
                                                      int32_t numFrames) {
    if (!is_recording_.load() || !aec3_processor_) {
        return oboe::DataCallbackResult::Continue;
    }
    
    // 确保帧大小正确
    if (numFrames != kFrameSize) {
        LOGV("🎙️ Frame size mismatch: expected %d, got %d", kFrameSize, numFrames);
        return oboe::DataCallbackResult::Continue;
    }
    
    const int16_t* input_data = static_cast<const int16_t*>(audioData);
    ProcessAudioFrame(input_data, numFrames);
    
    return oboe::DataCallbackResult::Continue;
}

void WqAec3Recorder::ProcessAudioFrame(const int16_t* input_data, int32_t frame_count) {
    if (!aec3_processor_ || frame_count != kFrameSize) {
        return;
    }
    
    // 获取当前C++播放的TTS参考帧
    std::vector<int16_t> tts_frame(kFrameSize);
    bool has_tts_reference = aec3_processor_->GetCurrentPlaybackTtsFrame(tts_frame.data());
    
    if (has_tts_reference) {
        // 处理TTS参考信号
        aec3_processor_->ProcessTtsAudio(tts_frame.data(), kFrameSize);
    }
    
    // 处理音频帧进行AEC
    std::vector<int16_t> clean_output(kFrameSize);
    bool success = aec3_processor_->ProcessMicrophoneAudio(
        input_data, clean_output.data(), kFrameSize);
    
    if (success) {
        // 存储清洁音频帧
        std::lock_guard<std::mutex> lock(clean_audio_mutex_);
        clean_audio_frames_.push_back(clean_output);
        
        // 限制缓冲区大小（最多保存10秒音频）
        const size_t max_frames = kSampleRate / kFrameSize * 10; // 10秒
        if (clean_audio_frames_.size() > max_frames) {
            clean_audio_frames_.erase(clean_audio_frames_.begin());
        }
    }
}

void WqAec3Recorder::OptimizeDelayThread() {
    LOGI("🎯 Starting adaptive delay optimization thread");
    
    const int delay_step = 10; // 10ms步长
    const int min_delay = 50;  // 最小延迟50ms
    const int max_delay = 300; // 最大延迟300ms
    const int optimization_interval_ms = 2000; // 每2秒优化一次
    
    int current_test_delay = optimal_delay_ms_.load();
    
    while (optimization_active_.load() && is_recording_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(optimization_interval_ms));
        
        if (!aec3_processor_ || !optimization_active_.load()) {
            break;
        }
        
        // 获取当前ERLE指标
        double erl, erle;
        int delay_ms;
        uint64_t render_frames, capture_frames;
        int optimal_delay;
        
        if (aec3_processor_->GetEnhancedMetrics(&erl, &erle, &delay_ms, &render_frames, &capture_frames, &optimal_delay)) {
            if (erle > best_erle_.load()) {
                best_erle_.store(erle);
                optimal_delay_ms_.store(optimal_delay);
            }
        }
        
        LOGV("🎯 Delay optimization: current_delay=%dms, erle=%.2fdB, best_erle=%.2fdB", 
             current_test_delay, erle, best_erle_.load());
        
        // 尝试下一个延迟值
        if (erle < best_erle_.load() - 2.0) { // 如果ERLE下降超过2dB
            // 尝试减少延迟
            current_test_delay = std::max(min_delay, current_test_delay - delay_step);
        } else if (erle > best_erle_.load() + 1.0) { // 如果ERLE改善超过1dB
            // 尝试进一步优化
            current_test_delay = std::max(min_delay, current_test_delay - delay_step);
        } else {
            // 尝试小幅调整
            current_test_delay += (current_test_delay % 2 == 0) ? delay_step : -delay_step;
            current_test_delay = std::clamp(current_test_delay, min_delay, max_delay);
        }
        
        // 应用新的延迟设置
        if (aec3_processor_) {
            aec3_processor_->SetStreamDelay(current_test_delay);
            LOGV("🎯 Testing delay: %dms", current_test_delay);
        }
    }
    
    LOGI("🎯 Adaptive delay optimization completed. Optimal delay: %dms", optimal_delay_ms_.load());
}

} // namespace webrtc_aec3_tts