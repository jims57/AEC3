#ifndef WQ_AEC3_PLAYER_H
#define WQ_AEC3_PLAYER_H

#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <memory>

#ifdef __ANDROID__
#include "oboe/Oboe.h"
#include <jni.h>
#endif

// Forward declaration
namespace webrtc_aec3_tts {
    class WqAec3Processor;
}

namespace webrtc_aec3_tts {

/**
 * WqAec3Player - PCM播放器类，专门用于WebRTC AEC3的TTS参考信号播放
 * 提供精确的时序控制和与AEC3处理器的集成
 * 使用Oboe (Android) 和 AudioToolbox (iOS) 实现跨平台音频播放
 * 日期: 2025-08-29
 */
class WqAec3Player 
#ifdef __ANDROID__
    : public oboe::AudioStreamDataCallback
#endif
{
public:
    WqAec3Player();
    ~WqAec3Player();
    
    /**
     * 设置AEC3处理器引用
     * @param processor AEC3处理器实例
     */
    void SetAec3Processor(WqAec3Processor* processor);
    
    /**
     * 设置JavaVM引用，用于Android初始化
     * @param vm JavaVM实例
     */
    void SetJavaVM(JavaVM* vm);
    
    /**
     * 播放PCM块数据，提供精确的时序控制
     * @param pcm_chunks PCM数据块向量
     * @param buffer_chunks 缓冲区块数量，默认为3
     * @param delay_ms 播放延迟毫秒数，用于精确时序同步，默认为0
     * @return 播放成功则返回true
     */
    bool PlayPcmChunks(const std::vector<std::vector<uint8_t>>& pcm_chunks, int buffer_chunks = 3, int delay_ms = 0);
    
    /**
     * 停止PCM播放
     */
    void StopPcmPlayback();
    
    /**
     * 检查是否正在播放
     * @return 正在播放则返回true
     */
    bool IsPlaying() const;

#ifdef __ANDROID__
    // Oboe callback for Android
    oboe::DataCallbackResult onAudioReady(oboe::AudioStream *audioStream, void *audioData, int32_t numFrames) override;
#endif

private:
    // AEC3处理器引用
    WqAec3Processor* aec3_processor_;
    
    // PCM播放状态管理
    std::atomic<bool> is_pcm_playing_;
    std::thread pcm_playback_thread_;
    std::vector<std::vector<uint8_t>> pcm_chunks_buffer_;
    std::mutex pcm_playback_mutex_;
    
    // 跨平台音频缓冲区
    std::vector<int16_t> audio_buffer_;
    std::mutex audio_buffer_mutex_;
    size_t buffer_read_position_;
    size_t buffer_write_position_;
    
#ifdef __ANDROID__
    // Android Oboe相关
    std::shared_ptr<oboe::AudioStream> audio_stream_;
    JavaVM* java_vm_;
    
    // 初始化和清理Oboe
    bool InitializeOboeStream();
    void CleanupOboeStream();
#endif
    
    // 跨平台方法
    void FillAudioBuffer(const std::vector<int16_t>& frame_data);
    bool GetAudioData(int16_t* output, int32_t numFrames);
};

} // namespace webrtc_aec3_tts

#endif // WQ_AEC3_PLAYER_H