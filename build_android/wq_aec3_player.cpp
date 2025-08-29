#include "wq_aec3_player.h"
#include "wq_aec3_processor.h"
#include <android/log.h>
#include <chrono>

#ifdef __ANDROID__
#include <jni.h>
#endif

#define LOG_TAG "WebRTC_AEC3_Player"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, LOG_TAG, __VA_ARGS__)

namespace webrtc_aec3_tts {

WqAec3Player::WqAec3Player() : 
    aec3_processor_(nullptr),
    is_pcm_playing_(false),
    buffer_read_position_(0),
    buffer_write_position_(0)
#ifdef __ANDROID__
    , java_vm_(nullptr)
#endif
{
    // 初始化音频缓冲区 (48kHz * 2秒 * mono)
    audio_buffer_.resize(48000 * 2, 0);
}

WqAec3Player::~WqAec3Player() {
    StopPcmPlayback();
#ifdef __ANDROID__
    CleanupOboeStream();
#endif
}

void WqAec3Player::SetAec3Processor(WqAec3Processor* processor) {
    aec3_processor_ = processor;
}

void WqAec3Player::SetJavaVM(JavaVM* vm) {
    java_vm_ = vm;
}

bool WqAec3Player::PlayPcmChunks(const std::vector<std::vector<uint8_t>>& pcm_chunks, int buffer_chunks, int delay_ms) {
    if (pcm_chunks.empty()) {
        LOGE("PlayPcmChunks: No PCM chunks provided");
        return false;
    }
    
    if (!aec3_processor_) {
        LOGE("PlayPcmChunks: AEC3 processor not set");
        return false;
    }
    
    // 停止任何现有的播放
    StopPcmPlayback();
    
    {
        std::lock_guard<std::mutex> lock(pcm_playback_mutex_);
        pcm_chunks_buffer_ = pcm_chunks;
        is_pcm_playing_ = true;
    }
    
    LOGI("Starting C++ PCM playback with %zu chunks, buffer_chunks=%d, delay_ms=%d", pcm_chunks.size(), buffer_chunks, delay_ms);
    
#ifdef __ANDROID__
    // 初始化Oboe音频流用于实际音频播放
    if (!InitializeOboeStream()) {
        LOGE("Failed to initialize Oboe audio stream");
        return false;
    }
#endif
    
    // 启动播放线程
    pcm_playback_thread_ = std::thread([this, buffer_chunks, delay_ms]() {
        try {
            // 使用简化的PCM播放逻辑，专注于时序同步
            const int kSampleRate = 48000;
            const int kFrameSize = 480; // 10ms at 48kHz
            const int kBytesPerSample = 2; // 16-bit PCM
            const int kFrameSizeBytes = kFrameSize * kBytesPerSample;
            
            size_t chunk_index = 0;
            size_t chunk_offset = 0;
            
            // 应用初始延迟以实现精确时序同步
            if (delay_ms > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
                LOGV("Applied initial delay: %dms for timing synchronization", delay_ms);
            }
            
            while (is_pcm_playing_ && chunk_index < pcm_chunks_buffer_.size()) {
                const auto& current_chunk = pcm_chunks_buffer_[chunk_index];
                
                // 确保有足够的数据
                if (chunk_offset + kFrameSizeBytes > current_chunk.size()) {
                    chunk_index++;
                    chunk_offset = 0;
                    continue;
                }
                
                // 提取480样本帧
                std::vector<int16_t> frame_data(kFrameSize);
                for (int i = 0; i < kFrameSize; ++i) {
                    size_t byte_pos = chunk_offset + i * 2;
                    if (byte_pos + 1 < current_chunk.size()) {
                        // Little-endian 16-bit PCM
                        frame_data[i] = static_cast<int16_t>(
                            current_chunk[byte_pos] | 
                            (static_cast<int16_t>(current_chunk[byte_pos + 1]) << 8)
                        );
                    }
                }
                
                // 将帧数据提供给AEC3作为参考信号
                if (aec3_processor_) {
                    aec3_processor_->ProcessTtsAudio(frame_data.data(), kFrameSize);
                    LOGV("C++ PCM: Processed frame %zu/%zu, chunk %zu", 
                         chunk_offset/kFrameSizeBytes, current_chunk.size()/kFrameSizeBytes, chunk_index);
                }
                
                // 将音频数据填充到缓冲区供Oboe回调使用
                FillAudioBuffer(frame_data);
                
                chunk_offset += kFrameSizeBytes;
                
                // 10ms播放间隔 (与帧大小匹配)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            LOGI("C++ PCM playback completed successfully");
            
        } catch (const std::exception& e) {
            LOGE("C++ PCM playback error: %s", e.what());
        }
        
        is_pcm_playing_ = false;
    });
    
    return true;
}

void WqAec3Player::StopPcmPlayback() {
    is_pcm_playing_ = false;
    
#ifdef __ANDROID__
    // 停止Oboe音频流
    if (audio_stream_) {
        audio_stream_->stop();
    }
#endif
    
    if (pcm_playback_thread_.joinable()) {
        pcm_playback_thread_.join();
    }
    
    {
        std::lock_guard<std::mutex> lock(pcm_playback_mutex_);
        pcm_chunks_buffer_.clear();
    }
    
#ifdef __ANDROID__
    CleanupOboeStream();
#endif
    LOGI("C++ PCM playback stopped");
}

bool WqAec3Player::IsPlaying() const {
    return is_pcm_playing_;
}

#ifdef __ANDROID__
bool WqAec3Player::InitializeOboeStream() {
    oboe::AudioStreamBuilder builder;
    
    // 配置Oboe音频流
    builder.setDirection(oboe::Direction::Output)
           ->setPerformanceMode(oboe::PerformanceMode::LowLatency)
           ->setSharingMode(oboe::SharingMode::Exclusive)
           ->setFormat(oboe::AudioFormat::I16)
           ->setChannelCount(1)  // Mono
           ->setSampleRate(48000)
           ->setDataCallback(this)
           ->setFramesPerDataCallback(480); // 10ms frames
    
    oboe::Result result = builder.openStream(audio_stream_);
    if (result != oboe::Result::OK) {
        LOGE("Failed to create Oboe stream: %s", oboe::convertToText(result));
        return false;
    }
    
    // 启动音频流
    result = audio_stream_->requestStart();
    if (result != oboe::Result::OK) {
        LOGE("Failed to start Oboe stream: %s", oboe::convertToText(result));
        audio_stream_.reset();
        return false;
    }
    
    LOGI("Oboe stream initialized successfully: 48kHz, mono, 16-bit, low-latency");
    return true;
}

void WqAec3Player::CleanupOboeStream() {
    if (audio_stream_) {
        audio_stream_->stop();
        audio_stream_->close();
        audio_stream_.reset();
    }
}

oboe::DataCallbackResult WqAec3Player::onAudioReady(oboe::AudioStream *audioStream, void *audioData, int32_t numFrames) {
    int16_t* outputData = static_cast<int16_t*>(audioData);
    
    // 从缓冲区获取音频数据
    if (GetAudioData(outputData, numFrames)) {
        return oboe::DataCallbackResult::Continue;
    } else {
        // 没有数据时输出静音
        std::fill(outputData, outputData + numFrames, 0);
        return oboe::DataCallbackResult::Continue;
    }
}
#endif

void WqAec3Player::FillAudioBuffer(const std::vector<int16_t>& frame_data) {
    std::lock_guard<std::mutex> lock(audio_buffer_mutex_);
    
    for (int16_t sample : frame_data) {
        audio_buffer_[buffer_write_position_] = sample;
        buffer_write_position_ = (buffer_write_position_ + 1) % audio_buffer_.size();
        
        // 防止写位置追上读位置
        if (buffer_write_position_ == buffer_read_position_) {
            buffer_read_position_ = (buffer_read_position_ + 1) % audio_buffer_.size();
        }
    }
}

bool WqAec3Player::GetAudioData(int16_t* output, int32_t numFrames) {
    std::lock_guard<std::mutex> lock(audio_buffer_mutex_);
    
    bool hasData = false;
    for (int32_t i = 0; i < numFrames; ++i) {
        if (buffer_read_position_ != buffer_write_position_) {
            output[i] = audio_buffer_[buffer_read_position_];
            buffer_read_position_ = (buffer_read_position_ + 1) % audio_buffer_.size();
            hasData = true;
        } else {
            output[i] = 0; // 静音
        }
    }
    
    return hasData;
}

} // namespace webrtc_aec3_tts