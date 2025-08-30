#ifndef TTS_AEC3_PROCESSOR_H
#define TTS_AEC3_PROCESSOR_H

#include <memory>
#include <vector>
#include <mutex>
#include <chrono>
#include <queue>
#include <deque>
#include <algorithm>
#include <cmath>
#include <atomic>
#include <thread>

#include "api/echo_canceller3_factory.h"
#include "api/echo_canceller3_config.h"
#include "audio_processing/audio_buffer.h"
#include "audio_processing/audio_frame.h"
#include "audio_processing/high_pass_filter.h"
#include "oboe/Oboe.h"

namespace webrtc_aec3_tts {

// 用于最优ERLE的增强时序同步缓冲区
struct TimedFrame {
    std::vector<int16_t> data;
    std::chrono::high_resolution_clock::time_point timestamp;
    uint64_t frame_id;
    
    TimedFrame(const int16_t* samples, size_t size, uint64_t id);
};

/**
 * 用于TTS回声消除的WebRTC AEC3处理器
 * 
 * 该类为TTS（文本转语音）应用提供生产级声学回声消除功能，
 * 使用WebRTC AEC3算法进行了特别优化。
 * 
 * 主要特性：
 * - 增强的ERLE性能（目标>12dB vs 标准6.2dB）
 * - 通用Android设备兼容性
 * - 精确时序同步
 * - 移动开发者参数控制
 * - 生产就绪的稳定性
 */
class WqAec3Processor : public oboe::AudioStreamCallback {
public:
    // 音频配置常量
    static constexpr int kSampleRate = 48000;
    static constexpr int kFrameSize = 480;  // 48kHz下10ms
    static constexpr int kChannels = 1;     // 单声道
    static constexpr int kStreamDelay = 100; // Android典型延迟

    // 增强ERLE优化常量
    static constexpr int kMaxDelayMs = 500; // 最大延迟时间（毫秒）
    static constexpr int kMinDelayMs = 20;  // 最小延迟时间（毫秒）
    static constexpr int kDelayBufferSize = kMaxDelayMs * kSampleRate / 1000 / kFrameSize; // 延迟缓冲区大小
    static constexpr double kTimingToleranceMs = 2.0; // 时序容忍度（毫秒）
    static constexpr int kDelayEstimationFrames = 50; // 延迟估计帧数
    static constexpr int kInitializationFrames = 100; // 初始化帧数

    /**
     * 使用优化默认参数的构造函数
     * 基于增强性能的调整结果
     */
    WqAec3Processor();
    
    /**
     * 析构函数 - 清理所有资源
     */
    ~WqAec3Processor();
    
    // 禁用拷贝构造函数和赋值操作符（因为包含atomic变量）
    WqAec3Processor(const WqAec3Processor&) = delete;
    WqAec3Processor& operator=(const WqAec3Processor&) = delete;

    // ========== 核心AEC3方法 ==========
    
    /**
     * 使用增强配置初始化AEC3处理器
     * @return 初始化成功则返回true
     */
    bool Initialize();

    /**
     * 处理TTS音频（参考信号）
     * 在通过扬声器播放TTS音频之前调用此方法
     * @param tts_data TTS音频样本（长度必须为kFrameSize）
     * @param length 样本数量（必须等于kFrameSize）
     * @return 处理成功则返回true
     */
    bool ProcessTtsAudio(const int16_t* tts_data, size_t length);

    /**
     * 处理麦克风音频并移除回声
     * @param mic_data 麦克风输入样本（长度必须为kFrameSize）
     * @param output_data 处理后音频的输出缓冲区（长度必须为kFrameSize）
     * @param length 样本数量（必须等于kFrameSize）
     * @return 处理成功则返回true
     */
    bool ProcessMicrophoneAudio(const int16_t* mic_data, int16_t* output_data, size_t length);

    /**
     * 处理TTS音频（参考信号）- 字节数组版本
     * 在通过扬声器播放TTS音频之前调用此方法
     * @param tts_byte_data TTS音频字节数据（长度必须为kFrameSize * 2）
     * @param byte_length 字节数量（必须等于kFrameSize * 2）
     * @return 处理成功则返回true
     */
    bool ProcessTtsAudioBytes(const uint8_t* tts_byte_data, size_t byte_length);

    /**
     * 处理麦克风音频并移除回声 - 字节数组版本
     * @param mic_byte_data 麦克风输入字节数据（长度必须为kFrameSize * 2）
     * @param output_byte_data 处理后音频的输出字节缓冲区（长度必须为kFrameSize * 2）
     * @param byte_length 字节数量（必须等于kFrameSize * 2）
     * @param enableAEC 是否启用AEC处理（true=启用AEC，false=直接返回原始音频）
     * @return 处理成功则返回true
     */
    bool ProcessMicrophoneAudioBytes(const uint8_t* mic_byte_data, uint8_t* output_byte_data, size_t byte_length, bool enableAEC);

    /**
     * 获取当前AEC性能指标
     * @param echo_return_loss 输出：ERL值
     * @param echo_return_loss_enhancement 输出：ERLE值
     * @param delay_ms 输出：检测到的延迟毫秒数
     * @return 成功检索指标则返回true
     */
    bool GetMetrics(double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms);

    /**
     * 获取带有详细信息的增强AEC性能指标
     * @param echo_return_loss 输出：ERL值
     * @param echo_return_loss_enhancement 输出：ERLE值
     * @param delay_ms 输出：检测到的延迟毫秒数
     * @param render_frames 输出：已处理的TTS帧总数
     * @param capture_frames 输出：已处理的麦克风帧总数
     * @param optimal_delay 输出：当前最优延迟设置
     * @return 成功检索指标则返回true
     */
    bool GetEnhancedMetrics(double* echo_return_loss, double* echo_return_loss_enhancement, 
                           int* delay_ms, uint64_t* render_frames, uint64_t* capture_frames, 
                           int* optimal_delay);

    // ========== 配置方法 ==========
    
    /**
     * 更新流延迟补偿
     * @param delay_ms 延迟毫秒数（Android通常为80-150ms）
     */
    void SetStreamDelay(int delay_ms);
    
    /**
     * 启用或禁用精确时序同步
     * @param enable true启用时序同步，false禁用
     * @return 设置成功应用则返回true
     */
    bool EnableTimingSync(bool enable);
    
    /**
     * 自动优化延迟以获得最大ERLE性能
     * @return 优化成功完成则返回true
     */
    bool AutoOptimizeDelay();

    // ========== 官方AEC3参数控制 ==========
    
    // 滤波器配置方法
    void SetConfigChangeDuration(int blocks);
    void SetInitialStateSeconds(float seconds);
    void SetConservativeInitialPhase(bool enable);
    
    // 抑制器正常调优方法
    void SetMaxDecFactorLF(float factor);
    void SetMaxIncFactor(float factor);
    
    // 抑制器近端调优方法
    void SetNearendMaxDecFactorLF(float factor);
    void SetNearendMaxIncFactor(float factor);
    
    // 主导近端检测方法
    void SetEnrThreshold(float threshold);
    void SetSnrThreshold(float threshold);
    void SetHoldDuration(int duration);
    void SetTriggerThreshold(int threshold);

    // ========== 移动开发者ERLE调整参数 ==========
    
    /**
     * 设置回声学习的滤波器长度块数
     * @param blocks 1-100范围，默认=25
     */
    void SetFilterLengthBlocks(int blocks);
    
    /**
     * 设置收敛时的滤波器泄漏以保持稳定性
     * @param leakage 0.000001-1.0范围，默认=0.000005
     */
    void SetFilterLeakageConverged(float leakage);
    
    /**
     * 设置发散时的滤波器泄漏以进行恢复
     * @param leakage 0.001-1.0范围，默认=0.005
     */
    void SetFilterLeakageDiverged(float leakage);
    
    /**
     * 设置延迟估计下采样因子以提高精度
     * @param factor 1-8范围，默认=2
     */
    void SetDelayDownSamplingFactor(int factor);
    
    /**
     * 设置延迟估计滤波器数量
     * @param filters 1-32范围，默认=16
     */
    void SetDelayNumFilters(int filters);
    
    /**
     * 设置延迟估计平滑因子以保持稳定性
     * @param smoothing 0.1-0.99范围，默认=0.98
     */
    void SetDelayEstimateSmoothing(float smoothing);

    // ========== C++ Oboe PCM播放方法 ==========
    
    /**
     * 初始化Oboe音频流用于C++级别的PCM播放
     * @return 初始化成功则返回true
     */
    bool InitializeOboePlayback();
    
    /**
     * 开始C++级别的PCM块播放，实现精确时序同步
     * @param pcm_chunks_path PCM块文件的路径（assets目录）
     * @param min_buffer_chunks 开始播放前需要准备的最小PCM块数量（默认100块，约1秒缓冲）
     * @return 播放开始成功则返回true
     */
    bool StartCppPcmPlayback(const std::string& pcm_chunks_path, int min_buffer_chunks = 100);
    
    /**
     * 停止C++级别的PCM播放
     */
    void StopCppPcmPlayback();
    
    /**
     * 检查C++播放是否正在进行
     * @return 正在播放则返回true
     */
    bool IsCppPlaybackActive() const;
    
    /**
     * 获取当前播放的PCM块索引
     * @return 当前播放的块索引
     */
    int GetCurrentPlaybackChunkIndex() const;

private:
    // 内部实现方法
    double CalculateFrameEnergy(const int16_t* samples, size_t length) const;
    const TimedFrame* FindOptimalReferenceFrame(const std::chrono::high_resolution_clock::time_point& capture_time);
    int EstimateOptimalDelay(const std::chrono::high_resolution_clock::time_point& capture_time,
                           const std::chrono::high_resolution_clock::time_point& render_time);
    void PerformDelayEstimationOptimization();
    int GetTimingBasedDelayEstimate();
    
    // 核心WebRTC组件
    std::mutex mutex_;
    std::unique_ptr<webrtc::EchoCanceller3Factory> aec_factory_;
    std::unique_ptr<webrtc::EchoControl> echo_controller_;
    std::unique_ptr<webrtc::AudioBuffer> audio_render_buffer_;
    std::unique_ptr<webrtc::AudioBuffer> audio_capture_buffer_;
    std::unique_ptr<webrtc::HighPassFilter> high_pass_filter_;
    
    // 增强时序同步
    std::deque<TimedFrame> render_buffer_;
    uint64_t frame_counter_;
    int last_delay_estimation_;
    int current_optimal_delay_ms_;
    int delay_estimation_counter_;
    uint64_t total_render_frames_;
    uint64_t total_capture_frames_;
    bool timing_sync_enabled_;
    
    // 初始化稳定化
    int initialization_frames_;
    bool is_initialization_complete_;
    
    // 自适应延迟管理
    int current_delay_ms_;
    int manual_delay_ms_;
    
    // 配置参数（运行时可调整）
    int config_change_duration_blocks_;
    float initial_state_seconds_;
    bool conservative_initial_phase_;
    float max_dec_factor_lf_;
    float max_inc_factor_;
    float nearend_max_dec_factor_lf_;
    float nearend_max_inc_factor_;
    float enr_threshold_;
    float snr_threshold_;
    int hold_duration_;
    int trigger_threshold_;
    
    // 移动开发者ERLE调整参数
    int filter_length_blocks_;
    float filter_leakage_converged_;
    float filter_leakage_diverged_;
    int delay_down_sampling_factor_;
    int delay_num_filters_;
    float delay_estimate_smoothing_;
    
    // 实时清洁音频缓冲系统
    std::vector<std::vector<float>> clean_audio_buffer_;
    std::mutex clean_audio_buffer_mutex_;
    
    // C++ PCM播放相关成员变量
    std::shared_ptr<oboe::AudioStream> oboe_playback_stream_;
    std::vector<std::vector<int16_t>> pcm_chunks_;
    std::mutex pcm_chunks_mutex_;
    std::atomic<bool> cpp_playback_active_{false};
    std::atomic<int> current_chunk_index_{0};
    std::atomic<int> current_chunk_offset_{0};
    std::atomic<int> min_buffer_chunks_{100};
    std::atomic<bool> async_loading_active_{false};
    std::thread playback_thread_;
    std::thread async_loading_thread_;
    
    // Oboe播放内部方法
    void PlaybackThreadFunction();
    bool LoadPcmChunks(const std::string& chunks_path);
    
    // Oboe AudioStreamCallback interface implementation
    oboe::DataCallbackResult onAudioReady(oboe::AudioStream* audioStream, void* audioData, int32_t numFrames) override;
    
public:
    // PCM块管理方法
    bool SetPcmChunks(const std::vector<std::vector<int16_t>>& chunks);
    bool AddPcmChunk(const std::vector<int16_t>& chunk);

private:
    

    
public:
    /**
     * 获取累积的清洁音频帧并清除缓冲区
     * @param outputFrames 接收清洁音频帧的输出向量
     * @return 检索到的帧数量
     */
    size_t GetAndClearCleanAudioBuffer(std::vector<std::vector<float>>& outputFrames);
    
    /**
     * 获取累积的清洁音频帧而不清除缓冲区
     * @param audioFrames 接收清洁音频帧的输出向量
     * @return 检索到的帧数量
     */
    size_t GetCleanAudioBuffer(std::vector<std::vector<float>>& audioFrames);
    
    /**
     * 清除清洁音频缓冲区
     */
    void ClearCleanAudioBuffer();
    
    // 字节数组转换工具方法
    bool ConvertByteArrayToShortArray(const uint8_t* byte_data, size_t byte_length, 
                                     int16_t* short_data, size_t expected_short_length) const;
    bool ConvertShortArrayToByteArray(const int16_t* short_data, size_t short_length,
                                     uint8_t* byte_data, size_t expected_byte_length) const;

};

} // namespace webrtc_aec3_tts

#endif // TTS_AEC3_PROCESSOR_H
