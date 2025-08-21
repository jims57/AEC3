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

#include "api/echo_canceller3_factory.h"
#include "api/echo_canceller3_config.h"
#include "audio_processing/audio_buffer.h"
#include "audio_processing/audio_frame.h"
#include "audio_processing/high_pass_filter.h"

namespace webrtc_aec3_tts {

// 用于最优ERLE的增强时序同步缓冲区
struct TimedFrame {
    std::vector<int16_t> data;
    std::chrono::high_resolution_clock::time_point timestamp;
    uint64_t frame_id;
    
    TimedFrame(const int16_t* samples, size_t size, uint64_t id);
};

// 🎯 Production-Grade ERLE Performance Metrics (2025-01-31)
struct ProductionErleMetrics {
    double erl_estimate;                    // Echo Return Loss estimate from WebRTC ErlEstimator
    double erle_fullband_log2;             // Fullband ERLE in log2 scale from ErleEstimator
    double erle_subband_average;           // Average subband ERLE performance
    double linear_filter_quality;         // Linear filter quality estimate [0-1]
    int matched_filter_delay_samples;     // Delay from matched filter estimator
    bool delay_estimate_reliable;         // Whether delay estimate is reliable
    double clockdrift_level;              // Clock drift detection level
    bool filter_converged;                // Whether adaptive filter has converged
    double timing_sync_accuracy_ms;       // Timing synchronization accuracy
};

/**
 * 用于TTS回声消除的WebRTC AEC3处理器 (2025-01-31)
 * 
 * 该类为TTS（文本转语音）应用提供生产级声学回声消除功能，
 * 使用WebRTC AEC3算法进行了特别优化，集成了内置的ERL/ERLE估计器。
 * 
 * 🎯 Production-Grade Features (2025-01-31):
 * - 集成WebRTC内置ErlEstimator和ErleEstimator实现生产级ERLE性能
 * - 使用EchoPathDelayEstimator进行精确时序同步和延迟估计
 * - 自适应滤波器收敛监控和优化
 * - 实时时钟漂移检测和补偿
 * - 生产环境稳定性和跨设备兼容性
 * 
 * 主要特性：
 * - 增强的ERLE性能（目标>15dB vs 标准6.2dB）
 * - WebRTC内置估计器集成优化
 * - 精确时序同步和延迟估计
 * - 移动开发者参数控制
 * - 生产就绪的稳定性
 */
class WqAec3Processor {
public:
    // 音频配置常量
    static constexpr int kSampleRate = 48000;
    static constexpr int kFrameSize = 480;  // 48kHz下10ms
    static constexpr int kChannels = 1;     // 单声道
    static constexpr int kStreamDelay = 100; // Android典型延迟

    // 增强ERLE优化常量
    static constexpr int kMaxDelayMs = 500;
    static constexpr int kMinDelayMs = 20;
    static constexpr int kDelayBufferSize = kMaxDelayMs * kSampleRate / 1000 / kFrameSize;
    static constexpr double kTimingToleranceMs = 1.0;    // Tighter timing tolerance
    static constexpr int kDelayEstimationFrames = 25;    // More frequent delay estimation
    static constexpr int kInitializationFrames = 150;    // Extended initialization for stability
    static constexpr double kTargetErleDb = 15.0;        // Production-grade ERLE target
    static constexpr double kMinAcceptableErleDb = 8.0;  // Minimum acceptable ERLE
    static constexpr int kErleMonitoringFrames = 100;    // ERLE monitoring frequency

    /**
     * 使用生产级优化参数的构造函数
     * 基于WebRTC内置估计器的增强性能调整
     */
    WqAec3Processor();
    
    /**
     * 析构函数 - 清理所有资源
     */
    ~WqAec3Processor();

    // ========== 核心AEC3方法 ==========
    
    /**
     * 使用生产级配置初始化AEC3处理器
     * 集成WebRTC内置ERL/ERLE估计器和延迟估计器
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
    
    /**
     * 获取生产级ERLE性能指标
     * 集成WebRTC内置ErlEstimator和ErleEstimator的详细性能数据
     * @return 生产级ERLE性能指标结构体
     */
    ProductionErleMetrics GetProductionErleMetrics();
    
    /**
     * 执行生产级ERLE性能优化
     * 基于WebRTC内置估计器进行自适应优化
     * @return 优化成功则返回true
     */
    bool OptimizeProductionErlePerformance();
    
    /**
     * 启用/禁用生产级精确时序同步
     * 使用WebRTC EchoPathDelayEstimator进行精确延迟估计
     * @param enable_precise_sync 是否启用精确时序同步
     * @param enable_clockdrift_detection 是否启用时钟漂移检测
     * @return 设置成功则返回true
     */
    bool EnableProductionTimingSync(bool enable_precise_sync, bool enable_clockdrift_detection);
    
    /**
     * 强制重新校准延迟估计
     * 使用WebRTC内置延迟估计器重新校准
     * @return 重新校准成功则返回true
     */
    bool RecalibrateDelayEstimation();

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

private:
    // 内部实现方法
    double CalculateFrameEnergy(const int16_t* samples, size_t length) const;
    const TimedFrame* FindOptimalReferenceFrame(const std::chrono::high_resolution_clock::time_point& capture_time);
    int EstimateOptimalDelay(const std::chrono::high_resolution_clock::time_point& capture_time,
                           const std::chrono::high_resolution_clock::time_point& render_time);
    void PerformDelayEstimationOptimization();
    int GetTimingBasedDelayEstimate();
    
    // 🎯 WebRTC Built-in Estimator Integration Methods
    void UpdateProductionErleEstimates();
    void MonitorAdaptiveFilterConvergence();
    void OptimizeBasedOnErleEstimates();
    void HandleClockdriftDetection();
    bool ValidateTimingSynchronization();
    void AdaptDelayBasedOnPathEstimator();

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
    
    // 🎯 Production-Grade ERLE Monitoring Variables
    ProductionErleMetrics current_production_metrics_;
    std::mutex production_metrics_mutex_;
    int erle_monitoring_counter_;
    bool production_timing_sync_enabled_;
    bool clockdrift_detection_enabled_;
    double last_erle_fullband_log2_;
    std::vector<double> erle_history_;
    int adaptive_filter_convergence_counter_;
    bool is_adaptive_filter_converged_;
    
    // 实时清洁音频缓冲系统 (2025-01-31)
    std::vector<std::vector<float>> clean_audio_buffer_;
    std::mutex clean_audio_buffer_mutex_;
    
public:
    /**
     * 获取累积的清洁音频帧并清除缓冲区
     * @param outputFrames 接收清洁音频帧的输出向量
     * @return 检索到的帧数量
     */
    size_t GetAndClearCleanAudioBuffer(std::vector<std::vector<float>>& outputFrames);
    
    /**
     * 获取累积的清洁音频帧而不清除缓冲区
     * @param outputFrames 接收清洁音频帧的输出向量
     * @return 检索到的帧数量
     */
    size_t GetCleanAudioBuffer(std::vector<std::vector<float>>& outputFrames);
    
    /**
     * 清除清洁音频缓冲区
     */
    void ClearCleanAudioBuffer();
};

} // namespace webrtc_aec3_tts

#endif // TTS_AEC3_PROCESSOR_H
