//
//  WQAecProcessor.h
//  WQDenoiser
//
//  Created by Jimmy Gan on 2025/1/31.
//  Copyright © 2025 WatchFun. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>

NS_ASSUME_NONNULL_BEGIN

/**
 * AEC性能指标类 - 与Android版本保持一致
 */
@interface WQAecMetrics : NSObject

@property (nonatomic, readonly) double echoReturnLoss;
@property (nonatomic, readonly) double echoReturnLossEnhancement;
@property (nonatomic, readonly) NSInteger delayMs;

- (instancetype)initWithERL:(double)erl ERLE:(double)erle delay:(NSInteger)delay;
- (NSString *)description;
- (NSString *)erleQuality;

@end

/**
 * 增强AEC性能指标类 - 与Android版本保持一致
 */
@interface WQEnhancedAecMetrics : NSObject

@property (nonatomic, readonly) double echoReturnLoss;
@property (nonatomic, readonly) double echoReturnLossEnhancement;
@property (nonatomic, readonly) NSInteger delayMs;
@property (nonatomic, readonly) uint64_t renderFrames;
@property (nonatomic, readonly) uint64_t captureFrames;
@property (nonatomic, readonly) NSInteger optimalDelayMs;

- (instancetype)initWithERL:(double)erl 
                       ERLE:(double)erle 
                      delay:(NSInteger)delay 
               renderFrames:(uint64_t)renderFrames 
              captureFrames:(uint64_t)captureFrames 
               optimalDelay:(NSInteger)optimalDelay;
- (NSString *)description;
- (NSString *)erleQuality;
- (BOOL)isFrameSynchronized;

@end

/**
 * WebRTC AEC3 TTS回声消除处理器 - iOS版本
 * 与Android WqAecProcessor保持API一致性
 */
@interface WQAecProcessor : NSObject

// 音频配置常量 - 与Android保持一致
@property (class, nonatomic, readonly) NSInteger sampleRate;     // 48000
@property (class, nonatomic, readonly) NSInteger frameSize;     // 480
@property (class, nonatomic, readonly) NSInteger channels;      // 1
@property (class, nonatomic, readonly) NSInteger bitsPerSample; // 16

// ========== 核心AEC3方法 ==========

/**
 * 初始化AEC处理器
 * @return 如果成功则返回YES
 */
- (BOOL)initialize;

/**
 * 清理资源
 */
- (void)destroy;

/**
 * 处理TTS音频块（参考信号）
 * @param ttsData TTS音频数据（480个样本，16位PCM）
 * @return 处理成功返回YES
 */
- (BOOL)processTtsAudio:(NSData *)ttsData;

/**
 * 处理麦克风音频并移除回声
 * @param micData 麦克风输入数据（480个样本，16位PCM）
 * @param enableAEC 是否启用AEC处理
 * @return 回声消除后的音频数据，如果出错则返回nil
 */
- (nullable NSData *)processMicrophoneAudio:(NSData *)micData enableAEC:(BOOL)enableAEC;

/**
 * 获取AEC性能指标
 * @return WQAecMetrics对象，包含性能数据
 */
- (nullable WQAecMetrics *)getMetrics;

/**
 * 获取包括时序信息在内的增强型指标
 * @return WQEnhancedAecMetrics对象，包含详细性能数据
 */
- (nullable WQEnhancedAecMetrics *)getEnhancedMetrics;

/**
 * 调整流延迟以获得最佳性能
 * @param delayMs 延迟毫秒数
 */
- (void)setStreamDelay:(NSInteger)delayMs;

// ========== 高级功能 ==========

/**
 * 自动优化延迟以获得最佳ERLE性能
 * @return 如果优化成功返回YES
 */
- (BOOL)autoOptimizeDelay;

/**
 * 启用或禁用精确时间同步
 * @param enable 是否启用精确时间同步
 * @return 如果操作成功返回YES
 */
- (BOOL)enableTimingSync:(BOOL)enable;

// ========== AEC3参数控制方法 ==========
// 这些方法直接控制WebRTC AEC3的配置参数

// 滤波器配置方法
- (void)setConfigChangeDuration:(NSInteger)blocks;
- (void)setInitialStateSeconds:(float)seconds;
- (void)setConservativeInitialPhase:(BOOL)enable;

// 抑制器正常调优方法
- (void)setMaxDecFactorLF:(float)factor;
- (void)setMaxIncFactor:(float)factor;

// 抑制器近端调优方法
- (void)setNearendMaxDecFactorLF:(float)factor;
- (void)setNearendMaxIncFactor:(float)factor;

// 主导近端检测方法
- (void)setEnrThreshold:(float)threshold;
- (void)setSnrThreshold:(float)threshold;
- (void)setHoldDuration:(NSInteger)duration;
- (void)setTriggerThreshold:(NSInteger)threshold;

// ERLE调整参数方法
- (void)setFilterLengthBlocks:(NSInteger)blocks;
- (void)setFilterLeakageConverged:(float)leakage;
- (void)setFilterLeakageDiverged:(float)leakage;
- (void)setDelayDownSamplingFactor:(NSInteger)factor;
- (void)setDelayNumFilters:(NSInteger)filters;
- (void)setDelayEstimateSmoothing:(float)smoothing;

// ========== 清洁音频转换方法 ==========

/**
 * 获取缓冲的清洁音频为WAV文件
 * @param outputSampleRate 目标采样率（例如：16000、24000、44100、48000）
 * @return WAV文件的数据，如果出错则返回nil
 */
- (nullable NSData *)getCleanAudioAsWAV:(NSInteger)outputSampleRate;

/**
 * 获取缓冲的清洁音频为WAV格式（默认采样率44100）
 * @return WAV文件的数据，如果没有可用音频则返回nil
 */
- (nullable NSData *)getCleanAudioAsWAV;

/**
 * 获取缓冲的清洁音频为PCM格式
 * @param outputSampleRate 目标采样率
 * @return PCM音频数据（16位小端序），如果出错则返回nil
 */
- (nullable NSData *)getCleanAudioAsPCM:(NSInteger)outputSampleRate;

/**
 * 获取缓冲的清洁音频为PCM格式（默认采样率44100）
 * @return PCM音频数据（16位小端序），如果没有可用音频则返回nil
 */
- (nullable NSData *)getCleanAudioAsPCM;

/**
 * 清除已累积的清洁音频缓冲区而不获取数据
 */
- (void)clearCleanAudioBuffer;

// ========== 数组转换工具方法 ==========

/**
 * 将字节数组转换为短整型数组
 * @param byteData 输入字节数据
 * @return 转换后的短整型数据，如果出错则返回nil
 */
+ (nullable NSData *)convertByteArrayToShortArray:(NSData *)byteData;

/**
 * 将短整型数组转换为字节数组
 * @param shortData 输入短整型数据
 * @return 转换后的字节数据，如果出错则返回nil
 */
+ (nullable NSData *)convertShortArrayToByteArray:(NSData *)shortData;

/**
 * 写入WAV文件头到数据中
 * @param audioDataSize 音频数据大小（字节）
 * @param sampleRate 采样率
 * @param channels 声道数（默认：1）
 * @param bitsPerSample 每样本位数（默认：16）
 * @return 包含WAV头的数据，如果出错则返回nil
 */
+ (nullable NSData *)createWavHeaderWithAudioDataSize:(NSInteger)audioDataSize
                                           sampleRate:(NSInteger)sampleRate
                                             channels:(NSInteger)channels
                                        bitsPerSample:(NSInteger)bitsPerSample;

@end

NS_ASSUME_NONNULL_END
