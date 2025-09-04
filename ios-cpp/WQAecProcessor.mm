//
//  WQAecProcessor.mm
//  WQDenoiser
//
//  Created by Jimmy Gan on 2025/1/31.
//  Copyright © 2025 WatchFun. All rights reserved.
//

#import "WQAecProcessor.h"
#import "WQDenoiserWrapper.h"
#import <os/log.h>

static os_log_t wq_log = nil;

@implementation WQAecMetrics

- (instancetype)initWithERL:(double)erl ERLE:(double)erle delay:(NSInteger)delay {
    self = [super init];
    if (self) {
        _echoReturnLoss = erl;
        _echoReturnLossEnhancement = erle;
        _delayMs = delay;
    }
    return self;
}

- (NSString *)description {
    return [NSString stringWithFormat:@"AEC Metrics: ERL=%.2fdB, ERLE=%.2fdB, Delay=%ldms", 
            _echoReturnLoss, _echoReturnLossEnhancement, (long)_delayMs];
}

- (NSString *)erleQuality {
    if (_echoReturnLossEnhancement >= 15.0) return @"Excellent";
    else if (_echoReturnLossEnhancement >= 10.0) return @"Good";
    else if (_echoReturnLossEnhancement >= 5.0) return @"Fair";
    else return @"Poor";
}

@end

@implementation WQEnhancedAecMetrics

- (instancetype)initWithERL:(double)erl 
                       ERLE:(double)erle 
                      delay:(NSInteger)delay 
               renderFrames:(uint64_t)renderFrames 
              captureFrames:(uint64_t)captureFrames 
               optimalDelay:(NSInteger)optimalDelay {
    self = [super init];
    if (self) {
        _echoReturnLoss = erl;
        _echoReturnLossEnhancement = erle;
        _delayMs = delay;
        _renderFrames = renderFrames;
        _captureFrames = captureFrames;
        _optimalDelayMs = optimalDelay;
    }
    return self;
}

- (NSString *)description {
    return [NSString stringWithFormat:@"Enhanced AEC Metrics: ERL=%.2fdB, ERLE=%.2fdB, Delay=%ldms, "
            "RenderFrames=%llu, CaptureFrames=%llu, OptimalDelay=%ldms", 
            _echoReturnLoss, _echoReturnLossEnhancement, (long)_delayMs, 
            _renderFrames, _captureFrames, (long)_optimalDelayMs];
}

- (NSString *)erleQuality {
    if (_echoReturnLossEnhancement >= 15.0) return @"Excellent";
    else if (_echoReturnLossEnhancement >= 10.0) return @"Good";
    else if (_echoReturnLossEnhancement >= 5.0) return @"Fair";
    else return @"Poor";
}

- (BOOL)isFrameSynchronized {
    if (_renderFrames == 0 || _captureFrames == 0) return NO;
    double ratio = (double)MIN(_renderFrames, _captureFrames) / MAX(_renderFrames, _captureFrames);
    return ratio > 0.95; // 在5%以内视为同步
}

@end

@interface WQAecProcessor ()
@property (nonatomic, assign) WQDenoiserHandle denoiserHandle;
@property (nonatomic, assign) BOOL initialized;
@end

@implementation WQAecProcessor

+ (void)initialize {
    if (self == [WQAecProcessor class]) {
        wq_log = os_log_create("cn.watchfun.wqaecprocessor", "AEC3");
    }
}

+ (NSInteger)sampleRate {
    return WQ_SAMPLE_RATE;
}

+ (NSInteger)frameSize {
    return WQ_FRAME_SIZE;
}

+ (NSInteger)channels {
    return WQ_CHANNELS;
}

+ (NSInteger)bitsPerSample {
    return 16;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        _denoiserHandle = wq_denoiser_create();
        _initialized = NO;
        
        if (!_denoiserHandle) {
            os_log_error(wq_log, "Failed to create WQ denoiser handle");
            return nil;
        }
        
        os_log_info(wq_log, "WQAecProcessor created successfully");
    }
    return self;
}

- (void)dealloc {
    [self destroy];
    [super dealloc];
}

// ========== 核心AEC3方法 ==========

- (BOOL)initialize {
    if (!_denoiserHandle) {
        os_log_error(wq_log, "Cannot initialize: denoiser handle is null");
        return NO;
    }
    
    if (_initialized) {
        os_log_info(wq_log, "AEC processor already initialized");
        return YES;
    }
    
    BOOL success = wq_denoiser_initialize(_denoiserHandle);
    if (success) {
        _initialized = YES;
        os_log_info(wq_log, "AEC processor initialized successfully");
    } else {
        os_log_error(wq_log, "Failed to initialize AEC processor");
    }
    
    return success;
}

- (void)destroy {
    if (_denoiserHandle) {
        wq_denoiser_destroy(_denoiserHandle);
        _denoiserHandle = NULL;
        _initialized = NO;
        os_log_info(wq_log, "AEC processor destroyed");
    }
}

- (BOOL)processTtsAudio:(NSData *)ttsData {
    if (!_initialized || !_denoiserHandle) {
        os_log_error(wq_log, "AEC processor not initialized");
        return NO;
    }
    
    if (!ttsData || ttsData.length != WQ_FRAME_SIZE * 2) {
        os_log_error(wq_log, "Invalid TTS data: length=%zu, expected=%d", 
                    ttsData.length, WQ_FRAME_SIZE * 2);
        return NO;
    }
    
    const uint8_t *bytes = (const uint8_t *)ttsData.bytes;
    BOOL success = wq_denoiser_process_tts_audio_bytes(_denoiserHandle, bytes, ttsData.length);
    
    if (!success) {
        os_log_error(wq_log, "Failed to process TTS audio");
    }
    
    return success;
}

- (nullable NSData *)processMicrophoneAudio:(NSData *)micData enableAEC:(BOOL)enableAEC {
    if (!_initialized || !_denoiserHandle) {
        os_log_error(wq_log, "AEC processor not initialized");
        return nil;
    }
    
    if (!micData || micData.length != WQ_FRAME_SIZE * 2) {
        os_log_error(wq_log, "Invalid microphone data: length=%zu, expected=%d", 
                    micData.length, WQ_FRAME_SIZE * 2);
        return nil;
    }
    
    const uint8_t *inputBytes = (const uint8_t *)micData.bytes;
    NSMutableData *outputData = [NSMutableData dataWithLength:micData.length];
    uint8_t *outputBytes = (uint8_t *)outputData.mutableBytes;
    
    BOOL success = wq_denoiser_process_microphone_audio_bytes(_denoiserHandle, 
                                                             inputBytes, 
                                                             outputBytes, 
                                                             micData.length, 
                                                             enableAEC);
    
    if (!success) {
        os_log_error(wq_log, "Failed to process microphone audio");
        return nil;
    }
    
    return outputData;
}

- (nullable WQAecMetrics *)getMetrics {
    if (!_initialized || !_denoiserHandle) {
        return nil;
    }
    
    double erl, erle;
    int delayMs;
    
    if (wq_denoiser_get_metrics(_denoiserHandle, &erl, &erle, &delayMs)) {
        return [[WQAecMetrics alloc] initWithERL:erl ERLE:erle delay:delayMs];
    }
    
    return nil;
}

- (nullable WQEnhancedAecMetrics *)getEnhancedMetrics {
    if (!_initialized || !_denoiserHandle) {
        return nil;
    }
    
    double erl, erle;
    int delayMs;
    uint64_t renderFrames, captureFrames;
    int optimalDelay;
    
    if (wq_denoiser_get_enhanced_metrics(_denoiserHandle, &erl, &erle, &delayMs, 
                                        &renderFrames, &captureFrames, &optimalDelay)) {
        return [[WQEnhancedAecMetrics alloc] initWithERL:erl 
                                                    ERLE:erle 
                                                   delay:delayMs 
                                            renderFrames:renderFrames 
                                           captureFrames:captureFrames 
                                            optimalDelay:optimalDelay];
    }
    
    return nil;
}

- (void)setStreamDelay:(NSInteger)delayMs {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_stream_delay(_denoiserHandle, (int)delayMs);
        os_log_info(wq_log, "Stream delay set to %ldms", (long)delayMs);
    }
}

// ========== 高级功能 ==========

- (BOOL)autoOptimizeDelay {
    if (!_initialized || !_denoiserHandle) {
        return NO;
    }
    
    BOOL success = wq_denoiser_auto_optimize_delay(_denoiserHandle);
    if (success) {
        os_log_info(wq_log, "Auto delay optimization completed");
    } else {
        os_log_error(wq_log, "Auto delay optimization failed");
    }
    
    return success;
}

- (BOOL)enableTimingSync:(BOOL)enable {
    if (!_initialized || !_denoiserHandle) {
        return NO;
    }
    
    BOOL success = wq_denoiser_enable_timing_sync(_denoiserHandle, enable);
    os_log_info(wq_log, "Timing synchronization %s", enable ? "enabled" : "disabled");
    
    return success;
}

// ========== AEC3参数控制方法 ==========

- (void)setConfigChangeDuration:(NSInteger)blocks {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_config_change_duration(_denoiserHandle, (int)blocks);
    }
}

- (void)setInitialStateSeconds:(float)seconds {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_initial_state_seconds(_denoiserHandle, seconds);
    }
}

- (void)setConservativeInitialPhase:(BOOL)enable {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_conservative_initial_phase(_denoiserHandle, enable);
    }
}

- (void)setMaxDecFactorLF:(float)factor {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_max_dec_factor_lf(_denoiserHandle, factor);
    }
}

- (void)setMaxIncFactor:(float)factor {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_max_inc_factor(_denoiserHandle, factor);
    }
}

- (void)setNearendMaxDecFactorLF:(float)factor {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_nearend_max_dec_factor_lf(_denoiserHandle, factor);
    }
}

- (void)setNearendMaxIncFactor:(float)factor {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_nearend_max_inc_factor(_denoiserHandle, factor);
    }
}

- (void)setEnrThreshold:(float)threshold {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_enr_threshold(_denoiserHandle, threshold);
    }
}

- (void)setSnrThreshold:(float)threshold {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_snr_threshold(_denoiserHandle, threshold);
    }
}

- (void)setHoldDuration:(NSInteger)duration {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_hold_duration(_denoiserHandle, (int)duration);
    }
}

- (void)setTriggerThreshold:(NSInteger)threshold {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_trigger_threshold(_denoiserHandle, (int)threshold);
    }
}

// ERLE调整参数方法
- (void)setFilterLengthBlocks:(NSInteger)blocks {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_filter_length_blocks(_denoiserHandle, (int)blocks);
    }
}

- (void)setFilterLeakageConverged:(float)leakage {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_filter_leakage_converged(_denoiserHandle, leakage);
    }
}

- (void)setFilterLeakageDiverged:(float)leakage {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_filter_leakage_diverged(_denoiserHandle, leakage);
    }
}

- (void)setDelayDownSamplingFactor:(NSInteger)factor {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_delay_down_sampling_factor(_denoiserHandle, (int)factor);
    }
}

- (void)setDelayNumFilters:(NSInteger)filters {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_delay_num_filters(_denoiserHandle, (int)filters);
    }
}

- (void)setDelayEstimateSmoothing:(float)smoothing {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_set_delay_estimate_smoothing(_denoiserHandle, smoothing);
    }
}

// ========== 清洁音频转换方法 ==========

- (nullable NSData *)getCleanAudioAsWAV:(NSInteger)outputSampleRate {
    if (!_initialized || !_denoiserHandle) {
        return nil;
    }
    
    size_t outputSize = 0;
    uint8_t *wavData = wq_denoiser_get_clean_audio_as_wav(_denoiserHandle, (int)outputSampleRate, &outputSize);
    
    if (wavData && outputSize > 0) {
        NSData *data = [NSData dataWithBytes:wavData length:outputSize];
        wq_denoiser_free_memory(wavData);
        return data;
    }
    
    return nil;
}

- (nullable NSData *)getCleanAudioAsWAV {
    return [self getCleanAudioAsWAV:44100];
}

- (nullable NSData *)getCleanAudioAsPCM:(NSInteger)outputSampleRate {
    if (!_initialized || !_denoiserHandle) {
        return nil;
    }
    
    size_t outputSize = 0;
    uint8_t *pcmData = wq_denoiser_get_clean_audio_as_pcm(_denoiserHandle, (int)outputSampleRate, &outputSize);
    
    if (pcmData && outputSize > 0) {
        NSData *data = [NSData dataWithBytes:pcmData length:outputSize];
        wq_denoiser_free_memory(pcmData);
        return data;
    }
    
    return nil;
}

- (nullable NSData *)getCleanAudioAsPCM {
    return [self getCleanAudioAsPCM:44100];
}

- (void)clearCleanAudioBuffer {
    if (_initialized && _denoiserHandle) {
        wq_denoiser_clear_clean_audio_buffer(_denoiserHandle);
    }
}

// ========== 数组转换工具方法 ==========

+ (nullable NSData *)convertByteArrayToShortArray:(NSData *)byteData {
    if (!byteData || byteData.length % 2 != 0) {
        return nil;
    }
    
    size_t shortLength = byteData.length / 2;
    NSMutableData *shortData = [NSMutableData dataWithLength:shortLength * sizeof(int16_t)];
    
    if (wq_denoiser_convert_byte_array_to_short_array((const uint8_t *)byteData.bytes, 
                                                     byteData.length,
                                                     (int16_t *)shortData.mutableBytes, 
                                                     shortLength)) {
        return shortData;
    }
    
    return nil;
}

+ (nullable NSData *)convertShortArrayToByteArray:(NSData *)shortData {
    if (!shortData || shortData.length % sizeof(int16_t) != 0) {
        return nil;
    }
    
    size_t shortLength = shortData.length / sizeof(int16_t);
    NSMutableData *byteData = [NSMutableData dataWithLength:shortLength * 2];
    
    if (wq_denoiser_convert_short_array_to_byte_array((const int16_t *)shortData.bytes, 
                                                     shortLength,
                                                     (uint8_t *)byteData.mutableBytes, 
                                                     byteData.length)) {
        return byteData;
    }
    
    return nil;
}

+ (nullable NSData *)createWavHeaderWithAudioDataSize:(NSInteger)audioDataSize
                                           sampleRate:(NSInteger)sampleRate
                                             channels:(NSInteger)channels
                                        bitsPerSample:(NSInteger)bitsPerSample {
    NSMutableData *headerData = [NSMutableData dataWithLength:44];
    
    if (wq_denoiser_write_wav_header((uint8_t *)headerData.mutableBytes, 
                                    audioDataSize, 
                                    (int)sampleRate, 
                                    (int)channels, 
                                    (int)bitsPerSample)) {
        return headerData;
    }
    
    return nil;
}

@end
