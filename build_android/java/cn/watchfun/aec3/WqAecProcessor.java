package cn.watchfun.aec3;

/**
 * 用于TTS回声消除的WebRTC AEC3包装器
 * 
 * 该类为WebRTC的声学回声消除(AEC3)提供了一个简单接口，
 * 专门为TTS(文本转语音)应用优化。
 * 
 * 使用方法：
 * 1. 初始化AEC处理器
 * 2. 对于每个TTS音频块：在播放前调用processTtsAudio()
 * 3. 对于每个麦克风音频块：调用processMicrophoneAudio()获取干净音频
 * 4. 使用getMetrics()监控性能
 * 
 * 重要提示：所有音频必须是48kHz、16位PCM、单声道、480个样本(10ms块)
 */
public class WqAecProcessor {
    static {
        System.loadLibrary("wq_aec3_tts");
    }

    // 音频配置常量
    public static final int SAMPLE_RATE = 48000;
    public static final int FRAME_SIZE = 480;  // 10ms at 48kHz
    public static final int CHANNELS = 1;      // Mono
    public static final int BITS_PER_SAMPLE = 16;

    /**
     * 初始化AEC处理器
     * @return 如果成功则返回true
     */
    public native boolean nativeInitialize();

    /**
     * 清理资源
     */
    public native void nativeDestroy();

    /**
     * 处理TTS音频（参考信号）
     * 在通过扬声器播放TTS音频之前调用此方法
     * 
     * @param ttsData TTS音频数据（480个样本，16位PCM）
     * @return 处理成功返回true
     */
    private native boolean nativeProcessTtsAudio(short[] ttsData);

    /**
     * 处理麦克风音频并移除回声
     * 
     * @param micData 麦克风输入数据（480个样本，16位PCM）
     * @param outputData 处理后音频的输出缓冲区（480个样本，16位PCM）
     * @return 处理成功返回true
     */
    private native boolean nativeProcessMicrophoneAudio(short[] micData, short[] outputData);

    /**
     * 处理TTS音频（参考信号）- 字节数组版本
     * 在通过扬声器播放TTS音频之前调用此方法
     * 
     * @param ttsData TTS音频字节数据（960字节，即480个样本 * 2字节）
     * @return 处理成功返回true
     */
    public native boolean nativeProcessTtsAudioBytes(byte[] ttsData);

    /**
     * 处理麦克风音频并移除回声 - 字节数组版本
     * 
     * @param micData 麦克风输入字节数据（960字节，即480个样本 * 2字节）
     * @param enableAEC 是否启用AEC处理
     * @return 处理后的音频字节数组，如果出错则返回null
     */
    public native byte[] nativeProcessMicrophoneAudioBytes(byte[] micData, boolean enableAEC);

    /**
     * 获取当前AEC性能指标
     * 
     * @return 返回[ERL, ERLE, delay_ms]数组，如果不可用则返回null
     */
    public native double[] nativeGetMetrics();

    /**
     * 更新流延迟补偿
     * 
     * @param delayMs 延迟毫秒数（Android通常为80-150ms）
     */
    public native void nativeSetStreamDelay(int delayMs);
    
    // 官方AEC3参数控制
    // 这些原生方法直接对应官方WebRTC AEC3配置参数
    
    // 滤波器配置原生方法
    public native void nativeSetConfigChangeDuration(int blocks);          // 0-1000 range, 0=default
    public native void nativeSetInitialStateSeconds(float seconds);        // 0.0-3.0 range, 0=default  
    public native void nativeSetConservativeInitialPhase(boolean enable);  // true/false
    
    // 抑制器常规调校原生方法
    public native void nativeSetMaxDecFactorLF(float factor);             // 0.0-100.0 range, 0=default
    public native void nativeSetMaxIncFactor(float factor);               // 0.0-100.0 range, 0=default
    
    // 近端抑制器调校原生方法
    public native void nativeSetNearendMaxDecFactorLF(float factor);      // 0.0-100.0 range, 0=default
    public native void nativeSetNearendMaxIncFactor(float factor);        // 0.0-100.0 range, 0=default
    
    // 主导近端检测原生方法
    public native void nativeSetEnrThreshold(float threshold);            // 0.0-1000.0 range, 0=default
    public native void nativeSetSnrThreshold(float threshold);            // 0.0-1000.0 range, 0=default
    public native void nativeSetHoldDuration(int duration);               // 0-10000 range, 0=default
    public native void nativeSetTriggerThreshold(int threshold);          // 0-10000 range, 0=default
    
    // 增强的ERLE优化方法
    public native boolean nativeAutoOptimizeDelay();               // Automatic delay optimization
    public native double[] nativeGetEnhancedMetrics();             // [ERL, ERLE, delay, render_frames, capture_frames, optimal_delay]
    public native boolean nativeEnableTimingSync(boolean enable);   // Enable/disable precise timing sync
    
    // 面向移动开发者的ERLE调整参数原生方法
    public native void nativeSetFilterLengthBlocks(int blocks);           // Filter length blocks (1-100)
    public native void nativeSetFilterLeakageConverged(float leakage);    // Filter leakage converged (0.000001-1.0)
    public native void nativeSetFilterLeakageDiverged(float leakage);     // Filter leakage diverged (0.001-1.0)
    public native void nativeSetDelayDownSamplingFactor(int factor);      // Delay down sampling factor (1-8)
    public native void nativeSetDelayNumFilters(int filters);             // Delay number of filters (1-32)
    public native void nativeSetDelayEstimateSmoothing(float smoothing);  // Delay estimate smoothing (0.1-0.99)
    
    // 干净音频转换原生方法
    public native byte[] nativeGetCleanAudioAsWAV(int outputSampleRate);  // Get buffered clean audio as WAV
    public native byte[] nativeGetCleanAudioAsPCM(int outputSampleRate);  // Get buffered clean audio as PCM
    public native void nativeClearCleanAudioBuffer();                     // Clear clean audio buffer
    
    // 数组转换工具原生方法
    public native boolean nativeConvertByteArrayToShortArray(byte[] byteArray, short[] shortArray);  // Convert byte[] to short[]
    public native boolean nativeConvertShortArrayToByteArray(short[] shortArray, byte[] byteArray);  // Convert short[] to byte[]
    
    // WAV文件头写入原生方法
    public native boolean nativeWriteWavHeader(byte[] buffer, int audioDataSize, int sampleRate, int channels, int bitsPerSample);  // Write WAV header to byte array
    
    // 字节数组版本的音频转换原生方法
    public native byte[] nativeConvertCleanAudioToWAVBytes(byte[][] audioFramesBytes, int inputSampleRate, int outputSampleRate);  // Convert clean audio frames to WAV using byte arrays
    public native byte[] nativeConvertPCMData(byte[] inputPcmData, int inputSampleRate, int outputSampleRate);  // Convert PCM data with resampling
    public native byte[][] nativeResamplePCMTo480SampleChunks(byte[] inputPcmData, int inputSampleRate, int outputSampleRate, boolean hasMoreData);  // Resample PCM and split into 480-sample chunks

    // 高级Java API
    private boolean initialized = false;

    /**
     * 初始化AEC处理器
     * @return true if successful
     */
    public boolean initialize() {
        if (!initialized) {
            initialized = nativeInitialize();
        }
        return initialized;
    }

    /**
     * 清理资源
     */
    public void destroy() {
        if (initialized) {
            nativeDestroy();
            initialized = false;
        }
    }

    /**
     * 处理TTS音频块
     * @param ttsData 音频数据（必须是480个样本）
     * @return 处理成功返回true
     */
    private boolean processTtsAudio(short[] ttsData) {
        if (!initialized || ttsData.length != FRAME_SIZE) {
            return false;
        }
        return nativeProcessTtsAudio(ttsData);
    }

    /**
     * 处理麦克风音频并获取回声消除后的音频
     * @param micData 麦克风输入（必须是480个样本）
     * @return 回声消除后的音频，如果出错则返回null
     */
    private short[] processMicrophoneAudio(short[] micData) {
        if (!initialized || micData.length != FRAME_SIZE) {
            return null;
        }
        
        short[] output = new short[FRAME_SIZE];
        if (nativeProcessMicrophoneAudio(micData, output)) {
            return output;
        }
        return null;
    }

    /**
     * 处理TTS音频块 - 字节数组版本（推荐使用）
     * @param ttsData 音频字节数据（必须是960字节，即480个样本 * 2字节）
     * @return 处理成功返回true
     */
    public boolean processTtsAudioBytes(byte[] ttsData) {
        if (!initialized || ttsData.length != FRAME_SIZE * 2) {
            return false;
        }
        return nativeProcessTtsAudioBytes(ttsData);
    }

    /**
     * 处理麦克风音频并获取回声消除后的音频 - 字节数组版本（推荐使用）
     * @param micData 麦克风输入字节数据（必须是960字节，即480个样本 * 2字节）
     * @param enableAEC 是否启用AEC处理
     * @return 回声消除后的音频字节数组，如果出错则返回null
     */
    public byte[] processMicrophoneAudioBytes(byte[] micData, boolean enableAEC) {
        if (!initialized || micData.length != FRAME_SIZE * 2) {
            return null;
        }
        return nativeProcessMicrophoneAudioBytes(micData, enableAEC);
    }

    /**
     * 获取AEC性能指标
     * 
     * @return AecMetrics对象，包含性能数据
     */
    public AecMetrics getMetrics() {
        if (!initialized) return null;
        
        double[] metrics = nativeGetMetrics();
        if (metrics != null && metrics.length == 3) {
            return new AecMetrics(metrics[0], metrics[1], (int)metrics[2]);
        }
        return null;
    }

    /**
     * 调整流延迟以获得最佳性能
     * @param delayMs 延迟毫秒数
     */
    public void setStreamDelay(int delayMs) {
        if (initialized) {
            nativeSetStreamDelay(delayMs);
        }
    }
    
    // 官方AEC3参数控制方法
    // 这些方法直接控制WebRTC AEC3的配置参数
    // 使用0值应用AEC3默认值，或设置特定值进行自定义调优
    
    // ======= 滤波器配置方法 =======
    
    /**
     * 设置AEC3配置更改持续时间（以块为单位）
     * 控制AEC3在不同配置之间转换的平滑度
     * @param blocks 0-1000范围，0=使用AEC3默认值，典型值：50-250块
     */
    public void setConfigChangeDuration(int blocks) {
        if (initialized) {
            nativeSetConfigChangeDuration(blocks);
        }
    }
    
    /**
     * 设置AEC3初始状态持续时间（秒）
     * AEC3在完全运行前花费在初始学习阶段的时间
     * @param seconds 0.0-3.0范围，0=使用AEC3默认值，典型值：0.5-2.5秒
     */
    public void setInitialStateSeconds(float seconds) {
        if (initialized) {
            nativeSetInitialStateSeconds(seconds);
        }
    }
    
    /**
     * 启用/禁用保守初始阶段
     * 保守模式 = 初始收敛较慢但更稳定
     * @param enable true=保守（更安全），false=激进（收敛更快）
     */
    public void setConservativeInitialPhase(boolean enable) {
        if (initialized) {
            nativeSetConservativeInitialPhase(enable);
        }
    }
    
    // ======= 抑制器常规调优方法 =======
    
    /**
     * 设置低频最大衰减因子（回声抑制强度）
     * 值越高 = 回声抑制越强，但可能影响语音质量
     * @param factor 0.0-100.0范围，0=使用AEC3默认值，典型值：2.0-25.0
     */
    public void setMaxDecFactorLF(float factor) {
        if (initialized) {
            nativeSetMaxDecFactorLF(factor);
        }
    }
    
    /**
     * 设置最大增加因子（语音恢复速度）
     * 值越高 = 回声抑制后语音恢复越快
     * @param factor 0.0-100.0范围，0=使用AEC3默认值，典型值：1.5-5.0
     */
    public void setMaxIncFactor(float factor) {
        if (initialized) {
            nativeSetMaxIncFactor(factor);
        }
    }
    
    // ======= 抑制器近端调优方法 =======
    
    /**
     * 设置近端低频最大衰减因子（语音保护）
     * 值越低 = 用户说话时语音保留越好
     * @param factor 0.0-100.0范围，0=使用AEC3默认值，典型值：1.0-8.0
     */
    public void setNearendMaxDecFactorLF(float factor) {
        if (initialized) {
            nativeSetNearendMaxDecFactorLF(factor);
        }
    }
    
    /**
     * 设置近端最大增加因子（近端语音恢复）
     * 值越高 = 用户说话时语音越清晰
     * @param factor 0.0-100.0范围，0=使用AEC3默认值，典型值：2.0-8.0
     */
    public void setNearendMaxIncFactor(float factor) {
        if (initialized) {
            nativeSetNearendMaxIncFactor(factor);
        }
    }
    
    // ======= 主导近端检测方法 =======
    
    /**
     * 设置语音检测的能量噪声比(ENR)阈值
     * 值越低 = 语音检测越敏感 = 语音保留越好
     * @param threshold 0.0-1000.0范围，0=使用AEC3默认值，典型值：0.1-1.0
     */
    public void setEnrThreshold(float threshold) {
        if (initialized) {
            nativeSetEnrThreshold(threshold);
        }
    }
    
    /**
     * 设置语音检测的信噪比(SNR)阈值
     * 值越低 = 语音检测越敏感 = 语音保留越好
     * @param threshold 0.0-1000.0范围，0=使用AEC3默认值，典型值：10.0-30.0
     */
    public void setSnrThreshold(float threshold) {
        if (initialized) {
            nativeSetSnrThreshold(threshold);
        }
    }
    
    /**
     * 设置主导近端状态的保持时间（以块为单位）
     * 检测到主导近端后保持该状态的时长
     * @param duration 0-10000范围，0=使用AEC3默认值，典型值：100-500块
     */
    public void setHoldDuration(int duration) {
        if (initialized) {
            nativeSetHoldDuration(duration);
        }
    }
    
    /**
     * 设置主导近端检测的触发阈值
     * 值越低 = 越容易检测到主导近端
     * @param threshold 0-10000范围，0=使用AEC3默认值，典型值：10-100
     */
    public void setTriggerThreshold(int threshold) {
        if (initialized) {
            nativeSetTriggerThreshold(threshold);
        }
    }
    
    // ======= 增强型ERLE优化方法 =======
    
    /**
     * 自动优化延迟以获得最佳ERLE性能
     * 将分析当前音频流并调整延迟
     * @return 如果优化成功返回true
     */
    public boolean autoOptimizeDelay() {
        if (!initialized) return false;
        return nativeAutoOptimizeDelay();
    }
    
    /**
     * 获取包括时序信息在内的增强型指标
     * @return 返回包含[ERL, ERLE, current_delay_ms, render_frames, capture_frames, optimal_delay_ms]的数组
     *         如果不可用则返回null
     */
    public EnhancedAecMetrics getEnhancedMetrics() {
        if (!initialized) return null;
        
        double[] metrics = nativeGetEnhancedMetrics();
        if (metrics != null && metrics.length == 6) {
            return new EnhancedAecMetrics(metrics[0], metrics[1], (int)metrics[2], 
                                        (long)metrics[3], (long)metrics[4], (int)metrics[5]);
        }
        return null;
    }
    
    /**
     * 启用或禁用精确时间同步
     * 启用后，使用系统时钟同步TTS和麦克风流
     * @param enable 是否启用精确时间同步
     * @return 如果操作成功返回true
     */
    public boolean enableTimingSync(boolean enable) {
        if (!initialized) return false;
        return nativeEnableTimingSync(enable);
    }
    
    // ======= ERLE调整参数 =======
    // Based on adjust-ERLE-result.md - fine-tune ERLE performance and convergence speed
    
    /**
     * 设置滤波器长度（以块为单位）
     * 值越高 = 回声学习越好，但收敛速度越慢
     * @param blocks 1-100范围，0=使用AEC3默认值，典型值：25块
     */
    public void setFilterLengthBlocks(int blocks) {
        if (initialized) {
            nativeSetFilterLengthBlocks(blocks);
        }
    }
    
    /**
     * 设置收敛时的滤波器泄漏值以保持稳定性
     * 值越低 = 越稳定，但对回声路径变化的适应越慢
     * @param leakage 0.000001-1.0范围，默认值=0.0005
     */
    public void setFilterLeakageConverged(float leakage) {
        if (initialized) {
            nativeSetFilterLeakageConverged(leakage);
        }
    }
    
    /**
     * 设置发散时的滤波器泄漏值以实现更快的重新收敛
     * 值越高 = 回声路径变化时恢复越快
     * @param leakage 0.001-1.0范围，默认值=0.1
     */
    public void setFilterLeakageDiverged(float leakage) {
        if (initialized) {
            nativeSetFilterLeakageDiverged(leakage);
        }
    }
    
    /**
     * 设置延迟估计的下采样因子
     * 值越高 = CPU使用率越低，但延迟估计越粗糙
     * @param factor 1-8范围，默认值=4
     */
    public void setDelayDownSamplingFactor(int factor) {
        if (initialized) {
            nativeSetDelayDownSamplingFactor(factor);
        }
    }
    
    /**
     * 设置用于延迟估计的滤波器数量
     * 滤波器越多 = 延迟估计越好，但CPU使用率越高
     * @param filters 1-32范围，默认值=12
     */
    public void setDelayNumFilters(int filters) {
        if (initialized) {
            nativeSetDelayNumFilters(filters);
        }
    }
    
    /**
     * 设置延迟估计的平滑因子以保持稳定性
     * 值越高 = 延迟估计越稳定
     * @param smoothing 0.1-0.99范围，默认值=0.98
     */
    public void setDelayEstimateSmoothing(float smoothing) {
        if (initialized) {
            nativeSetDelayEstimateSmoothing(smoothing);
        }
    }
    
    // ======= 干净音频转换方法 =======
    
    /**
     * 获取缓冲的干净音频为WAV文件
     * @param outputSampleRate 目标采样率（例如：16000、24000、44100、48000）
     * @return WAV文件的字节数组，如果出错则返回null
     */
    public byte[] getCleanAudioAsWAV(int outputSampleRate) {
        if (!initialized) return null;
        return nativeGetCleanAudioAsWAV(outputSampleRate);
    }
    
    /**
     * 获取缓冲的干净音频为WAV格式（默认采样率）
     * @return WAV文件的字节数组，如果没有可用音频则返回null
     */
    public byte[] getCleanAudioAsWAV() {
        return getCleanAudioAsWAV(44100);
    }
    public byte[] getCleanAudioAsPCM(int outputSampleRate) {
        if (!initialized) return null;
        return nativeGetCleanAudioAsPCM(outputSampleRate);
    }
    
    /**
     * 获取缓冲的干净音频为PCM格式（默认采样率）
     * @return PCM音频数据的字节数组（16位小端序），如果没有可用音频则返回null
     */
    public byte[] getCleanAudioAsPCM() {
        return getCleanAudioAsPCM(44100);
    }
    
    /**
     * 清除已累积的干净音频缓冲区而不获取数据
     * 在开始新的录音会话时使用此方法丢弃累积的音频
     */
    public void clearCleanAudioBuffer() {
        if (initialized) {
            nativeClearCleanAudioBuffer();
        }
    }
    
    // ======= 数组转换工具方法 =======
    
    /**
     * 将字节数组转换为短整型数组
     * @param byteArray 输入字节数组
     * @param shortArray 输出短整型数组（必须预分配正确大小：byteArray.length / 2）
     * @return 转换成功则返回true
     */
    public boolean convertByteArrayToShortArray(byte[] byteArray, short[] shortArray) {
        if (!initialized || byteArray == null || shortArray == null) {
            return false;
        }
        if (byteArray.length != shortArray.length * 2) {
            return false;
        }
        return nativeConvertByteArrayToShortArray(byteArray, shortArray);
    }
    
    /**
     * 将短整型数组转换为字节数组
     * @param shortArray 输入短整型数组
     * @param byteArray 输出字节数组（必须预分配正确大小：shortArray.length * 2）
     * @return 转换成功则返回true
     */
    public boolean convertShortArrayToByteArray(short[] shortArray, byte[] byteArray) {
        if (!initialized || shortArray == null || byteArray == null) {
            return false;
        }
        if (byteArray.length != shortArray.length * 2) {
            return false;
        }
        return nativeConvertShortArrayToByteArray(shortArray, byteArray);
    }
    
    /**
     * 写入WAV文件头到字节数组
     * @param buffer 输出字节数组（至少44字节）
     * @param audioDataSize 音频数据大小（字节）
     * @param sampleRate 采样率
     * @param channels 声道数（默认：1）
     * @param bitsPerSample 每样本位数（默认：16）
     * @return 成功时返回true
     */
    public boolean writeWavHeader(byte[] buffer, int audioDataSize, int sampleRate, int channels, int bitsPerSample) {
        if (!initialized || buffer == null || buffer.length < 44) {
            return false;
        }
        return nativeWriteWavHeader(buffer, audioDataSize, sampleRate, channels, bitsPerSample);
    }
    
    /**
     * 写入WAV文件头到字节数组（使用默认参数）
     * @param buffer 输出字节数组（至少44字节）
     * @param audioDataSize 音频数据大小（字节）
     * @param sampleRate 采样率
     * @return 成功时返回true
     */
    public boolean writeWavHeader(byte[] buffer, int audioDataSize, int sampleRate) {
        return writeWavHeader(buffer, audioDataSize, sampleRate, 1, 16);
    }
    
    /**
     * 将清洁音频帧转换为WAV格式 - 字节数组版本
     * @param audioFramesBytes 音频帧字节数据的二维数组
     * @param inputSampleRate 输入采样率
     * @param outputSampleRate 输出采样率
     * @return WAV数据的字节数组，出错时返回null
     */
    public byte[] convertCleanAudioToWAVBytes(byte[][] audioFramesBytes, int inputSampleRate, int outputSampleRate) {
        if (!initialized || audioFramesBytes == null) {
            return null;
        }
        return nativeConvertCleanAudioToWAVBytes(audioFramesBytes, inputSampleRate, outputSampleRate);
    }
    
    /**
     * 将PCM数据转换为不同采样率的PCM数据
     * @param inputPcmData 输入PCM字节数据
     * @param inputSampleRate 输入采样率
     * @param outputSampleRate 输出采样率
     * @return 转换后的PCM数据字节数组，出错时返回null
     */
    public byte[] convertPCMData(byte[] inputPcmData, int inputSampleRate, int outputSampleRate) {
        if (!initialized || inputPcmData == null) {
            return null;
        }
        return nativeConvertPCMData(inputPcmData, inputSampleRate, outputSampleRate);
    }
    
    /**
     * 将PCM数据重采样并分割为480样本块 (用于WebRTC AEC3)
     * @param inputPcmData 输入PCM字节数据
     * @param inputSampleRate 输入采样率
     * @param outputSampleRate 输出采样率 (通常48000)
     * @param hasMoreData 是否还有更多数据 (false时会flush剩余数据)
     * @return 480样本块的二维字节数组，出错时返回null
     */
    public byte[][] resamplePCMTo480SampleChunks(byte[] inputPcmData, int inputSampleRate, int outputSampleRate, boolean hasMoreData) {
        if (!initialized || inputPcmData == null) {
            return null;
        }
        return nativeResamplePCMTo480SampleChunks(inputPcmData, inputSampleRate, outputSampleRate, hasMoreData);
    }
    
    /**
     * 将PCM数据重采样并分割为480样本块 (默认输出48kHz)
     * @param inputPcmData 输入PCM字节数据
     * @param inputSampleRate 输入采样率
     * @param hasMoreData 是否还有更多数据
     * @return 480样本块的二维字节数组，出错时返回null
     */
    public byte[][] resamplePCMTo480SampleChunks(byte[] inputPcmData, int inputSampleRate, boolean hasMoreData) {
        return resamplePCMTo480SampleChunks(inputPcmData, inputSampleRate, 48000, hasMoreData);
    }
    
/**
 * 用于保存AEC性能指标的类
 */
public static class AecMetrics {
    public final double echoReturnLoss;
    public final double echoReturnLossEnhancement;
    public final int delayMs;

    public AecMetrics(double erl, double erle, int delay) {
        this.echoReturnLoss = erl;
        this.echoReturnLossEnhancement = erle;
        this.delayMs = delay;
    }

    @Override
    public String toString() {
        return String.format("AEC Metrics: ERL=%.2fdB, ERLE=%.2fdB, Delay=%dms", 
                           echoReturnLoss, echoReturnLossEnhancement, delayMs);
    }
}
    
    /**
     * 包含详细信息的增强型AEC性能指标
     */
    public static class EnhancedAecMetrics {
        public final double echoReturnLoss;
        public final double echoReturnLossEnhancement;
        public final int delayMs;
        public final long renderFrames;
        public final long captureFrames;
        public final int optimalDelayMs;

        public EnhancedAecMetrics(double erl, double erle, int delay, long renderFrames, long captureFrames, int optimalDelay) {
            this.echoReturnLoss = erl;
            this.echoReturnLossEnhancement = erle;
            this.delayMs = delay;
            this.renderFrames = renderFrames;
            this.captureFrames = captureFrames;
            this.optimalDelayMs = optimalDelay;
        }

        @Override
        public String toString() {
            return String.format("Enhanced AEC Metrics: ERL=%.2fdB, ERLE=%.2fdB, Delay=%dms, " +
                               "RenderFrames=%d, CaptureFrames=%d, OptimalDelay=%dms", 
                               echoReturnLoss, echoReturnLossEnhancement, delayMs, 
                               renderFrames, captureFrames, optimalDelayMs);
        }
        
        /**
         * 获取ERLE质量评估
         * @return 质量等级: "优秀" (>15dB), "良好" (>10dB), "一般" (>5dB), "差" (<5dB)
         */
        public String getErleQuality() {
            if (echoReturnLossEnhancement >= 15.0) return "Excellent";
            else if (echoReturnLossEnhancement >= 10.0) return "Good";
            else if (echoReturnLossEnhancement >= 5.0) return "Fair";
            else return "Poor";
        }
        
        /**
         * 检查帧是否同步（渲染和捕获帧数相等）
         * @return 如果帧同步良好则返回true
         */
        public boolean isFrameSynchronized() {
            if (renderFrames == 0 || captureFrames == 0) return false;
            double ratio = (double) Math.min(renderFrames, captureFrames) / Math.max(renderFrames, captureFrames);
            return ratio > 0.95; // 在5%以内视为同步
        }
    } // End of EnhancedAecMetrics class
} // End of WqAecProcessor class
