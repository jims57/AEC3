#include <jni.h>
#include <memory>
#include "wq_aec3_processor.h"
#include "wq_aec3_convertor.h"

// WebRTC AEC3 TTS回声消除的JNI实现 
// 该文件提供Java和C++之间的JNI桥梁，用于TTS AEC3处理器

// 全局处理器实例
static std::unique_ptr<webrtc_aec3_tts::WqAec3Processor> g_processor;

extern "C" {

// ========== 核心AEC3 JNI方法 ==========

/**
 * 初始化AEC3处理器
 * @return 初始化成功则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeInitialize(JNIEnv *env, jobject thiz) {
    g_processor = std::make_unique<webrtc_aec3_tts::WqAec3Processor>();
    return g_processor->Initialize() ? JNI_TRUE : JNI_FALSE;
}

/**
 * 清理并销毁AEC3处理器
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeDestroy(JNIEnv *env, jobject thiz) {
    g_processor.reset();
}

/**
 * 处理TTS音频（参考信号）
 * @param tts_data TTS音频样本（必须是480个样本）
 * @return 处理成功则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeProcessTtsAudio(JNIEnv *env, jobject thiz, jshortArray tts_data) {
    if (!g_processor) return JNI_FALSE;
    
    jsize length = env->GetArrayLength(tts_data);
    if (length != webrtc_aec3_tts::WqAec3Processor::kFrameSize) return JNI_FALSE;
    
    jshort* data = env->GetShortArrayElements(tts_data, nullptr);
    bool result = g_processor->ProcessTtsAudio(
        reinterpret_cast<const int16_t*>(data), length);
    env->ReleaseShortArrayElements(tts_data, data, JNI_ABORT);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

/**
 * 处理麦克风音频并移除回声
 * @param mic_data 麦克风输入样本（必须是480个样本）
 * @param output_data 处理后音频的输出缓冲区（必须是480个样本）
 * @return 处理成功则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeProcessMicrophoneAudio(JNIEnv *env, jobject thiz, 
                                                          jshortArray mic_data, jshortArray output_data) {
    if (!g_processor) return JNI_FALSE;
    
    jsize length = env->GetArrayLength(mic_data);
    if (length != webrtc_aec3_tts::WqAec3Processor::kFrameSize) return JNI_FALSE;
    
    jshort* input = env->GetShortArrayElements(mic_data, nullptr);
    jshort* output = env->GetShortArrayElements(output_data, nullptr);
    
    bool result = g_processor->ProcessMicrophoneAudio(
        reinterpret_cast<const int16_t*>(input), 
        reinterpret_cast<int16_t*>(output), length);
    
    env->ReleaseShortArrayElements(mic_data, input, JNI_ABORT);
    env->ReleaseShortArrayElements(output_data, output, 0);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

/**
 * 处理TTS音频（参考信号）- 字节数组版本
 * @param tts_byte_data TTS音频字节数据（必须是960字节，即480个样本 * 2字节）
 * @return 处理成功则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeProcessTtsAudioBytes(JNIEnv *env, jobject thiz, jbyteArray tts_byte_data) {
    if (!g_processor) return JNI_FALSE;
    
    jsize byte_length = env->GetArrayLength(tts_byte_data);
    if (byte_length != webrtc_aec3_tts::WqAec3Processor::kFrameSize * 2) return JNI_FALSE;
    
    jbyte* byte_data = env->GetByteArrayElements(tts_byte_data, nullptr);
    bool result = g_processor->ProcessTtsAudioBytes(
        reinterpret_cast<const uint8_t*>(byte_data), byte_length);
    env->ReleaseByteArrayElements(tts_byte_data, byte_data, JNI_ABORT);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

/**
 * 处理麦克风音频并移除回声 - 字节数组版本
 * @param mic_byte_data 麦克风输入字节数据（必须是960字节，即480个样本 * 2字节）
 * @return 处理后的音频字节数组，如果出错则返回null
 */
JNIEXPORT jbyteArray JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeProcessMicrophoneAudioBytes(JNIEnv *env, jobject thiz, jbyteArray mic_byte_data) {
    if (!g_processor) return nullptr;
    
    jsize byte_length = env->GetArrayLength(mic_byte_data);
    if (byte_length != webrtc_aec3_tts::WqAec3Processor::kFrameSize * 2) return nullptr;
    
    jbyte* input_bytes = env->GetByteArrayElements(mic_byte_data, nullptr);
    
    // Create output byte array
    jbyteArray output_byte_array = env->NewByteArray(byte_length);
    if (!output_byte_array) {
        env->ReleaseByteArrayElements(mic_byte_data, input_bytes, JNI_ABORT);
        return nullptr;
    }
    
    jbyte* output_bytes = env->GetByteArrayElements(output_byte_array, nullptr);
    
    bool result = g_processor->ProcessMicrophoneAudioBytes(
        reinterpret_cast<const uint8_t*>(input_bytes), 
        reinterpret_cast<uint8_t*>(output_bytes), byte_length);
    
    env->ReleaseByteArrayElements(mic_byte_data, input_bytes, JNI_ABORT);
    env->ReleaseByteArrayElements(output_byte_array, output_bytes, 0);
    
    if (!result) {
        return nullptr;
    }
    
    return output_byte_array;
}

/**
 * 获取当前AEC性能指标
 * @return double数组: [回声返回损耗, 回声返回损耗增强, 延迟毫秒数]
 */
JNIEXPORT jdoubleArray JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeGetMetrics(JNIEnv *env, jobject thiz) {
    if (!g_processor) return nullptr;
    
    double erl, erle;
    int delay_ms;
    if (!g_processor->GetMetrics(&erl, &erle, &delay_ms)) {
        return nullptr;
    }
    
    jdoubleArray result = env->NewDoubleArray(3);
    double metrics[] = {erl, erle, static_cast<double>(delay_ms)};
    env->SetDoubleArrayRegion(result, 0, 3, metrics);
    return result;
}

/**
 * 更新流延迟补偿
 * @param delay_ms 延迟毫秒数
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetStreamDelay(JNIEnv *env, jobject thiz, jint delay_ms) {
    if (g_processor) {
        g_processor->SetStreamDelay(delay_ms);
    }
}

// ========== 官方AEC3参数控制JNI方法 ==========

// 滤波器配置JNI方法
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetConfigChangeDuration(JNIEnv *env, jobject thiz, jint blocks) {
    if (g_processor) {
        g_processor->SetConfigChangeDuration(blocks);
    }
}

JNIEXPORT void JNICALL  
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetInitialStateSeconds(JNIEnv *env, jobject thiz, jfloat seconds) {
    if (g_processor) {
        g_processor->SetInitialStateSeconds(seconds);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetConservativeInitialPhase(JNIEnv *env, jobject thiz, jboolean enable) {
    if (g_processor) {
        g_processor->SetConservativeInitialPhase(enable == JNI_TRUE);
    }
}

// 抑制器正常调优JNI方法
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetMaxDecFactorLF(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetMaxDecFactorLF(factor);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetMaxIncFactor(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetMaxIncFactor(factor);
    }
}

// 抑制器近端调优JNI方法
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetNearendMaxDecFactorLF(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetNearendMaxDecFactorLF(factor);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetNearendMaxIncFactor(JNIEnv *env, jobject thiz, jfloat factor) {
    if (g_processor) {
        g_processor->SetNearendMaxIncFactor(factor);
    }
}

// 主导近端检测JNI方法
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetEnrThreshold(JNIEnv *env, jobject thiz, jfloat threshold) {
    if (g_processor) {
        g_processor->SetEnrThreshold(threshold);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetSnrThreshold(JNIEnv *env, jobject thiz, jfloat threshold) {
    if (g_processor) {
        g_processor->SetSnrThreshold(threshold);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetHoldDuration(JNIEnv *env, jobject thiz, jint duration) {
    if (g_processor) {
        g_processor->SetHoldDuration(duration);
    }
}

JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetTriggerThreshold(JNIEnv *env, jobject thiz, jint threshold) {
    if (g_processor) {
        g_processor->SetTriggerThreshold(threshold);
    }
}

// ========== 增强ERLE优化JNI方法 ==========

/**
 * 自动优化延迟以获得最大ERLE性能
 * @return 优化成功完成则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeAutoOptimizeDelay(JNIEnv *env, jobject thiz) {
    if (g_processor) {
        return g_processor->AutoOptimizeDelay() ? JNI_TRUE : JNI_FALSE;
    }
    return JNI_FALSE;
}

/**
 * 获取带有详细信息的增强AEC性能指标
 * @return double数组: [ERL, ERLE, 延迟, 渲染帧数, 捕获帧数, 最优延迟]
 */
JNIEXPORT jdoubleArray JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeGetEnhancedMetrics(JNIEnv *env, jobject thiz) {
    if (!g_processor) return nullptr;
    
    double erl, erle;
    int delay_ms;
    uint64_t render_frames, capture_frames;
    int optimal_delay;
    
    if (!g_processor->GetEnhancedMetrics(&erl, &erle, &delay_ms, &render_frames, &capture_frames, &optimal_delay)) {
        return nullptr;
    }
    
    jdoubleArray result = env->NewDoubleArray(6);
    double metrics[] = {erl, erle, static_cast<double>(delay_ms), 
                       static_cast<double>(render_frames), static_cast<double>(capture_frames),
                       static_cast<double>(optimal_delay)};
    env->SetDoubleArrayRegion(result, 0, 6, metrics);
    return result;
}

/**
 * 启用或禁用精确时序同步
 * @param enable true启用时序同步，false禁用
 * @return 设置成功应用则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeEnableTimingSync(JNIEnv *env, jobject thiz, jboolean enable) {
    if (g_processor) {
        return g_processor->EnableTimingSync(enable == JNI_TRUE) ? JNI_TRUE : JNI_FALSE;
    }
    return JNI_FALSE;
}

// ========== 移动开发者ERLE调整参数JNI方法 ==========

/**
 * 设置回声学习的滤波器长度块数
 * @param blocks 1-100范围，默认=25
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetFilterLengthBlocks(JNIEnv *env, jobject thiz, jint blocks) {
    if (g_processor) {
        g_processor->SetFilterLengthBlocks(blocks);
    }
}

/**
 * 设置收敛时的滤波器泄漏以保持稳定性
 * @param leakage 0.000001-1.0范围，默认=0.000005
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetFilterLeakageConverged(JNIEnv *env, jobject thiz, jfloat leakage) {
    if (g_processor) {
        g_processor->SetFilterLeakageConverged(leakage);
    }
}

/**
 * 设置发散时的滤波器泄漏以进行恢复
 * @param leakage 0.001-1.0范围，默认=0.005
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetFilterLeakageDiverged(JNIEnv *env, jobject thiz, jfloat leakage) {
    if (g_processor) {
        g_processor->SetFilterLeakageDiverged(leakage);
    }
}

/**
 * 设置延迟估计下采样因子以提高精度
 * @param factor 1-8范围，默认=2
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetDelayDownSamplingFactor(JNIEnv *env, jobject thiz, jint factor) {
    if (g_processor) {
        g_processor->SetDelayDownSamplingFactor(factor);
    }
}

/**
 * 设置延迟估计滤波器数量
 * @param filters 1-32范围，默认=16
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetDelayNumFilters(JNIEnv *env, jobject thiz, jint filters) {
    if (g_processor) {
        g_processor->SetDelayNumFilters(filters);
    }
}

/**
 * 设置延迟估计平滑因子以保持稳定性
 * @param smoothing 0.1-0.99范围，默认=0.98
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetDelayEstimateSmoothing(JNIEnv *env, jobject thiz, jfloat smoothing) {
    if (g_processor) {
        g_processor->SetDelayEstimateSmoothing(smoothing);
    }
}

// ========== 清洁音频转换JNI方法 ==========

/**
 * 获取清洁音频缓冲区并转换为WAV格式
 * @param outputSampleRate 输出采样率（默认：44100）
 * @return 包含WAV数据的字节数组，出错时返回null
 */
JNIEXPORT jbyteArray JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeGetCleanAudioAsWAV(JNIEnv *env, jobject thiz, jint outputSampleRate) {
    if (!g_processor) return nullptr;
    
    // 从处理器缓冲区获取清洁音频帧（不清除 - PCM方法会清除）
    std::vector<std::vector<float>> audioFrames;
    size_t frameCount = g_processor->GetCleanAudioBuffer(audioFrames);
    
    if (frameCount == 0) {
        return nullptr; // 没有可用的音频帧
    }
    
    // Convert float vectors to byte vectors
    std::vector<std::vector<uint8_t>> audioFramesBytes;
    audioFramesBytes.reserve(audioFrames.size());
    
    for (const auto& floatFrame : audioFrames) {
        std::vector<uint8_t> byteFrame;
        byteFrame.reserve(floatFrame.size() * sizeof(float));
        
        // Convert float samples to bytes (little-endian)
        for (float sample : floatFrame) {
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&sample);
            byteFrame.insert(byteFrame.end(), bytes, bytes + sizeof(float));
        }
        audioFramesBytes.push_back(std::move(byteFrame));
    }
    
    // 转换为WAV格式
    uint8_t* wavData = nullptr;
    size_t wavSize = 0;
    int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToWAV(
        audioFramesBytes, 48000, &wavData, &wavSize, outputSampleRate);
    
    if (result != 0 || !wavData || wavSize == 0) {
        if (wavData) free(wavData);
        return nullptr;
    }
    
    // 创建Java字节数组
    jbyteArray wavArray = env->NewByteArray(static_cast<jsize>(wavSize));
    if (!wavArray) {
        free(wavData);
        return nullptr;
    }
    
    env->SetByteArrayRegion(wavArray, 0, static_cast<jsize>(wavSize), 
                           reinterpret_cast<const jbyte*>(wavData));
    
    free(wavData);
    return wavArray;
}

/**
 * 获取清洁音频缓冲区并转换为PCM格式
 * @param outputSampleRate 输出采样率（默认：44100）
 * @return 包含PCM数据的字节数组，出错时返回null
 */
JNIEXPORT jbyteArray JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeGetCleanAudioAsPCM(JNIEnv *env, jobject thiz, jint outputSampleRate) {
    if (!g_processor) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3_TTS", "PCM: g_processor为空");
        return nullptr;
    }
    
    // 从处理器缓冲区获取清洁音频帧
    std::vector<std::vector<float>> audioFrames;
    size_t frameCount = g_processor->GetCleanAudioBuffer(audioFrames);
    
    __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: 从缓冲区检索到%zu帧", frameCount);
    
    if (frameCount == 0) {
        __android_log_print(ANDROID_LOG_WARN, "WebRTC_AEC3_TTS", "PCM: 没有可用的音频帧");
        return nullptr; // 没有可用的音频帧
    }
    
    // 调试：检查第一帧内容
    if (!audioFrames.empty() && !audioFrames[0].empty()) {
        float firstSample = audioFrames[0][0];
        float lastSample = audioFrames[0][audioFrames[0].size()-1];
        __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: 第一帧样本: 首个=%.6f, 最后=%.6f, 大小=%zu", 
                           firstSample, lastSample, audioFrames[0].size());
    }
    
    // Convert float vectors to byte vectors
    std::vector<std::vector<uint8_t>> audioFramesBytes;
    audioFramesBytes.reserve(audioFrames.size());
    
    for (const auto& floatFrame : audioFrames) {
        std::vector<uint8_t> byteFrame;
        byteFrame.reserve(floatFrame.size() * sizeof(float));
        
        // Convert float samples to bytes (little-endian)
        for (float sample : floatFrame) {
            const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&sample);
            byteFrame.insert(byteFrame.end(), bytes, bytes + sizeof(float));
        }
        audioFramesBytes.push_back(std::move(byteFrame));
    }
    
    // 转换为PCM格式
    uint8_t* pcmData = nullptr;
    size_t pcmSize = 0;
    int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToPCM(
        audioFramesBytes, 48000, &pcmData, &pcmSize, outputSampleRate);
    
    __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: 转换结果=%d, pcmData=%p, pcmSize=%zu", 
                       result, pcmData, pcmSize);
    
    if (result != 0 || !pcmData || pcmSize == 0) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3_TTS", "PCM: 转换失败 - 结果=%d, 数据=%p, 大小=%zu", 
                           result, pcmData, pcmSize);
        if (pcmData) free(pcmData);
        return nullptr;
    }
    
    // 调试：检查前几个PCM字节
    int16_t* pcmSamples = reinterpret_cast<int16_t*>(pcmData);
    __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: 前几个样本: [%d, %d, %d, %d]", 
                       pcmSamples[0], pcmSamples[1], pcmSamples[2], pcmSamples[3]);
    
    // 创建Java字节数组
    jbyteArray pcmArray = env->NewByteArray(static_cast<jsize>(pcmSize));
    if (!pcmArray) {
        __android_log_print(ANDROID_LOG_ERROR, "WebRTC_AEC3_TTS", "PCM: 创建Java字节数组失败");
        free(pcmData);
        return nullptr;
    }
    
    env->SetByteArrayRegion(pcmArray, 0, static_cast<jsize>(pcmSize), 
                           reinterpret_cast<const jbyte*>(pcmData));
    
    __android_log_print(ANDROID_LOG_INFO, "WebRTC_AEC3_TTS", "PCM: 成功创建大小为%zu的字节数组", pcmSize);
    
    free(pcmData);
    return pcmArray;
}

/**
 * 清除清洁音频缓冲区而不检索数据
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeClearCleanAudioBuffer(JNIEnv *env, jobject thiz) {
    if (g_processor) {
        g_processor->ClearCleanAudioBuffer();
    }
}

// ========== 数组转换工具JNI方法 ==========

/**
 * 将字节数组转换为短整型数组
 * @param byte_array 输入字节数组
 * @param short_array 输出短整型数组（必须预分配正确大小）
 * @return 转换成功则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeConvertByteArrayToShortArray(
    JNIEnv* env, jobject thiz, jbyteArray byte_array, jshortArray short_array) {
    
    if (!byte_array || !short_array) {
        return JNI_FALSE;
    }
    
    jsize byte_length = env->GetArrayLength(byte_array);
    jsize short_length = env->GetArrayLength(short_array);
    
    if (byte_length != short_length * 2) {
        return JNI_FALSE;
    }
    
    // Get array elements
    jbyte* byte_data = env->GetByteArrayElements(byte_array, nullptr);
    jshort* short_data = env->GetShortArrayElements(short_array, nullptr);
    
    if (!byte_data || !short_data) {
        if (byte_data) env->ReleaseByteArrayElements(byte_array, byte_data, JNI_ABORT);
        if (short_data) env->ReleaseShortArrayElements(short_array, short_data, JNI_ABORT);
        return JNI_FALSE;
    }
    
    // Use processor's conversion method
    bool success = false;
    if (g_processor) {
        success = g_processor->ConvertByteArrayToShortArray(
            reinterpret_cast<const uint8_t*>(byte_data), 
            byte_length,
            reinterpret_cast<int16_t*>(short_data), 
            short_length
        );
    }
    
    // Release arrays
    env->ReleaseByteArrayElements(byte_array, byte_data, JNI_ABORT);
    env->ReleaseShortArrayElements(short_array, short_data, success ? 0 : JNI_ABORT);
    
    return success ? JNI_TRUE : JNI_FALSE;
}

/**
 * 将短整型数组转换为字节数组
 * @param short_array 输入短整型数组
 * @param byte_array 输出字节数组（必须预分配正确大小）
 * @return 转换成功则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeConvertShortArrayToByteArray(
    JNIEnv* env, jobject thiz, jshortArray short_array, jbyteArray byte_array) {
    
    if (!short_array || !byte_array) {
        return JNI_FALSE;
    }
    
    jsize short_length = env->GetArrayLength(short_array);
    jsize byte_length = env->GetArrayLength(byte_array);
    
    if (byte_length != short_length * 2) {
        return JNI_FALSE;
    }
    
    // Get array elements
    jshort* short_data = env->GetShortArrayElements(short_array, nullptr);
    jbyte* byte_data = env->GetByteArrayElements(byte_array, nullptr);
    
    if (!short_data || !byte_data) {
        if (short_data) env->ReleaseShortArrayElements(short_array, short_data, JNI_ABORT);
        if (byte_data) env->ReleaseByteArrayElements(byte_array, byte_data, JNI_ABORT);
        return JNI_FALSE;
    }
    
    // Use processor's conversion method
    bool success = false;
    if (g_processor) {
        success = g_processor->ConvertShortArrayToByteArray(
            reinterpret_cast<const int16_t*>(short_data), 
            short_length,
            reinterpret_cast<uint8_t*>(byte_data), 
            byte_length
        );
    }
    
    // Release arrays
    env->ReleaseShortArrayElements(short_array, short_data, JNI_ABORT);
    env->ReleaseByteArrayElements(byte_array, byte_data, success ? 0 : JNI_ABORT);
    
    return success ? JNI_TRUE : JNI_FALSE;
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
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeWriteWavHeader(JNIEnv *env, jobject thiz, 
                                                          jbyteArray buffer, 
                                                          jint audioDataSize,
                                                          jint sampleRate,
                                                          jint channels,
                                                          jint bitsPerSample) {
    if (!buffer) {
        return JNI_FALSE;
    }
    
    jsize bufferLength = env->GetArrayLength(buffer);
    if (bufferLength < 44) {
        return JNI_FALSE;
    }
    
    jbyte* bufferData = env->GetByteArrayElements(buffer, nullptr);
    if (!bufferData) {
        return JNI_FALSE;
    }
    
    int result = webrtc_aec3_tts::WqAec3Convertor::writeWavHeader(
        reinterpret_cast<uint8_t*>(bufferData),
        static_cast<size_t>(audioDataSize),
        sampleRate,
        channels,
        bitsPerSample
    );
    
    env->ReleaseByteArrayElements(buffer, bufferData, result == 0 ? 0 : JNI_ABORT);
    
    return result == 0 ? JNI_TRUE : JNI_FALSE;
}

/**
 * 将清洁音频帧转换为WAV格式 - 字节数组版本
 * @param audioFramesBytes 音频帧字节数据的二维数组
 * @param inputSampleRate 输入采样率
 * @param outputSampleRate 输出采样率
 * @return WAV数据的字节数组，出错时返回null
 */
JNIEXPORT jbyteArray JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeConvertCleanAudioToWAVBytes(JNIEnv *env, jobject thiz,
                                                                       jobjectArray audioFramesBytes,
                                                                       jint inputSampleRate,
                                                                       jint outputSampleRate) {
    if (!audioFramesBytes) {
        return nullptr;
    }
    
    jsize frameCount = env->GetArrayLength(audioFramesBytes);
    if (frameCount == 0) {
        return nullptr;
    }
    
    // Convert Java byte[][] to C++ vector<vector<uint8_t>>
    std::vector<std::vector<uint8_t>> cppFrames;
    cppFrames.reserve(frameCount);
    
    for (jsize i = 0; i < frameCount; ++i) {
        jbyteArray frameArray = static_cast<jbyteArray>(env->GetObjectArrayElement(audioFramesBytes, i));
        if (!frameArray) continue;
        
        jsize frameLength = env->GetArrayLength(frameArray);
        jbyte* frameData = env->GetByteArrayElements(frameArray, nullptr);
        
        if (frameData && frameLength > 0) {
            std::vector<uint8_t> cppFrame(frameLength);
            memcpy(cppFrame.data(), frameData, frameLength);
            cppFrames.push_back(std::move(cppFrame));
        }
        
        if (frameData) {
            env->ReleaseByteArrayElements(frameArray, frameData, JNI_ABORT);
        }
        env->DeleteLocalRef(frameArray);
    }
    
    if (cppFrames.empty()) {
        return nullptr;
    }
    
    // Convert using C++ method
    uint8_t* wavData = nullptr;
    size_t wavSize = 0;
    // Call the C++ method with byte array input
    int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToWAV(cppFrames, inputSampleRate, &wavData, &wavSize, outputSampleRate);
    
    if (result != 0 || !wavData || wavSize == 0) {
        if (wavData) free(wavData);
        return nullptr;
    }
    
    // Create Java byte array
    jbyteArray wavArray = env->NewByteArray(static_cast<jsize>(wavSize));
    if (!wavArray) {
        free(wavData);
        return nullptr;
    }
    
    env->SetByteArrayRegion(wavArray, 0, static_cast<jsize>(wavSize), 
                           reinterpret_cast<const jbyte*>(wavData));
    
    free(wavData);
    return wavArray;
}

} // extern "C"
