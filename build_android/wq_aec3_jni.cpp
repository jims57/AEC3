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

// ========== Auto-Adaptive Control JNI Methods ==========

/**
 * 启用或禁用自动适配模式
 * @param enable true启用自动适配，false禁用
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeEnableAutoAdaptive(JNIEnv *env, jobject thiz, jboolean enable) {
    if (g_processor) {
        g_processor->EnableAutoAdaptive(enable);
    }
}

/**
 * 检查是否启用了自动适配模式
 * @return 如果启用了自动适配则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeIsAutoAdaptiveEnabled(JNIEnv *env, jobject thiz) {
    return g_processor ? g_processor->IsAutoAdaptiveEnabled() : JNI_FALSE;
}

/**
 * 获取自适应滤波器收敛状态
 * @return 收敛状态 (0-1, 1表示完全收敛)
 */
JNIEXPORT jfloat JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeGetAdaptiveFilterConvergence(JNIEnv *env, jobject thiz) {
    return g_processor ? g_processor->GetAdaptiveFilterConvergence() : 0.0f;
}

/**
 * 自动优化延迟以获得最佳ERLE性能
 * @return 优化成功完成则返回true
 */
JNIEXPORT jboolean JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeAutoOptimizeDelay(JNIEnv *env, jobject thiz) {
    return g_processor ? g_processor->AutoOptimizeDelay() : JNI_FALSE;
}

// ========== 增强ERLE优化JNI方法 ==========

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
 * 注意：此方法已弃用，使用自动适配模式
 * @param blocks 1-100范围，默认=25
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetFilterLengthBlocks(JNIEnv *env, jobject thiz, jint blocks) {
    // 已弃用 - 使用自动适配模式
}

/**
 * 设置收敛时的滤波器泄漏以保持稳定性
 * 注意：此方法已弃用，使用自动适配模式
 * @param leakage 0.000001-1.0范围，默认=0.000005
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetFilterLeakageConverged(JNIEnv *env, jobject thiz, jfloat leakage) {
    // 已弃用 - 使用自动适配模式
}

/**
 * 设置发散时的滤波器泄漏以进行恢复
 * 注意：此方法已弃用，使用自动适配模式
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetFilterLeakageDiverged(JNIEnv *env, jobject thiz, jfloat leakage) {
    // 已弃用 - 使用自动适配模式
}

/**
 * 设置延迟估计下采样因子以提高精度
 * 注意：此方法已弃用，使用自动适配模式
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetDelayDownSamplingFactor(JNIEnv *env, jobject thiz, jint factor) {
    // 已弃用 - 使用自动适配模式
}

/**
 * 设置延迟估计滤波器数量
 * 注意：此方法已弃用，使用自动适配模式
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetDelayNumFilters(JNIEnv *env, jobject thiz, jint filters) {
    // 已弃用 - 使用自动适配模式
}

/**
 * 设置延迟估计平滑因子以保持稳定性
 * 注意：此方法已弃用，使用自动适配模式
 */
JNIEXPORT void JNICALL
Java_cn_watchfun_aec3_WqAecProcessor_nativeSetDelayEstimateSmoothing(JNIEnv *env, jobject thiz, jfloat smoothing) {
    // 已弃用 - 使用自动适配模式
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
    
    // 转换为WAV格式
    uint8_t* wavData = nullptr;
    size_t wavSize = 0;
    int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToWAV(
        audioFrames, 48000, &wavData, &wavSize, outputSampleRate);
    
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
    
    // 转换为PCM格式
    uint8_t* pcmData = nullptr;
    size_t pcmSize = 0;
    int result = webrtc_aec3_tts::WqAec3Convertor::convertCleanAudioToPCM(
        audioFrames, 48000, &pcmData, &pcmSize, outputSampleRate);
    
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



} // extern "C"
