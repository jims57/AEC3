#pragma once

#include <vector>
#include <cstdint>

namespace webrtc_aec3_tts {

/**
 * 清洁音频输出的音频格式转换器
 * 提供将清洁音频帧转换为WAV和PCM格式的方法
 */
class WqAec3Convertor {
public:
    /**
     * 将清洁音频帧转换为内存中的WAV格式
     * @param audioFramesBytes 音频帧字节数据向量
     * @param inputSampleRate 输入音频帧的采样率
     * @param outputSampleRate 期望的输出采样率（默认：44100）
     * @param outputWavData WAV数据的输出缓冲区
     * @param outputSize 输出WAV数据的字节大小
     * @return 成功时返回0，失败时返回负错误码
     */
    static int convertCleanAudioToWAV(const std::vector<std::vector<uint8_t>>& audioFramesBytes,
                                     int inputSampleRate,
                                     uint8_t** outputWavData,
                                     size_t* outputSize,
                                     int outputSampleRate = 44100);

    /**
     * 将清洁音频帧转换为内存中的WAV格式 - 字节数组版本
     * @param audioFramesBytes 音频帧字节数据向量
     * @param inputSampleRate 输入音频帧的采样率
     * @param outputSampleRate 期望的输出采样率（默认：44100）
     * @param outputWavData WAV数据的输出缓冲区
     * @param outputSize 输出WAV数据的字节大小
     * @return 成功时返回0，失败时返回负错误码
     */
    static int convertCleanAudioToWAVBytes(const std::vector<std::vector<uint8_t>>& audioFramesBytes,
                                          int inputSampleRate,
                                          uint8_t** outputWavData,
                                          size_t* outputSize,
                                          int outputSampleRate = 44100);

    /**
     * 将清洁音频帧转换为内存中的PCM格式 - 浮点版本（向后兼容）
     * @param audioFrames 浮点音频帧向量（归一化到[-1.0, 1.0]）
     * @param inputSampleRate 输入音频帧的采样率
     * @param outputSampleRate 期望的输出采样率（默认：44100）
     * @param outputPcmData PCM数据的输出缓冲区
     * @param outputSize 输出PCM数据的字节大小
     * @return 成功时返回0，失败时返回负错误码
     */
    static int convertCleanAudioToPCM(const std::vector<std::vector<uint8_t>>& audioFramesBytes,
                                     int inputSampleRate,
                                     uint8_t** outputPcmData,
                                     size_t* outputSize,
                                     int outputSampleRate = 44100);

    /**
     * 将清洁音频帧转换为内存中的PCM格式 - 字节数组版本
     * @param audioFramesBytes 音频帧字节数据向量
     * @param inputSampleRate 输入音频帧的采样率
     * @param outputSampleRate 期望的输出采样率（默认：44100）
     * @param outputPcmData PCM数据的输出缓冲区
     * @param outputSize 输出PCM数据的字节大小
     * @return 成功时返回0，失败时返回负错误码
     */
    static int convertCleanAudioToPCMBytes(const std::vector<std::vector<uint8_t>>& audioFramesBytes,
                                          int inputSampleRate,
                                          uint8_t** outputPcmData,
                                          size_t* outputSize,
                                          int outputSampleRate = 44100);

    /**
     * 写入WAV文件头到字节缓冲区
     * @param buffer 输出缓冲区（至少44字节）
     * @param audioDataSize 音频数据大小（字节）
     * @param sampleRate 采样率
     * @param channels 声道数（默认：1）
     * @param bitsPerSample 每样本位数（默认：16）
     * @return 成功时返回0，失败时返回负错误码
     */
    static int writeWavHeader(uint8_t* buffer,
                             size_t audioDataSize,
                             int sampleRate,
                             int channels = 1,
                             int bitsPerSample = 16);

private:
    /**
     * 使用线性插值重新采样音频数据
     * @param inputData 输入音频样本
     * @param inputSampleRate 输入采样率
     * @param outputSampleRate 输出采样率
     * @return 重新采样的音频数据
     */
    static std::vector<float> resampleAudio(const std::vector<float>& inputData,
                                          int inputSampleRate,
                                          int outputSampleRate);

    /**
     * 以小端格式将32位整数写入缓冲区
     */
    static void writeInt32LE(uint8_t* buffer, uint32_t value);

    /**
     * 以小端格式将16位整数写入缓冲区
     */
    static void writeInt16LE(uint8_t* buffer, uint16_t value);
};

} // namespace webrtc_aec3_tts
