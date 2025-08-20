#pragma once

#include <vector>
#include <cstdint>

namespace webrtc_aec3_tts {

/**
 * Audio format converter for clean audio output (2025-01-31)
 * Provides methods to convert clean audio frames to WAV and PCM formats
 */
class WqAec3Convertor {
public:
    /**
     * Convert clean audio frames to WAV format in memory
     * @param audioFrames Vector of float audio frames (normalized to [-1.0, 1.0])
     * @param inputSampleRate Sample rate of input audio frames
     * @param outputSampleRate Desired output sample rate (default: 44100)
     * @param outputWavData Output buffer for WAV data
     * @param outputSize Size of output WAV data in bytes
     * @return 0 on success, negative error code on failure
     */
    static int convertCleanAudioToWAV(const std::vector<std::vector<float>>& audioFrames,
                                     int inputSampleRate,
                                     uint8_t** outputWavData,
                                     size_t* outputSize,
                                     int outputSampleRate = 44100);

    /**
     * Convert clean audio frames to PCM format in memory
     * @param audioFrames Vector of float audio frames (normalized to [-1.0, 1.0])
     * @param inputSampleRate Sample rate of input audio frames
     * @param outputSampleRate Desired output sample rate (default: 44100)
     * @param outputPcmData Output buffer for PCM data
     * @param outputSize Size of output PCM data in bytes
     * @return 0 on success, negative error code on failure
     */
    static int convertCleanAudioToPCM(const std::vector<std::vector<float>>& audioFrames,
                                     int inputSampleRate,
                                     uint8_t** outputPcmData,
                                     size_t* outputSize,
                                     int outputSampleRate = 44100);

private:
    /**
     * Resample audio data using linear interpolation
     * @param inputData Input audio samples
     * @param inputSampleRate Input sample rate
     * @param outputSampleRate Output sample rate
     * @return Resampled audio data
     */
    static std::vector<float> resampleAudio(const std::vector<float>& inputData,
                                          int inputSampleRate,
                                          int outputSampleRate);

    /**
     * Write 32-bit integer to buffer in little-endian format
     */
    static void writeInt32LE(uint8_t* buffer, uint32_t value);

    /**
     * Write 16-bit integer to buffer in little-endian format
     */
    static void writeInt16LE(uint8_t* buffer, uint16_t value);
};

} // namespace webrtc_aec3_tts
