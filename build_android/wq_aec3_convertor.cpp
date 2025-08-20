#include "wq_aec3_convertor.h"
#include <cstring>
#include <algorithm>
#include <cmath>

#ifdef ANDROID
#include <android/log.h>
#define LOG_TAG "WebRTC_AEC3_TTS"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGV(...) __android_log_print(ANDROID_LOG_VERBOSE, LOG_TAG, __VA_ARGS__)
#else
#include <cstdio>
#define LOGI(...) printf(__VA_ARGS__); printf("\n")
#define LOGE(...) printf(__VA_ARGS__); printf("\n")
#define LOGV(...) printf(__VA_ARGS__); printf("\n")
#endif

namespace webrtc_aec3_tts {

// Industry standard sample rates
static const int SUPPORTED_SAMPLE_RATES[] = {
    8000, 11025, 16000, 22050, 32000, 44100, 48000, 88200, 96000, 176400, 192000
};

static bool isSupportedSampleRate(int sampleRate) {
    for (int rate : SUPPORTED_SAMPLE_RATES) {
        if (rate == sampleRate) return true;
    }
    return false;
}

int WqAec3Convertor::convertCleanAudioToWAV(const std::vector<std::vector<float>>& audioFrames,
                                           int inputSampleRate,
                                           uint8_t** outputWavData,
                                           size_t* outputSize,
                                           int outputSampleRate) {
    if (audioFrames.empty() || !outputWavData || !outputSize) {
        LOGE("convertCleanAudioToWAV: Invalid parameters");
        return -1;
    }

    if (!isSupportedSampleRate(outputSampleRate)) {
        LOGE("convertCleanAudioToWAV: Unsupported output sample rate: %d", outputSampleRate);
        return -2;
    }

    try {
        // Step 1: Combine all audio frames into single vector
        std::vector<float> combinedAudio;
        size_t totalSamples = 0;
        for (const auto& frame : audioFrames) {
            totalSamples += frame.size();
        }
        combinedAudio.reserve(totalSamples);
        
        for (const auto& frame : audioFrames) {
            combinedAudio.insert(combinedAudio.end(), frame.begin(), frame.end());
        }

        LOGI("convertCleanAudioToWAV: Combined %zu frames into %zu samples", audioFrames.size(), totalSamples);

        // Step 2: Resample if needed
        std::vector<float> resampledAudio;
        if (inputSampleRate != outputSampleRate) {
            resampledAudio = resampleAudio(combinedAudio, inputSampleRate, outputSampleRate);
            LOGI("convertCleanAudioToWAV: Resampled from %dHz to %dHz, %zu->%zu samples", 
                 inputSampleRate, outputSampleRate, combinedAudio.size(), resampledAudio.size());
        } else {
            resampledAudio = std::move(combinedAudio);
        }

        // Step 3: Convert float samples to 16-bit PCM
        std::vector<int16_t> pcmSamples;
        pcmSamples.reserve(resampledAudio.size());
        for (float sample : resampledAudio) {
            // Clamp to [-1.0, 1.0] and convert to 16-bit
            float clamped = std::max(-1.0f, std::min(1.0f, sample));
            int16_t pcmSample = static_cast<int16_t>(clamped * 32767.0f);
            pcmSamples.push_back(pcmSample);
        }

        // Step 4: Calculate WAV file size
        size_t pcmDataSize = pcmSamples.size() * sizeof(int16_t);
        size_t wavHeaderSize = 44;
        size_t totalWavSize = wavHeaderSize + pcmDataSize;

        // Step 5: Allocate output buffer
        *outputWavData = static_cast<uint8_t*>(malloc(totalWavSize));
        if (!*outputWavData) {
            LOGE("convertCleanAudioToWAV: Failed to allocate output buffer");
            return -3;
        }

        uint8_t* buffer = *outputWavData;
        
        // Step 6: Write WAV header
        // RIFF header
        memcpy(buffer, "RIFF", 4);
        writeInt32LE(buffer + 4, static_cast<uint32_t>(totalWavSize - 8));
        memcpy(buffer + 8, "WAVE", 4);
        
        // fmt chunk
        memcpy(buffer + 12, "fmt ", 4);
        writeInt32LE(buffer + 16, 16);                    // fmt chunk size
        writeInt16LE(buffer + 20, 1);                     // PCM format
        writeInt16LE(buffer + 22, 1);                     // mono
        writeInt32LE(buffer + 24, outputSampleRate);      // sample rate
        writeInt32LE(buffer + 28, outputSampleRate * 2);  // byte rate
        writeInt16LE(buffer + 32, 2);                     // block align
        writeInt16LE(buffer + 34, 16);                    // bits per sample
        
        // data chunk
        memcpy(buffer + 36, "data", 4);
        writeInt32LE(buffer + 40, static_cast<uint32_t>(pcmDataSize));
        
        // Step 7: Write PCM data
        for (size_t i = 0; i < pcmSamples.size(); ++i) {
            writeInt16LE(buffer + wavHeaderSize + (i * 2), static_cast<uint16_t>(pcmSamples[i]));
        }

        *outputSize = totalWavSize;
        
        LOGI("convertCleanAudioToWAV: Created WAV file in memory: %zu bytes, %dHz, %zu samples", 
             totalWavSize, outputSampleRate, pcmSamples.size());
        
        return 0; // Success

    } catch (const std::exception& e) {
        LOGE("convertCleanAudioToWAV: Exception: %s", e.what());
        return -4;
    }
}

int WqAec3Convertor::convertCleanAudioToPCM(const std::vector<std::vector<float>>& audioFrames,
                                           int inputSampleRate,
                                           uint8_t** outputPcmData,
                                           size_t* outputSize,
                                           int outputSampleRate) {
    if (audioFrames.empty() || !outputPcmData || !outputSize) {
        LOGE("convertCleanAudioToPCM: Invalid parameters");
        return -1;
    }

    if (!isSupportedSampleRate(outputSampleRate)) {
        LOGE("convertCleanAudioToPCM: Unsupported output sample rate: %d", outputSampleRate);
        return -2;
    }

    try {
        // Step 1: Combine all audio frames into single vector
        std::vector<float> combinedAudio;
        size_t totalSamples = 0;
        for (const auto& frame : audioFrames) {
            totalSamples += frame.size();
        }
        combinedAudio.reserve(totalSamples);
        
        for (const auto& frame : audioFrames) {
            combinedAudio.insert(combinedAudio.end(), frame.begin(), frame.end());
        }

        LOGI("convertCleanAudioToPCM: Combined %zu frames into %zu samples", audioFrames.size(), totalSamples);

        // Step 2: Resample if needed
        std::vector<float> resampledAudio;
        if (inputSampleRate != outputSampleRate) {
            resampledAudio = resampleAudio(combinedAudio, inputSampleRate, outputSampleRate);
            LOGI("convertCleanAudioToPCM: Resampled from %dHz to %dHz, %zu->%zu samples", 
                 inputSampleRate, outputSampleRate, combinedAudio.size(), resampledAudio.size());
        } else {
            resampledAudio = std::move(combinedAudio);
        }

        // Step 3: Convert float samples to 16-bit PCM
        size_t pcmDataSize = resampledAudio.size() * sizeof(int16_t);
        
        // Step 4: Allocate output buffer
        *outputPcmData = static_cast<uint8_t*>(malloc(pcmDataSize));
        if (!*outputPcmData) {
            LOGE("convertCleanAudioToPCM: Failed to allocate output buffer");
            return -3;
        }

        int16_t* pcmBuffer = reinterpret_cast<int16_t*>(*outputPcmData);
        
        // Step 5: Convert and write PCM data
        for (size_t i = 0; i < resampledAudio.size(); ++i) {
            // Clamp to [-1.0, 1.0] and convert to 16-bit
            float clamped = std::max(-1.0f, std::min(1.0f, resampledAudio[i]));
            pcmBuffer[i] = static_cast<int16_t>(clamped * 32767.0f);
        }

        *outputSize = pcmDataSize;
        
        LOGI("convertCleanAudioToPCM: Created PCM data in memory: %zu bytes, %dHz, %zu samples", 
             pcmDataSize, outputSampleRate, resampledAudio.size());
        
        return 0; // Success

    } catch (const std::exception& e) {
        LOGE("convertCleanAudioToPCM: Exception: %s", e.what());
        return -4;
    }
}

std::vector<float> WqAec3Convertor::resampleAudio(const std::vector<float>& inputData,
                                                 int inputSampleRate,
                                                 int outputSampleRate) {
    if (inputSampleRate == outputSampleRate) {
        return inputData;
    }

    double ratio = static_cast<double>(outputSampleRate) / inputSampleRate;
    size_t outputLength = static_cast<size_t>(inputData.size() * ratio);
    std::vector<float> outputData;
    outputData.reserve(outputLength);

    for (size_t i = 0; i < outputLength; ++i) {
        double srcIndex = i / ratio;
        size_t index = static_cast<size_t>(srcIndex);
        double fraction = srcIndex - index;

        if (index < inputData.size() - 1) {
            // Linear interpolation
            float interpolated = static_cast<float>(
                inputData[index] * (1.0 - fraction) + inputData[index + 1] * fraction);
            outputData.push_back(interpolated);
        } else if (index < inputData.size()) {
            outputData.push_back(inputData[index]);
        } else {
            outputData.push_back(0.0f);
        }
    }

    return outputData;
}

void WqAec3Convertor::writeInt32LE(uint8_t* buffer, uint32_t value) {
    buffer[0] = static_cast<uint8_t>(value & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

void WqAec3Convertor::writeInt16LE(uint8_t* buffer, uint16_t value) {
    buffer[0] = static_cast<uint8_t>(value & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

} // namespace webrtc_aec3_tts
