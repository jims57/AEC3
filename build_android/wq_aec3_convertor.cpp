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


int WqAec3Convertor::convertCleanAudioToWAV(const std::vector<std::vector<uint8_t>>& audioFramesBytes,
                                           int inputSampleRate,
                                           uint8_t** outputWavData,
                                           size_t* outputSize,
                                           int outputSampleRate) {
    if (audioFramesBytes.empty() || !outputWavData || !outputSize) {
        LOGE("convertCleanAudioToWAV: Invalid parameters");
        return -1;
    }

    if (!isSupportedSampleRate(outputSampleRate)) {
        LOGE("convertCleanAudioToWAV: Unsupported output sample rate: %d", outputSampleRate);
        return -2;
    }

    try {
        // Step 1: Combine all audio frame bytes into single vector
        std::vector<uint8_t> combinedBytes;
        size_t totalBytes = 0;
        for (const auto& frame : audioFramesBytes) {
            totalBytes += frame.size();
        }
        combinedBytes.reserve(totalBytes);
        
        for (const auto& frame : audioFramesBytes) {
            combinedBytes.insert(combinedBytes.end(), frame.begin(), frame.end());
        }

        LOGI("convertCleanAudioToWAV: Combined %zu frames into %zu bytes", audioFramesBytes.size(), totalBytes);

        // Step 2: Convert bytes to 16-bit samples for resampling
        size_t numSamples = totalBytes / 2;
        std::vector<float> floatSamples;
        floatSamples.reserve(numSamples);
        
        for (size_t i = 0; i < totalBytes; i += 2) {
            int16_t sample = static_cast<int16_t>(combinedBytes[i] | (combinedBytes[i + 1] << 8));
            floatSamples.push_back(static_cast<float>(sample) / 32767.0f);
        }

        // Step 3: Resample if needed
        std::vector<float> resampledAudio;
        if (inputSampleRate != outputSampleRate) {
            resampledAudio = resampleAudio(floatSamples, inputSampleRate, outputSampleRate);
            LOGI("convertCleanAudioToWAV: Resampled from %dHz to %dHz, %zu->%zu samples", 
                 inputSampleRate, outputSampleRate, floatSamples.size(), resampledAudio.size());
        } else {
            resampledAudio = std::move(floatSamples);
        }

        // Step 4: Convert back to 16-bit PCM
        std::vector<int16_t> pcmSamples;
        pcmSamples.reserve(resampledAudio.size());
        for (float sample : resampledAudio) {
            float clamped = std::max(-1.0f, std::min(1.0f, sample));
            int16_t pcmSample = static_cast<int16_t>(clamped * 32767.0f);
            pcmSamples.push_back(pcmSample);
        }

        // Step 5: Calculate WAV file size
        size_t pcmDataSize = pcmSamples.size() * sizeof(int16_t);
        size_t wavHeaderSize = 44;
        size_t totalWavSize = wavHeaderSize + pcmDataSize;

        // Step 6: Allocate output buffer
        *outputWavData = static_cast<uint8_t*>(malloc(totalWavSize));
        if (!*outputWavData) {
            LOGE("convertCleanAudioToWAV: Failed to allocate output buffer");
            return -3;
        }

        uint8_t* buffer = *outputWavData;
        
        // Step 7: Write WAV header using writeWavHeader method
        int headerResult = writeWavHeader(buffer, pcmDataSize, outputSampleRate);
        if (headerResult != 0) {
            free(*outputWavData);
            *outputWavData = nullptr;
            return headerResult;
        }
        
        // Step 8: Write PCM data
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


int WqAec3Convertor::convertCleanAudioToPCM(const std::vector<std::vector<uint8_t>>& audioFramesBytes,
                                           int inputSampleRate,
                                           uint8_t** outputPcmData,
                                           size_t* outputSize,
                                           int outputSampleRate) {
    if (audioFramesBytes.empty() || !outputPcmData || !outputSize) {
        LOGE("convertCleanAudioToPCM: Invalid parameters");
        return -1;
    }

    if (!isSupportedSampleRate(outputSampleRate)) {
        LOGE("convertCleanAudioToPCM: Unsupported output sample rate: %d", outputSampleRate);
        return -2;
    }

    try {
        // Step 1: Combine all audio frame bytes into single vector
        std::vector<uint8_t> combinedBytes;
        size_t totalBytes = 0;
        for (const auto& frame : audioFramesBytes) {
            totalBytes += frame.size();
        }
        combinedBytes.reserve(totalBytes);
        
        for (const auto& frame : audioFramesBytes) {
            combinedBytes.insert(combinedBytes.end(), frame.begin(), frame.end());
        }

        LOGI("convertCleanAudioToPCM: Combined %zu frames into %zu bytes", audioFramesBytes.size(), totalBytes);

        // Step 2: Convert bytes to 16-bit samples for resampling
        size_t numSamples = totalBytes / 2;
        std::vector<float> floatSamples;
        floatSamples.reserve(numSamples);
        
        for (size_t i = 0; i < totalBytes; i += 2) {
            int16_t sample = static_cast<int16_t>(combinedBytes[i] | (combinedBytes[i + 1] << 8));
            floatSamples.push_back(static_cast<float>(sample) / 32767.0f);
        }

        // Step 3: Resample if needed
        std::vector<float> resampledAudio;
        if (inputSampleRate != outputSampleRate) {
            resampledAudio = resampleAudio(floatSamples, inputSampleRate, outputSampleRate);
            LOGI("convertCleanAudioToPCM: Resampled from %dHz to %dHz, %zu->%zu samples", 
                 inputSampleRate, outputSampleRate, floatSamples.size(), resampledAudio.size());
        } else {
            resampledAudio = std::move(floatSamples);
        }

        // Step 4: Convert back to 16-bit PCM
        size_t pcmDataSize = resampledAudio.size() * sizeof(int16_t);
        
        // Step 5: Allocate output buffer
        *outputPcmData = static_cast<uint8_t*>(malloc(pcmDataSize));
        if (!*outputPcmData) {
            LOGE("convertCleanAudioToPCM: Failed to allocate output buffer");
            return -3;
        }

        uint8_t* pcmBuffer = *outputPcmData;
        
        // Step 6: Convert and write PCM data (little-endian format)
        for (size_t i = 0; i < resampledAudio.size(); ++i) {
            float clamped = std::max(-1.0f, std::min(1.0f, resampledAudio[i]));
            int16_t pcmSample = static_cast<int16_t>(clamped * 32767.0f);
            writeInt16LE(pcmBuffer + (i * 2), static_cast<uint16_t>(pcmSample));
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

int WqAec3Convertor::writeWavHeader(uint8_t* buffer,
                                   size_t audioDataSize,
                                   int sampleRate,
                                   int channels,
                                   int bitsPerSample) {
    if (!buffer) {
        LOGE("writeWavHeader: Invalid buffer");
        return -1;
    }
    
    if (!isSupportedSampleRate(sampleRate)) {
        LOGE("writeWavHeader: Unsupported sample rate: %d", sampleRate);
        return -2;
    }
    
    if (channels < 1 || channels > 2) {
        LOGE("writeWavHeader: Invalid channels: %d", channels);
        return -3;
    }
    
    if (bitsPerSample != 16 && bitsPerSample != 24 && bitsPerSample != 32) {
        LOGE("writeWavHeader: Unsupported bits per sample: %d", bitsPerSample);
        return -4;
    }
    
    try {
        int bytesPerSample = bitsPerSample / 8;
        int byteRate = sampleRate * channels * bytesPerSample;
        int blockAlign = channels * bytesPerSample;
        uint32_t fileSize = static_cast<uint32_t>(36 + audioDataSize);
        
        // RIFF header
        memcpy(buffer, "RIFF", 4);
        writeInt32LE(buffer + 4, fileSize);
        memcpy(buffer + 8, "WAVE", 4);
        
        // fmt chunk
        memcpy(buffer + 12, "fmt ", 4);
        writeInt32LE(buffer + 16, 16);                    // fmt chunk size
        writeInt16LE(buffer + 20, 1);                     // PCM format
        writeInt16LE(buffer + 22, static_cast<uint16_t>(channels));
        writeInt32LE(buffer + 24, static_cast<uint32_t>(sampleRate));
        writeInt32LE(buffer + 28, static_cast<uint32_t>(byteRate));
        writeInt16LE(buffer + 32, static_cast<uint16_t>(blockAlign));
        writeInt16LE(buffer + 34, static_cast<uint16_t>(bitsPerSample));
        
        // data chunk
        memcpy(buffer + 36, "data", 4);
        writeInt32LE(buffer + 40, static_cast<uint32_t>(audioDataSize));
        
        LOGI("writeWavHeader: Created WAV header: %dHz, %d channels, %d bits, %zu bytes audio data", 
             sampleRate, channels, bitsPerSample, audioDataSize);
        
        return 0; // Success
        
    } catch (const std::exception& e) {
        LOGE("writeWavHeader: Exception: %s", e.what());
        return -5;
    }
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
