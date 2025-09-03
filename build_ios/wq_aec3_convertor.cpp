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
    8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000, 88200, 96000, 176400, 192000
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

    try {
        // Step 1: Use convertCleanAudioToPCM to get PCM data
        uint8_t* pcmData = nullptr;
        size_t pcmSize = 0;
        int pcmResult = convertCleanAudioToPCM(audioFramesBytes, inputSampleRate, &pcmData, &pcmSize, outputSampleRate);
        
        if (pcmResult != 0 || !pcmData || pcmSize == 0) {
            LOGE("convertCleanAudioToWAV: Failed to convert to PCM, result=%d", pcmResult);
            if (pcmData) free(pcmData);
            return pcmResult;
        }
        
        LOGI("convertCleanAudioToWAV: Got PCM data: %zu bytes", pcmSize);

        // Step 2: Calculate WAV file size
        size_t wavHeaderSize = 44;
        size_t totalWavSize = wavHeaderSize + pcmSize;

        // Step 3: Allocate output buffer
        *outputWavData = static_cast<uint8_t*>(malloc(totalWavSize));
        if (!*outputWavData) {
            LOGE("convertCleanAudioToWAV: Failed to allocate output buffer");
            free(pcmData);
            return -3;
        }

        uint8_t* buffer = *outputWavData;
        
        // Step 4: Write WAV header using writeWavHeader method
        int headerResult = writeWavHeader(buffer, pcmSize, outputSampleRate);
        if (headerResult != 0) {
            LOGE("convertCleanAudioToWAV: Failed to write WAV header, result=%d", headerResult);
            free(*outputWavData);
            *outputWavData = nullptr;
            free(pcmData);
            return headerResult;
        }
        
        // Step 5: Copy PCM data after header
        memcpy(buffer + wavHeaderSize, pcmData, pcmSize);
        
        *outputSize = totalWavSize;
        
        LOGI("convertCleanAudioToWAV: Created WAV file in memory: %zu bytes, %dHz", 
             totalWavSize, outputSampleRate);
        
        free(pcmData);
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

        // Step 2: Resample if needed using new uint8_t method
        std::vector<uint8_t> resampledBytes;
        if (inputSampleRate != outputSampleRate) {
            resampledBytes = resampleAudio(combinedBytes, inputSampleRate, outputSampleRate);
            LOGI("convertCleanAudioToPCM: Resampled from %dHz to %dHz, %zu->%zu bytes", 
                 inputSampleRate, outputSampleRate, combinedBytes.size(), resampledBytes.size());
        } else {
            resampledBytes = std::move(combinedBytes);
        }

        // Step 3: Allocate output buffer
        size_t pcmDataSize = resampledBytes.size();
        *outputPcmData = static_cast<uint8_t*>(malloc(pcmDataSize));
        if (!*outputPcmData) {
            LOGE("convertCleanAudioToPCM: Failed to allocate output buffer");
            return -3;
        }

        // Step 4: Copy resampled data to output buffer
        memcpy(*outputPcmData, resampledBytes.data(), pcmDataSize);
        *outputSize = pcmDataSize;
        
        LOGI("convertCleanAudioToPCM: Created PCM data in memory: %zu bytes, %dHz", 
             pcmDataSize, outputSampleRate);
        
        return 0; // Success

    } catch (const std::exception& e) {
        LOGE("convertCleanAudioToPCM: Exception: %s", e.what());
        return -4;
    }
}

std::vector<uint8_t> WqAec3Convertor::resampleAudio(const std::vector<uint8_t>& inputData,
                                                   int inputSampleRate,
                                                   int outputSampleRate) {
    if (inputSampleRate == outputSampleRate) {
        return inputData;
    }

    // Convert bytes to 16-bit samples for processing
    size_t numInputSamples = inputData.size() / 2;
    std::vector<int16_t> inputSamples;
    inputSamples.reserve(numInputSamples);
    
    for (size_t i = 0; i < inputData.size(); i += 2) {
        int16_t sample = static_cast<int16_t>(inputData[i] | (inputData[i + 1] << 8));
        inputSamples.push_back(sample);
    }

    double ratio = static_cast<double>(outputSampleRate) / inputSampleRate;
    size_t outputLength = static_cast<size_t>(inputSamples.size() * ratio);
    std::vector<int16_t> outputSamples;
    outputSamples.reserve(outputLength);

    for (size_t i = 0; i < outputLength; ++i) {
        double srcIndex = i / ratio;
        size_t index = static_cast<size_t>(srcIndex);
        double fraction = srcIndex - index;

        if (index < inputSamples.size() - 1) {
            // Linear interpolation
            double interpolated = inputSamples[index] * (1.0 - fraction) + inputSamples[index + 1] * fraction;
            outputSamples.push_back(static_cast<int16_t>(std::round(interpolated)));
        } else if (index < inputSamples.size()) {
            outputSamples.push_back(inputSamples[index]);
        } else {
            outputSamples.push_back(0);
        }
    }

    // Convert back to bytes
    std::vector<uint8_t> outputData;
    outputData.reserve(outputSamples.size() * 2);
    
    for (int16_t sample : outputSamples) {
        outputData.push_back(static_cast<uint8_t>(sample & 0xFF));
        outputData.push_back(static_cast<uint8_t>((sample >> 8) & 0xFF));
    }

    return outputData;
}

// Static remainder buffer for resamplePCMTo480SampleChunks
static std::vector<uint8_t> g_remainderBuffer;

int WqAec3Convertor::resamplePCMTo480SampleChunks(const std::vector<uint8_t>& inputPcmData,
                                                 int inputSampleRate,
                                                 int outputSampleRate,
                                                 std::vector<std::vector<uint8_t>>& outputChunks,
                                                 bool hasMoreData) {
    if (inputPcmData.empty()) {
        LOGE("resamplePCMTo480SampleChunks: Empty input data");
        return -1;
    }
    
    if (!isSupportedSampleRate(inputSampleRate) || !isSupportedSampleRate(outputSampleRate)) {
        LOGE("resamplePCMTo480SampleChunks: Unsupported sample rates: input=%d, output=%d", 
             inputSampleRate, outputSampleRate);
        return -2;
    }
    
    try {
        // Step 1: Combine remainder buffer with new input data
        std::vector<uint8_t> combinedData;
        combinedData.reserve(g_remainderBuffer.size() + inputPcmData.size());
        combinedData.insert(combinedData.end(), g_remainderBuffer.begin(), g_remainderBuffer.end());
        combinedData.insert(combinedData.end(), inputPcmData.begin(), inputPcmData.end());
        
        LOGI("resamplePCMTo480SampleChunks: Combined %zu remainder + %zu input = %zu total bytes", 
             g_remainderBuffer.size(), inputPcmData.size(), combinedData.size());
        
        // Step 2: Resample to target sample rate if needed
        std::vector<uint8_t> resampledData;
        if (inputSampleRate != outputSampleRate) {
            resampledData = resampleAudio(combinedData, inputSampleRate, outputSampleRate);
            LOGI("resamplePCMTo480SampleChunks: Resampled from %dHz to %dHz, %zu->%zu bytes", 
                 inputSampleRate, outputSampleRate, combinedData.size(), resampledData.size());
        } else {
            resampledData = std::move(combinedData);
        }
        
        // Step 3: Split into 480-sample chunks (480 samples * 2 bytes = 960 bytes per chunk)
        const size_t samplesPerChunk = 480;
        const size_t bytesPerChunk = samplesPerChunk * 2; // 16-bit samples
        
        size_t numCompleteChunks = resampledData.size() / bytesPerChunk;
        size_t remainderBytes = resampledData.size() % bytesPerChunk;
        
        LOGI("resamplePCMTo480SampleChunks: Creating %zu complete chunks, %zu remainder bytes", 
             numCompleteChunks, remainderBytes);
        
        // Step 4: Create complete 480-sample chunks
        outputChunks.clear();
        outputChunks.reserve(numCompleteChunks);
        
        for (size_t i = 0; i < numCompleteChunks; ++i) {
            size_t startIdx = i * bytesPerChunk;
            std::vector<uint8_t> chunk(resampledData.begin() + startIdx, 
                                     resampledData.begin() + startIdx + bytesPerChunk);
            outputChunks.push_back(std::move(chunk));
        }
        
        // Step 5: Handle remainder data
        if (remainderBytes > 0) {
            size_t remainderStartIdx = numCompleteChunks * bytesPerChunk;
            
            if (hasMoreData) {
                // Store remainder for next call
                g_remainderBuffer.assign(resampledData.begin() + remainderStartIdx, resampledData.end());
                LOGI("resamplePCMTo480SampleChunks: Stored %zu remainder bytes for next call", remainderBytes);
            } else {
                // Flush remainder by padding with silence to make complete 480-sample chunk
                std::vector<uint8_t> finalChunk(resampledData.begin() + remainderStartIdx, resampledData.end());
                
                // Pad with silence (zeros) to reach 480 samples
                size_t paddingBytes = bytesPerChunk - remainderBytes;
                finalChunk.resize(bytesPerChunk, 0);
                
                outputChunks.push_back(std::move(finalChunk));
                g_remainderBuffer.clear();
                
                LOGI("resamplePCMTo480SampleChunks: Flushed final chunk with %zu padding bytes", paddingBytes);
            }
        } else {
            // No remainder, clear buffer if not expecting more data
            if (!hasMoreData) {
                g_remainderBuffer.clear();
            }
        }
        
        LOGI("resamplePCMTo480SampleChunks: Successfully created %zu chunks", outputChunks.size());
        return 0; // Success
        
    } catch (const std::exception& e) {
        LOGE("resamplePCMTo480SampleChunks: Exception: %s", e.what());
        return -3;
    }
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
