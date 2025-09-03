#ifndef WQ_AEC3_C_API_H
#define WQ_AEC3_C_API_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque pointer to the AEC3 processor
typedef void* WqAec3ProcessorHandle;

/**
 * Create and initialize AEC3 processor
 * @return Handle to the processor, or NULL on failure
 */
WqAec3ProcessorHandle wq_aec3_processor_create(void);

/**
 * Initialize the AEC3 processor
 * @param handle Processor handle
 * @return true on success, false on failure
 */
bool wq_aec3_processor_initialize(WqAec3ProcessorHandle handle);

/**
 * Set stream delay for the AEC3 processor
 * @param handle Processor handle
 * @param delay_ms Delay in milliseconds
 */
void wq_aec3_processor_set_stream_delay(WqAec3ProcessorHandle handle, int delay_ms);

/**
 * Process TTS audio bytes (reference signal)
 * @param handle Processor handle
 * @param audio_data Audio data bytes
 * @param data_length Length of audio data
 * @return true on success, false on failure
 */
bool wq_aec3_processor_process_tts_audio_bytes(WqAec3ProcessorHandle handle, 
                                               const uint8_t* audio_data, 
                                               int data_length);

/**
 * Process microphone audio bytes with echo cancellation
 * @param handle Processor handle
 * @param mic_data Microphone audio data bytes
 * @param data_length Length of microphone data
 * @param enable_aec Whether to enable echo cancellation
 * @param output_data Output buffer for clean audio
 * @return true on success, false on failure
 */
bool wq_aec3_processor_process_microphone_audio_bytes(WqAec3ProcessorHandle handle,
                                                      const uint8_t* mic_data,
                                                      int data_length,
                                                      bool enable_aec,
                                                      uint8_t* output_data);

/**
 * Convert clean audio frames to WAV format
 * @param audio_frames Array of audio frame pointers
 * @param frame_sizes Array of frame sizes
 * @param frame_count Number of frames
 * @param input_sample_rate Input sample rate
 * @param output_sample_rate Output sample rate
 * @param wav_data Output WAV data pointer
 * @param wav_size Output WAV data size
 * @return true on success, false on failure
 */
bool wq_aec3_convertor_convert_clean_audio_to_wav_bytes(const uint8_t** audio_frames,
                                                        const int* frame_sizes,
                                                        int frame_count,
                                                        int input_sample_rate,
                                                        int output_sample_rate,
                                                        uint8_t** wav_data,
                                                        int* wav_size);

/**
 * Destroy the AEC3 processor
 * @param handle Processor handle
 */
void wq_aec3_processor_destroy(WqAec3ProcessorHandle handle);

#ifdef __cplusplus
}
#endif

#endif // WQ_AEC3_C_API_H
