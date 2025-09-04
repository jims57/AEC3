#ifndef WQDENOISERWRAPPER_H
#define WQDENOISERWRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// iOS AEC3处理器句柄
typedef void* WQDenoiserHandle;

// 音频配置常量
#define WQ_SAMPLE_RATE 48000
#define WQ_FRAME_SIZE 480
#define WQ_CHANNELS 1
#define WQ_STREAM_DELAY 100

// 核心AEC3方法
WQDenoiserHandle wq_denoiser_create(void);
bool wq_denoiser_initialize(WQDenoiserHandle handle);
void wq_denoiser_destroy(WQDenoiserHandle handle);

// 音频处理方法
bool wq_denoiser_process_tts_audio(WQDenoiserHandle handle, const int16_t* tts_data, size_t length);
bool wq_denoiser_process_microphone_audio(WQDenoiserHandle handle, const int16_t* mic_data, int16_t* output_data, size_t length);

// 字节数组版本（与Android AAR保持一致）
bool wq_denoiser_process_tts_audio_bytes(WQDenoiserHandle handle, const uint8_t* tts_byte_data, size_t byte_length);
bool wq_denoiser_process_microphone_audio_bytes(WQDenoiserHandle handle, const uint8_t* mic_byte_data, uint8_t* output_byte_data, size_t byte_length, bool enable_aec);

// 性能指标
bool wq_denoiser_get_metrics(WQDenoiserHandle handle, double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms);
bool wq_denoiser_get_enhanced_metrics(WQDenoiserHandle handle, double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms, uint64_t* render_frames, uint64_t* capture_frames, int* optimal_delay);

// 配置方法
void wq_denoiser_set_stream_delay(WQDenoiserHandle handle, int delay_ms);
bool wq_denoiser_enable_timing_sync(WQDenoiserHandle handle, bool enable);
bool wq_denoiser_auto_optimize_delay(WQDenoiserHandle handle);

// AEC3参数控制（与Android AAR保持一致）
void wq_denoiser_set_config_change_duration(WQDenoiserHandle handle, int blocks);
void wq_denoiser_set_initial_state_seconds(WQDenoiserHandle handle, float seconds);
void wq_denoiser_set_conservative_initial_phase(WQDenoiserHandle handle, bool enable);
void wq_denoiser_set_max_dec_factor_lf(WQDenoiserHandle handle, float factor);
void wq_denoiser_set_max_inc_factor(WQDenoiserHandle handle, float factor);
void wq_denoiser_set_nearend_max_dec_factor_lf(WQDenoiserHandle handle, float factor);
void wq_denoiser_set_nearend_max_inc_factor(WQDenoiserHandle handle, float factor);
void wq_denoiser_set_enr_threshold(WQDenoiserHandle handle, float threshold);
void wq_denoiser_set_snr_threshold(WQDenoiserHandle handle, float threshold);
void wq_denoiser_set_hold_duration(WQDenoiserHandle handle, int duration);
void wq_denoiser_set_trigger_threshold(WQDenoiserHandle handle, int threshold);

// ERLE调整参数（与Android AAR保持一致）
void wq_denoiser_set_filter_length_blocks(WQDenoiserHandle handle, int blocks);
void wq_denoiser_set_filter_leakage_converged(WQDenoiserHandle handle, float leakage);
void wq_denoiser_set_filter_leakage_diverged(WQDenoiserHandle handle, float leakage);
void wq_denoiser_set_delay_down_sampling_factor(WQDenoiserHandle handle, int factor);
void wq_denoiser_set_delay_num_filters(WQDenoiserHandle handle, int filters);
void wq_denoiser_set_delay_estimate_smoothing(WQDenoiserHandle handle, float smoothing);

// 清洁音频转换方法
uint8_t* wq_denoiser_get_clean_audio_as_wav(WQDenoiserHandle handle, int output_sample_rate, size_t* output_size);
uint8_t* wq_denoiser_get_clean_audio_as_pcm(WQDenoiserHandle handle, int output_sample_rate, size_t* output_size);
void wq_denoiser_clear_clean_audio_buffer(WQDenoiserHandle handle);

// 数组转换工具
bool wq_denoiser_convert_byte_array_to_short_array(const uint8_t* byte_data, size_t byte_length, int16_t* short_data, size_t expected_short_length);
bool wq_denoiser_convert_short_array_to_byte_array(const int16_t* short_data, size_t short_length, uint8_t* byte_data, size_t expected_byte_length);

// WAV文件头写入
bool wq_denoiser_write_wav_header(uint8_t* buffer, size_t audio_data_size, int sample_rate, int channels, int bits_per_sample);

// 内存管理
void wq_denoiser_free_memory(void* ptr);

#ifdef __cplusplus
}
#endif

#endif // WQDENOISERWRAPPER_H
