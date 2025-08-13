#include <jni.h>
#include <android/log.h>
#include <memory>
#include <vector>
#include <mutex>

#include "api/echo_canceller3_factory.h"
#include "api/echo_canceller3_config.h"
#include "audio_processing/audio_buffer.h"
#include "audio_processing/audio_frame.h"
#include "audio_processing/high_pass_filter.h"

#define LOG_TAG "WebRTC_AEC3_TTS"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

namespace webrtc_aec3_tts {

class TtsAec3Processor {
public:
    static constexpr int kSampleRate = 48000;
    static constexpr int kFrameSize = 480;  // 10ms at 48kHz
    static constexpr int kChannels = 1;     // Mono
    static constexpr int kStreamDelay = 100; // Android typical delay

    TtsAec3Processor() {
        Initialize();
    }

    ~TtsAec3Processor() {
        std::lock_guard<std::mutex> lock(mutex_);
        echo_controller_.reset();
        hp_filter_.reset();
        render_buffer_.reset();
        capture_buffer_.reset();
    }

    bool Initialize() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        try {
            // Configure AEC3 for TTS echo cancellation
            webrtc::EchoCanceller3Config config;
            config.filter.export_linear_aec_output = true;
            config.delay.default_delay = 5;
            config.delay.delay_headroom_samples = 32;
            config.delay.fixed_capture_delay_samples = kStreamDelay * kSampleRate / 1000;
            
            // Mobile optimization
            config.filter.main.length_blocks = 8;  // Shorter for mobile
            config.filter.shadow.length_blocks = 8;
            config.suppression_gain.normal_tuning.max_dec_factor_lf = 0.25f;
            
            // Create AEC3 factory and controller
            aec_factory_ = std::make_unique<webrtc::EchoCanceller3Factory>(config);
            echo_controller_ = aec_factory_->Create(kSampleRate, kChannels, kChannels);
            
            if (!echo_controller_) {
                LOGE("Failed to create AEC3 controller");
                return false;
            }

            // Create high-pass filter
            hp_filter_ = std::make_unique<webrtc::HighPassFilter>(kSampleRate, kChannels);

            // Create audio buffers
            render_buffer_ = std::make_unique<webrtc::AudioBuffer>(
                kSampleRate, kChannels, kSampleRate, kChannels, kSampleRate, kChannels);
            capture_buffer_ = std::make_unique<webrtc::AudioBuffer>(
                kSampleRate, kChannels, kSampleRate, kChannels, kSampleRate, kChannels);

            // Set initial delay
            echo_controller_->SetAudioBufferDelay(kStreamDelay);

            LOGI("TTS AEC3 initialized: %dHz, %d channels, %dms delay", 
                 kSampleRate, kChannels, kStreamDelay);
            return true;
        } catch (const std::exception& e) {
            LOGE("Exception during initialization: %s", e.what());
            return false;
        }
    }

    // Process TTS audio (reference signal) - call this BEFORE playback
    bool ProcessTtsAudio(const int16_t* tts_data, size_t length) {
        if (!echo_controller_ || length != kFrameSize) {
            LOGE("Invalid TTS data: length=%zu, expected=%d", length, kFrameSize);
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        
        try {
            // Create audio frame for TTS data
            webrtc::AudioFrame tts_frame;
            tts_frame.UpdateFrame(0, tts_data, kFrameSize, kSampleRate, 
                                webrtc::AudioFrame::kNormalSpeech, 
                                webrtc::AudioFrame::kVadActive, kChannels);

            // Copy to render buffer and process
            render_buffer_->CopyFrom(&tts_frame);
            render_buffer_->SplitIntoFrequencyBands();
            echo_controller_->AnalyzeRender(render_buffer_.get());
            render_buffer_->MergeFrequencyBands();

            return true;
        } catch (const std::exception& e) {
            LOGE("Exception in ProcessTtsAudio: %s", e.what());
            return false;
        }
    }

    // Process microphone audio and remove echo
    bool ProcessMicrophoneAudio(const int16_t* mic_data, int16_t* output_data, size_t length) {
        if (!echo_controller_ || length != kFrameSize) {
            LOGE("Invalid mic data: length=%zu, expected=%d", length, kFrameSize);
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        
        try {
            // Create audio frame for microphone data
            webrtc::AudioFrame mic_frame;
            mic_frame.UpdateFrame(0, mic_data, kFrameSize, kSampleRate, 
                                webrtc::AudioFrame::kNormalSpeech, 
                                webrtc::AudioFrame::kVadActive, kChannels);

            // Copy to capture buffer
            capture_buffer_->CopyFrom(&mic_frame);

            // Analyze capture signal
            echo_controller_->AnalyzeCapture(capture_buffer_.get());

            // Apply frequency domain processing
            capture_buffer_->SplitIntoFrequencyBands();

            // Apply high-pass filter
            hp_filter_->Process(capture_buffer_.get(), true);

            // Perform echo cancellation
            echo_controller_->ProcessCapture(capture_buffer_.get(), false);

            // Merge back to time domain
            capture_buffer_->MergeFrequencyBands();

            // Copy processed audio to output
            webrtc::AudioFrame output_frame;
            capture_buffer_->CopyTo(&output_frame);
            memcpy(output_data, output_frame.data(), kFrameSize * sizeof(int16_t));

            return true;
        } catch (const std::exception& e) {
            LOGE("Exception in ProcessMicrophoneAudio: %s", e.what());
            return false;
        }
    }

    // Get current AEC metrics for monitoring
    bool GetMetrics(double* echo_return_loss, double* echo_return_loss_enhancement, int* delay_ms) {
        if (!echo_controller_) return false;
        
        std::lock_guard<std::mutex> lock(mutex_);
        
        try {
            auto metrics = echo_controller_->GetMetrics();
            *echo_return_loss = metrics.echo_return_loss;
            *echo_return_loss_enhancement = metrics.echo_return_loss_enhancement;
            *delay_ms = metrics.delay_ms;
            return true;
        } catch (const std::exception& e) {
            LOGE("Exception in GetMetrics: %s", e.what());
            return false;
        }
    }

    // Update stream delay if needed
    void SetStreamDelay(int delay_ms) {
        if (echo_controller_) {
            std::lock_guard<std::mutex> lock(mutex_);
            echo_controller_->SetAudioBufferDelay(delay_ms);
            LOGI("Stream delay updated to %dms", delay_ms);
        }
    }

private:
    std::mutex mutex_;
    std::unique_ptr<webrtc::EchoCanceller3Factory> aec_factory_;
    std::unique_ptr<webrtc::EchoControl> echo_controller_;
    std::unique_ptr<webrtc::HighPassFilter> hp_filter_;
    std::unique_ptr<webrtc::AudioBuffer> render_buffer_;
    std::unique_ptr<webrtc::AudioBuffer> capture_buffer_;
};

// Global processor instance
static std::unique_ptr<TtsAec3Processor> g_processor;

} // namespace webrtc_aec3_tts

extern "C" {

// JNI function implementations
JNIEXPORT jboolean JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeInitialize(JNIEnv *env, jobject thiz) {
    webrtc_aec3_tts::g_processor = std::make_unique<webrtc_aec3_tts::TtsAec3Processor>();
    return webrtc_aec3_tts::g_processor->Initialize();
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeDestroy(JNIEnv *env, jobject thiz) {
    webrtc_aec3_tts::g_processor.reset();
}

JNIEXPORT jboolean JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeProcessTtsAudio(JNIEnv *env, jobject thiz, jshortArray tts_data) {
    if (!webrtc_aec3_tts::g_processor) return JNI_FALSE;
    
    jsize length = env->GetArrayLength(tts_data);
    if (length != webrtc_aec3_tts::TtsAec3Processor::kFrameSize) return JNI_FALSE;
    
    jshort* data = env->GetShortArrayElements(tts_data, nullptr);
    bool result = webrtc_aec3_tts::g_processor->ProcessTtsAudio(
        reinterpret_cast<const int16_t*>(data), length);
    env->ReleaseShortArrayElements(tts_data, data, JNI_ABORT);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeProcessMicrophoneAudio(JNIEnv *env, jobject thiz, 
                                                          jshortArray mic_data, jshortArray output_data) {
    if (!webrtc_aec3_tts::g_processor) return JNI_FALSE;
    
    jsize length = env->GetArrayLength(mic_data);
    if (length != webrtc_aec3_tts::TtsAec3Processor::kFrameSize) return JNI_FALSE;
    
    jshort* input = env->GetShortArrayElements(mic_data, nullptr);
    jshort* output = env->GetShortArrayElements(output_data, nullptr);
    
    bool result = webrtc_aec3_tts::g_processor->ProcessMicrophoneAudio(
        reinterpret_cast<const int16_t*>(input), 
        reinterpret_cast<int16_t*>(output), length);
    
    env->ReleaseShortArrayElements(mic_data, input, JNI_ABORT);
    env->ReleaseShortArrayElements(output_data, output, 0);
    
    return result ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jdoubleArray JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeGetMetrics(JNIEnv *env, jobject thiz) {
    if (!webrtc_aec3_tts::g_processor) return nullptr;
    
    double erl, erle;
    int delay_ms;
    if (!webrtc_aec3_tts::g_processor->GetMetrics(&erl, &erle, &delay_ms)) {
        return nullptr;
    }
    
    jdoubleArray result = env->NewDoubleArray(3);
    double metrics[] = {erl, erle, static_cast<double>(delay_ms)};
    env->SetDoubleArrayRegion(result, 0, 3, metrics);
    return result;
}

JNIEXPORT void JNICALL
Java_com_tts_aec3_WebRtcAec3_nativeSetStreamDelay(JNIEnv *env, jobject thiz, jint delay_ms) {
    if (webrtc_aec3_tts::g_processor) {
        webrtc_aec3_tts::g_processor->SetStreamDelay(delay_ms);
    }
}

} // extern "C"
