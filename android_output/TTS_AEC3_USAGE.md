# WebRTC AEC3 TTS Echo Cancellation - Android Usage Guide

## Overview
This AAR provides WebRTC AEC3-based echo cancellation specifically optimized for TTS (Text-to-Speech) applications.

## Key Features
- **High-Quality Echo Cancellation**: Based on WebRTC AEC3 algorithm
- **TTS Optimized**: Designed for TTS playback scenarios
- **Mobile Optimized**: Efficient performance on Android devices
- **Real-time Processing**: 10ms latency, suitable for live applications
- **Easy Integration**: Simple Java API

## Technical Specifications
- **Sample Rate**: 48kHz (mandatory)
- **Frame Size**: 480 samples (10ms chunks)
- **Channels**: Mono (1 channel)
- **Bit Depth**: 16-bit PCM
- **Latency**: ~10ms processing + system delay
- **CPU Usage**: Optimized for mobile ARM processors

## Integration Steps

### 1. Add AAR to Your Project
```gradle
implementation files('libs/webrtc-aec3-tts-1.0.aar')
```

### 2. Add Permissions
```xml
<uses-permission android:name="android.permission.RECORD_AUDIO" />
<uses-permission android:name="android.permission.MODIFY_AUDIO_SETTINGS" />
```

### 3. Basic Usage
```java
import com.tts.aec3.WebRtcAec3;

public class TtsEchoCancellation {
    private WebRtcAec3 aec3;
    
    public void initializeAec() {
        aec3 = new WebRtcAec3();
        if (aec3.initialize()) {
            Log.i("AEC3", "Echo cancellation initialized successfully");
        }
    }
    
    public void processTtsChunk(short[] ttsAudio) {
        // CRITICAL: Call this BEFORE playing TTS audio
        aec3.processTtsAudio(ttsAudio);
        
        // Now play the TTS audio through speakers
        playTtsAudio(ttsAudio);
    }
    
    public short[] processMicrophoneChunk(short[] micAudio) {
        // Process microphone input and get echo-cancelled output
        return aec3.processMicrophoneAudio(micAudio);
    }
    
    public void cleanup() {
        if (aec3 != null) {
            aec3.destroy();
        }
    }
}
```

### 4. Complete TTS Integration Example
```java
public class TtsWithEchoCancellation {
    private WebRtcAec3 aec3;
    private AudioTrack audioTrack;
    private AudioRecord audioRecord;
    private boolean isRecording = false;
    
    public void startTtsSession() {
        // Initialize AEC3
        aec3 = new WebRtcAec3();
        aec3.initialize();
        
        // Setup audio playback (48kHz, 16-bit, mono)
        int bufferSize = AudioTrack.getMinBufferSize(
            WebRtcAec3.SAMPLE_RATE,
            AudioFormat.CHANNEL_OUT_MONO,
            AudioFormat.ENCODING_PCM_16BIT
        );
        
        audioTrack = new AudioTrack(
            AudioManager.STREAM_MUSIC,
            WebRtcAec3.SAMPLE_RATE,
            AudioFormat.CHANNEL_OUT_MONO,
            AudioFormat.ENCODING_PCM_16BIT,
            bufferSize,
            AudioTrack.MODE_STREAM
        );
        
        // Setup audio recording (48kHz, 16-bit, mono)
        int recordBufferSize = AudioRecord.getMinBufferSize(
            WebRtcAec3.SAMPLE_RATE,
            AudioFormat.CHANNEL_IN_MONO,
            AudioFormat.ENCODING_PCM_16BIT
        );
        
        audioRecord = new AudioRecord(
            MediaRecorder.AudioSource.MIC,
            WebRtcAec3.SAMPLE_RATE,
            AudioFormat.CHANNEL_IN_MONO,
            AudioFormat.ENCODING_PCM_16BIT,
            recordBufferSize
        );
        
        // Start recording
        startMicrophoneRecording();
    }
    
    public void playTtsResponse(byte[] ttsData) {
        // Convert to 16-bit samples
        short[] samples = bytesToShorts(ttsData);
        
        // Process in 10ms chunks (480 samples)
        for (int i = 0; i < samples.length; i += WebRtcAec3.FRAME_SIZE) {
            short[] chunk = Arrays.copyOfRange(samples, i, 
                Math.min(i + WebRtcAec3.FRAME_SIZE, samples.length));
            
            if (chunk.length == WebRtcAec3.FRAME_SIZE) {
                // Feed to AEC3 BEFORE playback
                aec3.processTtsAudio(chunk);
                
                // Play through speakers
                audioTrack.write(chunk, 0, chunk.length);
            }
        }
    }
    
    private void startMicrophoneRecording() {
        isRecording = true;
        audioRecord.startRecording();
        
        new Thread(() -> {
            short[] buffer = new short[WebRtcAec3.FRAME_SIZE];
            
            while (isRecording) {
                int read = audioRecord.read(buffer, 0, buffer.length);
                
                if (read == WebRtcAec3.FRAME_SIZE) {
                    // Process with AEC3 to remove echo
                    short[] cleanAudio = aec3.processMicrophoneAudio(buffer);
                    
                    if (cleanAudio != null) {
                        // Send clean audio to your TTS service
                        sendToTtsService(cleanAudio);
                    }
                }
            }
        }).start();
    }
    
    public void stopSession() {
        isRecording = false;
        
        if (audioRecord != null) {
            audioRecord.stop();
            audioRecord.release();
        }
        
        if (audioTrack != null) {
            audioTrack.stop();
            audioTrack.release();
        }
        
        if (aec3 != null) {
            aec3.destroy();
        }
    }
}
```

## Performance Monitoring and ERLE Optimization
```java
// Get enhanced AEC performance metrics with detailed information
WebRtcAec3.EnhancedAecMetrics metrics = aec3.getEnhancedMetrics();
if (metrics != null) {
    Log.i("AEC3", "Performance: " + metrics.toString());
    Log.i("AEC3", "ERLE Quality: " + metrics.getErleQuality());
    Log.i("AEC3", "Frame Sync: " + metrics.isFrameSynchronized());
    
    // Excellent AEC performance indicators (Enhanced 2025-01-30):
    // - Echo Return Loss Enhancement > 15dB (Excellent)
    // - Echo Return Loss Enhancement > 10dB (Good)  
    // - Frame synchronization > 95%
    // - Optimal delay within 20-500ms range
    
    // Auto-optimize if performance is poor
    if (metrics.echoReturnLossEnhancement < 10.0) {
        Log.w("AEC3", "Poor ERLE detected, running auto-optimization...");
        if (aec3.autoOptimizeDelay()) {
            Log.i("AEC3", "Delay optimization completed");
        }
    }
}

// Enable/disable timing synchronization for CPU optimization
aec3.enableTimingSync(true); // Enable for maximum ERLE
// aec3.enableTimingSync(false); // Disable for lower CPU usage
```

## Optimization Tips

### 1. Delay Compensation
```java
// Android devices typically have 80-150ms latency
// Fine-tune based on your specific device/setup
aec3.setStreamDelay(100); // Start with 100ms
```

### 2. Audio Format Requirements
- **MUST use 48kHz sample rate** - AEC3 will not work with other rates
- **MUST use 16-bit PCM format**
- **MUST process in 10ms chunks (480 samples)**
- **Recommended: Use mono audio** for better performance

### 3. Threading Considerations
- AEC3 processing is thread-safe
- Process TTS and microphone audio on separate threads for best performance
- Always call `processTtsAudio()` BEFORE playing the audio

### 4. Memory Management
- Reuse audio buffers to reduce GC pressure
- Call `destroy()` when done to free native resources
- Monitor memory usage in long-running sessions

## Troubleshooting

### Common Issues
1. **Silent Output**: Check sample rate is exactly 48kHz
2. **Poor Echo Cancellation**: Ensure TTS is processed before playback
3. **High CPU Usage**: Verify you're using 10ms chunks (480 samples)
4. **Crashes**: Check all audio arrays are exactly 480 samples

### Debugging
```java
// Enable verbose logging
adb shell setprop log.tag.WebRTC_AEC3_TTS VERBOSE
```

## License
Based on WebRTC project (BSD-style license)
