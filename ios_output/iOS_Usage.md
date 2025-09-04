# WQDenoiser iOS XCFramework 使用指南

## 概述
WQDenoiser是一个基于WebRTC AEC3的iOS回声消除库，专为TTS（文本转语音）应用优化。

## 集成步骤

### 1. 添加XCFramework到项目
1. 将`WQAec.xcframework`拖拽到Xcode项目中
2. 在目标设置中，确保XCFramework被添加到"Frameworks, Libraries, and Embedded Content"

### 2. 导入头文件
```objc
#import <WQAec/WQDenoiserWrapper.h>
```

### 3. 基本使用示例
```objc
// 创建和初始化处理器
WQDenoiserHandle denoiser = wq_denoiser_create();
if (!wq_denoiser_initialize(denoiser)) {
    NSLog(@"初始化失败");
    return;
}

// 处理TTS音频（参考信号）
int16_t ttsData[WQ_FRAME_SIZE];
wq_denoiser_process_tts_audio(denoiser, ttsData, WQ_FRAME_SIZE);

// 处理麦克风音频（移除回声）
int16_t micData[WQ_FRAME_SIZE];
int16_t cleanData[WQ_FRAME_SIZE];
wq_denoiser_process_microphone_audio(denoiser, micData, cleanData, WQ_FRAME_SIZE);

// 获取性能指标
double erl, erle;
int delay;
if (wq_denoiser_get_metrics(denoiser, &erl, &erle, &delay)) {
    NSLog(@"ERLE: %.2f dB, 延迟: %d ms", erle, delay);
}

// 清理
wq_denoiser_destroy(denoiser);
```

### 4. 权限设置
在Info.plist中添加麦克风权限：
```xml
<key>NSMicrophoneUsageDescription</key>
<string>此应用需要麦克风权限进行回声消除功能</string>
```

## API参考
详细的API文档请参考WQDenoiserWrapper.h头文件。

## 配置参数
- 采样率: 48000 Hz
- 帧大小: 480 samples (10ms)
- 声道数: 1 (单声道)
- 默认延迟: 100ms

## 技术支持
如有问题，请联系开发团队。
