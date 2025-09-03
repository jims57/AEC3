# WebRTC AEC3 iOS XCFramework 使用指南

## 集成步骤

### 1. 添加XCFramework到项目
1. 将 `WqAec3.xcframework` 拖拽到你的Xcode项目中
2. 在项目设置中，选择 "Embed & Sign" 为 "Do Not Embed"（静态库）
3. 确保在 "Link Binary With Libraries" 中添加了该框架

### 2. 导入头文件
```objc
#import "wq_aec3_processor.h"
```

### 3. 基本使用
```objc
// 初始化AEC处理器
webrtc_aec3_tts::WqAec3Processor* processor = new webrtc_aec3_tts::WqAec3Processor();
processor->Initialize();

// 处理TTS音频（参考信号）
processor->ProcessTtsAudio(tts_samples, frame_size);

// 处理麦克风音频并移除回声
processor->ProcessMicrophoneAudio(mic_samples, output_samples, frame_size);

// 获取性能指标
double erl, erle;
int delay_ms;
processor->GetMetrics(&erl, &erle, &delay_ms);
```

## 技术规格
- 采样率: 48kHz
- 帧大小: 480样本（10ms）
- 声道: 单声道
- 架构支持: arm64（仅真机）

## 注意事项
- 该XCFramework为静态库，无需设置"Embed & Sign"
- 仅支持iOS真机（arm64），不包含模拟器支持
- 确保音频数据格式为16位PCM
