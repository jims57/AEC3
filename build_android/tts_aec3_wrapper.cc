// TTS AEC3包装器 - 主入口点
// 该文件组合了WebRTC AEC3 TTS库的所有C++组件

// 包含录音器实现
#include "wq_aec3_recorder.cpp"

// 包含主处理器实现
#include "wq_aec3_processor.cpp"

// 包含WebRTC兼容层
#include "webrtc_compat.cpp"

// 包含音频转换器实现
#include "wq_aec3_convertor.cpp"

// 包含JNI实现
#include "wq_aec3_jni.cpp"
