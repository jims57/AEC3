// iOS AEC3包装器 - 主入口点
// 该文件组合了WebRTC AEC3 TTS库的所有C++组件（iOS版本）

// 包含主处理器实现（iOS兼容版本）
#include "wq_aec3_processor_ios.cpp"

// 包含WebRTC兼容层
#include "webrtc_compat.cpp"

// 包含音频转换器实现
#include "wq_aec3_convertor.cpp"

// 注意：iOS版本不包含JNI实现
