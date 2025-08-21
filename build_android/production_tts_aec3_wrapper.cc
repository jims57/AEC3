// Production-Grade TTS AEC3 Wrapper - Main entry point 
// This file combines all C++ components for the WebRTC AEC3 TTS library
// with enhanced ERLE performance and precise timing synchronization

// Include the main processor implementation
#include "wq_aec3_processor.cpp"

// Include WebRTC compatibility layer
#include "webrtc_compat.cpp"

// Include audio converter implementation
#include "wq_aec3_convertor.cpp"

// Include JNI implementation  
#include "wq_aec3_jni.cpp"
