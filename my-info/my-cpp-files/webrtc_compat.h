#ifndef WEBRTC_COMPAT_H
#define WEBRTC_COMPAT_H

// Architecture-specific optimization stubs for missing SSE2 functions on ARM
// This header provides compatibility layer for WebRTC components across different architectures

#if !defined(__i386__) && !defined(__x86_64__)

namespace webrtc {

// Provide fallback implementations for SSE2 functions when building for ARM
// In production, these should call the NEON equivalent or C fallback implementations

/**
 * FFT-related SSE2 function stubs for ARM compatibility
 * These are called by WebRTC's ooura_fft implementation
 */
void rftfsub_128_SSE2(float* a);
void rftbsub_128_SSE2(float* a);
void cft1st_128_SSE2(float* a);
void cftmdl_128_SSE2(float* a);

namespace {

/**
 * Resampler SSE2 function stubs for ARM compatibility
 * These are called by WebRTC's sinc_resampler implementation
 */
class SincResampler {
public:
    static float Convolve_SSE(const float* input_ptr, const float* k1, 
                             const float* k2, double kernel_interpolation_factor);
};

} // anonymous namespace

} // namespace webrtc

#endif // !defined(__i386__) && !defined(__x86_64__)

#endif // WEBRTC_COMPAT_H
