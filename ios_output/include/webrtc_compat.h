#ifndef WEBRTC_COMPAT_H
#define WEBRTC_COMPAT_H

// ARM平台上缺少SSE2函数的架构特定优化存根
// 该头文件为不同架构下的WebRTC组件提供兼容层

#if !defined(__i386__) && !defined(__x86_64__)

namespace webrtc {

// 为ARM构建时提供SSE2函数的回退实现
// 在生产环境中，这些应该调用NEON等效实现或C语言回退实现

/**
 * 用于ARM兼容性的FFT相关SSE2函数存根
 * 这些由WebRTC的ooura_fft实现调用
 */
void rftfsub_128_SSE2(float* a);
void rftbsub_128_SSE2(float* a);
void cft1st_128_SSE2(float* a);
void cftmdl_128_SSE2(float* a);

namespace {

/**
 * 用于ARM兼容性的重采样器SSE2函数存根
 * 这些由WebRTC的sinc_resampler实现调用
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
