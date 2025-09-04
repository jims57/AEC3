#include <pthread.h>
#include <unistd.h>

// iOS threading implementation for WebRTC compatibility

namespace rtc {

pthread_t CurrentThreadId() {
    return pthread_self();
}

pthread_t CurrentThreadRef() {
    return pthread_self();
}

bool IsThreadRefEqual(const pthread_t& a, const pthread_t& b) {
    return pthread_equal(a, b) != 0;
}

} // namespace rtc
