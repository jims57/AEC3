#ifndef SYS_PRCTL_H_IOS_COMPAT
#define SYS_PRCTL_H_IOS_COMPAT

// iOS兼容的prctl.h实现
#define PR_SET_NAME 15

#ifdef __cplusplus
extern "C" {
#endif

// iOS上的prctl实现，支持多种参数类型
static inline int prctl(int option, ...) {
    // iOS上的空实现，仅用于编译兼容性
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif // SYS_PRCTL_H_IOS_COMPAT
