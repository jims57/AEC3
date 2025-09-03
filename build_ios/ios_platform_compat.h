#ifndef IOS_PLATFORM_COMPAT_H
#define IOS_PLATFORM_COMPAT_H

// iOS平台兼容性适配
#ifdef __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_IOS

// 为iOS提供缺失的Linux特定定义
#include <stddef.h>
#include <stdint.h>

// 为splitting_filter_c.c提供size_t定义
#ifndef _SIZE_T
#define _SIZE_T
typedef unsigned long size_t;
#endif

#endif // TARGET_OS_IOS
#endif // __APPLE__

#endif // IOS_PLATFORM_COMPAT_H
