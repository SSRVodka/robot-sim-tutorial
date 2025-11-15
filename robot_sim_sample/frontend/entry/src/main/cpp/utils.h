
// Node APIs are not fully supported. To solve the compilation error of the interface cannot be found,
// please include "napi/native_api.h".

#pragma once

#include "napi/native_api.h"
#include <dlfcn.h>


#include <hilog/log.h>

#define MY_LOG_DOMAIN 0x000
#define MY_LOG_TAG "[SERVO-CONTROLLER NAPI]"

#ifndef LOGI
#define LOGI(...) ((void)OH_LOG_Print(LOG_APP, LOG_INFO, MY_LOG_DOMAIN, MY_LOG_TAG, __VA_ARGS__))
#endif

#ifndef LOGD
#define LOGD(...) ((void)OH_LOG_Print(LOG_APP, LOG_DEBUG, MY_LOG_DOMAIN, MY_LOG_TAG, __VA_ARGS__))
#endif

#ifndef LOGW
#define LOGW(...) ((void)OH_LOG_Print(LOG_APP, LOG_WARN, MY_LOG_DOMAIN, MY_LOG_TAG, __VA_ARGS__))
#endif

#ifndef LOGE
#define LOGE(...) ((void)OH_LOG_Print(LOG_APP, LOG_ERROR, MY_LOG_DOMAIN, MY_LOG_TAG, __VA_ARGS__))
#endif


template <typename Func> static bool load_symbol(void *handle, const char *symname, Func &sym) {
    // 清除之前的错误
    dlerror();

    // 加载符号
    sym = reinterpret_cast<Func>(dlsym(handle, symname));
    const char *dlsym_error = dlerror();
    if (!sym) {
        LOGE("Cannot load symbol: %{public}s", dlsym_error);
        return false;
    }
    LOGI("load symbol %{public}s", symname);
    return true;
}

// 终止递归的函数
static bool load_symbols(void *handle) { return true; }


// 可变参数模板函数，递归展开处理所有符号
template <typename Func, typename... Args>
static bool load_symbols(void *handle, const char *symname, Func &sym, Args &...args) {
    if (!load_symbol(handle, symname, sym))
        return false;
    return load_symbols(handle, args...);
}


template <typename... Args> static bool load_so(const char *so_name, Args &... args) {
    void *handle = dlopen(so_name, RTLD_LAZY);
    if (!handle) {
        LOGE("Cannot open library: %{public}s", dlerror());
        return false;
    }

    bool success = load_symbols(handle, args...);
    if (!success) {
        dlclose(handle);
        return false;
    }

    // 在这里不关闭句柄，因为符号可能还在使用中
    // 用户需要在合适的时候调用 dlclose(handle)
    return true;
}

