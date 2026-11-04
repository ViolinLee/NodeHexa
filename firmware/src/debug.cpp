#include "debug.h"

#include <stdio.h>
#include <stdarg.h>  // For va_start, etc.

namespace {
    std::function<void(const char*)> _writer = nullptr;
    std::function<int(void)> _time_func = nullptr;
}

void _my_log_impl(const char* format, ...)
{
    if (!_writer || !format) return;

    char buffer[256] = {};
    int pos = 0;

    if(_time_func) {
        pos = snprintf(buffer, sizeof(buffer), "[%d]", _time_func());
        if (pos < 0 || static_cast<size_t>(pos) >= sizeof(buffer)) return;
    }

    va_list ap;
    va_start(ap, format);
    vsnprintf(buffer + pos, sizeof(buffer) - pos, format, ap);
    va_end(ap);
    _writer(buffer);
}

namespace hexapod {

    void initLogOutput(std::function<void(const char*)> writer, std::function<int(void)> time_func) {
        _writer = writer;
        _time_func = time_func;
    }

}
