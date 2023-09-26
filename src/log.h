#ifndef __cnes_log_h__
#define __cnes_log_h__

#ifdef _WIN32
#define BREAK() __debugbreak()
#else
#include <signal.h>
#define BREAK() raise(SIGINT)
#endif


typedef enum log_level_t {
	LOG_LEVEL_TRACE,
	LOG_LEVEL_DEBUG,
	LOG_LEVEL_INFO,
	LOG_LEVEL_WARN,
	LOG_LEVEL_ERROR,
	LOG_LEVEL_FATAL,
	LOG_LEVEL_OFF,
} log_level_t;

void log_log(log_level_t level, const char *file, int line, const char *func, const char *fmt, ...);

#define log_trace(fmt, ...)  log_log(LOG_LEVEL_TRACE, __FILE__, __LINE__, __func__, fmt, __VA_ARGS__)
#define log_debug(fmt, ...)  log_log(LOG_LEVEL_DEBUG, __FILE__, __LINE__, __func__, fmt, __VA_ARGS__)
#define log_info(fmt, ...)   log_log(LOG_LEVEL_INFO,  __FILE__, __LINE__, __func__, fmt, __VA_ARGS__)
#define log_warn(fmt, ...)   log_log(LOG_LEVEL_WARN,  __FILE__, __LINE__, __func__, fmt, __VA_ARGS__)
#define log_error(fmt, ...)  (log_log(LOG_LEVEL_ERROR, __FILE__, __LINE__, __func__, fmt, __VA_ARGS__), BREAK())
#define log_fatal(fmt, ...)  (log_log(LOG_LEVEL_FATAL, __FILE__, __LINE__, __func__, fmt, __VA_ARGS__), BREAK())
#define log(level, fmt, ...) { log_log(level, __FILE__, __LINE__, __func__, fmt, __VA_ARGS__); if (level >= LOG_LEVEL_WARN) { BREAK(); } }


#endif /* __cnes_log_h__ */
