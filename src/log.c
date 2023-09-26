#include "log.h"

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

static const char *LEVEL_STR[] = { "TRACE", "DEBUG", "INFO ", "WARN ", "ERROR", "FATAL" };
static const char *LEVEL_COL[] = { "\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m" };

void log_log(log_level_t level, const char *file, int line, const char *func, const char *fmt, ...) {
	va_list va;

	va_start(va, fmt);
	size_t size = vsnprintf(NULL, 0, fmt, va);
	va_end(va);

	char *buffer = malloc(1 + size);
	if (buffer != NULL) {
		va_start(va, fmt);
		vsnprintf(buffer, 1 + size, fmt, va);
		va_end(va);
	}

	printf("[%s%s\x1b[0m] %s:%d @ %s: %s\n", LEVEL_COL[level], LEVEL_STR[level], file, line, func, buffer);
}
