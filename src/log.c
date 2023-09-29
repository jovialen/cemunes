#include "log.h"

#include <stdarg.h>
#include <stdlib.h>

static const char *LEVEL_STR[] = { "TRACE", "DEBUG", "INFO ", "WARN ", "ERROR", "FATAL" };
static const char *LEVEL_COL[] = { "\x1b[94m", "\x1b[36m", "\x1b[32m", "\x1b[33m", "\x1b[31m", "\x1b[35m" };

static FILE *log_file = NULL;

FILE *log_set_output_file(FILE *file) {
	FILE *prev = log_file;
	log_file = file;
	return prev;
}

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

	const char *log_fmt = "[%s%s\x1b[0m] %s:%d @ %s: %s\n";
	size = snprintf(NULL, 0, log_fmt, LEVEL_COL[level], LEVEL_STR[level], file, line, func, buffer);
	char *output = malloc(1 + size);
	if (output != NULL) {
		snprintf(output, 1 + size, log_fmt, LEVEL_COL[level], LEVEL_STR[level], file, line, func, buffer);
	}

	printf("%s", output);
	if (log_file != NULL && output != NULL) {
		fwrite(output, 1, 1 + size, log_file);
	}

	free(buffer);
	free(output);
}
