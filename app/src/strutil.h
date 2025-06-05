#pragma once

#include <stdbool.h>

/**
 * Destuctively splits string at first delimiter, by overwriting 0 to str.
 *
 * @param str string pointer. after the call, it will point the first part.
 * @param delim Delimiter character to split on
 * @return NULL if no delimiter found. Otherwise returns pointer to the second
 * part of the string.
 */
char* split_at(char* str, char delim);

/**
 * Parse string to int with validation.
 * @param str String to parse
 * @param value Output int value
 * @return true if valid int, false otherwise
 */
bool parse_int(const char* str, int* value);

/**
 * Parse string to float with validation.
 * @param str String to parse
 * @param value Output float value
 * @return true if valid float, false otherwise
 */
bool parse_float(const char* str, float* value);
