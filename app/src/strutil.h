#pragma once

/**
 * Destructive string parsing helper
 * Splits string at first delimiter, null-terminates first part
 * Updates str pointer to point to rest of string (after delimiter)
 * @param str Pointer to string pointer - will be updated to point to remainder
 * @param delim Delimiter character to split on
 * @return Pointer to first part (before delimiter) or NULL if no more tokens
 */
char* split_front(char** str, char delim);
