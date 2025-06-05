#pragma once

/**
 * Destuctively splits string at first delimiter, by overwriting 0 to str.
 * 
 * @param str string pointer. after the call, it will point the first part.
 * @param delim Delimiter character to split on
 * @return NULL if no delimiter found. Otherwise returns pointer to the second part of the string.
 */
char* split_at(char* str, char delim);
