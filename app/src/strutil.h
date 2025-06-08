/**
 * (Stateless) String parsing utilities.
 */
#pragma once

#include <stdbool.h>

/**
 * Destuctively splits string at first delimiter, by overwriting 0 to str.
 * Returns non-NULL even if the second part is empty.
 *
 * @param str string pointer. after the call, it will point the first part.
 * @param delim Delimiter character to split on
 * @return NULL if no delimiter found. Otherwise returns pointer to the second
 * part of the string (may be empty "").
 *
 * Examples:
 *   split_at("a.b", '.') -> str="a", return="b"
 *   split_at("a.", '.') -> str="a", return=""
 *   split_at("a", '.') -> str="a", return=NULL
 *   split_at("", '.') -> str="", return=NULL
 */
char* split_at(char* str, char delim);

/**
 * Destructively splits string at first whitespace sequence, by overwriting 0 to
 * str. Skips multiple consecutive whitespace characters (spaces, tabs, etc).
 * Returns NULL if no meaningful content follows the whitespace.
 *
 * @param str string pointer. after the call, it will point the first part.
 * @return NULL if no whitespace found or only whitespace remains. Otherwise
 * returns pointer to the second part of the string (after skipping all
 * whitespace).
 *
 * Examples:
 *   split_by_space("a b") -> str="a", return="b"
 *   split_by_space("a  b") -> str="a", return="b"
 *   split_by_space("a ") -> str="a", return=NULL
 *   split_by_space("a") -> str="a", return=NULL
 *   split_by_space("") -> str="", return=NULL
 *   split_by_space(" ") -> str="", return=NULL
 */
char* split_by_space(char* str);

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
