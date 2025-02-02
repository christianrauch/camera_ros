#pragma once
#include <string>
#include <vector>


/**
 * @brief encode a 2D integer/float/double array as a string
 * @param array
 * @return the provided 2D array encoded as a string
 *         with JSON-like formatting
 */
template<typename T>
std::string
encode_2d_numeric_array(const std::vector<std::vector<T>> &array);

/**
 * @brief decode a 2D integer/float/double array from a string
 * @param array_str
 * @return the provided string-encoded 2D array as a 2D vector
 *         of the specified numeric type (int/float/double)
 */
template<typename T>
std::vector<std::vector<T>>
decode_2d_numeric_array(std::string_view array_str);