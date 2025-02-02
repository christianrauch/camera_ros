#include "array_string_utils.hpp"
#include <algorithm>
#include <cctype>
#include <regex>
#include <sstream>
#include <vector>


template<typename T>
std::string
encode_2d_numeric_array(const std::vector<std::vector<T>> &array)
{
  std::ostringstream encoded_stream;
  encoded_stream << "[";

  size_t row_index = 0;
  for (auto &row : array) {
    encoded_stream << "[";

    size_t col_index = 0;
    for (auto &col : row) {
      encoded_stream << col;

      if (col_index != row.size() - 1) {
        encoded_stream << ", ";
      }

      ++col_index;
    }

    encoded_stream << "]";

    if (row_index != array.size() - 1) {
      encoded_stream << ", ";
    }

    ++row_index;
  }

  encoded_stream << "]";
  return encoded_stream.str();
}

std::string
normalize_str(const std::string_view str)
{
  std::string result = str.data();

  // Remove newline, carriage return and spaces
  result.erase(std::remove(result.begin(), result.end(), '\r'), result.end());
  result.erase(std::remove(result.begin(), result.end(), '\n'), result.end());
  result.erase(std::remove(result.begin(), result.end(), ' '), result.end());

  // Make all alphabetic symbols lowercase
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    return std::tolower(c);
  });

  return result;
}

std::string
trim_outer_brackets(const std::string_view str)
{
  if (str.front() != '[' || str.back() != ']') {
    throw std::invalid_argument("Invalid format: missing enclosing brackets!");
  }

  std::string result = str.data();
  result = result.substr(1, str.size() - 2);
  return result;
}

bool
str_is_numeric(const std::string_view str)
{
  const std::regex pattern(R"(^([+-]?(inf|(\d+(\.\d*)?|\.\d+)))$)");
  return std::regex_match(str.data(), pattern);
}

template<typename T>
std::vector<T>
parse_single_numeric_array(const std::string_view array_str)
{
  std::stringstream array_ss(array_str.data());
  std::string item;
  std::vector<T> result;

  while (std::getline(array_ss, item, ',')) {
    try {
      if (!str_is_numeric(item)) {
        throw std::invalid_argument("Invalid format: array item is non-numeric!");
      }

      if constexpr (std::is_same_v<T, int>) {
        result.push_back(std::stoi(item));
      }
      else if constexpr (std::is_same_v<T, float>) {
        result.push_back(std::stof(item));
      }
      else if constexpr (std::is_same_v<T, double>) {
        result.push_back(std::stod(item));
      }
      else {
        throw std::invalid_argument("Unsupported array item type!");
      }
    }
    catch (const std::invalid_argument &e) {
      throw std::invalid_argument("Invalid format: unable to parse numerical value '" + item + "'! \n" + e.what());
    }
  }

  return result;
}

template<typename T>
std::vector<std::vector<T>>
decode_2d_numeric_array(const std::string_view array_str)
{
  std::string normalized_str = normalize_str(array_str);

  if (normalized_str.empty() || normalized_str.size() <= 4) {
    throw std::invalid_argument("Invalid format, array is empty!");
  }

  // Trim twice to remove both outer brackets
  normalized_str = trim_outer_brackets(trim_outer_brackets(normalized_str));

  std::vector<std::vector<T>> result;
  size_t start_pos = 0, end_pos = 0;

  while ((end_pos = normalized_str.find("],[", start_pos)) != std::string::npos) {
    std::string sing_arr = normalized_str.substr(start_pos, end_pos - start_pos);
    result.push_back(parse_single_numeric_array<T>(sing_arr));
    start_pos = end_pos + 3;
  }

  if (start_pos < normalized_str.size()) {
    result.push_back(parse_single_numeric_array<T>(normalized_str.substr(start_pos)));
  }

  return result;
}


template std::string
encode_2d_numeric_array<int>(const std::vector<std::vector<int>> &array);
template std::string
encode_2d_numeric_array<float>(const std::vector<std::vector<float>> &array);
template std::string
encode_2d_numeric_array<double>(const std::vector<std::vector<double>> &array);
template std::vector<std::vector<int>>
decode_2d_numeric_array<int>(std::string_view array_str);
template std::vector<std::vector<float>>
decode_2d_numeric_array<float>(std::string_view array_str);
template std::vector<std::vector<double>>
decode_2d_numeric_array<double>(std::string_view array_str);