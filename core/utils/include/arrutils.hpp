/**
 * @file arrutils.hpp
 * @brief Utility functions for manipulating vectors as if they were arrays.
 */

#ifndef ARRAY_UTILS_HPP
#define ARRAY_UTILS_HPP

#include <iterator>
#include <ostream>
#include <vector>

/**
 * @fn std::ostream& operator<<(std::ostream& out, const std::vector<T>& v)
 * @brief Overloads cout to accept a vector input
 * @details Prints vector out sequentially from index 0
 * 
 * @param out The ostream object
 * @param v The vector object to print
 */
template <typename T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &v) {
    if (!v.empty()) {
        out << '[';
        std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
        out << "\b\b]";
    }
    return out;
}

// Get the first element of a vector as a vector.
template <typename T>
std::vector<T> getFirst(const std::vector<T> &arr) {
    if (arr.empty())
        return std::vector<T>{};

    return std::vector<T>({arr.front()});
}

// Get the last element of a vector as a vector.
template <typename T>
std::vector<T> getLast(const std::vector<T> &arr) {
    if (arr.empty())
        return std::vector<T>{};

    return std::vector<T>({arr.back()});
}

// Numpy-style indexing for std::vector. No bounds checking!
template <typename T>
std::vector<T> indexArray(const std::vector<T> &arr, const std::vector<int> &indices) {
    std::vector<T> result{};

    for (const int &ind : indices)
        result.push_back(arr[ind]);

    return result;
}

/**
 * Numpy-style linspace
 * 
 * @param a Starting value.
 * @param b Ending value.
 * @param num Total number of elements in return vector
 * @param endPoint Whether to include the ending value or not.
 */
std::vector<double> linspace(double a, double b, int num, bool endPoint = true);

/**
 * Numpy-style arange
 * 
 * @param start Starting value
 * @param stop End value (exclusive)
 * @param step Increment amount per iteration
 */
template <typename T>
std::vector<T> arange(T start, T stop, T step = 1) {
    std::vector<T> values;
    for (T value = start; value < stop; value += step)
        values.push_back(value);
    return values;
}

// Call push_back on a vector with the specified value, n times.
void push_n(std::vector<double>& vec, const double val, const size_t n);


#endif  //MEASURING_ARRUTILS_HPP
