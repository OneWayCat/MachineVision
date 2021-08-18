#include "arrutils.hpp"

// Numpy-style linspace
std::vector<double> linspace(double a, double b, int num, bool endPoint) {
    // Create a vector of length num
    std::vector<double> v(num);

    int n = num;
    if (endPoint)
        n = n - 1;

    // Trust the programmer that n is not 0
    double step = (b - a) / n;
    double val = a;

    // now assign the values to the vector
    for (int i = 0; i < num; i++) {
        v[i] = val;
        val += step;
    }

    return v;
}

void push_n(std::vector<double>& vec, const double val, const size_t n) {
	for (size_t i = 0; i < n; i++)
		vec.push_back(val);
}