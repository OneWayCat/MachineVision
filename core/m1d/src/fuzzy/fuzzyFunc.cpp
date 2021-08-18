#include "measureHandle.hpp"

// Constructor
FuzzyFunc::FuzzyFunc(const std::vector<double> &xVals_, const std::vector<double> &yVals_) {
    // Validate size of vectors
    size_t n = xVals_.size();

    if (n != yVals_.size() or n < 2)
        throw std::invalid_argument("xVals and yVals must have matching sizes >= 2");

    // Ensure that xVals is in ascending order with no duplicates
    for (size_t i = 1; i < n; i++) {
        if (xVals_[i] <= xVals_[i - 1])
            throw std::invalid_argument("xVals is not in ascending order");
    }

    // Ensure that the elements in yVals is between 0 and 1
    for (size_t i = 0; i < n; i++) {
        if (yVals_[i] < 0 or yVals_[i] > 1)
            throw std::invalid_argument("y coordinate out of range");
    }

    xVals = xVals_;
    yVals = yVals_;
}

// Interpolation
double FuzzyFunc::interpolate(double x) const {
    if (x <= xVals.front())
        return yVals.front();

    if (x >= xVals.back())
        return yVals.back();

    for (size_t i = 0; i < yVals.size() - 1; i++) {
        // Check if x is between the y value at current and next index
        double currentX = xVals[i];
        double nextX = xVals[i + 1];

        double currentY = yVals[i];
        double nextY = yVals[i + 1];

        if (x >= currentX and x < nextX)
            return currentY + (x - currentX) / (nextX - currentX) * (nextY - currentY);
    }

    // This shouldn't happen
    return 0;
}

// Scale each x coordinate by a factor
void FuzzyFunc::scale(double factor) {
    if (factor <= 0)
        throw std::invalid_argument("pairSize must be a positive number");

    for (double &x : xVals)
        x *= factor;
}