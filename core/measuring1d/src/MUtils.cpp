#include "MUtils.hpp"
#include <cmath>

using namespace cv;
using namespace std;

// Helper functions for measuring and fuzzy measuring
Mat nonMaxSuppress(Mat &profile) {
    if (profile.cols < 2)
        return profile;

    Mat absProfile = abs(profile);
    Mat_<double> suppressed = Mat_<double>::zeros(1, profile.cols);

    auto profptr = profile.ptr<double>();
    auto absProfptr = absProfile.ptr<double>();
    auto suppressedPtr = suppressed.ptr<double>();

    // Edge cases (first and last one)
    if (absProfptr[0] > absProfptr[1])
        suppressedPtr[0] = profptr[0];

    int lastIndex = profile.cols - 1;

    if (absProfptr[lastIndex] >= absProfptr[lastIndex - 1])
        suppressedPtr[lastIndex] = profptr[lastIndex];

    // General case
    for (int i = 1; i < profile.cols - 1; i++) {
        if (absProfptr[i] >= absProfptr[i - 1] && absProfptr[i] > absProfptr[i + 1])
            suppressedPtr[i] = profptr[i];
    }

    return suppressed;
}

void validateArgs(const Mat &img, double sigma, double threshold) {
    // Input validation
    // Ensure image is single channel
    CV_Assert(img.depth() == CV_8U && img.channels() == 1);

    // Validate sigma
    if (sigma < 0.4 || sigma > 100)
        throw std::invalid_argument("Sigma must be between 0.4 and 100");

    // Validate threshold
    if (threshold < 0 || threshold > 255)
        throw std::invalid_argument("Threshold must be between 0 and 255");
}

int nextSignedElement(const std::vector<double> &arr, bool sign, int index, bool findStrongest) {
    if (index >= arr.size())
        return -1;

    int first = -1;

    // Find the first element with the given sign
    for (int i = index; i <= arr.size(); i++) {
        if (signbit(arr[i]) == sign) {
            first = i;
            break;
        }
    }

    if (first == -1 || !findStrongest)
        return first;

    // Find the strongest element with given sign (until we reach an element with the opposite sign)
    int i = first;
    double max = 0;
    int maxIndex = 0;

    while (i < arr.size() && signbit(arr[i]) == sign) {
        if (abs(arr[i]) > abs(max)) {
            max = arr[i];
            maxIndex = i;
        }

        i++;
    }

    return maxIndex;
}
