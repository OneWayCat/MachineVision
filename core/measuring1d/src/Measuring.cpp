#include "Measuring.hpp"

using namespace cv;

std::pair <std::vector<double>, std::vector<double>> findSubpixelPos(const Mat &profile, const std::vector<int> &aboveThreshold) {
    std::vector<double> subpixelPos{};
    std::vector<double> amplitudes{};

    auto profptr = profile.ptr<double>();

    // Do polynomial interpolation
    for (const int &idx : aboveThreshold) {
        // Get the profile values at the minima/maxima and the neighboring points
        double middle = profptr[idx];
        double left = idx == 0 ? middle : profptr[idx - 1];
        double right = idx == profile.cols - 1 ? middle : profptr[idx + 1];

        // Fit to y = Ax^2 + Bx + C
        double A = 0.5 * (left - 2 * middle + right);
        double B = 0.5 * (right - middle);
        double C = middle;

        // Find the local maximum / minimum
        if (A != 0) {
            double subpixel = -B / (2 * A);
            double amplitude = C - B * B / (4 * A);

            subpixelPos.push_back(idx + subpixel);
            amplitudes.push_back(amplitude);
        }
    }

    return std::make_pair(subpixelPos, amplitudes);
}

std::pair <std::vector<double>, std::vector<double>> getEdgeAmplitudes(Mat &profile, const EdgeElement &measureHandle, double sigma, double threshold, TransitionType transition) {

    //    std::ofstream profileOutIO("D:/Programming/Workspaces/Zebra/H-Rev-C/noise_testing/projection.txt", std::ios::app);
    //    for (int i = 0; i < profile.cols; i++) {
    //        if (profile.at<double>(0, i) != 0) {
    //            profileOutIO << profile.at<double>(0, i) << ",";
    //        }
    //    }
    //    profileOutIO << "\n";
    //    profileOutIO.close();

    // Gaussian smoothing before detecting edges
    Size kernelSize{3, 1};
    GaussianBlur(profile, profile, kernelSize, sigma, sigma, BORDER_REPLICATE);

    // Take the derivative of the 1D profile
    Mat_<double> kernel = (Mat_<double>(1, 3) << -1, 0, 1);
    filter2D(profile, profile, -1, kernel, Point(-1, -1), 0, BORDER_REPLICATE);

    //    Mat_<double> gaussianKernel = getGaussianKernel(3, sigma);
    // Scale edge amplitudes to account for gaussian smoothing
    //    double scalingFactor = 1.0 / (gaussianKernel(0, 0) + gaussianKernel(1, 0));
    //    profile *= scalingFactor;

    // Suppress edges that we don't want
    if (transition != TransitionType::ALL) {
        // If transition is positive, we want to get rid of negative edges, which have a sign bit of 1
        // And vice versa

        // Signbit returns true if the value is negative
        bool sign = transition == TransitionType::NEGATIVE;
        auto profptr = profile.ptr<double>(0);

        for (int i = 0; i < profile.cols; i++) {
            if (signbit(profptr[i]) != sign)
                profptr[i] = 0;
        }
    }

//    std::ofstream profileOutIO("D:/Programming/Workspaces/Zebra/H-Rev-C/noise_testing/projection.txt", std::ios::app);
//    for (int i = 0; i < profile.cols; i++) {
//        profileOutIO << profile.at<double>(0, i) << ",";
//    }
//    profileOutIO << "\n";
//    profileOutIO.close();

    // Non maximum suppression to thin the edges
    Mat suppressed = nonMaxSuppress(profile);

    // Take the ones above the threshold
    std::vector<int> aboveThreshold;
    std::vector<double> amplitudes;

    auto suppressedProfptr = suppressed.ptr<double>();

    for (int i = 0; i < profile.cols; i++) {
        if (abs(suppressedProfptr[i]) > threshold)
            aboveThreshold.push_back(i);
    }

    return findSubpixelPos(profile, aboveThreshold);
}

bool betweenInclusive(double a, double b, double c) {
    return signbit(c - a) != signbit(c - b);
}
