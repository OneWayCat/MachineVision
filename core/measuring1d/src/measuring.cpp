#include "measuring.hpp"

#include <numeric>
#include <fstream>

#include "arrutils.hpp"
#include "mutils.hpp"
#include "drawSubpixel.hpp"

using namespace cv;

bool betweenInclusive(double a, double b, double c) {
    return signbit(c - a) != signbit(c - b);
}

Mat MeasureHandle::measureProjection_(const Mat &img) const {
    // Ensure image is single channel
    CV_Assert(img.depth() == CV_8U and img.channels() == 1);

    // Dimensions of the image
    int imgHeight = img.rows;
    int imgWidth = img.cols;

    // Coordinates of the bounding box of the measure handle
    int boxMinC = minC + column;
    int boxMinR = minR + row;

    size_t numCols = size_t(maxC) - minC + 1;
    size_t numRows = size_t(maxR) - minR + 1;

    // Project down to 1D profile
    Mat_<double> profile = Mat_<double>::zeros(1, (int) binCounts.size());
    auto profPtr = profile.ptr<double>();  // Pointer to the profile array

    // Main loop to iterate over each (r, c) pair inside the bounding box
    for (size_t r = 0; r < numRows; r++) {
        int imgRow = (int) r + boxMinR;

        // Treat as 0 if out of bounds
        if (imgRow < 0 or imgRow >= imgHeight)
            continue;

        const auto rowPtr = img.ptr<uchar>(imgRow);  // Pointer to the row of the image

        for (int c = 0; c < numCols; c++) {
            int binNumber = binNumbers[r * numCols + c];
            int imgCol = c + boxMinC;

            // Inside check
            if (binNumber != -1 and imgCol >= 0 and imgCol < imgWidth) {
                // Add the pixel of the image to the correct bin
                profPtr[binNumber] += rowPtr[imgCol];
            }
        }
    }

    // Divide the profile by binCounts to get the mean
    for (int i = 0; i < binCounts.size(); i++) {
        if (binCounts[i] != 0)
            profPtr[i] = (double) profPtr[i] / binCounts[i];
    }

    return profile;
}

Mat measureProjection(const Mat &img, const MeasureHandle &measureHandle) {
    return measureHandle.measureProjection_(img);
}

#define DRAW_SP 0

Mat MeasureHandle::measureProjection_(const Mat &img, double cx, double cy) const {
    // Ensure image is single channel
    CV_Assert(img.depth() == CV_8U and img.channels() == 1);

    // Dimensions of the image
    int imgHeight = img.rows;
    int imgWidth = img.cols;

    // Coordinates of the bounding box of the measure handle
    int boxMinC = minC + column;
    int boxMinR = minR + row;

    size_t numCols = size_t(maxC) - minC + 1;
    size_t numRows = size_t(maxR) - minR + 1;

    // Project down to 1D profile
    Mat_<double> profile = Mat_<double>::zeros(1, (int) binCounts.size());
    auto profPtr = profile.ptr<double>();  // Pointer to the profile array

    double distToMeasureCenter = sqrt(pow(column - cx, 2) + pow(row - cy, 2));
    double dist2LeftEdge = distToMeasureCenter - (profileLength / 2);
    std::vector<int> pixelCounts(binCounts.size());
//    std::vector<int> pixelVals;
//    std::vector<int> inds;

#if DRAW_SP
    Mat colorClone = img.clone();
    cvtColor(colorClone, colorClone, COLOR_GRAY2BGR);
    SubpixelWorkspace spWS(colorClone, row, column, 10, 50);

    std::vector<Point2d> corners = getRectPoints(*reinterpret_cast<const MeasureRectangle *>(this));
    for (int i = 0; i < corners.size(); i++) {
        corners[i] += Point2d(row, column);
    }

    spWS.drawRect(corners, Scalar(0, 255, 0), false);
    std::vector<Scalar> colors = {
            Scalar(255, 0, 0),
            Scalar(0, 255, 0),
            Scalar(0, 0, 255)
    };

#endif

    // Main loop to iterate over each (r, c) pair inside the bounding box
    for (size_t r = 0; r < numRows; r++) {
        int imgRow = (int) r + boxMinR;

        // Treat as 0 if out of bounds
        if (imgRow < 0 or imgRow >= imgHeight)
            continue;

        const auto rowPtr = img.ptr<uchar>(imgRow);  // Pointer to the row of the image

        for (int c = 0; c < numCols; c++) {
            int binNumber = binNumbers[r * numCols + c];
            int imgCol = c + boxMinC;

            // Inside check
            if (binNumber != -1 and imgCol >= 0 and imgCol < imgWidth) {
                double dist2Pt = sqrt(pow(imgCol - cx, 2) + pow(imgRow - cy, 2));
//                double phi = atan2(imgRow - cy, imgCol - cx);
//                double x = imgCol + rad * cos(phi);
//                double y = imgRow + rad * sin(phi);
                int radialBin = (int) round(abs(dist2Pt - dist2LeftEdge));
                if (radialBin >= binCounts.size())
                    radialBin = binCounts.size() - 1;
                else if (radialBin < 0)
                    radialBin = 0;
                pixelCounts[radialBin] += 1;
                profPtr[radialBin] += rowPtr[imgCol];
#if DRAW_SP
                spWS.drawPoint(imgRow, imgCol, colors[radialBin % 3]);
//                std::cout << radialBin << "\n";
#endif

            }
        }
    }
#if DRAW_SP
    cv::imshow("SP", spWS.getZoomedImage());
    cv::waitKey(0);
#endif

    // Divide the profile by binCounts to get the mean
    for (int i = 0; i < pixelCounts.size(); i++) {
        if (pixelCounts[i] != 0) {
            profPtr[i] = (double) profPtr[i] / pixelCounts[i];
        }
    }

    return profile;
}

Mat measureProjection(const Mat &img, const MeasureHandle &measureHandle, double cx, double cy) {
    return measureHandle.measureProjection_(img, cx, cy);
}

/* Find the subpixel position and the amplitude of the edges at that position given all the
 * edge amplitudes and the position of the local maxima/minima  */
std::pair<std::vector<double>, std::vector<double>>
findSubpixelPos(const Mat &profile, const std::vector<int> &aboveThreshold) {
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

std::pair<std::vector<double>, std::vector<double>>
getEdgeAmplitudes(Mat &profile, const MeasureHandle &measureHandle, double sigma, double threshold,
                  TransitionType transition) {

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

    std::ofstream profileOutIO("D:/Programming/Workspaces/Zebra/H-Rev-C/noise_testing/projection.txt", std::ios::app);
    for (int i = 0; i < profile.cols; i++) {
        profileOutIO << profile.at<double>(0, i) << ",";
    }
    profileOutIO << "\n";
    profileOutIO.close();

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

template<typename T>
MeasurePosResult measurePos(const Mat &img, const T &measureHandle, double sigma, double threshold,
                            TransitionType transition, SelectType select) {
    validateArgs(img, sigma, threshold);
    Mat profile = measureProjection(img, measureHandle);

    auto returnVal = getEdgeAmplitudes(profile, measureHandle, sigma, threshold, transition);
    std::vector<double> &aboveThreshold = returnVal.first;
    std::vector<double> &amplitudes = returnVal.second;

    if (!amplitudes.empty()) {
        if (select == SelectType::FIRST)
            amplitudes = getFirst(amplitudes);

        else if (select == SelectType::LAST)
            amplitudes = getLast(amplitudes);
    }

    PosDistanceResult pos = findEdgePos(measureHandle, aboveThreshold, select);
    return MeasurePosResult(pos, amplitudes);
}

template MeasurePosResult
measurePos<MeasureRectangle>(const Mat &img, const MeasureRectangle &measureHandle, double sigma, double threshold,
                             TransitionType transition, SelectType select);

template MeasurePosResult
measurePos<MeasureArc>(const Mat &img, const MeasureArc &measureHandle, double sigma, double threshold,
                       TransitionType transition, SelectType select);

template MeasurePosResult
measurePos<MeasureArcTransposed>(const Mat &img, const MeasureArcTransposed &measureHandle, double sigma, double threshold,
                                 TransitionType transition, SelectType select);

template<typename T>
MeasurePairsResult
measurePairs(const Mat &img, const T &measureHandle, double sigma, double threshold,
             TransitionType transition, SelectType select, bool pickStrongest) {
    // Detect all edges
    MeasurePosResult edges = measurePos(img, measureHandle, sigma, threshold);

    // Make sure there are at least 2 edges (for at least one edge pair)
    if (edges.amplitudes.size() < 2)
        return MeasurePairsResult{};

    // If transition is ALL, the first edge determines the polarity of edge pairs to detect
    if (transition == TransitionType::ALL)
        transition = edges.amplitudes.front() > 0 ? TransitionType::POSITIVE : TransitionType::NEGATIVE;

    // Iterate through edges to find the edge pairs
    int sign = transition == TransitionType::POSITIVE ? 0 : 1;

    int index = 0;                   // Current index
    std::vector<int> firstIndex{};   // Indices within amplitudes for the first edge of each edge pair
    std::vector<int> secondIndex{};  // Indices within amplitudes for the second edge of each edge pair

    while (true) {
        // Find the first edge of the pair
        int temp = nextSignedElement(edges.amplitudes, sign, index, pickStrongest);

        if (temp == -1)
            break;

        firstIndex.push_back(temp);
        index = temp + 1;

        // Find the second edge of the pair
        temp = nextSignedElement(edges.amplitudes, !sign, index, pickStrongest);

        if (temp == -1)
            break;

        secondIndex.push_back(temp);
        index = temp + 1;
    }

    // Make sure firstIndex and secondIndex have the same length
    firstIndex.resize(secondIndex.size());

    // Initialize vectors to return
    size_t n = firstIndex.size();

    auto posFirst = indexArray(edges.pos, firstIndex);
    auto amplitudesFirst = indexArray(edges.amplitudes, firstIndex);

    auto posSecond = indexArray(edges.pos, secondIndex);
    auto amplitudesSecond = indexArray(edges.amplitudes, secondIndex);

    // Find intraDistance and interDistance
    std::vector<double> intraDistance(n);
    std::vector<double> interDistance(n == 0 ? 0 : n - 1);

    auto distancePtr = edges.distances.cbegin();

    for (size_t i = 0; i < n; i++)
        intraDistance[i] = std::accumulate(distancePtr + firstIndex[i], distancePtr + secondIndex[i], 0.0);

    for (size_t i = 0; i < interDistance.size(); i++)
        interDistance[i] = std::accumulate(distancePtr + secondIndex[i], distancePtr + firstIndex[i + 1], 0.0);

    auto result = MeasurePairsResult{posFirst, amplitudesFirst, posSecond,
                                     amplitudesSecond, intraDistance, interDistance};

    // Return values
    switch (select) {
        case SelectType::ALL:
            return result;

        case SelectType::FIRST:
            return result.front();

        default:
            return result.back();
    }
}

template MeasurePairsResult
measurePairs<MeasureRectangle>(const Mat &img, const MeasureRectangle &measureHandle, double sigma,
                               double threshold,
                               TransitionType transition, SelectType select, bool pickStrongest);

template MeasurePairsResult
measurePairs<MeasureArc>(const Mat &img, const MeasureArc &measureHandle, double sigma, double threshold,
                         TransitionType transition, SelectType select, bool pickStrongest);

template<typename T>
PosDistanceResult measureThresh(const Mat &img, const T &measureHandle, double sigma, double threshold,
                                SelectType select) {
    validateArgs(img, sigma, threshold);

    // Get the 1D profile
    Mat profile = measureProjection(img, measureHandle);

    // Gaussian smoothing before detecting edges
    GaussianBlur(profile, profile, Size(3, 1), sigma, 0, BORDER_REPLICATE);

    // Find the points where profile crosses the threshold
    std::vector<double> matchThreshold{};
    auto profptr = profile.ptr<double>();

    for (int i = 0; i < profile.cols; i++) {
        // Exact match
        if (threshold == profptr[i])
            matchThreshold.push_back(i);

        // Crosses the threshold
        if (i < profile.cols - 1 and betweenInclusive(profptr[i], profptr[i + 1], threshold)) {
            // Do linear interpolation
            double subpixel = (threshold - profptr[i]) / (profptr[i + 1] - profptr[i]);
            matchThreshold.push_back(i + subpixel);
        }
    }

    return findEdgePos(measureHandle, matchThreshold, select);
}

template PosDistanceResult
measureThresh<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
                                double threshold, SelectType select);

template PosDistanceResult
measureThresh<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma,
                          double threshold, SelectType select);
