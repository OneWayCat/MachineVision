#include "measuring.hpp"

#include <numeric>
#include <fstream>

#include "arrutils.hpp"
#include "mutils.hpp"
#include "drawSubpixel.hpp"

using namespace cv;



#define DRAW_SP 0

//Mat MeasureHandle::measureProjection_(const Mat &img, double cx, double cy) const {
//    // Ensure image is single channel
//    CV_Assert(img.depth() == CV_8U and img.channels() == 1);
//
//    // Dimensions of the image
//    int imgHeight = img.rows;
//    int imgWidth = img.cols;
//
//    // Coordinates of the bounding box of the measure handle
//    int boxMinC = minC + column;
//    int boxMinR = minR + row;
//
//    size_t numCols = size_t(maxC) - minC + 1;
//    size_t numRows = size_t(maxR) - minR + 1;
//
//    // Project down to 1D profile
//    Mat_<double> profile = Mat_<double>::zeros(1, (int) binCounts.size());
//    auto profPtr = profile.ptr<double>();  // Pointer to the profile array
//
//    double distToMeasureCenter = sqrt(pow(column - cx, 2) + pow(row - cy, 2));
//    double dist2LeftEdge = distToMeasureCenter - (profileLength / 2);
//    std::vector<int> pixelCounts(binCounts.size());
////    std::vector<int> pixelVals;
////    std::vector<int> inds;
//
//#if DRAW_SP
//    Mat colorClone = img.clone();
//    cvtColor(colorClone, colorClone, COLOR_GRAY2BGR);
//    SubpixelWorkspace spWS(colorClone, row, column, 10, 50);
//
//    std::vector<Point2d> corners = getRectPoints(*reinterpret_cast<const MeasureRectangle *>(this));
//    for (int i = 0; i < corners.size(); i++) {
//        corners[i] += Point2d(row, column);
//    }
//
//    spWS.drawRect(corners, Scalar(0, 255, 0), false);
//    std::vector<Scalar> colors = {
//            Scalar(255, 0, 0),
//            Scalar(0, 255, 0),
//            Scalar(0, 0, 255)
//    };
//
//#endif
//
//    // Main loop to iterate over each (r, c) pair inside the bounding box
//    for (size_t r = 0; r < numRows; r++) {
//        int imgRow = (int) r + boxMinR;
//
//        // Treat as 0 if out of bounds
//        if (imgRow < 0 or imgRow >= imgHeight)
//            continue;
//
//        const auto rowPtr = img.ptr<uchar>(imgRow);  // Pointer to the row of the image
//
//        for (int c = 0; c < numCols; c++) {
//            int binNumber = binNumbers[r * numCols + c];
//            int imgCol = c + boxMinC;
//
//            // Inside check
//            if (binNumber != -1 and imgCol >= 0 and imgCol < imgWidth) {
//                double dist2Pt = sqrt(pow(imgCol - cx, 2) + pow(imgRow - cy, 2));
////                double phi = atan2(imgRow - cy, imgCol - cx);
////                double x = imgCol + rad * cos(phi);
////                double y = imgRow + rad * sin(phi);
//                int radialBin = (int) round(abs(dist2Pt - dist2LeftEdge));
//                if (radialBin >= binCounts.size())
//                    radialBin = binCounts.size() - 1;
//                else if (radialBin < 0)
//                    radialBin = 0;
//                pixelCounts[radialBin] += 1;
//                profPtr[radialBin] += rowPtr[imgCol];
//#if DRAW_SP
//                spWS.drawPoint(imgRow, imgCol, colors[radialBin % 3]);
////                std::cout << radialBin << "\n";
//#endif
//
//            }
//        }
//    }
//#if DRAW_SP
//    cv::imshow("SP", spWS.getZoomedImage());
//    cv::waitKey(0);
//#endif
//
//    // Divide the profile by binCounts to get the mean
//    for (int i = 0; i < pixelCounts.size(); i++) {
//        if (pixelCounts[i] != 0) {
//            profPtr[i] = (double) profPtr[i] / pixelCounts[i];
//        }
//    }
//
//    return profile;
//}

//Mat measureProjection(const Mat &img, const MeasureHandle &measureHandle, double cx, double cy) {
//    return measureHandle.measureProjection_(img, cx, cy);
//}

/* Find the subpixel position and the amplitude of the edges at that position given all the
 * edge amplitudes and the position of the local maxima/minima  */

//template MeasurePosResult
//measurePos<MeasureRectangle>(const Mat &img, const MeasureRectangle &measureHandle, double sigma, double threshold,
//                             TransitionType transition, SelectType select);
//
//template MeasurePosResult
//measurePos<MeasureArc>(const Mat &img, const MeasureArc &measureHandle, double sigma, double threshold,
//                       TransitionType transition, SelectType select);
//
//template MeasurePosResult
//measurePos<MeasureArcTransposed>(const Mat &img, const MeasureArcTransposed &measureHandle, double sigma, double threshold,
//                                 TransitionType transition, SelectType select);

//template<typename T>
//MeasurePairsResult
//measurePairs(const Mat &img, const T &measureHandle, double sigma, double threshold,
//             TransitionType transition, SelectType select, bool pickStrongest)

//template MeasurePairsResult
//measurePairs<MeasureRectangle>(const Mat &img, const MeasureRectangle &measureHandle, double sigma,
//                               double threshold,
//                               TransitionType transition, SelectType select, bool pickStrongest);
//
//template MeasurePairsResult
//measurePairs<MeasureArc>(const Mat &img, const MeasureArc &measureHandle, double sigma, double threshold,
//                         TransitionType transition, SelectType select, bool pickStrongest);

//template<typename T>
//PosDistanceResult measureThresh(const Mat &img, const T &measureHandle, double sigma, double threshold,
//                                SelectType select)

//template PosDistanceResult
//measureThresh<MeasureRectangle>(const cv::Mat &img, const MeasureRectangle &measureHandle, double sigma,
//                                double threshold, SelectType select);
//
//template PosDistanceResult
//measureThresh<MeasureArc>(const cv::Mat &img, const MeasureArc &measureHandle, double sigma,
//                          double threshold, SelectType select);
