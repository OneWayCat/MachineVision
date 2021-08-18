#ifndef MENISCUS_HPP
#define MENISCUS_HPP

#include "measuring.hpp"

enum class MeniscusShape {
    CONCAVE,
    CONVEX
};

struct MeniscusParams {
    unsigned int numMeasures = 20;
    // The half height parameter for each measure handle.
    int measureHalfHeight = 10;
    // Phi parameter for each measure handle
    double measurePhi = boost::math::double_constants::half_pi;
    // Sigma for edge detection
    double measureSigma = 1.1;
    // The proportion of edges to trim from each side
    double trimProportion = 0.25;
    // Edge ampltidue threshold to use for measure position call
    double edgeThresh = 20;
    // Value from MeniscusShape enum to select for concave or convex meniscus
    MeniscusShape meniscusShape;
};

/**
 * @brief Find the vertex point of a meniscus in a user selected region in an image
 * @details User specifies convex or concave meniscus to detect. Internally calls 1D Measuring functions
 * to assist with edge detection. A parabola least squares fit is used to determine the local minimum / maximum.
 * 
 * @param img Grayscale image of interest
 * @param topLeft Top left point of a rectangular region of interest in (row, col) format
 * @param bottomRight Bottom right point of a rectangular region of interest
 * @param meniscusParams Parameter struct for hyper parameters for meniscus detection
 */
cv::Point2d findMeniscus(const cv::Mat& img, const cv::Point2d& topLeft, const cv::Point2d& bottomRight, MeniscusParams meniscusParams);

#endif
