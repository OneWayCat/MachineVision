/**
 * @file ransacFit.hpp
 * @brief Functions for fitting shapes to points.
 */

#ifndef METROLOGY_RANSAC_FIT_HPP
#define METROLOGY_RANSAC_FIT_HPP

#include <ciso646>
#include <opencv2/opencv.hpp>

// The coordinate system is (r, c) -> (x, y)

/**
 * @brief This struct holds the center and radius which defines a circle.
 */
struct CircleParams {
    cv::Point2d center;
    double radius;
};

/**
 * @brief This struct holds the center, radii and rotation angle which defines a rotated ellipse.
 */
struct EllipseParams {
    cv::Point2d center;
    double shortRadius;
    double longRadius;
    double phi;
};

/** 
 * @brief This struct represents the equation of a line in ax + by + c = 0 form
 */
struct LineEqParams {
    double a;
    double b;
    double c;
};
/**
 * @brief This struct represents a line SEGMENT with start and end points.
 */
struct LineParams {
    cv::Point2d startPoint;
    cv::Point2d endPoint;
};

/**
 * @brief This struct holds the row, column, half width, half height and phi of a rotated rectangle.
 */
struct RectParams {
    double row;
    double column;
    double halfWidth;
    double halfHeight;
    double phi;

    std::vector<cv::Point2d> rectPoints;  // The 4 corners of the rectangle in counterclockwise order
};

/**
 * @brief Fit a circle through 3 or more points.
 * @param points Points in (row, column) format
 * @return The center and the radius of the circle.
 */
CircleParams fitCircle(const std::vector<cv::Point2d> &points);

/**
 * @brief Fit an ellipse through exactly 5 points.
 * @param points Points in (row, column) format
 * @return The parameters of the ellipse. If the ellipse is invalid, then return a zero-initialized struct.
 */
EllipseParams fitEllipse5(const std::vector<cv::Point2d> &points);

/**
 * @brief Fit an ellipse through 5 or more points.
 * @param points Points in (row, column) format
 * @return The parameters of the ellipse.
 */
EllipseParams fitEllipse(const std::vector<cv::Point2d> &points);

/**
 * @brief Fit a line through exactly 2 points.
 * @param points Points in (row, column) format
 * @return The parameters of the line.
 */
LineEqParams fitLine2(const std::vector<cv::Point2d> &points);

/**
 * @brief Fit a line through 2 or more points.
 * @param points Points in (row, column) format
 * @return The parameters of the line.
 */
LineEqParams fitLine(const std::vector<cv::Point2d> &points);

/**
 * @brief Fit a rectangle through 8 or more points.
 * @param points Points in (row, column) format.
 * @param pointsPerSide The number of points per side.
 * @return The center and half lengths of the rectangle.
 */
RectParams fitRect(const std::vector<cv::Point2d> &points, const std::vector<size_t> &pointsPerSide);

#endif  //MEASURING_RANSACFIT_HPP
