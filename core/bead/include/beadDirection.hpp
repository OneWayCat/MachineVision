#ifndef BEAD_DIRECTION_HPP
#define BEAD_DIRECTION_HPP

#include <opencv2/core/core.hpp>

/* Array of directions in 2 dimensions.
 * The first element is row direction and the second is column direction.
 * Starting from east, it is ordered in counterclockwise order. */
const cv::Point2i Dirs2D[8] = {
		{0,  1},
		{-1, 1},
		{-1, 0},
		{-1, -1},
		{0,  -1},
		{1,  -1},
		{1,  0},
		{1,  1}
};


/* An integer that represents the index within the Dirs2D array.
 * Must be in range 0 - 7. */
typedef int Direction;

constexpr int DIR_MASK = 0b111;

/**
 * @param dir Reference direction
 * @return Integer value of direction 45 degrees to the left, relative to parameter.
 */
inline Direction turnLeft(const Direction dir) {
	return (dir + 1) & DIR_MASK;
}

/**
 * @param dir Reference direction
 * @return Integer value of direction 45 degrees to the right, relative to parameter.
 */
inline Direction turnRight(const Direction dir) {
	return (dir - 1) & DIR_MASK;
}

/**
 * @param dir Reference direction
 * @return Integer value of direction to the left, relative to parameter.
 */
inline Direction left(const Direction dir) {
	return (dir + 2) & DIR_MASK;
}

/**
 * @param dir Reference direction
 * @return Integer value of direction to the right, relative to parameter.
 */
inline Direction right(const Direction dir) {
	return (dir - 2) & DIR_MASK;
}

/**
 * @brief Get the coordinates of a neighbor pixel.
 *
 * @param pt Reference point
 * @param dir Direction to extract point from
 * @return Neighbor point
 */
inline cv::Point2i getPoint(const cv::Point2i &pt, const Direction dir) {
	return pt + Dirs2D[dir];
}

/**
 * @brief Given a point and a direction, get the pixel value at that neighbor pixel.
 *
 * @param img Reference image
 * @param pt Reference point
 * @param dir Direction to retrieve a pixel from relative to reference point.
 * @return Pixel value of neighbor point
 */
inline int getPixelValue(const cv::Mat &img, const cv::Point2i &pt, const Direction dir) {
	const int r = pt.x;
	const int c = pt.y;
	const cv::Size imgSize = img.size();

	if (r < 0 or c < 0 or r >= imgSize.height or c >= imgSize.width)
		throw std::out_of_range("Point access out of bounds in getPixelValue(). Unable to track bead properly");

	const cv::Point2i &nextPt = getPoint(pt, dir);
	return img.at<uchar>(nextPt.x, nextPt.y);
}

/**
 * @brief Given an angle in radians, convert it to one of the indicies in Dirs2D
 *
 * @param angle Angle in radians
 * @return Direction of the angle given
 */
inline Direction angle2Dir(double angle) {
	if (angle < 0)
		angle += boost::math::double_constants::two_pi;

	constexpr double DIR_STEP = boost::math::double_constants::quarter_pi;
	return int(round(angle / DIR_STEP)) % 8;
}

#endif
