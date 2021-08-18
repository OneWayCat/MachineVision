#ifndef BEAD_POINT_HPP
#define BEAD_POINT_HPP

#include "beadModel.hpp"
#include "beadDirection.hpp"

typedef std::vector<cv::Point2i> Contour;

/* Types of the result from making a step along the bead. */
enum class BeadIntersectionType {
	// If there was no intersection at all with either binary image
	NONE,
	// If the point intersected its own binary image
	SELF,
	// If the point intersected the other bead's binary image
	OTHER,
	// If the point hit the final anchor point
	FINAL
};

/* A pair of points along the bead where the width was taken and the validity
 * of the bead was determined. */
struct WidthSample {
	cv::Point2i pt1;
	cv::Point2i pt2;
	ErrorType errorType;

	WidthSample(cv::Point2i &point1, cv::Point2i &point2, ErrorType errorType)
			: pt1{point1}, pt2{point2}, errorType{errorType} {};
};

// When tracking the edge, we need to know which side is dark and which one is bright
enum class BrighterSide {
	LEFT,
	RIGHT
};


// A class to represent tracking along one side of the bead.
class Bead {
private:
	// The main image
	const cv::Mat &image;

	// The NURBS spline where the centers of the measure handles are.
	const tinynurbs::RationalCurve<double> spline;

	/* When calculating contrast, the brighter pixel must be above this value, and the darker
	 * pixel must be below this value. */
	int pixLimit = -1;

	// The final anchor point to go towards.
	cv::Point2i endPt{}; 

	Direction nextDir = -1;

	// Polarity of the side of the bead.
	BrighterSide polarity{};

	// Get the difference between the left and the right pixels given a point and a direction.
	int getContrast(const cv::Point2i &pt, Direction dir) const;

	/**
	 * @brief Return the point in the specified direction
	 *
	 * @param dir
	 * @return The next point in the tracking step.
	 */
	cv::Point2i peekAheadPt(Direction dir) const;

	/**
	 * @brief Return the point in the specified direction relative to a specific point.
	 * @param pt
	 * @param dir
	 * @return The next point in the tracking step.
	*/
	static cv::Point2i peekAheadPt(const cv::Point2i &pt, Direction dir);

	/**
	 * @brief Calculate the next direction in the tracking step
	 * @details Gets contrast of the point in the forward, left and right directions and returns
	 * the direction with the most significant contrast.
	 *
	 * @return New direction to move to.
	 */
	Direction proposeNextDir();

	/**
	 * @brief Overloaded variant which calculates the next direction in a tracking step relative to point given
	 *
	 * @param startPt Point which tracking is performed at
	 * @param startDir Next direction relative to point given
	 * @return New direction to move to
	 */
	Direction proposeNextDir(const cv::Point2i &startPt, Direction startDir, bool useMagnitude = false) const;

	/**
	 * @brief Set the internal current point to the next point in the tracking step.
	 * @details Sets current point in binary image to 255. Moves current point into trackedPoints.
	 * Change current point and current direction to next point and direction.
	 *
	 * @return The next point. Next is relative to the "current point" at the moment of method call.
	 */
	cv::Point2i moveAhead();

	/**
	 * @brief Checks if there is an intersection between the point given and both this bead
	 * and the other bead.
	 *
	 * @param pt
	 * @param otherBead
	 * @return Intersection type. Refer to BeadIntersectionType for more details.
	 */
	BeadIntersectionType checkIntersection(const cv::Point2i &pt, const Bead &otherBead) const;

public:
	// The binary image to keep track of points already tracked
	cv::Mat &binaryImage;

	cv::Point2i currentPt{};
	Direction currentDir{};

	// ----------- //
	// Constructor //
	// ----------- //

	/**
	 * @brief Construct a bead object that is designed to track one edge of a bead.
	 * @details This class is only intended to be used in pairs to track both edges of a bead
	 * and to communicate with each other to determine stopping points.
	 *
	 * @param img Grayscale image to track on
	 * @param binImg Black image that is the same size as img. Used as binary image to check for intersections.
	 * @param side Whether bead is upper or lower (0 or 1)
	 * @param beadModel A reference to the bead model.	 */
	Bead(const cv::Mat &img, cv::Mat &binImg, const cv::Point2d &end,
		 const tinynurbs::RationalCurve<double> &splineCurve);

	//TODO: Doc
	void setTracking(const cv::Point2d &start, const cv::Point2d &to);
	void setTracking(const cv::Point2d &start, Direction towards);

	// -------------------------------------------------------------------------- //
	// Foundational GETTERS. Everything that doesn't call other internal methods. //
	// -------------------------------------------------------------------------- //

	/**
	 * @brief Get the squared distance from the currentPt field to otherPt parameter.
	 *
	 * @param otherPt Other point to get distance to
	 * @return Euclidean distance squared.
	 */
	int getDistanceSq(const cv::Point2i &otherPt) const;

	/**
	 * @brief Try making a certain amount of steps and return the resulting points.
	 * Doesn't actually make the steps, so the calling object is not changed.
	 * @param n Number of points to look ahead.
	 * @return The points returned by looking ahead.
	*/
	Contour lookAhead(int n) const;

	//TODO: Doc
	static Direction getDirP2P(const cv::Point2i &pt1, const cv::Point2i &pt2);

	// ------------ //
	// Core Methods //
	// ------------ //

	/**
	 * @brief Proceed to the next point in tracking.
	 * @details Goes through entire pipleine of checking intersection of the next proposed point.
	 * If there is no intersection, step is successful and moveAhead is called. If not, the bead does not move ahead.
	 *
	 * @param otherBead
	 * @return Intersection type. Refer to BeadIntersectionType for more details.
	 */
	BeadIntersectionType step(const Bead& otherBead);
};

#endif
