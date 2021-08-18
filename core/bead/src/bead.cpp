#include "bead.hpp"
#include "mathutils.hpp"

using namespace cv;


inline bool close(const Point2i &p1, const Point2i &p2) {
	constexpr int threshold = 2;

	const int rowDiff = abs(p1.x - p2.x);
	const int colDiff = abs(p1.y - p2.y);

	return rowDiff <= threshold and colDiff <= threshold;
}

BrighterSide findPolarity(const Mat &img, const Point2i &startPt, const Direction startDir) {
	// Find the polarity of the edges
	const int leftOfStart = getPixelValue(img, startPt, left(startDir));
	const int rightOfStart = getPixelValue(img, startPt, right(startDir));

	if (leftOfStart >= rightOfStart)
		return BrighterSide::LEFT;
	else
		return BrighterSide::RIGHT;

}

// Get the angle of the vector from pt1 to pt2
Direction Bead::getDirP2P(const Point2i &pt1, const Point2i &pt2) {
	const int rowDiff = -(pt2.x - pt1.x);
	const int colDiff = pt2.y - pt1.y;
	return angle2Dir(atan2(rowDiff, colDiff));
}

// ----------- //
// Constructor //
// ----------- //

Bead::Bead(const cv::Mat &img, cv::Mat &binImg, const cv::Point2d &end,
		   const tinynurbs::RationalCurve<double> &splineCurve)
		: image{img},
		  binaryImage{binImg},
		  endPt{end},
		  spline{splineCurve} {}

void Bead::setTracking(const cv::Point2d &start, const cv::Point2d &to) {
	const Direction approxDir = getDirP2P(currentPt, to);
	setTracking(start, approxDir);
}

void Bead::setTracking(const Point2d &start, Direction towards) {
	// Determine the starting point and direction
	currentPt = start;

	/* Using the starting point and the point to go towards, get an approximate direction
	 * to start the tracking. */
	nextDir = -1;

	// With approxDir, get a more accurate estimate for the starting direction.
	currentDir = proposeNextDir(currentPt, towards, true);
	polarity = findPolarity(image, currentPt, currentDir);

	const int leftPix = getPixelValue(image, currentPt, left(currentDir));
	const int rightPix = getPixelValue(image, currentPt, right(currentDir));
	pixLimit = (leftPix + rightPix) / 2;
}

// -------------------------------------------------------------------------- //
// Foundational GETTERS. Everything that doesn't call other internal methods. //
// -------------------------------------------------------------------------- //

int Bead::getDistanceSq(const Point2i &otherPt) const {
	return square(currentPt.x - otherPt.x) + square(currentPt.y - otherPt.y);
}

Point2i Bead::peekAheadPt(Direction dir) const {
	return currentPt + Dirs2D[dir];
}

Point2i Bead::peekAheadPt(const Point2i &pt, Direction dir) {
	return pt + Dirs2D[dir];
}

int Bead::getContrast(const Point2i &pt, const Direction dir) const {
	constexpr double diagonalFactor = 0.7;

	const int leftPix = getPixelValue(image, pt, left(dir));
	const int rightPix = getPixelValue(image, pt, right(dir));

	int brightPix, darkPix;

	if (polarity == BrighterSide::LEFT) {
		brightPix = leftPix;
		darkPix = rightPix;
	} else {
		brightPix = rightPix;
		darkPix = leftPix;
	}

	if (min(darkPix, brightPix) > pixLimit)
		return 0;

	const int contrast = brightPix - darkPix;

	// Discount contrast by a certain factor if the direction is diagonal
	if (dir & 1)
		return int(contrast * diagonalFactor);

	return contrast;
}

Direction Bead::proposeNextDir(const Point2i &startPt, const Direction startDir, const bool useMagnitude) const {
	constexpr int NUM_DIRECTIONS = 5;
	int contrasts[NUM_DIRECTIONS];

	Direction directions[NUM_DIRECTIONS] = {
			startDir,
			turnRight(startDir),
			turnLeft(startDir),
			right(startDir),
			left(startDir)
	};

	for (int i = 0; i < NUM_DIRECTIONS; i++) {
		contrasts[i] = getContrast(peekAheadPt(startPt, directions[i]), directions[i]);

		if (useMagnitude)
			contrasts[i] = abs(contrasts[i]);
	}

	const auto maxContrastPtr = std::max_element(contrasts, contrasts + NUM_DIRECTIONS);
	const auto dirChange = maxContrastPtr - contrasts;

	return directions[dirChange];
}

Direction Bead::proposeNextDir() {
	if (nextDir == -1)
		nextDir = proposeNextDir(currentPt, currentDir);

	return nextDir;
}

//------------------------------------------------------------------------------------------------------ //
// Derivative GETTERS. Methods that call other internal methods and act like a wrapper for cleaner usage //
// ----------------------------------------------------------------------------------------------------- //

Contour Bead::lookAhead(const int n) const {
	Point2i cPt = currentPt;
	Direction cDir = currentDir;
	Direction nDir;  // Next direction

	Contour lookAheadPts;
	lookAheadPts.reserve(n);
	lookAheadPts.push_back(cPt);

	for (int j = 0; j < n - 1; j++) {
		nDir = proposeNextDir(cPt, cDir);

		// moveAhead
		cPt = peekAheadPt(cPt, nDir);
		lookAheadPts.push_back(cPt);
		cDir = nDir;
	}

	return lookAheadPts;
}

// ----------------------------------------------------------------------------------------------------- //
// Derivative SETTERS. Methods that call other internal methods and act like a wrapper for cleaner usage //
// ----------------------------------------------------------------------------------------------------- //

Point2i Bead::moveAhead() {
	const Direction dir = proposeNextDir();

	binaryImage.at<uchar>(currentPt.x, currentPt.y) = 255;

	currentPt = peekAheadPt(dir);
	currentDir = dir;

	return currentPt;
}

// ------------ //
// Core Methods //
// ------------ //

BeadIntersectionType Bead::checkIntersection(const Point2i &pt, const Bead &otherBead) const {
	const Point2i thisDirVec = Dirs2D[currentDir];
	const Point2i otherDirVec = Dirs2D[otherBead.currentDir];
	const Point2i thisToOther = otherBead.currentPt - currentPt;

	if (binaryImage.at<uchar>(pt.x, pt.y) == 255)
		return BeadIntersectionType::SELF;

	else if (thisDirVec.dot(otherDirVec) == -1 and thisToOther.dot(otherDirVec) >= 0)
		return BeadIntersectionType::OTHER;

	else if (close(currentPt, endPt))
		return BeadIntersectionType::FINAL;

	return BeadIntersectionType::NONE;
}

BeadIntersectionType Bead::step(const Bead &otherBead) {
	BeadIntersectionType ret = checkIntersection(peekAheadPt(proposeNextDir()), otherBead);

	if (ret == BeadIntersectionType::NONE)
		moveAhead();

	nextDir = -1;
	return ret;
}
