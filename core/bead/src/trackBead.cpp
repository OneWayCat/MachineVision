#include "fitSpline.hpp"
#include "bead.hpp"
#include "arrutils.hpp"
#include "posBead.hpp"

using namespace cv;
using namespace boost::math;

#define BEAD_DEBUGGING 0

// Constants for indexing
constexpr int LOWER = 0, UPPER = 1;

#if BEAD_DEBUGGING
Mat color;
const std::vector<Vec3b> COLORS = {
		//            Vec3b(148, 0, 211),
		//            Vec3b(255, 165, 0),
		Vec3b(0, 255, 255),
};
const std::vector<Vec3b> LOOKAHEADCOLORS = {
		Vec3b(0, 255, 255),
		Vec3b(212, 189, 145),
		Vec3b(55, 54, 130),
		Vec3b(170, 103, 178),
		Vec3b(193, 181, 93)
};
size_t colorInd = 0;
bool skipTracking = false;

Point reverse(const Point &pt) {
	return {pt.y, pt.x};
}

#endif

// Zoom in on an image to view the pixels more closely.
// Center is in (row, column), so no need to coordinate swap!
Mat getZoomedImg(const Mat &img, const Point2i &center, const int buffer, const int scale) {
	const int roiLength = buffer * 2 + 1;
	const Point2i topLeft{center.y - buffer, center.x - buffer};
	Mat roi(img, Rect(topLeft, Size(roiLength, roiLength)));

	resize(roi, roi, Size(), scale, scale, INTER_NEAREST);
	return roi;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

int getNextCrvPt(const Mat &img, const Point2d &currentPt, const Contour &splineAnchors) {
	if (splineAnchors.size() <= 1) {
		throw std::invalid_argument(
				"ERROR in getNextCrvPt. "
				"The number of potential locations for MeasureHandles should be atleast 1\n");
	}

	double minDistSq = std::numeric_limits<double>::max();
	int retInd = -1;

	for (size_t i = 1; i < splineAnchors.size(); i++) {
		const double distSq = distPtToSegSq(currentPt, splineAnchors[i - 1], splineAnchors[i]);

#if BEAD_DEBUGGING
		Mat temp = img.clone();
		cvtColor(temp, temp, COLOR_GRAY2BGR);
		line(temp, reverse(splineAnchors[i - 1]), reverse(splineAnchors[i]), Scalar(0, 0, 255));
		line(temp, reverse(currentPt), reverse(splineAnchors[i]), Scalar(0, 255, 0));
		line(temp, reverse(currentPt), reverse(splineAnchors[i - 1]), Scalar(0, 255, 0));

		imshow("temp", temp);
		waitKey(0);
#endif

		if (distSq <= minDistSq) {
			retInd = i;
			minDistSq = distSq;
		}
	}

#if BEAD_DEBUGGING
	destroyWindow("temp");
#endif

	return retInd;
}

// Track a bead section, until it stops for some reason. The main logic of the below two functions.
std::pair<BeadIntersectionType, std::vector<WidthSample>> trackSection(Bead &upper, Bead &lower,
																	   const BeadModel &beadModel) {

	// Number of points to look ahead when syncing the upper and lower beads.
	constexpr int LOOK_AHEAD = 5;
	unsigned int iterations = 0;

	BeadIntersectionType interUpper = BeadIntersectionType::NONE;
	BeadIntersectionType interLower = BeadIntersectionType::NONE;

	const int minWidthSq = square(beadModel.targetThickness - beadModel.thicknessTol);
	const int maxWidthSq = square(beadModel.targetThickness + beadModel.thicknessTol);
	const int maxDistFromContour = square(beadModel.positionTol);

	std::vector<WidthSample> widthSamples;

	while (interUpper == BeadIntersectionType::NONE && interLower == BeadIntersectionType::NONE) {
		if (iterations >= beadModel.getIterationLimit()) {
			std::cerr
					<< "WARNING: Iteration limit reached during bead tracking. Returning early with whatever results acquired.\n";
			return std::make_pair(BeadIntersectionType::FINAL, widthSamples);
		}

		const Contour upperLookAhead = upper.lookAhead(LOOK_AHEAD);
		const Contour lowerLookAhead = lower.lookAhead(LOOK_AHEAD);

		int upperDists[LOOK_AHEAD];
		int lowerDists[LOOK_AHEAD];

		for (int i = 0; i < LOOK_AHEAD; i++) {
			upperDists[i] = upper.getDistanceSq(lowerLookAhead[i]);
			lowerDists[i] = lower.getDistanceSq(upperLookAhead[i]);
		}

		const int *upperMinPtr = std::min_element(upperDists, upperDists + LOOK_AHEAD);
		const auto upperMinInd = upperMinPtr - upperDists;

		const int *lowerMinPtr = std::min_element(lowerDists, lowerDists + LOOK_AHEAD);
		const auto lowerMinInd = lowerMinPtr - lowerDists;

#if BEAD_DEBUGGING
		Mat allTheAnnotations = color.clone();

		std::vector<int> tempLeadDists(upperDists, upperDists + sizeof upperDists / sizeof upperDists[0]);
		std::cout << "Follow Points: \n" << lowerLookAhead << "\n";
		std::cout << "Leader CPT: " << upper.currentPt << "\n";
		std::cout << "Leader Dists: " << tempLeadDists << "\n";
//		std::cout << "-- " << upperMinInd << ", " << upperMinDist << "\n";

		std::vector<int> tempFollowDists(lowerDists, lowerDists + sizeof lowerDists / sizeof lowerDists[0]);
		std::cout << "Leader Points: \n" << upperLookAhead << "\n";
		std::cout << "Follow Dists: " << tempFollowDists << "\n";
		std::cout << "Follow CPT: " << lower.currentPt << "\n";
//		std::cout << "-- " << lowerMinInd << ", " << lowerMinDist << "\n";
#endif

		if (upperMinInd == lowerMinInd) {
			interUpper = upper.step(lower);
			interLower = lower.step(upper);

			auto errorType = ErrorType::VALID;
			const int widthSq = upper.getDistanceSq(lower.currentPt);

			if (interUpper == BeadIntersectionType::OTHER or
				interLower == BeadIntersectionType::OTHER)
				errorType = ErrorType::NO_BEAD;

			else if (widthSq < minWidthSq)
				errorType = ErrorType::TOO_THIN;

			else if (widthSq > maxWidthSq)
				errorType = ErrorType::TOO_THICK;

			else {
				const Point2d centerPt = (upper.currentPt + lower.currentPt) / 2.0;
//				const double distFromContour = distPtToContourSq(centerPt, beadModel.beadContour);
				const std::pair<int, int> distInds = gsSearch(centerPt, beadModel.denseSpline, 0,
															  beadModel.denseSpline.size());

				const double distFromContour = std::min(euclideanDistance(centerPt, beadModel.denseSpline[distInds.first], false),
											 euclideanDistance(centerPt, beadModel.denseSpline[distInds.second],false));

				if (distFromContour > maxDistFromContour)
					errorType = ErrorType::OUT_OF_RANGE;
			}

			widthSamples.emplace_back(upper.currentPt, lower.currentPt, errorType);

#if BEAD_DEBUGGING
			color.at<Vec3b>(upper.currentPt.x, upper.currentPt.y) = Vec3b(148, 0, 211);
			color.at<Vec3b>(lower.currentPt.x, lower.currentPt.y) = Vec3b(148, 0, 211);

			allTheAnnotations.at<Vec3b>(upper.currentPt.x, upper.currentPt.y) = COLORS[colorInd];
			allTheAnnotations.at<Vec3b>(lower.currentPt.x, lower.currentPt.y) = COLORS[colorInd];

			line(allTheAnnotations, Point2i(upper.currentPt.y, upper.currentPt.x),
				 Point2i(lower.currentPt.y, lower.currentPt.x), COLORS[colorInd]);

			colorInd = (colorInd + 1) % COLORS.size();
#endif

		} else if (upperMinInd > lowerMinInd) {
			interLower = lower.step(upper);

#if BEAD_DEBUGGING
			color.at<Vec3b>(lower.currentPt.x, lower.currentPt.y) = Vec3b(255, 0, 0);
			allTheAnnotations.at<Vec3b>(lower.currentPt.x, lower.currentPt.y) = Vec3b(255, 0, 0);
#endif

		} else {
			interUpper = upper.step(lower);

#if BEAD_DEBUGGING
			color.at<Vec3b>(upper.currentPt.x, upper.currentPt.y) = Vec3b(0, 255, 0);
			allTheAnnotations.at<Vec3b>(upper.currentPt.x, upper.currentPt.y) = Vec3b(0, 255, 0);
#endif
		}

#if BEAD_DEBUGGING
		for (int i = 0; i < LOOK_AHEAD; i++) {
			allTheAnnotations.at<Vec3b>(lowerLookAhead[i].x, lowerLookAhead[i].y) = LOOKAHEADCOLORS[i];
			allTheAnnotations.at<Vec3b>(upperLookAhead[i].x, upperLookAhead[i].y) = LOOKAHEADCOLORS[i];
		}

		Mat mainROI = getZoomedImg(color, (lower.currentPt + upper.currentPt) / 2, 30, 10);
		Mat annotROI = getZoomedImg(allTheAnnotations, (lower.currentPt + upper.currentPt) / 2, 30, 10);

		if (upperMinInd == lowerMinInd) {
			putText(annotROI, format("%d", upper.getDistanceSq(lower.currentPt)),
					(annotROI.size() / 2) - Size(50, 150),
					FONT_HERSHEY_SIMPLEX, 0.7, Vec3b(0, 255, 255), 2);
		}

		std::cout << "\n";
		if (not skipTracking) {
			imshow("Color", color);
			imshow("mainROI", mainROI);
			imshow("allTheAnnotations", allTheAnnotations);
			imshow("annotROI", annotROI);

			int keyPressed = waitKey(0);
			if (keyPressed == 'q') {
				skipTracking = true;
				destroyWindow("Color");
				destroyWindow("mainROI");
				destroyWindow("allTheAnnotations");
				destroyWindow("annotROI");

			}
		}

#endif

		iterations++;
	}

	const BeadIntersectionType ret = (interUpper == BeadIntersectionType::NONE) ? interLower :
									 interUpper;

	return std::make_pair(ret, widthSamples);
}

// Backtrack after hitting a dead end.
std::pair<BeadIntersectionType, std::vector<WidthSample>> backtrack(const Mat &img, Bead &upperBead, Bead &lowerBead,
																	BeadModel &beadModel) {

	const Point2d averagePt = (upperBead.currentPt + lowerBead.currentPt) / 2.0;
	const Contour &measureCenters = beadModel.getMeasureCenters();

	int anchorInd = getNextCrvPt(img, averagePt, measureCenters);
	std::vector<Point2i> startingAnchorPts = beadModel.getAnchorPts(img, anchorInd);

	const Point2i holder{-1, -1};

	while (startingAnchorPts[0] == holder || startingAnchorPts[1] == holder)
		startingAnchorPts = beadModel.getAnchorPts(img, ++anchorInd);

	Direction approxDir = Bead::getDirP2P(measureCenters[anchorInd], measureCenters[size_t(anchorInd) - 1]);
	upperBead.setTracking(startingAnchorPts[UPPER], approxDir);
	lowerBead.setTracking(startingAnchorPts[LOWER], approxDir);

	auto ret = trackSection(upperBead, lowerBead, beadModel);

	// Flip the direction for the next sections
	approxDir = Bead::getDirP2P(measureCenters[anchorInd], measureCenters[size_t(anchorInd) + 1]);
	upperBead.setTracking(startingAnchorPts[UPPER], approxDir);
	lowerBead.setTracking(startingAnchorPts[LOWER], approxDir);

	return ret;
}

BeadInspectionResult BeadModel::trackBead(const Mat &img) {
#if BEAD_DEBUGGING
	color = img.clone();
	cvtColor(color, color, COLOR_GRAY2BGR);
#endif

	// Initialize the binary images
	Mat lowerBin = Mat::zeros(img.size(), CV_8U);
	Mat upperBin = Mat::zeros(img.size(), CV_8U);

	const std::vector<Point2i> startingAnchorPts = getAnchorPts(img, 0);
	const std::vector<Point2i> finalAnchorPts = getAnchorPts(img, numMeasures - 1);

	Bead lowerBead = Bead(img, lowerBin, finalAnchorPts[LOWER], spline);
	Bead upperBead = Bead(img, upperBin, finalAnchorPts[UPPER], spline);

	Direction approxDir = Bead::getDirP2P(measureCenters[0], measureCenters[1]);
	lowerBead.setTracking(startingAnchorPts[LOWER], approxDir);
	upperBead.setTracking(startingAnchorPts[UPPER], approxDir);

	// Where the tracked points will be stored
	std::vector<WidthSample> widthSamples;
	auto intersectionType = BeadIntersectionType::NONE;

	while (intersectionType != BeadIntersectionType::FINAL) {
#if BEAD_DEBUGGING
		skipTracking = false;
#endif
		auto ret = trackSection(upperBead, lowerBead, *this);

		intersectionType = ret.first;
		const auto &sectionWidthSamples = ret.second;

		widthSamples.insert(widthSamples.end(), sectionWidthSamples.begin(), sectionWidthSamples.end());

		if (intersectionType == BeadIntersectionType::OTHER or
			intersectionType == BeadIntersectionType::SELF) {
#if BEAD_DEBUGGING
			skipTracking = false;
#endif
			ret = backtrack(img, upperBead, lowerBead, *this);

			widthSamples.insert(widthSamples.end(), sectionWidthSamples.rbegin(), sectionWidthSamples.rend());
		}
	}

	std::vector<std::pair<Contour, ErrorType>> leftContours;
	std::vector<std::pair<Contour, ErrorType>> rightContours;

	for (const WidthSample &ws : widthSamples) {
		const Point2i &leftPt = ws.pt1;
		const Point2i &rightPt = ws.pt2;
		const ErrorType errorType = ws.errorType;

		if (leftContours.empty() or errorType != leftContours.back().second) {
			leftContours.emplace_back(Contour{}, errorType);
			rightContours.emplace_back(Contour{}, errorType);
		}

		leftContours.back().first.push_back(leftPt);
		rightContours.back().first.push_back(rightPt);
	}

	return BeadInspectionResult{leftContours, rightContours};
}
