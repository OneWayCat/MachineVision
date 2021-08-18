/**
* @file beadModel.hpp
* @brief BeadModel class for bead inspection.
*/

#ifndef BEAD_MODEL
#define BEAD_MODEL

#include "beadMeasuring.hpp"
#include "tinynurbs/tinynurbs.h"

typedef std::vector<cv::Point2i> Contour;

/**
 * @brief Types of invalid bead segments.
*/
enum class ErrorType {
	VALID,
	/** No bead could be detected at the segment. */
	NO_BEAD,
	/** Bead position is out of range */
	OUT_OF_RANGE,
	/** The bead is too thin. */
	TOO_THIN,
	/** The bead is too thick. */
	TOO_THICK
};

/**
 * @brief Structure representing the results of the applyBeadModel fuction.
*/
struct BeadInspectionResult {
	/** Left contours of the beads. */
	std::vector<std::pair<Contour, ErrorType>> leftContours;
	/** Right contours of the beads. */
	std::vector<std::pair<Contour, ErrorType>> rightContours;

	/** Empty constructor */
	BeadInspectionResult() = default;

	/**
	 * @brief Constructs a BeadInspectionResult from series of points and the
	 * bead ErrorTypes of each of those points.
	 * @details Used as final return format for the whole of bead inspection.
	 *
	 * @param leftContours The first of a pair of a series of points and their ErrorTypes
	 * @param rightContours The second of a pair of a series of points and their ErrorTypes
	 */
	BeadInspectionResult(std::vector<std::pair<Contour, ErrorType>> &leftContours,
						 std::vector<std::pair<Contour, ErrorType>> &rightContours) :
			leftContours{std::move(leftContours)},
			rightContours{std::move(rightContours)} {}
};

/**
 * @brief A model for inspecting beads in images.
*/
class BeadModel {
private:
	/** The original beadContour fit to a NURBS b-spline.*/
	tinynurbs::RationalCurve<double> spline{};

	/** The centers of each measure handle, interpolated from the spline */
	Contour measureCenters{};

	/** Maximum number of iterations for tracking*/
	unsigned int iterationLimit = 10000;

	/** Structures to handle edge detection*/
	MeasureRectTemplate handleFunctor{};
	FuzzyMeasuringFunctor fuzzyFunctor{};

	/** Vector to store anchor points that were already calculated*/
	std::vector<std::vector<cv::Point2i>> anchorPts{};


public:
	/** Contour representing the expected position of the bead. */
	Contour beadContour{};
	/** Vector to store a dense represenation of the spline */
	std::vector<cv::Point2i> denseSpline{};
	/** Expected thickness of the bead. */
	int targetThickness = 50;
	/** Tolerance of the thickness parameter. */
	int thicknessTol = 15;
	/** Tolerance of the position of the bead. */
	int positionTol = 15;
	/** Number of measure handles to use */
	int numMeasures{};

	/** Measures aligned to the contour user gives as a template. */
	std::vector<MeasureRectangle> modelMeasures{};

	/**
	 * @brief Default constructor
	 * @details Accepts a series of points which act as the template to perform inspection on
	 *
	 * @param beadContour Points of a template
	 */
	explicit BeadModel(const Contour &beadContour)
			: beadContour{beadContour},
			  numMeasures{int(beadContour.size())} {
		if (beadContour.size() < 2)
			throw std::invalid_argument("beadContour must have at least 2 points");
	}

	// TODO: Fix doc
	/**
	 * @brief Parameter constructor
	 * @details Accepts all pertinent hyper parameters as input
	 *
	 * @param beadContour Points of a template
	 * @param targetThick Expected thickness of the bead.
	 * @param thickTol Tolerance of the thickness parameter.
	 * @param posTol Tolerance of the position of the bead.
	 * @param numMeas Number of measure handles to use
	 */
	BeadModel(const Contour &beadContour, unsigned int targetThick, unsigned int thickTol, unsigned int posTol,
			  unsigned int iterLimit = 10000)
			: BeadModel{beadContour} {

		targetThickness = int(targetThick);
		thicknessTol = int(thickTol);
		positionTol = int(posTol);
		iterationLimit = iterLimit;
	}

	/**
	 * @brief Inspect beads in an image.
	 *
	 * @param img Image to perform bead inspection on
	 * @param rectTemplate A struct serving to contain all of the hyper parameters necessary for generating MeasureRectangles along the user given contour
	 * @param measureFunctor A functor which wraps fuzzyMeasurePairs which helps with hyper parameter organization.
	 * @param iterationLimit Maximum number of iterations for tracking.
	 * @return
	 */
	BeadInspectionResult applyBeadModel(const cv::Mat &img, const MeasureRectTemplate &rectTemplate,
										const FuzzyMeasuringFunctor &measureFunctor);

	/**
	 * @brief Get the anchor points along the bead at a specified index.
	 * @param img
	 * @param index
	 * @return A vector of length 2 containing the points on both sides of the bead.
	*/
	std::vector<cv::Point2i> getAnchorPts(const cv::Mat &img, int index);

	const tinynurbs::RationalCurve<double> &getSpline() const {
		return spline;
	}

	unsigned int getIterationLimit() const {
		return iterationLimit;
	};

	const Contour &getMeasureCenters() const {
		return measureCenters;
	}

private:
	/**
	 * @brief Track the location of the bead and find the invalid segments.
	 * @param img
	 * @return The valid and invalid sections of the bead.
	*/
	BeadInspectionResult trackBead(const cv::Mat &img);
};

#endif  // BEAD_MODEL
