#include "inRegion.hpp"
#include "mathutils.hpp"
#include "ransacError.hpp"

// A circle is defined by at least 3 points
constexpr int MIN_NUM_MEASURES = 3;

using namespace boost::math;

// Constructor
ArcObject::ArcObject(const MeasureParams &measureParams, int row, int col, int radius, double angleStart,
                     double angleExtent)
        : MetrologyObject{measureParams, MIN_NUM_MEASURES},
          row{row},
          column{col},
          radius{radius},

          angleStart{mapTo2Pi(angleStart)},
          angleExtent{angleExtent} {

    if (abs(angleExtent) > double_constants::two_pi)
        throw std::invalid_argument("Angle extent must be between -2pi and 2pi");

    if (radius < 1)
        throw std::invalid_argument("Radius must be positive");

    // UNCOMMENT FOR DEBUGGING
    // initMeasures();
}

int ArcObject::findNumMeasures() {
    int numMeasures = measureParams.numMeasures;

    if (numMeasures <= 0) {
        // Use separation
        double arcSeparation = measureParams.separation / radius;
        numMeasures = int(abs(angleExtent) / arcSeparation);
    }

    return std::max(numMeasures, MIN_NUM_MEASURES);
}

void ArcObject::initMeasures() {
    validateMeasureParams();

    if (measureParams.perpLength >= radius)
        throw std::invalid_argument("measurePerpLength must be less than the radius");

    int numMeasures = findNumMeasures();

    // Find the separation between consecutive measures (in radians)
    double arcSeparation = angleExtent / numMeasures;

    // Find the starting polar angle (along the arc) for the measure handles
    double angle = angleStart + arcSeparation / 2.0;
    angle += double_constants::half_pi;  // Add pi/2 because angle is measured from horizontal axis

    metResults.clear();
    measures.clear();

    for (int i = 0; i < numMeasures; i++) {
        // Find the center of the measure handle
        double centerRow = row + radius * cos(angle);
        double centerCol = column + radius * sin(angle);

        // Map angle to [-pi, pi]
        measures.emplace_back(measureParams.perpLength, measureParams.tangLength,
                              int(round(centerRow)), int(round(centerCol)), mapToPi(angle - double_constants::half_pi));

        angle += arcSeparation;
    }

    initMeasuresFuzzy();
}

std::vector<int>
ArcObject::getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                      double distThresh) {

    auto shape = fitCircle(fitPoints);
    return arcInliers(shape, testPoints, distThresh);
}

bool ArcObject::addResult(const std::vector<cv::Point2d> &inliers, bool allowOutsideRegion) {
    CircleParams shape = fitCircle(inliers);

    if (!allowOutsideRegion and !inArc(*this, shape))
        return false;

    metResults.push_back(shape);
    return true;
}