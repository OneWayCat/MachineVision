#include "inRegion.hpp"
#include "mathutils.hpp"
#include "ransacError.hpp"

// A circle is defined by at least 3 points
constexpr int MIN_NUM_MEASURES = 3;

using namespace boost::math;

// Constructor
ArcProjectionObject::ArcProjectionObject(const MeasureParams &measureParams, int row, int col, int radius, double angleStart,
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
//     initMeasures();
}

int ArcProjectionObject::findNumMeasures() {
    int numMeasures = measureParams.numMeasures;

    if (numMeasures <= 0) {
        // Use separation
        double arcSeparation = measureParams.separation / radius;
        numMeasures = int(abs(angleExtent) / arcSeparation);
    }

    return std::max(numMeasures, MIN_NUM_MEASURES);
}

void ArcProjectionObject::initMeasures() {
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

    double angleIncrement = (angleStart + angleExtent) / numMeasures;
    double currentAngle = angleStart;

    for (int i = 0; i < numMeasures; i++) {
        measures.emplace_back(radius + 20, radius - 20, row, column, currentAngle, angleIncrement);
        currentAngle += angleIncrement;
        if (currentAngle >= double_constants::pi) {
            currentAngle = 0;
            angleIncrement = -angleIncrement;
        }
    }

//    for (int i = 0; i < numMeasures; i++) {
//        // Find the center of the measure handle
//        double centerRow = row + radius * cos(angle);
//        double centerCol = column + radius * sin(angle);
//
//        // Map angle to [-pi, pi]
//        measures.emplace_back(measureParams.perpLength, measureParams.tangLength,
//                              int(round(centerRow)), int(round(centerCol)), mapToPi(angle - double_constants::half_pi));
//
//        angle += arcSeparation;
//    }
//
    initMeasuresFuzzy();


}

std::vector<int>
ArcProjectionObject::getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                                double distThresh) {

    auto shape = fitCircle(fitPoints);
    return arcInliers(shape, testPoints, distThresh);
}

bool ArcProjectionObject::addResult(const std::vector<cv::Point2d> &inliers, bool allowOutsideRegion) {
    CircleParams shape = fitCircle(inliers);

    if (!allowOutsideRegion and !inArc(*this, shape))
        return false;

    metResults.push_back(shape);
    return true;
}