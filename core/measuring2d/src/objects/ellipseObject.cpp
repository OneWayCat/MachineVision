#include <boost/math/special_functions/ellint_2.hpp>
#include <boost/math/tools/roots.hpp>

#include "inRegion.hpp"
#include "mathutils.hpp"
#include "ransacError.hpp"

// An ellipse is defined by at least 5 points
constexpr int MIN_NUM_MEASURES = 5;

using namespace boost::math;

// Constructor
EllipseObject::EllipseObject(const MeasureParams &measureParams, int row, int col, int longRadius, int shortRadius,
                             double phi, double angleStart, double angleExtent)
        : MetrologyObject{measureParams, MIN_NUM_MEASURES},
          row{row},
          column{col},
          longRadius{longRadius},
          shortRadius{shortRadius},

          phi{mapToPi(phi)},
          angleStart{mapToPi(angleStart)},
          angleExtent{angleExtent} {

    if (longRadius < 1 or shortRadius < 1)
        throw std::invalid_argument("Radius must be positive");

    if (shortRadius > longRadius)
        std::swap(shortRadius, longRadius);

    if (abs(angleExtent) > double_constants::two_pi)
        throw std::invalid_argument("Angle extent must be between -2pi and 2pi");
}

// Function object to calculate the elliptical integral
struct circumFunctor {
    circumFunctor(double target, int longRadius, double eccentricity) :
            target{target}, longRadius{longRadius}, eccentricity{eccentricity} {}

    double operator()(double angle) const {
        double arcLength = 0;

        if (fmod(angle, double_constants::pi) == 0) 
            arcLength = longRadius * ellint_2(eccentricity, double_constants::half_pi) * (angle / double_constants::half_pi);
        else
			arcLength = longRadius * ellint_2(eccentricity, angle);

        return arcLength - target;
    }

   private:
    double target;  // The arc length of the angle we're trying to find
    int longRadius;
    double eccentricity;
};

// Convert a parametric ellipse angle to polar angle
double toPolar(double angle, double shortRadius, double longRadius) {
    double polar = atan2(shortRadius * sin(angle), longRadius * cos(angle));

    if (polar - angle > double_constants::pi)
        polar -= double_constants::two_pi;

    else if (angle - polar > double_constants::pi)
        polar += double_constants::two_pi;

    return polar;
}

int EllipseObject::findNumMeasures(double circum) {
    int numMeasures = measureParams.numMeasures;

    if (numMeasures <= 0) {
        // Use separation
        numMeasures = int(circum / measureParams.separation);
    }

    return std::max(numMeasures, MIN_NUM_MEASURES);
}

void EllipseObject::initMeasures() {
    validateMeasureParams();

    if (measureParams.perpLength >= shortRadius)
        throw std::invalid_argument("measurePerpLength must be less than the radius");

    double angleEnd = angleStart + angleExtent;

    // Convert angleStart and angleEnd into polar angles
    double polarAngleStart = toPolar(angleStart, shortRadius, longRadius);
    double polarAngleEnd = toPolar(angleEnd, shortRadius, longRadius);

    // Calculate the circumference of the elliptical arc
    double eccentricity = sqrt(1.0 - (double(shortRadius) * shortRadius) / (double(longRadius) * longRadius));

    double circumStart = longRadius * ellint_2(eccentricity, polarAngleStart);
    double circumEnd = 0;

    // This part is necessary because calling ellint_2 with 2pi returns a nan...
    if (polarAngleEnd == double_constants::two_pi)
        circumEnd = 4.0 * longRadius * ellint_2(eccentricity, double_constants::half_pi);

    else if (polarAngleEnd == -double_constants::two_pi)
        circumEnd = 4.0 * longRadius * ellint_2(eccentricity, -double_constants::half_pi);

    else
        circumEnd = longRadius * ellint_2(eccentricity, polarAngleEnd);

    // Arc length of the ellipse. This value is negative if angleExtent is negative.
    double circum = circumEnd - circumStart;

    int numMeasures = findNumMeasures(abs(circum));
    // Arc length separation between two consecutive measure handles.
    double circumStep = circum / numMeasures;
    double target = circumStart + circumStep / 2.0;

    double sinPhi = sin(phi);
    double cosPhi = cos(phi);

    /* To find where the centers of the measure handles should be, we first need to find their polar angles.
     * Since we know that they are equally spaced, we know the arc length from the starting point to the
     * center of the measure handle. Therefore, we can solve E(eccentricity, angle) = arc_length for angle. */
    double angleMin = std::min(polarAngleStart, polarAngleEnd);
    double angleMax = std::max(polarAngleStart, polarAngleEnd);
    double radiusRatio = double(longRadius) / shortRadius;

    metResults.clear();
    measures.clear();

    // Number of bits of precision for root finding
    constexpr unsigned int PRECISION = 16;

    for (int i = 0; i < numMeasures; i++) {
        auto result = tools::bisect(circumFunctor{target, longRadius, eccentricity}, angleMin, angleMax,
                                    tools::eps_tolerance<double>(PRECISION));

        double polarAngle = (result.first + result.second) / 2.0;
        // Convert to parametric angle
        double paraAngle = atan2(radiusRatio * sin(polarAngle), cos(polarAngle));

        // Calculate x, y coordinates based on the parametric angle
        double x = longRadius * cos(paraAngle);
        double y = shortRadius * sin(paraAngle);

        // Rotate x, y by phi
        rotate(x, y, sinPhi, cosPhi);

        // Adjust to center of ellipse
        double c = x + column;
        double r = -y + row;

        // Find the perpendicular angle to the tanget line to the ellipse at this point
        double perpAngle = double_constants::half_pi - atan2(shortRadius * cos(paraAngle), longRadius * sin(paraAngle));

        measures.emplace_back(measureParams.perpLength, measureParams.tangLength, int(round(r)), int(round(c)),
                              mapToPi(perpAngle + phi));
        target += circumStep;
    }

    initMeasuresFuzzy();
}

std::vector<int>
EllipseObject::getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                      double distThresh) {

    auto shape = fitEllipse5(fitPoints);

    // Return an empty vector if the ellipse is invalid
    if (shape.longRadius <= 0)
        return std::vector<int>{};

    return ellipseInliers(shape, testPoints, distThresh);
}

bool EllipseObject::addResult(const std::vector<cv::Point2d> &inliers, bool allowOutsideRegion) {
    EllipseParams shape = fitEllipse(inliers);

    if (!allowOutsideRegion and !inEllipse(*this, shape))
        return false;

    shape.phi += boost::math::double_constants::half_pi;  // Add pi/2 to account for coordinate change
    metResults.push_back(shape);

    return true;
}
