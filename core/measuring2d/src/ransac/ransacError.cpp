#include "ransacError.hpp"
#include "mathutils.hpp"

using namespace cv;

inline double square(double x) {
    return x * x;
}

std::vector<int> arcInliers(const CircleParams &shape, const RansacPoints &testPoints, double threshold) {
    // This vector stores the index of the inlier point per each measure handle. -1 if no inliers.
    std::vector<int> inliers{};
    double a = shape.center.x;
    double b = shape.center.y;
    double r = shape.radius;

    // Iterate across each measure
    for (int i = 0; i < testPoints.size(); i++) {
        const std::vector<Point2d> &pointsOnMeasure = testPoints[i];

        // Find the point with the smallest distance from the circle
        // -1 if not found
        double minDist = std::numeric_limits<double>::infinity();
        int minIndex = -1;

        for (int j = 0; j < pointsOnMeasure.size(); j++) {
            double x = pointsOnMeasure[j].x;
            double y = pointsOnMeasure[j].y;

			double dist = abs(euclideanDistance(a, x, b, y) - r);

            if (dist < minDist and dist < threshold) {
                minDist = dist;
                minIndex = j;
            }
        }
        
        inliers.push_back(minIndex);
    }

    return inliers;
}

/* Determine whether pt is inside the ellipse defined by center, a, b, sinPhi, cosPhi.
 * We take in sinPhi and cosPhi because we don't want to recalculate that for every point. */
bool insideEllipse(const Point2d &pt, const Point2d &center, double a, double b, double sinPhi, double cosPhi) {
    // Use the standard form of the ellipse equation
    double result = square((pt.x - center.x) * cosPhi + (pt.y - center.y) * sinPhi) / (a * a);
    result += square((pt.x - center.x) * sinPhi - (pt.y - center.y) * cosPhi) / (b * b);
    result -= 1;

    return result <= 0;
}

std::vector<int>
ellipseInliers(const EllipseParams &shape, const RansacPoints &testPoints, double threshold) {
    /* Since calculating the distance from a point to an ellipse is long and complicated, we don't actually calculate
     * the distance, we simply see if it's within the distance threshold */
    std::vector<int> inliers{};
    Point2d center = shape.center;
    double a = shape.longRadius;
    double b = shape.shortRadius;

    double sinPhi = sin(shape.phi);
    double cosPhi = cos(shape.phi);

    for (int i = 0; i < testPoints.size(); i++) {
        const std::vector<Point2d>& pointsOnMeasure = testPoints[i];
        int index = -1;

        for (int j = 0; j < pointsOnMeasure.size(); j++) {
            bool insideOuterEllipse = insideEllipse(pointsOnMeasure[j], center, a + threshold, b + threshold, sinPhi, cosPhi);
            bool insideInnerEllipse = insideEllipse(pointsOnMeasure[j], center, a - threshold, b - threshold, sinPhi, cosPhi);

            if (insideOuterEllipse and !insideInnerEllipse) {
                index = j;
                break;
            }
        }

        inliers.push_back(index);
    }

    return inliers;
}

std::vector<int> lineInliers(const LineEqParams &shape, const RansacPoints &testPoints, double threshold) {
    std::vector<int> inliers{};
    double a = shape.a;
    double b = shape.b;
    double c = shape.c;

    for (int i = 0; i < testPoints.size(); i++) {
		const std::vector<Point2d> &pointsOnMeasure = testPoints[i];
        double minDist = std::numeric_limits<double>::infinity();
        int minIndex = -1;

        for (int j = 0; j < pointsOnMeasure.size(); j++) {
            double x = pointsOnMeasure[j].x;
            double y = pointsOnMeasure[j].y;

            double dist = abs((a * x) + (b * y) + c);

            if (dist < minDist and dist < threshold) {
                minDist = dist;
                minIndex = j;
            }
        }

        inliers.push_back(minIndex);
    }

    return inliers;
}
