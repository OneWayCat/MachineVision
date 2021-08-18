#include "inRegion.hpp"
#include "mutils.hpp"

constexpr int NUM_SIDES = 4;  // Number of sides in a rectangle

using namespace cv;

inline double square(double x) {
    return x * x;
}

// Adjust the vector of the corners of the MeasureRectangle for the center.
void adjustForCenter(std::vector<Point2d> &rectPoints, const Point2d &center) {
    for (Point2d &pt : rectPoints)
        pt += center;
}

// Get the profile line of a measure rectangle: (start, end).
std::pair<Point2d, Point2d> getProfileLine(const MeasureRectangle &handle) {
    std::vector<Point2d> rectPoints = getRectPoints(handle);
    adjustForCenter(rectPoints, Point2d(handle.getRow(), handle.getColumn()));

    Point2d profileStart = (rectPoints[0] + rectPoints[1]) / 2;
    Point2d profileEnd = (rectPoints[2] + rectPoints[3]) / 2;

    return std::make_pair(profileStart, profileEnd);
}

bool inArc(const ArcObject &arcObj, const CircleParams &arcShape) {
    double r = arcShape.center.x;
    double c = arcShape.center.y;
    double radiusSq = square(arcShape.radius);

    auto circleEq = [=](double x, double y) {
        return square(x - r) + square(y - c) - radiusSq;
    };

    for (const MeasureRectangle &handle : arcObj.getMeasures()) {
        auto profileLine = getProfileLine(handle);
        Point2d &profileStart = profileLine.first;
        Point2d &profileEnd = profileLine.second;

        double v1 = circleEq(profileStart.x, profileStart.y);
        double v2 = circleEq(profileEnd.x, profileEnd.y);

        if (v1 > 0 == v2 > 0)
            return false;
    }

    return true;
}

bool inArc(const ArcProjectionObject &arcObj, const CircleParams &arcShape) {
    double r = arcShape.center.x;
    double c = arcShape.center.y;
    double radiusSq = square(arcShape.radius);

    auto circleEq = [=](double x, double y) {
        return square(x - r) + square(y - c) - radiusSq;
    };

    for (const MeasureRectangle &handle : arcObj.getMeasures()) {
        auto profileLine = getProfileLine(handle);
        Point2d &profileStart = profileLine.first;
        Point2d &profileEnd = profileLine.second;

        double v1 = circleEq(profileStart.x, profileStart.y);
        double v2 = circleEq(profileEnd.x, profileEnd.y);

        if (v1 > 0 == v2 > 0)
            return false;
    }

    return true;
}

bool inEllipse(const EllipseObject &ellipseObj, const EllipseParams &ellipseShape) {
    double r = ellipseShape.center.x;
    double c = ellipseShape.center.y;

    double aSq = square(ellipseShape.longRadius);
    double bSq = square(ellipseShape.shortRadius);
    double phi = ellipseShape.phi;

    double cosPhi = cos(phi);
    double sinPhi = sin(phi);

    auto ellipseEq = [=](double x, double y) {
        return square((x - r) * cosPhi + (y - c) * sinPhi) / aSq +
               square((x - r) * sinPhi - (y - c) * cosPhi) / bSq - 1;
    };

    for (const MeasureRectangle &handle : ellipseObj.getMeasures()) {
        auto profileLine = getProfileLine(handle);
        Point2d &profileStart = profileLine.first;
        Point2d &profileEnd = profileLine.second;

        double v1 = ellipseEq(profileStart.x, profileStart.y);
        double v2 = ellipseEq(profileEnd.x, profileEnd.y);

        if (v1 > 0 == v2 > 0)
            return false;
    }

    return true;
}

// Source for line intersection: https://algorithmtutor.com/Computational-Geometry/Check-if-two-line-segment-intersect/
double direction(const Point2d &a, const Point2d &b, const Point2d &c) {
    // Calculate the cross product of a->b and b->c
    Point2d ab = b - a;
    Point2d bc = c - b;

    return ab.cross(bc);
}

bool left(const Point2d &a, const Point2d &b, const Point2d &c) {
    // Determine if c is on the left of the vector a->b
    return direction(a, b, c) > 0;
}

bool collinear(const Point2d &a, const Point2d &b, const Point2d &c) {
    // Determine if a, b, c are collinear
    return direction(a, b, c) == 0;
}

bool intersectProp(const Point2d &a, const Point2d &b, const Point2d &c, const Point2d &d) {
    // Determine if the line segment ab intersects properly the line (not line segment!) cd.
    return left(c, d, a) != left(c, d, b);
}

bool intersect(const Point2d &a, const Point2d &b, const Point2d &c, const Point2d &d) {
    // Determine if the line segment ab crosses or touches the line (not line segment!) cd.
    if (intersectProp(a, b, c, d))
        return true;

    return collinear(c, d, a) or collinear(c, d, b);
}

bool inLine(const LineObject &lineObj, const LineParams &lineShape) {
    Point2d fStart = lineShape.startPoint;
    Point2d fEnd = lineShape.endPoint;

    for (const MeasureRectangle &handle : lineObj.getMeasures()) {
        auto profileLine = getProfileLine(handle);
        Point2d &pStart = profileLine.first;
        Point2d &pEnd = profileLine.second;

        // pStart -> pEnd is the profile line, fStart -> fEnd is the fitted line
        // Determine if the line segment pStart<->pEnd intersects or touches the line fStart<->fEnd
        if (!intersect(pStart, pEnd, fStart, fEnd))
            return false;
    }

    return true;
}

bool intersectRect(const Point2d &a, const Point2d &b, const std::vector<Point2d> &rectPoints) {
    // Determine if the line segment ab crosses or touches the rectangle determined by rectPoints.
    for (int i = 0; i < NUM_SIDES - 1; i++)
        if (intersect(a, b, rectPoints[i], rectPoints[i + 1]))
            return true;

    return intersect(a, b, rectPoints.back(), rectPoints.front());
}

bool inRect(const RectObject &rectObj, const RectParams &rectShape) {
    for (const MeasureRectangle &handle : rectObj.getMeasures()) {
        auto profileLine = getProfileLine(handle);
        Point2d &pStart = profileLine.first;
        Point2d &pEnd = profileLine.second;

        if (!intersectRect(pStart, pEnd, rectShape.rectPoints))
            return false;
    }

    return true;
}