#include "RectElement.hpp"
#include "mathutils.hpp"

using namespace cv;

/**
 * Determines if the point test is to the left or on the line segment
 * determined by (a -> b) by calculating cross product.
 *
 * @param a Starting point of the line.
 * @param b Ending point of the line.
 * @param test Point to test.
 * @return Whether test is to the left or on the line (a -> b).
 */
bool leftOn(const Point2d &a, const Point2d &b, const Point2d &test) {
    double area = (b.x - a.x) * (test.y - a.y) - (test.x - a.x) * (b.y - a.y);
    return area >= 0;
}

/**
 * Determines if the point test is inside the rectangle defined by the four points (a, b, c, d).
 * The four points must be provided in counterclockwise order.
 *
 * @param a
 * @param b
 * @param c
 * @param d
 * @param test Point to test.
 * @return Whether test is inside the rectangle (a, b, c, d).
 */
bool insideRect(const Point2d &a, const Point2d &b, const Point2d &c, const Point2d &d, const Point2d &test) {
    return leftOn(a, b, test) && leftOn(b, c, test) && leftOn(c, d, test) && leftOn(d, a, test);
}

/**
 * Projects a points to the line defined by the 2 points (projL -> projR), and finds the distance
 * from projL to the projected point.
 *
 * @param projL
 * @param projR
 * @param toProject Point to project to the line.
 * @return The distance from the projected point to projL.
 */
double findBinNumber(const Point2d &projL, const Point2d &projR, const Point2d &toProject) {
    // Project the point p to the line defined by the projection line (projL -> projR)
    double projectedX, projectedY;

    // If line is vertical (to avoid dividing by 0)
    if (projL.x == projR.x) {
        projectedX = projL.x;
        projectedY = toProject.y;
    } else {
        // Calculate the slope and intercept of the line defined by (projL -> projR)
        double m = (projR.y - projL.y) / (projR.x - projL.x);
        double b = projL.y - m * projL.x;

        double mm1 = m * m + 1;

        // Find the projected point onto the line
        projectedX = (m * toProject.y + toProject.x - m * b) / mm1;
        projectedY = (m * m * toProject.y + m * toProject.x + b) / mm1;
    }

    // Find the bin number by calculating the distance from projL
    return euclideanDistance(projectedX, projL.x, projectedY, projL.y);
}

/* Comparison functions to use with std::min_element to calculate the minimum row and column coordinate
   of the bounding box of the measure rectangle */
bool compareRow(const Point2d &a, const Point2d &b) { return a.x < b.x; }

bool compareCol(const Point2d &a, const Point2d &b) { return a.y < b.y; }

double RectangleElement::findDistance(double firstCoord, double secondCoord) {
    return secondCoord - firstCoord;
}

EdgeResults RectangleElement::findEdgePos(const std::vector<double> &coords, SelectType select) {
    // No edges detected
    if (coords.empty())
        return EdgeResults{};

    // Find the appropriate number of edges to return
    size_t numEdgesDetected = coords.size();
    size_t numEdges = select == SelectType::ALL ? numEdgesDetected : 1;

    std::vector<Point2d> posEdges(numEdges);
    std::vector<double> distances(numEdges - 1);

    // Add pi/2 to angle because phi is measured from horizontal axis and with the (r, c) coordinate system
    // the horizontal axis is 90 degrees further clockwise
    double angle = phi + (CV_PI / 2);
    double sinTheta = sin(angle);
    double cosTheta = cos(angle);

    // Calculate where the 0th coordinate (with respect to profile line) is
    double rowPos = -halfWidth * cosTheta;
    double colPos = -halfWidth * sinTheta;

    // Adjust for center of measure handle
    rowPos += row;
    colPos += column;

    // Calculate edgePositions based on coords
    if (select != SelectType::LAST) {
        posEdges.front().x = colPos + coords.front() * sinTheta;
        posEdges.front().y = rowPos + coords.front() * cosTheta;
    }

    if (select != SelectType::FIRST) {
        posEdges.back().x = colPos + coords.back() * sinTheta;
        posEdges.back().y = rowPos + coords.back() * cosTheta;
    }

    if (select == SelectType::ALL) {
        for (size_t i = 1; i < numEdgesDetected; i++) {
            posEdges[i].x = colPos + coords[i] * sinTheta;
            posEdges[i].y = rowPos + coords[i] * cosTheta;
            distances[i - 1] = coords[i] - coords[i - 1];
        }
    }

    return EdgeResults{posEdges, distances};
}

// Constructor
RectangleElement::RectangleElement(int halfWidth_, int halfHeight_, int row, int column, double phi_, bool angleInDegrees) : EdgeElement(row, column) {
    // Check arguments
    if (halfWidth_ < 1 || halfHeight_ < 0)
        throw std::invalid_argument("Width or height value must be positive");

    if (angleInDegrees) {
        if (abs(phi_) > 180)
            throw std::invalid_argument("phi must be between -180 and 180");
    } else {
        if (abs(phi_) > CV_PI)
            throw std::invalid_argument("phi must be between -pi and pi");
    }

    halfWidth = halfWidth_;
    halfHeight = halfHeight_;
    phi = phi_;

    if (angleInDegrees)
        phi = deg2rad(phi_);

    // Find the bin numbers for projection
    profileLength = double(halfWidth) * 2;

    // Get the four points of the rectangle as (r, c) coordinates
    std::vector<Point2d> rectPoints = getCornerPoints();

    Point2d rect0 = rectPoints[0];
    Point2d rect1 = rectPoints[1];
    Point2d rect2 = rectPoints[2];
    Point2d rect3 = rectPoints[3];

    // Find the lower and upper limits of the bounding box
    minC = (int) (*std::min_element(rectPoints.begin(), rectPoints.end(), compareCol)).y;
    minR = (int) (*std::min_element(rectPoints.begin(), rectPoints.end(), compareRow)).x;
    maxC = -minC + 1;
    maxR = -minR + 1;

    // Coordinates of the endpoints of the line to project to
    Point2d projR = (rect0 + rect1) / 2.0;
    Point2d projL = -projR;

    // Make a 2D array of bin numbers for each (r, c) pair
    size_t numRows = size_t(maxR) - minR + 1;
    size_t numCols = size_t(maxC) - minC + 1;

    binNumbers = std::vector<int>(numRows * numCols);
    int numberOfBins = 2 * halfWidth + 1;
    binCounts = std::vector<int>(numberOfBins);

    double cosPhi = cos(phi);

    // Main loop to iterate over each (r, c) pair inside the bounding box
    for (size_t r = 0; r < numRows; r++) {
        double binNumber = -1;  // The bin that the projected point belongs to

        for (size_t c = 0; c < numCols; c++) {
            Point2d toProject = Point2d(double(r) + minR, double(c) + minC);

            if (!insideRect(rect0, rect1, rect2, rect3, toProject))
                binNumbers[r * numCols + c] = -1;

            else {
                // For the first point in the row, we need to directly find the projected point
                if (binNumber < 0)
                    binNumber = findBinNumber(projL, projR, toProject);

                else  // After that, we just need to keep adding cos(phi)
                    binNumber += cosPhi;

                // Add the pixel of the image to the correct bin
                int intBinNumber = (int) round(binNumber);
                binNumbers[r * numCols + c] = intBinNumber;
                binCounts[intBinNumber]++;
            }
        }
    }
}