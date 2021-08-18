#include "meniscus.hpp"

using namespace cv;

// Fit a parabola to 3 or more points and get the y coordinate of the vertex
Point2d getVertex(const std::vector<Point2d> &fitPoints) {
    const int n = (int)fitPoints.size();

    // Coefficient matrix
    Mat_<double> A(n, 3);

    // Y values
    Mat_<double> rhs(n, 1);

    // Result
    Mat_<double> coeffs(3, 1);

    for (int i = 0; i < n; i++) {
        double *rowPtr = A.ptr<double>(i);
        const Point2d &pt = fitPoints[i];

        rowPtr[0] = pt.x * pt.x;
        rowPtr[1] = pt.x;
        rowPtr[2] = 1;

        rhs(i, 0) = pt.y;
    }

    solve(A, rhs, coeffs, DECOMP_NORMAL);

    const double a = coeffs(0, 0);
    const double b = coeffs(1, 0);
    const double c = coeffs(2, 0);

    // Get the vertex
    const double vX = -b / (2 * a);
    const double vY = a * vX * vX + b * vX + c;
    return {vX, vY};
}

// Boundary checks for inputs to findMeniscus function.
void validateArgs(const Mat &img, const Point2d &topLeft, const Point2d &bottomRight) {
    // Points given in (row, column)
    if (topLeft.x < 0 or topLeft.x >= img.rows or
        topLeft.y < 0 or topLeft.y >= img.cols)
        throw std::invalid_argument("topLeft is out of bounds");

    if (bottomRight.x < 0 or bottomRight.x >= img.rows or
        bottomRight.y < 0 or bottomRight.y >= img.cols)
        throw std::invalid_argument("topLeft is out of bounds");

    if (bottomRight.x <= topLeft.x or bottomRight.y <= topLeft.y)
        throw std::invalid_argument("bottomRight must be to the bottom and right of topLeft");
}

Point2d findMeniscus(const Mat &img, const Point2d &topLeft, const Point2d &bottomRight, MeniscusParams meniscusParams) {
    validateArgs(img, topLeft, bottomRight);

    // The half width parameter for each measure handle.
    const int measureHalfWidth = double(bottomRight.x - topLeft.x) / 2;
    const int centerRow = (topLeft.x + bottomRight.x) / 2;
    double centerCol = topLeft.y;
    const double colStep = double(bottomRight.y - topLeft.y) / meniscusParams.numMeasures;

    // Extracting variables from struct to make rest of the code look cleaner.
    MeniscusShape shape = meniscusParams.meniscusShape;
    unsigned int numMeasures = meniscusParams.numMeasures;
    int halfHeight = meniscusParams.measureHalfHeight;
    double phi = meniscusParams.measurePhi;
    double sigma = meniscusParams.measureSigma;
    double threshold = meniscusParams.edgeThresh;
    double trimProportion = meniscusParams.trimProportion;

    // If meniscus is expected to be concave, choose the bottommost edge. Otherwise, choose the topmost edge.
    const SelectType measureSelect = shape == MeniscusShape::CONCAVE ? SelectType::FIRST : SelectType::LAST;

    std::vector<Point2d> fitPoints;

    for (unsigned int i = 0; i < numMeasures; i++) {
        const MeasureRectangle rect(measureHalfWidth, halfHeight, centerRow, int(round(centerCol)), phi);
        const MeasurePosResult result = measurePos(img, rect, sigma, threshold, TransitionType::ALL, measureSelect);

        const std::vector<Point2d> &resultPos = result.pos;

        if (!resultPos.empty()) {
            // (col, row) format for parabola fitting
            const Point2d &edge = resultPos.front();
            fitPoints.emplace_back(edge.y, edge.x);
        }

        centerCol = centerCol + colStep;
    }

    // Trim some points from the edges of the fitPoints vector
    const int pointsToTrim = int(fitPoints.size() * trimProportion);
    std::vector<Point2d> trimmedFitPoints;
    std::copy(fitPoints.begin() + pointsToTrim, fitPoints.end() - pointsToTrim, std::back_inserter(trimmedFitPoints));

    const Point2d vertex = getVertex(trimmedFitPoints);

    // Return vertex in (row, col) format
    return {vertex.y, vertex.x};
}