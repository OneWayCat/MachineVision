#include "arrutils.hpp"
#include "mathutils.hpp"
#include "inRegion.hpp"
#include "ransacError.hpp"

constexpr int MIN_NUM_MEASURES = 2;

using namespace cv;
using namespace boost::math;

LineObject::LineObject(const MeasureParams &measureParams, int rowStart, int colStart, int rowEnd, int colEnd)
        : MetrologyObject{measureParams, MIN_NUM_MEASURES},
          rowStart{rowStart},
          columnStart{colStart},
          rowEnd{rowEnd},
          columnEnd{colEnd} {
}

int LineObject::findNumMeasures() {
    int numMeasures = measureParams.numMeasures;

    if (numMeasures <= 0) {
        double x1 = columnStart;
        double y1 = rowStart;
        double x2 = columnEnd;
        double y2 = rowEnd;
        double lineLength = euclideanDistance(x1, x2, y1, y2);

        numMeasures = int(lineLength / measureParams.separation);
    }

    return std::max(numMeasures, MIN_NUM_MEASURES);
}

void LineObject::initMeasures() {
    validateMeasureParams();

    int numMeasures = findNumMeasures();

    // Calculate angle of the line
    int rowDiff = rowEnd - rowStart;
    int colDiff = columnEnd - columnStart;
    double angle = atan2(double(colDiff), double(rowDiff));

    // Offset the first and last measure handles from the extents of the line
    // by their tangential length
    double offsetC = measureParams.tangLength * sin(angle);
    double offsetR = measureParams.tangLength * cos(angle);

    std::vector<double> rowSpacing = linspace(rowStart + offsetR, rowEnd - offsetR, numMeasures);
    std::vector<double> colSpacing = linspace(columnStart + offsetC, columnEnd - offsetC, numMeasures);

    auto rowIter = rowSpacing.begin();
    auto colIter = colSpacing.begin();

    // Set angle of measure handles to be 90 degree offset from angle of line
    double correctedAngle = mapToPi(angle - double_constants::pi);

    // Initialize MeasureHandles
    metResults.clear();
    measures.clear();

    while (rowIter < rowSpacing.end() && colIter < colSpacing.end()) {
        measures.emplace_back(measureParams.perpLength, measureParams.tangLength,
                              int(*rowIter), int(*colIter), correctedAngle);

        std::advance(rowIter, 1);
        std::advance(colIter, 1);
    }

    initMeasuresFuzzy();
}

std::vector<int>
LineObject::getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                       double distThresh) {

    auto shape = fitLine2(fitPoints);
    return lineInliers(shape, testPoints, distThresh);
}

bool LineObject::addResult(const std::vector<Point2d> &inliers, bool allowOutsideRegion) {
    LineEqParams lineEq = fitLine(inliers);
    const Point2d &firstPoint = inliers.front();
    const Point2d &lastPoint = inliers.back();

    double rStart, rEnd, cStart, cEnd;
    double a = lineEq.a;
    double b = lineEq.b;
    double c = lineEq.c;

    // Now we have to convert this to (start point) -> (end point)
    if (abs(a) <= abs(b)) {
        // Use the row coordinates of the first and the last point
        rStart = firstPoint.x;
        rEnd = lastPoint.x;

        if (b == 0)
            cStart = cEnd = -a * c;
        else {
            cStart = -(a * rStart + c) / b;
            cEnd = -(a * rEnd + c) / b;
        }
    } else {
        // Use the column coordinates of the first and the last point
        cStart = firstPoint.y;
        cEnd = lastPoint.y;

        if (a == 0)
            rStart = rEnd = -b * c;
        else {
            rStart = -(b * cStart + c) / a;
            rEnd = -(b * cEnd + c) / a;
        }
    }

    LineParams shape{Point2d{rStart, cStart}, Point2d{rEnd, cEnd}};

    if (!allowOutsideRegion and !inLine(*this, shape))
        return false;

    metResults.push_back(shape);
    return true;
}