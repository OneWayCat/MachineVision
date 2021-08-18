#include "inRegion.hpp"
#include "mathutils.hpp"
#include "metrologyObject.hpp"
#include "mutils.hpp"
#include "ransacError.hpp"

constexpr int MIN_NUM_MEASURES = 8;

using namespace cv;

RectObject::RectObject(const MeasureParams &measureParams, int row, int col, double phi, int halfWidth, int halfHeight)
        : MetrologyObject{measureParams, MIN_NUM_MEASURES},
          row{row},
          column{col},
          phi{mapToPi(phi)},
          halfWidth{halfWidth},
          halfHeight{halfHeight},
          rectPoints{4} {

    if (measureParams.perpLength >= halfWidth)
        throw std::invalid_argument("MeasureHandle perpendicular length must be less than the RectObject's half width");

    if (measureParams.perpLength >= halfHeight)
        throw std::invalid_argument(
                "MeasureHandle perpendicular length must be less than the RectObject's half height");

    rectPoints = ::getRectPoints(MeasureRectangle{halfWidth, halfHeight, row, col, phi});
    Point2d center{double(row), double(col)};

    // Adjust the points to the center of the rectangle
    for (auto &point : rectPoints)
        point += center;

    // UNCOMMENT FOR DEBUGGING
    // initMeasures();
}

Point2i round(const Point2d& pt) {
    return Point2i{ int(round(pt.x)), int(round(pt.y)) };
}

void RectObject::initMeasures() {
    validateMeasureParams();

    metResults.clear();
    measures.clear();

    for (size_t i = 0; i < rectPoints.size() - 1; i++) {
        Point2i current = round(rectPoints[i]);
        Point2i next = round(rectPoints[i + 1]);

        LineObject temp = LineObject(measureParams, current.x, current.y, next.x, next.y);
        temp.initMeasures();

        const std::vector<MeasureRectangle> &handles = temp.getMeasures();
        measures.insert(std::end(measures), std::begin(handles), std::end(handles));
    }

    Point2i bottomLeft = round(rectPoints[3]);
    Point2i bottomRight = round(rectPoints[0]);

    LineObject temp = LineObject(measureParams, bottomLeft.x, bottomLeft.y, bottomRight.x, bottomRight.y);
    temp.initMeasures();

    const std::vector<MeasureRectangle> &handles = temp.getMeasures();
    measures.insert(std::end(measures), std::begin(handles), std::end(handles));

    initMeasuresFuzzy();
}

// Fit a line through the fitting points of one side, and return the inliers.
std::vector<int>
RectObject::getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                       double distThresh) {

    auto shape = fitLine(fitPoints);
    return lineInliers(shape, testPoints, distThresh);
}

bool RectObject::addResult(const std::vector<cv::Point2d> &inliers, const std::vector<size_t> &inliersPerSide,
                           bool allowOutsideRegion) {
    RectParams shape = fitRect(inliers, inliersPerSide);

    if (!allowOutsideRegion and !inRect(*this, shape))
        return false;

    metResults.push_back(shape);
    return true;
}