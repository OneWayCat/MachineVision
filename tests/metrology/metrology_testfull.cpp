#include "arrutils.hpp"
#include "drawMetrology.hpp"
#include "drawSubpixel.hpp"
#include "mathutils.hpp"
#include "metcout.hpp"
#include "metrologyObject.hpp"

using namespace boost::math;
using namespace cv;
using namespace std::chrono;

std::vector<int> circleRows;
std::vector<int> circleCols;
std::vector<int> circleRadii;
std::vector<double> circleAngleStart;
std::vector<double> circleAngleEnd;
MeasureParams circleParams;

void circleInitialization() {
    // Initialize circle rows
    circleRows = arange<int>(52, 500, 89);
    circleRows.insert(circleRows.end(), circleRows.begin(), circleRows.end());

    // Initialize circle column
    circleCols = std::vector<int>(6, 348);
    std::vector<int> circleRightCols = std::vector<int>(6, 438);
    circleCols.insert(circleCols.end(), circleRightCols.begin(), circleRightCols.end());

    // Initialize circle radii
    circleRadii = std::vector<int>(12, 23);

    // Initialize circle start and end angles.
    // For loops are used in case you want to hot-tweak a specific measure.
    circleAngleStart = std::vector<double>(12);
    for (double &i : circleAngleStart) {
        i = 0;
    }

    circleAngleEnd = std::vector<double>(12);
    for (double &i : circleAngleEnd) {
        i = double_constants::two_pi;
    }

    // Initialize circle MeasureParams
    circleParams.perpLength = 12;
    circleParams.tangLength = 10;
    circleParams.threshold = 2;
    circleParams.transition = TransitionType::POSITIVE;

    circleParams.numMeasures = 10;
}

std::vector<int> rectRows;
std::vector<int> rectCols;
std::vector<int> rectPhis;
std::vector<int> rectHalfWidths;
std::vector<int> rectHalfHeights;
MeasureParams rectParams;

void rectInitialization() {
    rectRows = {410, 410};
    rectCols = {215, 562};
    rectPhis = {0, 0};
    rectHalfWidths = {85, 85};
    rectHalfHeights = {88, 88};

    rectParams.perpLength = 10;
    rectParams.tangLength = 5;
    rectParams.threshold = 1;
    rectParams.transition = TransitionType::POSITIVE;
}

int main(int argc, char *argv[]) {
    circleInitialization();
    rectInitialization();

    Mat image;
    Mat copy;
    Mat gray;
    //image = imread(argv[1]);
    image = imread("D:/Programming/Workspaces/Zebra/H-Rev-C/images/pads.png");
    image.copyTo(copy);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    std::vector<RectObject> rectObjects;
    std::vector<ArcObject> arcObjects;

    for (int i = 0; i < rectRows.size(); i++) {
        rectObjects.emplace_back(rectParams, rectRows[i], rectCols[i], rectPhis[i], rectHalfHeights[i],
                                 rectHalfWidths[i]);
    }

    for (int i = 0; i < circleRows.size(); i++) {
        arcObjects.emplace_back(circleParams, circleRows[i], circleCols[i], circleRadii[i]);
    }

    Mat subpixelCopy;
    copy.copyTo(subpixelCopy);

    for (RectObject &rect : rectObjects) {
        int row = rect.getRow();
        int col = rect.getColumn();
        int extent = std::max({rect.getHalfHeight(), rect.getHalfWidth()});
        const std::vector<Point2d> &corners = rect.getRectPoints();

        SubpixelWorkspace subpixelWS = SubpixelWorkspace(subpixelCopy, row, col, 4, extent + 10);
        Mat zoomed = subpixelWS.getZoomedImage();

        auto startTime = high_resolution_clock::now();

        rect.applyMetrologyModel(gray, 0.95, 1, 3, -1, 0, false);

        auto stopTime = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stopTime - startTime);
        std::cout << duration.count() << " microseconds" << std::endl;

        drawRectObject(copy, rect);

        for (const RectParams &shape : rect.getMetrologyResults()) {
            subpixelWS.drawRect(shape.rectPoints, COLOR_GREEN);

            for (const MeasurePosResult &result : rect.getMeasureResults()) {

                for (const Point2d &pt : result.pos) 
					subpixelWS.drawPoint(pt.x, pt.y, COLOR_RED); 
            }
        }

        imshow("zoomed", zoomed);
        imshow("image", copy);
        //waitKey(0);
    }
    
    for (ArcObject &arc : arcObjects) {
        int row = arc.getRow();
        int col = arc.getColumn();

        SubpixelWorkspace subpixelWS = SubpixelWorkspace(subpixelCopy, row, col, 10, circleRadii[0] + 20);
        const Mat &zoomed = subpixelWS.getZoomedImage();

        auto startTime = high_resolution_clock::now();

        arc.applyMetrologyModel(gray, 0.95, 1, 3, -1, 0, false);

        auto stopTime = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(stopTime - startTime);
        std::cout << duration.count() << " microseconds" << std::endl;

        drawArcObject(copy, arc);

        for (const CircleParams &shape : arc.getMetrologyResults()) {
            subpixelWS.drawCircle(shape.center.x, shape.center.y, shape.radius, COLOR_GREEN);

            for (const MeasurePosResult& result : arc.getMeasureResults()) {

                for (const Point2d &pt : result.pos)
                    subpixelWS.drawPoint(pt.x, pt.y, COLOR_RED);
            }
        }

        cv::imshow("zoomed", zoomed);
        cv::imshow("image", copy);
        //cv::waitKey(0);
    }
}
