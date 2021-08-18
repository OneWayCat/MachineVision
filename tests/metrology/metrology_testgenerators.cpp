#include "drawingDefs.hpp"
#include "metrologyGen.hpp"
#include "metrologyObject.hpp"

#define DARK_GRAY Scalar(100, 100, 100)
#define LIGHT_GRAY Scalar(125, 125, 125)

using namespace boost::math;
using namespace cv;

int main(int argc, char *argv[]) {
    MeasureParams measureParams;
    measureParams.perpLength = 40;
    measureParams.threshold = 10;
    measureParams.transition = TransitionType::NEGATIVE;
    std::vector<Scalar> colors = {LIGHT_GRAY};

    ArcGeneratorParams arcGenParams(5, std::vector<double>{double_constants::half_pi, double_constants::half_pi},
                                    std::vector<double>{1.5 * double_constants::pi, 1.5 * double_constants::pi});
    arcGenParams.numInstances = 4;
    arcGenParams.instanceSeparation = 10;

    ArcObject arcObj{measureParams, 350, 350, 200, 3 * double_constants::half_pi, -double_constants::pi};
    ArcGenerator arcGen{arcObj, arcGenParams};

    while (true) {
        arcGen.spoofImage(Size(700, 700), DARK_GRAY, colors);
        Mat spoofed = arcGen.getSpoofedImage();
        Mat gray;
        cvtColor(spoofed, gray, COLOR_BGR2GRAY);

        arcObj.applyMetrologyModel(gray, 0.95, 4, 3.5, -1, 0, false);

        for (const CircleParams &shape : arcObj.getMetrologyResults()) {
            circle(spoofed, reverse(shape.center), int(shape.radius), COLOR_GREEN);

            for (const MeasurePosResult &result : arcObj.getMeasureResults()) {
                for (const Point2d &pt : result.pos) {
                    circle(spoofed, reverse(pt), 1, COLOR_RED, -1);
                }
            }
        }

        imshow("gray", spoofed);
        auto keyPressed = waitKey(0);

        if (keyPressed == int('q')) {
            destroyAllWindows();
            break;
        }
    }

    EllipseGeneratorParams ellipseGenParams(0, 0, std::vector<double>{0, 0},
                                            std::vector<double>{1.5 * double_constants::pi,
                                                                1.5 * double_constants::pi});
    ellipseGenParams.numInstances = 4;
    ellipseGenParams.instanceSeparation = 10;

    EllipseObject ellipseObj{measureParams, 350, 350, 300, 150, double_constants::sixth_pi, 1.5 * double_constants::pi,
                             -1.5 * double_constants::pi};
    EllipseGenerator ellipseGen{ellipseObj, ellipseGenParams};

    while (true) {
        ellipseGen.spoofImage(Size(700, 700), DARK_GRAY, colors);
        Mat spoofed = ellipseGen.getSpoofedImage();
        Mat gray;
        cvtColor(spoofed, gray, COLOR_BGR2GRAY);

        ellipseObj.applyMetrologyModel(gray, 0.95, 2, 3.5, -1, 0, false);

        for (const EllipseParams &shape : ellipseObj.getMetrologyResults()) {
            int row = (int)shape.center.x;
            int col = (int)shape.center.y;
            int lr = (int)shape.longRadius;
            int sr = (int)shape.shortRadius;

            ellipse(spoofed, Point2i(col, row), Size(lr, sr), -rad2deg(shape.phi), 0, 360, COLOR_GREEN);

            for (const MeasurePosResult &result : ellipseObj.getMeasureResults()) {
                for (const Point2d &pt : result.pos) {
                    circle(spoofed, reverse(pt), 1, COLOR_RED, -1);
                }
            }
        }

        imshow("gray", spoofed);
        auto keyPressed = waitKey(0);

        if (keyPressed == int('q')) {
            destroyAllWindows();
            break;
        }
    }

    // SAUSAGE! :D
    LineGeneratorParams lineGenParams;
    lineGenParams.numInstances = 3;
    lineGenParams.instanceSeparation = 20;

    LineObject lineObj{measureParams, 200, 200, 200, 300};
    LineGenerator lineGen{lineObj, lineGenParams, 5};

    while (true) {
        lineGen.spoofImage(Size(700, 700), DARK_GRAY, colors);
        Mat spoofed = lineGen.getSpoofedImage();
        Mat gray;
        cvtColor(spoofed, gray, COLOR_BGR2GRAY);

        lineObj.applyMetrologyModel(gray, 0.8, 3, 5, 1000, 0, false);

        for (const LineParams &shape : lineObj.getMetrologyResults()) {
            Point2d start = shape.startPoint;
            Point2d end = shape.endPoint;

            line(spoofed, reverse(start), reverse(end), COLOR_GREEN);

            for (const MeasurePosResult &result : lineObj.getMeasureResults()) {
                for (const Point2d &pt : result.pos) {
                    circle(spoofed, reverse(pt), 1, COLOR_RED, -1);
                }
            }
        }

        imshow("spoofed", spoofed);
        auto keyPressed = waitKey(0);

        if (keyPressed == int('q')) {
            destroyAllWindows();
            break;
        }
    }

    RectGeneratorParams rectGenParams;
    rectGenParams.numInstances = 4;
    rectGenParams.instanceSeparation = 10;

    RectObject rectObj{measureParams, 350, 350, double_constants::sixth_pi, 150, 150};

    RectGenerator rectGen = {rectObj, rectGenParams};

    while (true) {
        rectGen.spoofImage(Size(700, 700), DARK_GRAY, colors);
        Mat spoofed = rectGen.getSpoofedImage();
        Mat gray;
        cvtColor(spoofed, gray, COLOR_BGR2GRAY);

        rectObj.applyMetrologyModel(gray, 0.5, 2, 2, -1, 0, true);

        for (const RectParams &shape : rectObj.getMetrologyResults()) {
            for (size_t i = 0; i < 3; i++)
                line(spoofed, reverse(shape.rectPoints[i]), reverse(shape.rectPoints[i + 1]), COLOR_GREEN);

            line(spoofed, reverse(shape.rectPoints[3]), reverse(shape.rectPoints[0]), COLOR_GREEN);

            for (const MeasurePosResult &result : rectObj.getMeasureResults()) {
                for (const Point2d &pt : result.pos) {
                    circle(spoofed, reverse(pt), 1, COLOR_RED, -1);
                }
            }
        }

        imshow("spoofed", spoofed);
        auto keyPressed = waitKey(0);

        if (keyPressed == int('q')) {
            destroyAllWindows();
            break;
        }
    }
}
