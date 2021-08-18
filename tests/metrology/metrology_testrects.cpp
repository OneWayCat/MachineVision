#include "mathutils.hpp"
#include "metcout.hpp"
#include "metrologyObject.hpp"
#include "drawMetrology.hpp"

using namespace boost::math;
using namespace cv;

int main(int argc, char *argv[]) {
    Mat image;
    Mat gray;

    image = imread(argv[1]);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    srand((unsigned int) time(0));

    while (true) {
        Mat copy;
        image.copyTo(copy);

        MeasureParams params{};
        params.numMeasures = 2 + rand() % 10;
        params.perpLength = 10;
        params.tangLength = 5;

        int randRow = 250 + (rand() % 250);
        int randCol = 250 + (rand() % 250);
        int randPhi = rand() % 90;
        int hh = rand() % 150 + 15;
        int hw = rand() % 150 + 15;

        RectObject obj{params, randRow, randCol, deg2rad(randPhi), hh, hw};
        drawRectObject(copy, obj, Scalar(0, 255, 0), true);
        putText(copy, "Num Measures: " + std::to_string(params.numMeasures), Point2d(50, 50), FONT_HERSHEY_SIMPLEX, 1,
                Scalar(0, 255, 0));
        std::cout << obj;

        imshow("image", copy);
        int keyPressed = waitKey(0);
        if (keyPressed == int('q')) {
            destroyAllWindows();
            break;
        }
    }
}
