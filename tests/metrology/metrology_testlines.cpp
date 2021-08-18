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
    Size shape = image.size();

    while (true) {
        Mat copy;
        image.copyTo(copy);

        int randRowStart = rand() % shape.height;
        int randColStart = rand() % shape.width;
        int randRowEnd = rand() % shape.height;
        int randColEnd = rand() % shape.width;

        MeasureParams lineParams{};
        lineParams.numMeasures = 10;
        LineObject lineObj{lineParams, randRowStart, randColStart, randRowEnd, randColEnd};

        drawLineObject(copy, lineObj, Scalar(0,255,0), false);
        std::cout << lineObj;

        imshow("image", copy);
        int keyPressed = waitKey(0);
        if (keyPressed == int('q')) {
            destroyAllWindows();
            break;
        }
    }
}
