#include "fuzzyMeasuring.hpp"

#include "drawFuzzy.hpp"
#include "drawHandles.hpp"
#include "mcout.hpp"

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
    // Read the image
    Mat image;
    Mat gray;

    image = imread(argv[1]);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    // FuzzyType::SIZE
    std::vector<double> xVals({20, 50, 80});
    std::vector<double> yVals({0, 1, 0});

    FuzzyFunc func(xVals, yVals);
    MeasureRectangle mHandle(250, 5, 10, 256, 0, true);  // fuzzy_tester.png
                                                         //    MeasureArc mHandle(200, 150, 271, 271); // fuzzy_arc_test.png

    mHandle.setFuzzyFunc(func, FuzzyType::SIZE);
    //    mHandle.setFuzzyFunc(func, FuzzyType::POSITION);

    //    auto result = fuzzyMeasurePos(gray, mHandle, 1, 10.0, 0.5, TransitionType::ALL);
    auto result = fuzzyMeasurePairs(gray, mHandle, 1, 10.0, 0.5, TransitionType::ALL);

    std::cout << result << "\n";

    Mat imgCopy = image.clone();
    drawMeasureHandle(imgCopy, mHandle);
    drawFuzzyPairs(imgCopy, result);

    imshow("img", imgCopy);
    waitKey(0);
}