#include "MeasuringCore.hpp"

#include "drawHandles.hpp"
#include "drawMeasuring.hpp"

#include <filesystem>

using namespace cv;
using namespace std;

int main(int argc, char *argv[]) {

    std::cout << std::filesystem::current_path() << "\n";

    // Read the image
    Mat image;
    Mat gray;

    //image = imread(argv[1]);
    image = imread("D:/Programming/Workspaces/Zebra/H-Rev-C/images/fuzzy_arc_test.png");
    cvtColor(image, gray, COLOR_BGR2GRAY);

    Mat profile;

    // MeasureRectangle mHandle(60, 10, 290, 170, -130, true);  // switch.png
    // MeasureRectangle mHandle(250, 5, 10, 256, 0, true); // fuzzy_tester.png
    // MeasureArc mHandle(30, 20, 450, 450, 0, 180, true); // rings_and_nuts.png
    ArcElement mHandle(200, 150, 271, 271);  // fuzzy_arc_test.png

    auto result = measurePairs(gray, mHandle, 1.0, 10.0);
    // auto result = measurePos(gray, mHandle, 1.1, 20);

    cout << result << "\n";

    Mat imgCopy = image.clone();
    drawMeasureHandle(imgCopy, mHandle);
    drawMeasurePairs(imgCopy, result);

    imshow("img", imgCopy);
    waitKey(0);

    // Print out the contents of the profile
    cout << profile << endl;
}