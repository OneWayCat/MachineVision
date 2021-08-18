
#include <iostream>

#include "drawHandles.hpp"
#include "measuring.hpp"
#include "meniscus.hpp"

using namespace boost::math;
using namespace cv;

struct ImageStruct {
    Mat *img;
};

Point2d topLeftSelection;
bool selectedTopLeft = false;
Point2d bottomRightSelection;
bool selectedBotRight = false;

void mouse_callback(int event, int x, int y, int flag, void *param) {
    ImageStruct imgStruct = *(ImageStruct *)param;
    Mat origImg = *imgStruct.img;

    if (event == EVENT_LBUTTONDOWN) {
        if (selectedTopLeft == false) {
            topLeftSelection = Point2d(y, x);
            circle(origImg, topLeftSelection, 3, Scalar(255, 0, 0), -1);
            selectedTopLeft = true;
        } else if (selectedBotRight == false) {
            bottomRightSelection = Point2d(y, x);
            circle(origImg, Point2d(x, y), 3, Scalar(0, 255, 0), -1);
            selectedBotRight = true;
        }

        imshow("orig", origImg);
    }
}

int main(int argc, char *argv[]) {
    Mat image;
    Mat gray;
    image = imread(argv[1]);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    ImageStruct imgStruct;
    imgStruct.img = &image;

    namedWindow("orig");
    setMouseCallback("orig", mouse_callback, &imgStruct);

    imshow("orig", image);

    bool ranDetection = false;

    // TODO: THIS SHOULD BE CHANGED FOR TESTING
    MeniscusParams params;
    params.edgeThresh = 30;
    params.meniscusShape = MeniscusShape::CONVEX;

    while (true) {
        if (selectedTopLeft && selectedBotRight && !ranDetection) {
            Point2d vertex = findMeniscus(gray, topLeftSelection, bottomRightSelection, params);

            std::cout << "Row: " << vertex.x << std::endl;
            std::cout << "Column: " << vertex.y << std::endl;

            ranDetection = true;
            circle(gray, Point2i{int(vertex.y), int(vertex.x)}, 3, COLOR_BLACK, -1);
        }

        imshow("orig", gray);
        int keyPressed = waitKey(1);
        if (keyPressed == int('q')) {
            destroyAllWindows();
            break;
        }
    }
}