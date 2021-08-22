#include "ArcTransposedElement.hpp"

using namespace cv;


EdgeResults ArcTransposedElement::findEdgePos(const std::vector<double> &coords, SelectType select) const {
    // No edges detected
    if (coords.empty())
        return EdgeResults{};

    // Find the appropriate number of edges to return
    size_t numEdgesDetected = coords.size();
    size_t numEdges = select == SelectType::ALL ? numEdgesDetected : 1;

    std::vector<Point2d> posEdges(numEdges);
    std::vector<double> distances(numEdges - 1);

    int centerRow = row;
    int centerColumn = column;
    double middleAngle = angleStart + angleExtent / 2 + (CV_PI / 2);

    double cosTheta = cos(middleAngle);
    double sinTheta = sin(middleAngle);

    // Calculate edgePositions based on coords
    if (select != SelectType::LAST) {
        posEdges.front().x = centerColumn + (coords[0] + innerRadius) * sinTheta;
        posEdges.front().y = centerRow + (coords[0] + innerRadius) * cosTheta;
    }

    if (select != SelectType::FIRST) {
        posEdges.back().x = centerColumn + (coords[0] + innerRadius) * sinTheta;
        posEdges.back().y = centerRow + (coords[0] + innerRadius) * cosTheta;
    }

    if (select == SelectType::ALL) {
        for (size_t i = 1; i < numEdgesDetected; i++) {
            posEdges[i].x = centerColumn + (coords[i] + innerRadius) * sinTheta;
            posEdges[i].y = centerRow + (coords[i] + innerRadius) * cosTheta;
            distances[i - 1] = coords[i] - coords[i - 1];
        }
    }

    return EdgeResults{posEdges, distances};
}


// Constructor
ArcTransposedElement::ArcTransposedElement(int radius_, int innerRadius_, int row_, int column_, double angleStart_, double angleExtent_, bool angleInDegrees)
        : EdgeElement(row_, column_),
          ArcElement(radius_, innerRadius_, row_, column_, angleStart_, angleExtent_, angleInDegrees) {

    profileLength = abs(radius_ - innerRadius_);

    // Make a 2D array of bin numbers for each (r, c) pair
    int numRows = maxR - minR + 1;
    int numCols = maxC - minC + 1;

    double angleEnd = angleStart + angleExtent;

    int numberOfBins = int(profileLength);
    binCounts = std::vector<int>(numberOfBins);

    //    Mat drawImg = Mat::zeros(500, 500, CV_8UC1);
    //    circle(drawImg, Point(column_, row_), 1, Scalar(255), -1);
    //    imshow("temp", drawImg);

    // Main loop to iterate over each (r, c) pair inside the bounding box
    for (int r = 0; r < numRows; r++) {
        for (int c = 0; c < numCols; c++) {
            Point2d toProject = Point2d(double(c) + minC, double(r) + minR);

            double pointNorm = norm(toProject);
            //            double pointAngle = atan2(toProject.y, toProject.x) - (CV_PI / 2);
            double pointAngle = atan2(toProject.y, toProject.x);

            // Map to (-pi, pi]
            if (pointAngle <= -CV_PI)
                pointAngle += CV_2PI;

            if (!insideArc(innerRadius, radius, angleStart, angleEnd, pointNorm, pointAngle))
                binNumbers[size_t(r) * numCols + c] = -1;

            else {
                // Calculate bin number
                Point trueImgPoint(c + column_ + minC, r + row_ + minR);
                double intBinNumber = (int) round(sqrt(pow(trueImgPoint.x - column_, 2) + pow(trueImgPoint.y - row_, 2)) - innerRadius_);
                if (intBinNumber >= profileLength)
                    intBinNumber = profileLength - 1;
                else if (intBinNumber < 0)
                    intBinNumber = 0;

                //                drawImg.at<uchar>(int(trueImgPoint.y), int(trueImgPoint.x)) = 255;

                // Add the pixel of the image to the correct bin
                //                int intBinNumber = (int) round(binNumber);

                binNumbers[size_t(r) * numCols + c] = (int) intBinNumber;
                binCounts[(int) intBinNumber]++;
            }
        }
        //        imshow("temp", drawImg);
        //        waitKey(0);
    }
}