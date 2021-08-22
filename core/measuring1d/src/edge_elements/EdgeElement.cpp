#include <stdexcept>

#include "EdgeElement.hpp"

using namespace std;
using namespace cv;

Mat EdgeElement::projectOnToProfileLine(const Mat &img) const {
    // Ensure image is single channel
    CV_Assert(img.depth() == CV_8U && img.channels() == 1);

    // Dimensions of the image
    int imgHeight = img.rows;
    int imgWidth = img.cols;

    // Coordinates of the bounding box of the measure handle
    int boxMinC = minC + column;
    int boxMinR = minR + row;

    size_t numCols = size_t(maxC) - minC + 1;
    size_t numRows = size_t(maxR) - minR + 1;

    // Project down to 1D profile
    Mat_<double> profile = Mat_<double>::zeros(1, (int) binCounts.size());
    auto profPtr = profile.ptr<double>();  // Pointer to the profile array

    // Main loop to iterate over each (r, c) pair inside the bounding box
    for (size_t r = 0; r < numRows; r++) {
        int imgRow = (int) r + boxMinR;

        // Treat as 0 if out of bounds
        if (imgRow < 0 || imgRow >= imgHeight)
            continue;

        const auto rowPtr = img.ptr<uchar>(imgRow);  // Pointer to the row of the image

        for (int c = 0; c < numCols; c++) {
            int binNumber = binNumbers[r * numCols + c];
            int imgCol = c + boxMinC;

            // Inside check
            if (binNumber != -1 && imgCol >= 0 && imgCol < imgWidth) {
                // Add the pixel of the image to the correct bin
                profPtr[binNumber] += rowPtr[imgCol];
            }
        }
    }

    // Divide the profile by binCounts to get the mean
    for (int i = 0; i < binCounts.size(); i++) {
        if (binCounts[i] != 0)
            profPtr[i] = (double) profPtr[i] / binCounts[i];
    }

    return profile;
}

// Initialization constructor
EdgeElement::EdgeElement(int row_, int column_) :
        row{row_},
        column{column_},
        pairSize{0} {

    if (row_ < 0 || column_ < 0)
        throw invalid_argument("Row or column value cannot be negative");
}

// Translates a measure object
void EdgeElement::translateMeasure(int newRow, int newCol) {
    if (newRow < 0 || newCol < 0)
        throw invalid_argument("Row or column value cannot be negative");

    row = newRow;
    column = newCol;
}

// Set fuzzy function
void EdgeElement::setFuzzyFunc(const FuzzyFunction &func, FuzzyType type) {
    if (type == FuzzyType::SIZE_DIFF || type == FuzzyType::SIZE_ABS_DIFF)
        throw invalid_argument("Fuzzy type only available for normalized fuzzy function");

    fuzzySet.insert({type, func});
}

// Set normalized fuzzy function
void EdgeElement::setNormFuzzyFunc(const FuzzyFunction &func, FuzzyType type, double pairSize_) {
    if (type == FuzzyType::CONTRAST || type == FuzzyType::GRAY)
        throw invalid_argument("Fuzzy type not available for normalized fuzzy function");

    pairSize = pairSize_;
    fuzzySet.insert({type, func});

    // Multiply each x-coordinate by pairSize
    fuzzySet.at(type).scale(pairSize_);
}