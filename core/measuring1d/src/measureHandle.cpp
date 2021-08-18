#include <stdexcept>
#include "measureHandle.hpp"

using namespace std;

// Initialization constructor
MeasureHandle::MeasureHandle(int row_, int column_) :
        row{row_},
        column{column_},
        pairSize{0} {

    if (row_ < 0 or column_ < 0)
        throw invalid_argument("Row or column value cannot be negative");
}

// Translates a measure object
void MeasureHandle::translateMeasure(int newRow, int newCol) {
    if (newRow < 0 or newCol < 0)
        throw invalid_argument("Row or column value cannot be negative");

    row = newRow;
    column = newCol;
}

// Set fuzzy function
void MeasureHandle::setFuzzyFunc(const FuzzyFunc &func, FuzzyType type) {
    if (type == FuzzyType::SIZE_DIFF or type == FuzzyType::SIZE_ABS_DIFF)
        throw invalid_argument("Fuzzy type only available for normalized fuzzy function");

    fuzzySet.insert({type, func});
}

// Set normalized fuzzy function
void MeasureHandle::setNormFuzzyFunc(const FuzzyFunc &func, FuzzyType type, double pairSize_) {
    if (type == FuzzyType::CONTRAST or type == FuzzyType::GRAY)
        throw invalid_argument("Fuzzy type not available for normalized fuzzy function");

    pairSize = pairSize_;
    fuzzySet.insert({type, func});

    // Multiply each x-coordinate by pairSize
    fuzzySet.at(type).scale(pairSize_);
}

// Destructor
MeasureHandle::~MeasureHandle() = default;