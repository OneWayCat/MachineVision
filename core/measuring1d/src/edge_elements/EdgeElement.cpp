#include <stdexcept>

#include "EdgeElement.hpp"

using namespace std;

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