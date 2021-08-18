#include "metrologyGen.hpp"
#include "mathutils.hpp"

using namespace cv;

LineGenerator::LineGenerator(const LineObject& metobj, LineGeneratorParams& genParams, int thickness) {
    anchor.row = metobj.getRowStart();
    anchor.column = metobj.getColumnStart();
    anchor.endRow = metobj.getRowEnd();
    anchor.endColumn = metobj.getColumnEnd();
    anchor.thickness = thickness;

    params = genParams;
}

LineGenerator::LineGenerator(const LineParams& fitParams, LineGeneratorParams& genParams) {
    anchor.row = (int) fitParams.startPoint.x;
    anchor.column = (int) fitParams.startPoint.y;
    anchor.endRow = (int) fitParams.endPoint.x;
    anchor.endColumn = (int) fitParams.endPoint.y;

    params = genParams;
}

LineGenerator::LineGenerator(LineAnchor& anchor, LineGeneratorParams& genParams) : anchor{anchor}, params{genParams} {}

void LineGenerator::spoofImage(Size imgShape, Scalar backgroundColor, std::vector<Scalar> fillColors) {
    Mat img = Mat(imgShape, CV_8UC3, backgroundColor);

    int colorPicker = 0;
    int direction = -1;

    int rowStart = anchor.row + randomRange(-params.jitterRow, params.jitterRow);
    int colStart = anchor.column + randomRange(-params.jitterColumn, params.jitterColumn);
    int rowEnd = anchor.endRow + randomRange(-params.jitterEndRow, params.jitterEndRow);
    int colEnd = anchor.endColumn + randomRange(-params.jitterEndColumn, params.jitterEndColumn);
    int thickness = anchor.thickness + randomRange(0, params.jitterThickness);

    int halfRowDiff = abs(rowEnd - rowStart) / 2;
    int halfColDiff = abs(colEnd - colStart) / 2;

    for (int i = 0; i < params.numInstances; i++) {
        colorPicker = i % fillColors.size();

        Point2d offsetPoint = getPerpOffset(Point2d(rowStart, colStart), Point2d(rowEnd, colEnd), i * direction * params.instanceSeparation);

        rowStart = (int) offsetPoint.x - halfRowDiff;
        colStart = (int) offsetPoint.y - halfColDiff;
        rowEnd = (int) offsetPoint.x + halfRowDiff;
        colEnd = (int) offsetPoint.y + halfColDiff;

        line(img, Point2d(colStart, rowStart), Point2d(colEnd, rowEnd), fillColors[colorPicker], thickness);

        direction = (direction == -1) ? 1 : -1;
    }

    spoofedImage = img;
}
