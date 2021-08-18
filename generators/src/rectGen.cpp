#include "metrologyGen.hpp"

using namespace cv;

RectGenerator::RectGenerator(const RectObject &metobj, RectGeneratorParams &genParams) {
    anchor.row = metobj.getRow();
    anchor.column = metobj.getColumn();
    anchor.phi = metobj.getPhi();
    anchor.halfWidth = metobj.getHalfWidth();
    anchor.halfHeight = metobj.getHalfHeight();

    params = genParams;
}

RectGenerator::RectGenerator(const RectParams &fitParams, RectGeneratorParams &genParams) {
    anchor.row = (int) fitParams.row;
    anchor.column = (int) fitParams.column;
    anchor.phi = fitParams.phi;
    anchor.halfWidth = (int) fitParams.halfWidth;
    anchor.halfHeight = (int) fitParams.halfHeight;

    params = genParams;
}

RectGenerator::RectGenerator(RectAnchor &anchor, RectGeneratorParams &genParams) : anchor{anchor}, params{genParams} {}

void RectGenerator::spoofImage(Size imgShape, Scalar backgroundColor, std::vector<Scalar> fillColors) {
    Mat img = Mat(imgShape, CV_8UC3, backgroundColor);

    fillColors.push_back(backgroundColor);
    int colorPicker = 0;

    int row = anchor.row + randomRange(-params.jitterRow, params.jitterRow);
    int col = anchor.column + randomRange(-params.jitterColumn, params.jitterColumn);
    double phi = anchor.phi + randomRange(-params.jitterPhi, params.jitterPhi);
    int halfWidth = anchor.halfWidth + randomRange(-params.jitterHalfWidth, params.jitterHalfWidth);
    int halfHeight = anchor.halfHeight + randomRange(-params.jitterHalfHeight, params.jitterHalfHeight);

    for (int i = 0; i < params.numInstances; i++) {
        colorPicker = i % fillColors.size();

        RotatedRect rotRect = RotatedRect(Point2d(col, row), Size(halfWidth * 2, halfHeight * 2), (float) rad2deg(-phi));
        Point2f vertices2f[4];
        rotRect.points(vertices2f);

        Point vertices[4];
        for (int i = 0; i < 4; i++) {
            vertices[i] = vertices2f[i];
        }

        fillConvexPoly(img, vertices, 4, fillColors[colorPicker]);

        halfWidth -= params.instanceSeparation;
        halfHeight -= params.instanceSeparation;
    }

    spoofedImage = img;
}
