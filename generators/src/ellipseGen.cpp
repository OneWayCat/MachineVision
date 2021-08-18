#include "mathutils.hpp"
#include "metrologyGen.hpp"

using namespace cv;

EllipseGenerator::EllipseGenerator(const EllipseObject& metobj, EllipseGeneratorParams& genParams) : params{genParams} {
    anchor.row = metobj.getRow();
    anchor.column = metobj.getColumn();
    anchor.phi = metobj.getPhi();
    anchor.shortRadius = metobj.getShortRadius();
    anchor.longRadius = metobj.getLongRadius();
}

EllipseGenerator::EllipseGenerator(const EllipseParams& fitParams, EllipseGeneratorParams& genParams) : params{genParams} {
    anchor.row = (int) fitParams.center.x;
    anchor.column = (int) fitParams.center.y;
    anchor.phi = fitParams.phi;
    anchor.shortRadius = (int) fitParams.shortRadius;
    anchor.longRadius = (int) fitParams.longRadius;
}

EllipseGenerator::EllipseGenerator(EllipseAnchor& anchor, EllipseGeneratorParams& genParams) : anchor{anchor}, params{genParams} {}

void EllipseGenerator::spoofImage(Size imgShape, Scalar backgroundColor, std::vector<Scalar> fillColors) {
    Mat img = Mat(imgShape, CV_8UC3, backgroundColor);

    fillColors.push_back(backgroundColor);
    int colorPicker = 0;

    int row = anchor.row + randomRange(-params.jitterRow, params.jitterRow);
    int column = anchor.column + randomRange(-params.jitterColumn, params.jitterColumn);
    double phi = anchor.phi + randomRange(-params.jitterPhi, params.jitterPhi);
    int shortRadius = anchor.shortRadius + randomRange(-params.jitterShortRadius, params.jitterShortRadius);
    int longRadius = anchor.longRadius + randomRange(-params.jitterLongRadius, params.jitterLongRadius);
    double startAngle = randomRange(params.angleStartRange[0], params.angleStartRange[1]);
    double endAngle = randomRange(params.angleEndRange[0], params.angleEndRange[1]);

    for (int i = 0; i < params.numInstances; i++) {
        colorPicker = i % fillColors.size();

        ellipse(img, Point2d(column, row), Size(longRadius, shortRadius), -rad2deg(phi), -rad2deg(startAngle), -rad2deg(endAngle), fillColors[colorPicker], -1);

        shortRadius -= params.instanceSeparation;
        longRadius -= params.instanceSeparation;
    }

    spoofedImage = img;
}