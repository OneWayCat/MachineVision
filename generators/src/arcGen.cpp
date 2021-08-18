#include "metrologyGen.hpp"

using namespace cv;

ArcGenerator::ArcGenerator(const ArcObject &metobj, ArcGeneratorParams &genParams) : params{genParams} {
    anchor.row = metobj.getRow();
    anchor.column = metobj.getColumn();
    anchor.radius = metobj.getRadius();
}

ArcGenerator::ArcGenerator(const CircleParams &fitParams, ArcGeneratorParams &genParams) : params{genParams} {
    anchor.row = (int) fitParams.center.x;
    anchor.column = (int) fitParams.center.y;
    anchor.radius = (int) fitParams.radius;
}

ArcGenerator::ArcGenerator(ArcAnchor &anchor, ArcGeneratorParams &genParams) : anchor{anchor}, params{genParams} {}

void ArcGenerator::spoofImage(Size imgShape, Scalar backgroundColor, std::vector<Scalar> fillColors) {
    Mat img = Mat(imgShape, CV_8UC3, backgroundColor);

    fillColors.push_back(backgroundColor);
    int colorPicker = 0;

    int row = anchor.row + randomRange(-params.jitterRow, params.jitterRow);
    int column = anchor.column + randomRange(-params.jitterColumn, params.jitterColumn);
    int radius = anchor.radius + randomRange(-params.jitterRadius, params.jitterRadius);
    double startAngle = randomRange(params.angleStartRange[0], params.angleStartRange[1]);
    double endAngle = randomRange(params.angleEndRange[0], params.angleEndRange[1]);

    for (int i = 0; i < params.numInstances; i++) {
        colorPicker = i % fillColors.size();
        
        ellipse(img, Point2d(column, row), Size(radius, radius), 0, -rad2deg(startAngle), -rad2deg(endAngle), fillColors[colorPicker], -1);
        
        radius -= params.instanceSeparation;
    }

    spoofedImage = img;
}
