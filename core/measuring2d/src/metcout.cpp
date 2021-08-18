#include "metcout.hpp"

#include <iterator>

std::ostream &operator<<(std::ostream &o, const MeasureParams &mP) {
    std::cout << "[\n";
    std::cout << "\tPerpendicular Length:\t" << mP.perpLength << "\n";
    std::cout << "\tTangential Length:\t" << mP.tangLength << "\n";
    std::cout << "\tSeparation:\t\t" << mP.separation << "\n";
    std::cout << "\tNumber of Measures:\t" << mP.numMeasures << "\n";
    std::cout << "\tSigma:\t\t\t" << mP.sigma << "\n";

    std::cout << "]\n";

    return o;
}

std::ostream &operator<<(std::ostream &o, const ArcObject &obj) {
    std::cout << "[\n";
    std::cout << "\tRow:\t" << obj.getRow() << "\n";
    std::cout << "\tColumn:\t" << obj.getColumn() << "\n";
    std::cout << "\tRadius:\t" << obj.getRadius() << "\n";
    std::cout << "\tAngleStart:\t" << obj.getAngleStart() << "\n";
    std::cout << "\tAngleExtent:\t" << obj.getAngleExtent() << "\n";
    std::cout << "]\n";

    return o;
}

std::ostream &operator<<(std::ostream &o, const EllipseObject &obj) {
    std::cout << "[\n";
    std::cout << "\tRow:\t" << obj.getRow() << "\n";
    std::cout << "\tColumn:\t" << obj.getColumn() << "\n";
    std::cout << "\tRadii (S/L):\t" << '(' << obj.getShortRadius() << ',' << obj.getLongRadius() << ')' << "\n";
    std::cout << "\tPhi:\t" << obj.getPhi() << "\n";
    std::cout << "\tAngle (S/E):\t" << '(' << obj.getAngleStart() << ',' << obj.getAngleExtent() << ')' << "\n";
    std::cout << "]\n";

    return o;
}

std::ostream &operator<<(std::ostream &o, const LineObject &obj) {
    std::cout << "[\n";
    std::cout << "\tStart:\t"
              << "(" << obj.getRowStart() << ", " << obj.getColumnStart() << ")"
              << "\n";
    std::cout << "\tEnd:\t"
              << "(" << obj.getRowEnd() << ", " << obj.getColumnEnd() << ")"
              << "\n";
    std::cout << "]\n";

    return o;
}

std::ostream &operator<<(std::ostream &o, const RectObject &obj) {
    std::cout << "[\n";
    std::cout << "\tRow:\t\t" << obj.getRow() << "\n";
    std::cout << "\tColumn:\t\t" << obj.getColumn() << "\n";
    std::cout << "\tPhi:\t\t" << obj.getPhi() << "\n";
    std::cout << "\tHalf Width:\t" << obj.getHalfWidth() << "\n";
    std::cout << "\tHalf Height:\t" << obj.getHalfHeight() << "\n";
    std::cout << "]\n";

    return o;
}

std::ostream &operator<<(std::ostream &o, const CircleParams &obj) {
    std::cout << "[\n";
    std::cout << "\tCenter:\t" << obj.center << "\n";
    std::cout << "\tRadius:\t" << obj.radius << "\n";
    std::cout << "]\n";

    return o;
}

std::ostream &operator<<(std::ostream &o, const EllipseParams &obj) {
    std::cout << "[\n";
    std::cout << "\tCenter:\t\t" << obj.center << "\n";
    std::cout << "\tPhi:\t\t" << obj.phi << "\n";
    std::cout << "\tShort Radius:\t" << obj.shortRadius << "\n";
    std::cout << "\tLong Radius:\t" << obj.longRadius << "\n";
    std::cout << "]\n";

    return o;
}

std::ostream &operator<<(std::ostream &o, const LineParams &obj) {
    std::cout << "[\n";
    std::cout << "\tStart:\t" << obj.startPoint << "\n";
    std::cout << "\tEnd:\t" << obj.endPoint << "\n";
    std::cout << "]\n";

    return o;
}

std::ostream &operator<<(std::ostream &o, const LineEqParams &obj) {
    std::cout << "[\n";
    std::cout << "\tCoeff 'a':\t" << obj.a << "\n";
    std::cout << "\tCoeff 'b':\t" << obj.b << "\n";
    std::cout << "\tCoeff 'c':\t" << obj.c << "\n";
    std::cout << "]\n";

    return o;
}

std::ostream &operator<<(std::ostream &o, const RectParams &obj) {
    std::cout << "[\n";
    std::cout << "\tRow:\t\t" << obj.row << "\n";
    std::cout << "\tColumn:\t\t" << obj.column << "\n";
    std::cout << "\tPhi:\t\t" << obj.phi << "\n";
    std::cout << "\tHalf Width:\t" << obj.halfWidth << "\n";
    std::cout << "\tHalf Height:\t" << obj.halfHeight << "\n";
    std::cout << "]\n";

    return o;
}
