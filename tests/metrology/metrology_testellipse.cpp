#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

#include "metrologyObject.hpp"
#include "drawMetrology.hpp"
#include "metcout.hpp"

using namespace boost::math;
using namespace boost::random;
using namespace cv;

int main(int argc, char *argv[]) {
    Mat image;
    Mat gray;

    image = imread(argv[1]);
    cvtColor(image, gray, COLOR_BGR2GRAY);

    srand((unsigned int) time(0));
    Size shape = image.size();

    mt19937 rng;
    rng.seed((unsigned int) time(0));

    auto randomAngle = uniform_real_distribution<double>(-double_constants::two_pi, double_constants::two_pi);

    while (true) {
        Mat copy;
        image.copyTo(copy);

        double phi = randomAngle(rng);
        double angleStart = randomAngle(rng);
        double angleExtent = randomAngle(rng);

        int shortRadius = shape.width / 4 + rand() % (shape.width / 12);
        int longRadius = shape.width / 3 + rand() % (shape.width / 6);

        MeasureParams params{};
        params.numMeasures = 10;
        EllipseObject ellipseObject{params, 271, 271, longRadius, shortRadius, phi, angleStart, angleExtent};

        drawEllipseObject(copy, ellipseObject);
        std::cout << ellipseObject;

        imshow("image", copy);
        int keyPressed = waitKey(0);
        if (keyPressed == int('q')) {
            destroyAllWindows();
            break;
        }
    }
}
