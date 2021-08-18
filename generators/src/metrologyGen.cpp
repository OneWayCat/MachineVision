#include "metrologyGen.hpp"
using namespace cv;

void TestsGenerator::applyGaussianNoise(Scalar mean, Scalar sigma) {
    Mat noise = Mat(spoofedImage.size(), CV_16SC3);
    randn(noise, mean, sigma);

    int originalType = spoofedImage.type();
    spoofedImage.convertTo(spoofedImage, CV_16SC3);
    addWeighted(spoofedImage, 1.0, noise, 1.0, 0.0, spoofedImage);
    spoofedImage.convertTo(spoofedImage, originalType);
}

void TestsGenerator::applySaltPepper(double cutoff) {
    Mat noise = Mat::zeros(spoofedImage.size(), CV_8U);
    randu(noise, 0, 255);
    Mat black = noise < (255 * cutoff);
    Mat white = noise > (255 * (1.0 - cutoff));

    spoofedImage.setTo(255, white);
    spoofedImage.setTo(0, black);
}

