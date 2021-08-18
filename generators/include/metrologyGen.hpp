#ifndef METROLOGY_GENERATORS_HPP
#define METROLOGY_GENERATORS_HPP

#include "generatorAnchors.hpp"
#include "generatorParams.hpp"
#include "metrologyObject.hpp"
#include "ransacFit.hpp"

/**
 * @brief Pure abstract class which provides the template for other Metrology test case generators.
 */
class TestsGenerator {
   protected:
    cv::Mat spoofedImage;

   public:
    /**
     * @brief Generates a test image based on Anchor and Generator structs given (or derived from) on construction. 
     * 
     * @param imgShape Size of the image to create.
     * @param backGroundColor The background color of the image.
     * @param fillColors A vector of colors to use when creating multiple instances of a shape. Loops back to background color when exhausted.
    */
    virtual void spoofImage(cv::Size imgShape, cv::Scalar backgroundColor, std::vector<cv::Scalar> fillColors) = 0;
    
    /**
     * @brief Applies gaussian noise to the generated image.
     * 
     * @param mean Mean kernel for the gaussian filter.
     * @param sigma Sigma kernel fo the gaussian filter.
     */
    void applyGaussianNoise(cv::Scalar mean = cv::Scalar(0, 0, 0), cv::Scalar sigma = cv::Scalar(10, 10, 10));

    /**
     * @brief Applies salt and pepper noise to the generated image
     * 
     * @param cutoff Top and bottom percent of the image to apply salt and pepper noise to.
     */
    void applySaltPepper(double cutoff = 0.05);

    /**
     * @brief Returns the image with the generated shape(s).
     */
    cv::Mat getSpoofedImage() { return spoofedImage; };
};

/**
 * @brief Generator for circular Metrology test cases.
 * @details All generator subclasses have three constructors which accept the generator parameters 
 * and either their corresponding Anchor struct or a relevant object/struct to derive the anchor from.
 */
class ArcGenerator : public TestsGenerator {
   private:
    ArcAnchor anchor;
    ArcGeneratorParams params;

   public:
    ArcGenerator(const ArcObject& metobj, ArcGeneratorParams& genParams);
    ArcGenerator(const CircleParams& fitParams, ArcGeneratorParams& genParams);
    ArcGenerator(ArcAnchor& anchor, ArcGeneratorParams& genParams);
    void spoofImage(cv::Size imgShape, cv::Scalar backgroundColor, std::vector<cv::Scalar> fillColors) override;
};

/**
 * @brief Generator for elliptical Metrology test cases.
 * @details All generator subclasses have three constructors which accept the generator parameters 
 * and either their corresponding Anchor struct or a relevant object/struct to derive the anchor from.
 */
class EllipseGenerator : public TestsGenerator {
   private:
    EllipseAnchor anchor;
    EllipseGeneratorParams params;

   public:
    EllipseGenerator(const EllipseObject& metobj, EllipseGeneratorParams& genParams);
    EllipseGenerator(const EllipseParams& fitParams, EllipseGeneratorParams& genParams);
    EllipseGenerator(EllipseAnchor& anchor, EllipseGeneratorParams& genParams);
    void spoofImage(cv::Size imgShape, cv::Scalar backgroundColor, std::vector<cv::Scalar> fillColors) override;
};

/**
 * @brief Generator for linear Metrology test cases.
 * @details All generator subclasses have three constructors which accept the generator parameters 
 * and either their corresponding Anchor struct or a relevant object/struct to derive the anchor from.
 */
class LineGenerator : public TestsGenerator {
   private:
    LineAnchor anchor;
    LineGeneratorParams params;

   public:
    LineGenerator(const LineObject& metobj, LineGeneratorParams& genParams, int thickness);
    LineGenerator(const LineParams& fitParams, LineGeneratorParams& genParams);
    LineGenerator(LineAnchor& anchor, LineGeneratorParams& genParams);
    void spoofImage(cv::Size imgShape, cv::Scalar backgroundColor, std::vector<cv::Scalar> fillColors) override;
};

/**
 * @brief Generator for rectangular Metrology test cases.
 * @details All generator subclasses have three constructors which accept the generator parameters 
 * and either their corresponding Anchor struct or a relevant object/struct to derive the anchor from.
 */
class RectGenerator : public TestsGenerator {
   private:
    RectAnchor anchor;
    RectGeneratorParams params;

   public:
    RectGenerator(const RectObject& metobj, RectGeneratorParams& genParams);
    RectGenerator(const RectParams& fitParams, RectGeneratorParams& genParams);
    RectGenerator(RectAnchor& anchor, RectGeneratorParams& genParams);
    void spoofImage(cv::Size imgShape, cv::Scalar backgroundColor, std::vector<cv::Scalar> fillColors) override;
};
#endif