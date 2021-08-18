/**
 * @file metrologyObject.hpp
 * @brief MetrologyObject classes.
 */

#ifndef METROLOGY_METROLOGY_OBJECTS_HPP
#define METROLOGY_METROLOGY_OBJECTS_HPP

#include "measuring.hpp"
#include "ransacFit.hpp"

/* Data structure to represent the points used for Ransac. Represented as a 2D vector, where the rows are
 * the points from each measure region. */
typedef std::vector<std::vector<cv::Point2d>> RansacPoints;

/**
 * @brief A structure to represent the parameters for the measure regions of the metrology object
 * and the parameters for edge detection within the measure regions.
 */
struct MeasureParams {
    /** Half-length of the measure regions in the direction perpendicular to the perimeter of the object. */
    int perpLength = 20;
    /** Half-length of the measure regions in the direction tangent to the perimeter of the object. */
    int tangLength = 5;

    /** Distance between centers of two consecutive measure regions. If the distance is too long, it will
     * be shortened so that the requirement for minimum number of measure regions is satisfied. If numMeasures
     * is greater than 0, this value is ignored. */
    double separation = 10.0;
    /** Desired number of measure regions. If this number is too small, it will be increased so that the
     * requirement for minimum number of measure regions is satisfied. If this value is greater than 0,
     * the separation parameter will be ignored and numMeasures will be used instead. */
    int numMeasures = 0;

    /** Sigma for Gaussian smoothing to use in edge detection. Same behavior as in measurePos. */
    double sigma = 1.0;
    /** Minimum edge amplitude for edge detection. */
    double threshold = 30.0;
    /** Whether to detect the first, last, or all edges within each measure region. */
    SelectType select = SelectType::ALL;
    /** Whether to detect dark->light edges or light->dark edges. TransitionType::ALL is not supported. */
    TransitionType transition = TransitionType::POSITIVE;
};

/**
 * @brief This abstract class defines a template shape that we want to match. It is composed of
 * measure regions around the shape's perimeter.
 */
class MetrologyObject {
protected:
    /** Minimum fuzzy score. */
    double fuzzyThresh = 0.5;

    /** Fuzzy functions to use. */
    std::unordered_map<FuzzyType, FuzzyFunc, EnumClassHash> fuzzySet;

    /** Minimum number of points needed for RANSAC fitting. */
    int minNumMeasures;

    /** List of measure regions around the perimeter of the shape. */
    std::vector<MeasureRectangle> measures;

    /** Edges detected from each measureHandle */
    std::vector<MeasurePosResult> measureResults;

public:
    /** Parameters for the size and position of measure regions, and for edge detection. */
    MeasureParams measureParams;

protected:
    // Validate the measure parameters of the object.
    void validateMeasureParams() const;

    /**
     * @brief Initialize measure handles along the object's perimeter.
     * @details Since different shapes have different parameters, this method is implemented in all subclasses.
     */
    virtual void initMeasures() = 0;

    /**
     * @brief Set fuzzy functions for all of the measure handles created for this object.
     * Should always be called after initMeasures().
     */
    void initMeasuresFuzzy();

    virtual /**
     * @brief Call measurePos or fuzzyMeasurePos on each measure handle and stores the results
     * in the measuresResult variable.
     * Must be called after initMeasures.
     */
    void applyMeasures(const cv::Mat &img);

    /**
     * @brief Fit a shape with fitPoints and return the indices of the inliers among testPoints.
     * This method should really be a static method, but its behavior differs from shape to
     * shape and since it's called by a class method, this was the simplest way to implement it.
     * @param fitPoints Points used to fit a shape
     * @param testPoints Points to test for inliers
     * @param distThresh The maximum distance between the point and the shape for it to be considered an inlier
     * @return The indices of the inliers among the testPoints vector
     */
    virtual std::vector<int>
    getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
               double distThresh) = 0;

    /**
     * @brief Fit the final shape using the inlier points, and then add it to metResults.
     * @param inliers
     */
    virtual bool addResult(const std::vector<cv::Point2d> &inliers, bool allowOutsideRegion) { return true; }

public:
    /** Construct a MetrologyObject. */
    explicit MetrologyObject(const MeasureParams &params, int minNumMeasures)
            : measureParams{params},
              minNumMeasures{minNumMeasures} {}

    /** Get the fuzzy score threshold. */
    double getFuzzyThresh() const { return fuzzyThresh; }

    /** Set the fuzzy score threshold. */
    void setFuzzyThresh(double score);

    /**
     * @brief Set the fuzzy type and its fuzzy function to score edges with.
     * @param func Fuzzy function to use to score edges with.
     * @param type Fuzzy criteria to use.
     */
    void setFuzzyFunc(const FuzzyFunc &func, FuzzyType type) { fuzzySet.insert({type, func}); }

    /**
     * @brief Get the fuzzy function for the specified fuzy type.
     * @param type
     * @return Fuzzy function set for this object for the specified type.
     */
    const FuzzyFunc &getFuzzyFunc(FuzzyType type) const { return fuzzySet.at(type); }

    /**
     * @brief Reset all fuzzy functions of a metrology object.
     */
    void resetFuzzy() { fuzzySet.clear(); }

    /**
     * @brief Get the reference to the list of measure handles of the object.
     * @return A const reference to the vector of measure handles of the object.
     */
    const std::vector<MeasureRectangle> &getMeasures() const { return measures; }

    /**
     * @brief Get the reference to the list of measure results of the object, in the same order as getMeasures().
     * @return A const reference to the vector of 1D measuring results of each measure handle of the object.
     */
    const std::vector<MeasurePosResult> &getMeasureResults() const { return measureResults; }

    /**
     * @brief Find close matches to the shape determined by the MetrologyObject.
     * @details First, this function performs edge detection on all of its measures, and
     * using these edge points, uses RANSAC to fit a shape.
     * @param img Single-channel input image.
     * @param minScore Minimum score for a valid instance of the metrology model. The score is determined by
     *  the number of measure handles with inlier points divided by the total number of measure handles.
     *  This should be an estimate of the proportion of measures with inlier points.
     * @param numInstances Maximum number of valid instances of the metrology model to find. Must be 1, 2, 3 or 4.
     * @param distThresh The maximum distance between a point and a shape for the point to be considered an inlier.
     * @param maxIters Maximum number of iterations for RANSAC. If this value is negative, the upper limit will
     *  be automatically calculated.
     * @param randSeed Seed for the random number generator to use for RANSAC. If this is 0, the current time is used.
     * @param allowOutsideRegion Whether to allow shapes that are ouside the measure regions.
     */
    virtual void applyMetrologyModel(const cv::Mat &img, double minScore, unsigned int numInstances,
                                     double distThresh, int maxIters, unsigned int randSeed, bool allowOutsideRegion);
};

/**
 * @brief A circular template to match.
 */
class ArcObject : public MetrologyObject {
private:
    int row;
    int column;
    int radius;

    double angleStart;
    double angleExtent;
    std::vector<CircleParams> metResults;

    // Initialize the measure handles of the object around its perimeter.
    void initMeasures() override;

    // Find the number of measure handles needed around the perimeter
    int findNumMeasures();

    std::vector<int> getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                                double distThresh) override;

    bool addResult(const std::vector<cv::Point2d> &inliers, bool allowOutsideRegion) override;

public:
    /**
     * @brief Construct an ArcObject.
     * @param measureParams Parameters for measure handles and edge detection.
     * @param row Row coordinate of center.
     * @param col Column coordinate of center.
     * @param radius Radius of the circle or the arc.
     * @param angleStart Starting angle of the arc, measured from positive horizontal axis, counterclockwise. Angles are given in radians.
     * @param angleExtent Angular extent of the arc, in radians. If this value is negative, the arc is clockwise. Must be between -2pi and 2pi.
     */
    ArcObject(const MeasureParams &measureParams, int row, int col, int radius, double angleStart = 0.0,
              double angleExtent = boost::math::double_constants::two_pi);

    // Getters
    int getRow() const { return row; }

    int getColumn() const { return column; }

    int getRadius() const { return radius; }

    double getAngleStart() const { return angleStart; }

    double getAngleExtent() const { return angleExtent; }

    /**
     * @return Get the list of shapes returned by the applyMetrologyModel method.
     */
    const std::vector<CircleParams> &getMetrologyResults() { return metResults; }
};

/**
 * @brief An elliptical template to match.
 */
class EllipseObject : public MetrologyObject {
private:
    int row;
    int column;
    int longRadius;
    int shortRadius;

    double phi;
    double angleStart;
    double angleExtent;
    std::vector<EllipseParams> metResults;

    void initMeasures() override;

    int findNumMeasures(double circum);

    std::vector<int> getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                                double distThresh) override;

    bool addResult(const std::vector<cv::Point2d> &inliers, bool allowOutsideRegion) override;

public:
    /**
     * @brief Construct an EllipseObject.
     * @param measureParams Parameters for measure handles and edge detection.
     * @param row Row coordinate of center.
     * @param col Column coordinate of center.
     * @param longRadius Major radius of the ellipse.
     * @param shortRadius Minor radius of the ellipse.
     * @param phi Angle of the major axis measured from positive horizontal axis. All angles given in radians.
     * @param angleStart Starting angle of the elliptical arc. angleStart and angleExtent are given in parametric angles.
     * @param angleExtent Angular extent of the elliptical arc, in radians. If this value is negative, the arc is clockwise. Must be between -2pi and 2pi.
     */
    EllipseObject(const MeasureParams &measureParams, int row, int col, int longRadius, int shortRadius,
                  double phi = 0.0, double angleStart = 0.0,
                  double angleExtent = boost::math::double_constants::two_pi);

    // Getters
    int getRow() const { return row; }

    int getColumn() const { return column; }

    int getShortRadius() const { return shortRadius; }

    int getLongRadius() const { return longRadius; }

    double getPhi() const { return phi; }

    double getAngleStart() const { return angleStart; }

    double getAngleExtent() const { return angleExtent; }

    /**
     * @return Get the list of shapes returned by the applyMetrologyModel method.
     */
    const std::vector<EllipseParams> &getMetrologyResults() { return metResults; }
};

/**
 * @brief A line template to match.
 */
class LineObject : public MetrologyObject {
private:
    int rowStart;
    int columnStart;
    int rowEnd;
    int columnEnd;
    std::vector<LineParams> metResults;

    // Initialize the measure handles of the object around its perimeter.
    void initMeasures() override;

    // Find the number of measure handles needed around the perimeter
    int findNumMeasures();

    std::vector<int> getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                                double distThresh) override;

    bool addResult(const std::vector<cv::Point2d> &inliers, bool allowOutsideRegion) override;

    // This is done so RectObject can access the initMeasures() function
    friend class RectObject;

public:
    /**
         * @brief Construct a LineObject.
         * @param measureParams Parameters for measure handles and edge detection.
         * @param rowStart Row coordinate of first point on line.
         * @param colStart Column coordinate of first point on line
         * @param rowEnd Row coordinate of second point on line.
         * @param colEnd Column coordinate of second point on line.
         */
    LineObject(const MeasureParams &measureParams, int rowStart, int colStart, int rowEnd, int colEnd);

    int getRowStart() const { return rowStart; }

    int getColumnStart() const { return columnStart; }

    int getRowEnd() const { return rowEnd; }

    int getColumnEnd() const { return columnEnd; }

    /**
     * @return Get the list of shapes returned by the applyMetrologyModel method.
     */
    const std::vector<LineParams> &getMetrologyResults() { return metResults; }
};

/**
 * @brief A rectangular template to match.
 */
class RectObject : public MetrologyObject {
private:
    int row;
    int column;
    double phi;
    int halfWidth;
    int halfHeight;
    std::vector<cv::Point2d> rectPoints;

    std::vector<RectParams> metResults;

    void initMeasures() override;

    std::vector<int> getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                                double distThresh) override;

    bool
    addResult(const std::vector<cv::Point2d> &inliers, const std::vector<size_t> &inliersPerSide, bool allowOutsideRegion);

public:
    /**
         * @brief Construct a RectObject.
         * @param measureParams Parameters for measure handles and edge detection.
         * @param row Row coordinate of center of rectangular template.
         * @param col Column coordinate of center of rectangular template.
         * @param phi Rotation of rectangular template in radians.
         * @param halfWidth Half width of rectangular template.
         * @param halfHeight Half height of rectangular template
         */
    RectObject(const MeasureParams &measureParams, int row, int col, double phi, int halfWidth, int halfHeight);

    int getRow() const { return row; }

    int getColumn() const { return column; }

    double getPhi() const { return phi; }

    int getHalfWidth() const { return halfWidth; }

    int getHalfHeight() const { return halfHeight; }

    const std::vector<cv::Point2d> &getRectPoints() const { return rectPoints; }

    /**
     * @return Get the list of shapes returned by the applyMetrologyModel method.
     */
    const std::vector<RectParams> &getMetrologyResults() { return metResults; }

    // Since the algorithm is quite different for rectangles, it is done as a separate method that's overriden.
    void applyMetrologyModel(const cv::Mat &img, double minScore, unsigned int numInstances,
                             double distThresh, int maxIters, unsigned int randSeed, bool allowOutsideRegion) override;
};


class ArcProjectionObject : public MetrologyObject {
private:
    int row;
    int column;
    int radius;

    double angleStart;
    double angleExtent;
    std::vector<CircleParams> metResults;

    std::vector<MeasureArcTransposed> measures;

    // Initialize the measure handles of the object around its perimeter.
    void initMeasures() override;

    // Find the number of measure handles needed around the perimeter
    int findNumMeasures();

    std::vector<int> getInliers(const std::vector<cv::Point2d> &fitPoints, const RansacPoints &testPoints,
                                double distThresh) override;

    bool addResult(const std::vector<cv::Point2d> &inliers, bool allowOutsideRegion) override;

public:
    /**
     * @brief Construct an ArcObject.
     * @param measureParams Parameters for measure handles and edge detection.
     * @param row Row coordinate of center.
     * @param col Column coordinate of center.
     * @param radius Radius of the circle or the arc.
     * @param angleStart Starting angle of the arc, measured from positive horizontal axis, counterclockwise. Angles are given in radians.
     * @param angleExtent Angular extent of the arc, in radians. If this value is negative, the arc is clockwise. Must be between -2pi and 2pi.
     */
    ArcProjectionObject(const MeasureParams &measureParams, int row, int col, int radius, double angleStart = 0.0,
                        double angleExtent = boost::math::double_constants::two_pi);

    void ArcProjectionObject::applyMeasures(const cv::Mat &img);

    // Getters
    int getRow() const { return row; }

    int getColumn() const { return column; }

    int getRadius() const { return radius; }

    double getAngleStart() const { return angleStart; }

    double getAngleExtent() const { return angleExtent; }

    std::vector<MeasureArcTransposed> getOtherMeasures() { return measures; }

    /**
     * @return Get the list of shapes returned by the applyMetrologyModel method.
     */
    const std::vector<CircleParams> &getMetrologyResults() { return metResults; }
};

#endif
