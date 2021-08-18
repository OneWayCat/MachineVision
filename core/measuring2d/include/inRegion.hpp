/**
 * @file inRegion.hpp
 * @brief Functions for inside region checking.
 */

#ifndef METROLOGY_IN_REGION_HPP
#define METROLOGY_IN_REGION_HPP

#include "metrologyObject.hpp"
#include "ransacFit.hpp"

/**
 * @brief Determines if a predicted shape is within the MeasureHandle constraints of the ArcObject.
 * @details Performs an intersection test between the shape and the profile lines of all of the MeasureHandles of the ArcObject.
 * The shape is considered "inside" the ArcObject "region" ONLY IF it intersects ALL of the MeasureHandles.
 * 
 * @param arcObj The ArcObject to test region inclusion with.
 * @param arcShape The predicted circle being tested.
 * @return True if shape is "inside the region".
 */
bool inArc(const ArcObject &arcObj, const CircleParams &arcShape);
bool inArc(const ArcProjectionObject &arcObj, const CircleParams &arcShape);

/**
 * @brief Determines if a predicted shape is within the MeasureHandle constraints of the EllipseObject.
 * @details Performs an intersection test between the shape and the profile lines of all of the MeasureHandles of the EllipseObject.
 * The shape is considered "inside" the EllipseObject "region" ONLY IF it intersects ALL of the MeasureHandles.
 * 
 * @param ellipseObj The EllipseObject to test region inclusion with.
 * @param ellipseShape The predicted ellipse being tested.
 * @return True if shape is "inside the region".
 */
bool inEllipse(const EllipseObject &ellipseObj, const EllipseParams &ellipseShape);

/**
 * @brief Determines if a predicted shape is within the MeasureHandle constraints of the LineObject.
 * @details Performs an intersection test between the shape and the profile lines of all of the MeasureHandles of the LineObject.
 * The shape is considered "inside" the LineObject "region" ONLY IF it intersects ALL of the MeasureHandles.
 * 
 * @param lineObj The LineObject to test region inclusion with.
 * @param lineShape The predicted line being tested.
 * @return True if shape is "inside the region".
 */
bool inLine(const LineObject &lineObj, const LineParams &lineShape);

/**
 * @brief Determines if a predicted shape is within the MeasureHandle constraints of the RectObject.
 * @details Performs an intersection test between the shape and the profile lines of all of the MeasureHandles of the RectObject.
 * The shape is considered "inside" the RectObject "region" ONLY IF it intersects ALL of the MeasureHandles.
 * 
 * @param rectObj The RectObject to test region inclusion with.
 * @param rectShape The predicted rectangle being tested.
 * @return True if shape is "inside the region".
 */
bool inRect(const RectObject &rectObj, const RectParams &rectShape);

#endif