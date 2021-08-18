/**
 * @file metcout.hpp
 * @brief std::cout overloads to accept Metrology related structs and objects.
 */

#ifndef METROLOGY_COUT_OVERLOADS_HPP
#define METROLOGY_COUT_OVERLOADS_HPP

#include "metrologyObject.hpp"

/**
 * @fn std::ostream &operator<<(std::ostream &out, const MeasureParams measureParams)
 * @brief Overloads cout to accept a MeasureParams object
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param mP The MeasureParams object to print
 */
std::ostream &operator<<(std::ostream &o, const MeasureParams &mP);

/**
 * @fn std::ostream &operator<<(std::ostream &o, const ArcObject &obj)
 * @brief Overloads cout to accept an ArcOBject
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param obj The ArcObject object to print
 */
std::ostream &operator<<(std::ostream &o, const ArcObject &obj);

/**
 * @fn std::ostream &operator<<(std::ostream &o, const EllipseObject &obj)
 * @brief Overloads cout to accept an EllipseObject
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param obj The EllipseObject object to print
 */
std::ostream &operator<<(std::ostream &o, const EllipseObject &obj);

/**
 * @fn std::ostream &operator<<(std::ostream &o, const LineObject &obj)
 * @brief Overloads cout to accept an LineObject
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param obj The LineObject object to print
 */
std::ostream &operator<<(std::ostream &o, const LineObject &obj);

/**
 * @fn std::ostream &operator<<(std::ostream &o, const RectObject &obj)
 * @brief Overloads cout to accept an RectObject
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param obj The RectObject object to print
 */
std::ostream &operator<<(std::ostream &o, const RectObject &obj);

/**
 * @fn std::ostream &operator<<(std::ostream &o, const CircleParams &obj)
 * @brief Overloads cout to accept CircleParams
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param obj The CircleParams object to print
 */
std::ostream &operator<<(std::ostream &o, const CircleParams &obj);

/**
 * @fn std::ostream &operator<<(std::ostream &o, const EllipseParams &obj)
 * @brief Overloads cout to accept EllipseParams
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param obj The EllipseParams object to print
 */
std::ostream &operator<<(std::ostream &o, const EllipseParams &obj);

/**
 * @fn std::ostream &operator<<(std::ostream &o, const LineParams &obj)
 * @brief Overloads cout to accept LineParams
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param obj The LineParams object to print
 */
std::ostream &operator<<(std::ostream &o, const LineParams &obj);

/**
 * @fn std::ostream &operator<<(std::ostream &o, const LineEqParams &obj)
 * @brief Overloads cout to accept LineEqParams
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param obj The LineEqParams object to print
 */
std::ostream &operator<<(std::ostream &o, const LineEqParams &obj);

/**
 * @fn std::ostream &operator<<(std::ostream &o, const RectParams &obj)
 * @brief Overloads cout to accept RectParams
 * @details Prints each parameter of the object on tab-indented new lines
 * 
 * @param o The ostream object
 * @param obj The RectParams object to print
 */
std::ostream &operator<<(std::ostream &o, const RectParams &obj);

#endif