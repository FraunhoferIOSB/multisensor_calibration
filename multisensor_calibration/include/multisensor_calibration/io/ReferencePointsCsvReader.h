/***********************************************************************
*
* Copyright (c) Fraunhofer Institute of Optronics,
* System Technologies and Image Exploitation IOSB
*
**********************************************************************/
/***********************************************************************
*
* Reader for CSV files holding named 3D reference points (name, x, y, z
* per row).
*
**********************************************************************/

#ifndef MULTISENSORCALIBRATION_IO_REFERENCEPOINTSCSVREADER_H
#define MULTISENSORCALIBRATION_IO_REFERENCEPOINTSCSVREADER_H

// Std
#include <filesystem>
#include <string>
#include <vector>

namespace multisensor_calibration
{

/**
 * @ingroup io
 * @brief Named 3D point read from a CSV file.
 */
struct ReferencePoint
{
    /// Point name / number as given in the first column.
    std::string name;

    /// Coordinates as given in the second to fourth column.
    double x;
    double y;
    double z;
};

/**
 * @ingroup io
 * @brief Content of a reference points CSV file.
 */
struct ReferencePointsCsvContent
{
    /// Successfully parsed points in file order.
    std::vector<ReferencePoint> points;

    /// 1-based line numbers of rows that could not be parsed.
    std::vector<int> invalidLines;
};

/**
 * @ingroup io
 * @brief Read reference points from a CSV file.
 *
 * Each row is expected to hold at least 4 fields: name, x, y, z. Further fields are ignored.
 * The delimiter (tab, ';' or ',') is detected from the first non-empty line. This line is
 * treated as header if its coordinates are not numeric. Numbers must use '.' as decimal
 * separator. Empty lines are skipped.
 *
 * @param[in] iFilePath Path to the CSV file.
 * @param[out] oContent Parsed points and line numbers of invalid rows.
 * @return False, if the file could not be opened.
 */
bool readReferencePointsFromCsv(const std::filesystem::path& iFilePath,
                                ReferencePointsCsvContent& oContent);

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_IO_REFERENCEPOINTSCSVREADER_H
