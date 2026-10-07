/***********************************************************************
*
* Copyright (c) Fraunhofer Institute of Optronics,
* System Technologies and Image Exploitation IOSB
*
**********************************************************************/
/***********************************************************************
*
* Parsing of reference points CSV files (name, x, y, z per row).
*
**********************************************************************/

#include "multisensor_calibration/io/ReferencePointsCsvReader.h"

// Std
#include <charconv>
#include <cmath>
#include <fstream>

namespace multisensor_calibration
{

namespace
{

std::string trim(const std::string& str)
{
    const char* WHITESPACE = " \t\r\n";
    const size_t first     = str.find_first_not_of(WHITESPACE);
    if (first == std::string::npos)
        return "";
    const size_t last = str.find_last_not_of(WHITESPACE);
    return str.substr(first, last - first + 1);
}

std::vector<std::string> split(const std::string& line, char delimiter)
{
    std::vector<std::string> fields;
    size_t start = 0;
    size_t pos;
    while ((pos = line.find(delimiter, start)) != std::string::npos)
    {
        fields.push_back(trim(line.substr(start, pos - start)));
        start = pos + 1;
    }
    fields.push_back(trim(line.substr(start)));
    return fields;
}

// std::from_chars instead of std::stod, since the latter depends on the C locale, which Qt sets
// from the environment (e.g. ',' as decimal separator for de_DE).
bool parseDouble(const std::string& str, double& oValue)
{
    if (str.empty())
        return false;
    const char* end = str.data() + str.size();
    auto result     = std::from_chars(str.data(), end, oValue);
    return result.ec == std::errc() && result.ptr == end && std::isfinite(oValue);
}

char detectDelimiter(const std::string& line)
{
    // ';' and tab take precedence, since ',' may also appear as decimal separator.
    for (char candidate : {'\t', ';'})
    {
        if (line.find(candidate) != std::string::npos)
            return candidate;
    }
    return ',';
}

bool parseRow(const std::string& line, char delimiter, ReferencePoint& oPoint)
{
    std::vector<std::string> fields = split(line, delimiter);
    if (fields.size() < 4 || fields[0].empty())
        return false;

    oPoint.name = fields[0];
    return parseDouble(fields[1], oPoint.x) &&
           parseDouble(fields[2], oPoint.y) &&
           parseDouble(fields[3], oPoint.z);
}

} // namespace

//==================================================================================================
bool readReferencePointsFromCsv(const std::filesystem::path& iFilePath,
                                ReferencePointsCsvContent& oContent)
{
    oContent = ReferencePointsCsvContent();

    std::ifstream file(iFilePath);
    if (!file.is_open())
        return false;

    const std::string UTF8_BOM = "\xEF\xBB\xBF";

    std::string line;
    int lineNumber       = 0;
    char delimiter       = ',';
    bool isFirstDataLine = true;
    while (std::getline(file, line))
    {
        ++lineNumber;

        if (lineNumber == 1 && line.compare(0, UTF8_BOM.size(), UTF8_BOM) == 0)
            line.erase(0, UTF8_BOM.size());

        if (trim(line).empty())
            continue;

        if (isFirstDataLine)
            delimiter = detectDelimiter(line);

        ReferencePoint point;
        if (parseRow(line, delimiter, point))
            oContent.points.push_back(point);
        else if (!isFirstDataLine)
            oContent.invalidLines.push_back(lineNumber);

        isFirstDataLine = false;
    }

    return true;
}

} // namespace multisensor_calibration
