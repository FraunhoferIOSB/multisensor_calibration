/***********************************************************************
 *
 * Copyright (c) Fraunhofer Institute of Optronics,
 * System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/
/***********************************************************************
 *
 * Unit tests for readReferencePointsFromCsv.
 *
 **********************************************************************/

// Std
#include <filesystem>
#include <fstream>
#include <string>

// GTest
#include <gtest/gtest.h>

// multisensor_calibration
#include <multisensor_calibration/io/ReferencePointsCsvReader.h>

namespace fs = std::filesystem;
using multisensor_calibration::readReferencePointsFromCsv;
using multisensor_calibration::ReferencePointsCsvContent;

class ReferencePointsCsvReaderTest : public ::testing::Test
{
  protected:
    void SetUp() override
    {
        tmpDir_ = fs::temp_directory_path() /
                  ("msc_csv_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
                   "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
        fs::create_directories(tmpDir_);
    }

    void TearDown() override
    {
        fs::remove_all(tmpDir_);
    }

    fs::path writeFile(const std::string& iContent) const
    {
        fs::path filePath = tmpDir_ / "points.csv";
        std::ofstream file(filePath);
        file << iContent;
        return filePath;
    }

    fs::path tmpDir_;
};

TEST_F(ReferencePointsCsvReaderTest, NonExistentFileReturnsFalse)
{
    ReferencePointsCsvContent content;
    EXPECT_FALSE(readReferencePointsFromCsv(tmpDir_ / "does_not_exist.csv", content));
}

TEST_F(ReferencePointsCsvReaderTest, CommaSeparatedWithHeader)
{
    ReferencePointsCsvContent content;
    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("name,x,y,z\n"
                "P1,1.0,2.0,3.0\n"
                "P2,-0.5,0.25,10\n"),
      content));

    ASSERT_EQ(content.points.size(), 2u);
    EXPECT_TRUE(content.invalidLines.empty());
    EXPECT_EQ(content.points[0].name, "P1");
    EXPECT_DOUBLE_EQ(content.points[0].x, 1.0);
    EXPECT_DOUBLE_EQ(content.points[0].y, 2.0);
    EXPECT_DOUBLE_EQ(content.points[0].z, 3.0);
    EXPECT_EQ(content.points[1].name, "P2");
    EXPECT_DOUBLE_EQ(content.points[1].x, -0.5);
    EXPECT_DOUBLE_EQ(content.points[1].y, 0.25);
    EXPECT_DOUBLE_EQ(content.points[1].z, 10.0);
}

TEST_F(ReferencePointsCsvReaderTest, SemicolonAndTabDelimiters)
{
    ReferencePointsCsvContent content;
    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("P1;1.0;2.0;3.0\n"), content));
    ASSERT_EQ(content.points.size(), 1u);
    EXPECT_DOUBLE_EQ(content.points[0].y, 2.0);

    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("P1\t4.0\t5.0\t6.0\n"), content));
    ASSERT_EQ(content.points.size(), 1u);
    EXPECT_DOUBLE_EQ(content.points[0].z, 6.0);
}

TEST_F(ReferencePointsCsvReaderTest, HeaderWithoutNumericCoordinatesIsNotInvalid)
{
    ReferencePointsCsvContent content;
    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("id,east,north,height\n"
                "P1,1,2,3\n"),
      content));
    ASSERT_EQ(content.points.size(), 1u);
    EXPECT_TRUE(content.invalidLines.empty());
}

TEST_F(ReferencePointsCsvReaderTest, FirstLineWithNumericCoordinatesIsParsedAsPoint)
{
    ReferencePointsCsvContent content;
    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("P1,1,2,3\n"
                "P2,4,5,6\n"),
      content));
    ASSERT_EQ(content.points.size(), 2u);
}

TEST_F(ReferencePointsCsvReaderTest, InvalidLinesAreReportedOneBased)
{
    ReferencePointsCsvContent content;
    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("name,x,y,z\n"
                "P1,1,2,3\n"
                "P2,not_a_number,2,3\n"
                "P3,1,2\n"
                "P4,4,5,6\n"),
      content));

    ASSERT_EQ(content.points.size(), 2u);
    ASSERT_EQ(content.invalidLines.size(), 2u);
    EXPECT_EQ(content.invalidLines[0], 3);
    EXPECT_EQ(content.invalidLines[1], 4);
}

TEST_F(ReferencePointsCsvReaderTest, EmptyLinesAreSkipped)
{
    ReferencePointsCsvContent content;
    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("\n"
                "name,x,y,z\n"
                "\n"
                "P1,1,2,3\n"
                "   \n"),
      content));
    ASSERT_EQ(content.points.size(), 1u);
    EXPECT_TRUE(content.invalidLines.empty());
}

TEST_F(ReferencePointsCsvReaderTest, Utf8BomIsIgnored)
{
    ReferencePointsCsvContent content;
    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("\xEF\xBB\xBFP1,1,2,3\n"), content));
    ASSERT_EQ(content.points.size(), 1u);
    EXPECT_EQ(content.points[0].name, "P1");
}

TEST_F(ReferencePointsCsvReaderTest, ExtraFieldsAreIgnored)
{
    ReferencePointsCsvContent content;
    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("P1,1,2,3,extra,fields\n"), content));
    ASSERT_EQ(content.points.size(), 1u);
    EXPECT_DOUBLE_EQ(content.points[0].z, 3.0);
}

TEST_F(ReferencePointsCsvReaderTest, FieldsAreTrimmed)
{
    ReferencePointsCsvContent content;
    ASSERT_TRUE(readReferencePointsFromCsv(
      writeFile("  P1 , 1.5 , 2.5 , 3.5 \r\n"), content));
    ASSERT_EQ(content.points.size(), 1u);
    EXPECT_EQ(content.points[0].name, "P1");
    EXPECT_DOUBLE_EQ(content.points[0].x, 1.5);
}
