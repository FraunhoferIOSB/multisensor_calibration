// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Std
#include <regex>

// Qt
#include <QDate>
#include <QTextStream>
#include <QXmlStreamReader>

// OpenCV
#include <opencv2/core/version.hpp>

// PCL
#include <pcl/pcl_config.h>

// Multisensor Calibration
#include "multisensor_calibration/ui/AboutDialog.h"
#include "ui_AboutDialog.h"

namespace multisensor_calibration
{

//==================================================================================================
AboutDialog::AboutDialog(QWidget* parent) :
  QDialog(parent),
  ui(new Ui::AboutDialog)
{
    //--- set up UI
    ui->setupUi(this);
    ui->titleIconLabel->setPixmap(QIcon(":/icons/icons8-sensor-100.png").pixmap(50, 50));
    this->setWindowIcon(QIcon(":/icons/icons8-sensor-100_filled.png"));

    //--- read and parse resource files
    readAndParsePackageXml();
    readAndParseChangelog();

    //--- populate content
    updateVersionAndYearTag();
    populateDependenciesTextEdit();
    populateChangelogTextEdit();
    populateDevelopersTextEdit();
    populateContributorsTextEdit();
    populateLicenseTextEdit();
}

//==================================================================================================
AboutDialog::~AboutDialog()
{
    delete ui;
}

//==================================================================================================
void AboutDialog::readAndParsePackageXml()
{
    QFile pFile(":/package.xml");
    if (!pFile.open(QIODevice::ReadOnly | QIODevice::Text))
        return;

    QXmlStreamReader xmlReader(&pFile);
    QString previousElement;
    while (!xmlReader.atEnd())
    {
        xmlReader.readNext();

        if (!xmlReader.isStartElement())
            continue;

        QString tagName = xmlReader.name().toString();
        if (tagName == QStringLiteral("author"))
            developers_.append(xmlReader.readElementText());
    }
}

//==================================================================================================
void AboutDialog::readAndParseChangelog()
{
    QFile clFile(":/CHANGELOG.rst");
    if (!clFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        changeLogContents_ = "n/a";
        return;
    }

    std::regex contributorsRegex(R"(^Contributors:\s*(.+))");
    std::regex nameRegex(R"(\s*([^,]+)\s*,?)");
    std::smatch match;

    QTextStream in(&clFile);
    while (!in.atEnd())
    {
        QString line = in.readLine();
        changeLogContents_ += (line + "\n");

        //--- check for 'Contributors:' and extract names
        std::string lineStr = line.toStdString();
        if (std::regex_search(lineStr, match, contributorsRegex))
        {
            std::string namesPart = match[1].str(); // everything after "Contributors:"

            // extract comma-separated names
            std::sregex_iterator it(namesPart.begin(), namesPart.end(), nameRegex);
            std::sregex_iterator end;

            while (it != end)
            {
                QString name = QString::fromStdString(it->str(1));
                if (!developers_.contains(name) && !contributors_.contains(name))
                    contributors_.append(name);

                ++it;
            }
        }
    }
}

//==================================================================================================
void AboutDialog::updateVersionAndYearTag()
{
    //--- version
    QString presetLabelText = ui->versionLabel->text();
    QString newLabelText    = presetLabelText.arg(QString::number(VERSION_MAJOR))
                             .arg(QString::number(VERSION_MINOR))
                             .arg(QString::number(VERSION_PATCH));

    ui->versionLabel->setText(newLabelText);

    //--- year
    int currentYear = QDate::currentDate().year();

    presetLabelText = ui->copyrightLabel->text();
    newLabelText    = presetLabelText.arg(("2024 - " + QString::number(currentYear)));

    ui->copyrightLabel->setText(newLabelText);
}

//==================================================================================================
void AboutDialog::populateDependenciesTextEdit()
{
    QString contents;

    contents.append(QString("ROS\t\t%1\n").arg(QString(std::getenv("ROS_DISTRO"))));
    contents.append(QString("Qt\t\t%1\n").arg(QString(qVersion())));
    contents.append(QString("OpenCV\t\t%1.%2.%3\n")
                      .arg(QString::number(CV_VERSION_MAJOR))
                      .arg(QString::number(CV_VERSION_MINOR))
                      .arg(QString::number(CV_VERSION_REVISION)));
    contents.append(QString("PCL\t\t%1.%2.%3\n")
                      .arg(QString::number(PCL_MAJOR_VERSION))
                      .arg(QString::number(PCL_MINOR_VERSION))
                      .arg(QString::number(PCL_REVISION_VERSION)));
#ifdef WITH_OPENMP
    contents.append(QString("OpenMP\n"));
#endif

#ifdef SMALL_GICP_HEAD_COMMIT_HASH
    QString small_gicp_head_commit = QString(SMALL_GICP_HEAD_COMMIT_HASH);
    contents.append(QString("small_gicp\t\t%1\n")
                      .arg(small_gicp_head_commit.mid(small_gicp_head_commit.size() - 8)));
#else
    contents.append(QString("small_gicp\n"));
#endif

    ui->dependenciesTextEdit->setText(contents);
}

//==================================================================================================
void AboutDialog::populateChangelogTextEdit()
{
    ui->changelogTextEdit->setText(changeLogContents_);
}

//==================================================================================================
void AboutDialog::populateDevelopersTextEdit()
{
    QString contents;
    for (QString name : developers_)
        contents += (name + "\n");
    ui->developerTextEdit->setText(contents);
}

//==================================================================================================
void AboutDialog::populateContributorsTextEdit()
{
    QString contents;
    for (QString name : contributors_)
        contents += (name + "\n");
    ui->contributorTextEdit->setText(contents);
}

//==================================================================================================
void AboutDialog::populateLicenseTextEdit()
{
    QFile lFile(":/LICENSE");
    if (!lFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        ui->licenseTextEdit->setText("n/a");
        return;
    }

    QTextStream in(&lFile);
    QString contents;
    while (!in.atEnd())
    {
        QString line = in.readLine();
        contents += (line + "\n");
    }

    ui->licenseTextEdit->setText(contents);
}

} // namespace multisensor_calibration
