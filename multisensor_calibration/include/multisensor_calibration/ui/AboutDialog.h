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

#ifndef MULTISENSORCALIBRATION_UI_ABOUTDIALOG_H
#define MULTISENSORCALIBRATION_UI_ABOUTDIALOG_H

// Qt
#include <QAbstractButton>
#include <QDialog>
#include <QDir>
#include <QLabel>
#include <QSettings>
#include <Qt>

namespace multisensor_calibration
{

namespace Ui
{
class AboutDialog;
}

/**
 * @ingroup ui
 * @brief About dialog.
 *
 */
class AboutDialog : public QDialog
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Construct a new Calibration Config Dialog object.
     *
     * @param[in] parent Parent Widget
     */
    AboutDialog(QWidget* parent = nullptr);

    /**
     * @brief Destroy the Calibration Config Dialog object.
     */
    ~AboutDialog();

  private:
    /**
     * @brief Read and parse package xml file.
     *
     * This will read the contents into file and also parse the authors.
     */
    void readAndParsePackageXml();

    /**
     * @brief Read and parse changelog file.
     *
     * This will read the contents into file and also parse the contributors.
     */
    void readAndParseChangelog();

    /**
     * @brief Update version and year label.
     *
     */
    void updateVersionAndYearTag();

    /**
     * @brief Method to populate the text edit object with the dependencies
     *
     */
    void populateDependenciesTextEdit();

    /**
     * @brief Method to populate the text edit object with the changelog
     *
     */
    void populateChangelogTextEdit();

    /**
     * @brief Method to populate the text edit object with the developers
     *
     */
    void populateDevelopersTextEdit();

    /**
     * @brief Method to populate the text edit object with the contributors
     *
     */
    void populateContributorsTextEdit();

    /**
     * @brief Method to populate the text edit object with the License
     *
     */
    void populateLicenseTextEdit();

    //--- MEMBER DECLARATION ---//

  private:
    /// Pointer to UI
    Ui::AboutDialog* ui;

    /// List of developers name, extracted from package.xml
    QStringList developers_;

    /// List of developers name, extracted from changelog.rst
    QStringList contributors_;

    /// String holding contents of changelog.
    QString changeLogContents_;
};

} // namespace multisensor_calibration
#endif // MULTISENSORCALIBRATION_UI_ABOUTDIALOG_H
