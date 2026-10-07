/***********************************************************************
*
* Copyright (c) Fraunhofer Institute of Optronics,
* System Technologies and Image Exploitation IOSB
*
**********************************************************************/
/***********************************************************************
*
* Dialog to assign reference points read from a CSV file to target pose
* and marker IDs before they are entered as marker observations.
*
**********************************************************************/

#ifndef MULTISENSORCALIBRATION_UI_IMPORTREFERENCEPOINTSDIALOG_H
#define MULTISENSORCALIBRATION_UI_IMPORTREFERENCEPOINTSDIALOG_H

// Std
#include <vector>

// Qt
#include <QDialog>

// multisensor_calibration
#include "../io/ReferencePointsCsvReader.h"

class QTableWidgetItem;

namespace multisensor_calibration
{

namespace Ui
{
class ImportReferencePointsDialog;
}

/**
 * @ingroup ui
 * @brief Reference point with the target pose and marker it has been assigned to.
 */
struct ReferencePointAssignment
{
    int poseId;
    int markerId;
    ReferencePoint point;
};

/**
 * @ingroup ui
 * @brief Dialog listing reference points read from a CSV file, in which the user assigns
 * target pose and marker IDs to the points that are to be imported.
 */
class ImportReferencePointsDialog : public QDialog
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//

  public:
    /**
     * @brief Constructor
     *
     * @param[in] iPoints Points to list in the dialog.
     * @param[in] parent Parent widget.
     */
    ImportReferencePointsDialog(const std::vector<ReferencePoint>& iPoints,
                                QWidget* parent = nullptr);

    /**
     * @brief Destructor
     */
    ~ImportReferencePointsDialog() override;

    /**
     * @brief Get the ticked points with their assignment, sorted by pose ID and marker ID.
     */
    std::vector<ReferencePointAssignment> getAssignedPoints() const;

  private slots:

    /**
     * @brief Handle QT signal emitted by QTableWidget whenever the data of an item has changed.
     */
    void handleTableWidgetItemChanged(QTableWidgetItem* item);

  private:
    /**
     * @brief Validate the assignments, update status column and summary, and enable or disable
     * the OK button.
     */
    void validateAssignments();

    //--- MEMBER DECLARATION ---//

  private:
    /// Pointer to UI
    Ui::ImportReferencePointsDialog* pUi_;

    /// Points listed in the table, one per row.
    std::vector<ReferencePoint> points_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_UI_IMPORTREFERENCEPOINTSDIALOG_H
