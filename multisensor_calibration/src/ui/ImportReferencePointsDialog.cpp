/***********************************************************************
*
* Copyright (c) Fraunhofer Institute of Optronics,
* System Technologies and Image Exploitation IOSB
*
**********************************************************************/
/***********************************************************************
*
* Implementation of the dialog to assign CSV reference points to target
* poses and markers, including validation of the assignments.
*
**********************************************************************/

#include "multisensor_calibration/ui/ImportReferencePointsDialog.h"

// Std
#include <algorithm>
#include <array>
#include <map>
#include <utility>

// Qt
#include <QBrush>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTableWidget>

// multisensor_calibration
#include "multisensor_calibration/ui/PositiveIntegerCellDelegate.h"
#include "ui_ImportReferencePointsDialog.h"

namespace multisensor_calibration
{

namespace
{

enum ImportTableColumn
{
    IMPORT_COL_USE = 0,
    IMPORT_COL_NAME,
    IMPORT_COL_X,
    IMPORT_COL_Y,
    IMPORT_COL_Z,
    IMPORT_COL_POSE_ID,
    IMPORT_COL_MARKER_ID,
    IMPORT_COL_STATUS
};

// The calibration node requires exactly these markers per pose
// (see CalibrationTarget::computePoseFromTopLeftMarkerCorners).
constexpr std::array<int, 4> REQUIRED_MARKER_IDS = {1, 2, 3, 4};

QTableWidgetItem* createReadOnlyItem(const QString& text, bool isNumber)
{
    QTableWidgetItem* item = new QTableWidgetItem(text);
    item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    if (isNumber)
        item->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    return item;
}

} // namespace

//==================================================================================================
ImportReferencePointsDialog::ImportReferencePointsDialog(const std::vector<ReferencePoint>& iPoints,
                                                         QWidget* parent) :
  QDialog(parent),
  pUi_(new Ui::ImportReferencePointsDialog),
  points_(iPoints)
{
    pUi_->setupUi(this);

    QTableWidget* table = pUi_->pointsTableWidget;
    table->setItemDelegateForColumn(IMPORT_COL_POSE_ID, new PositiveIntegerCellDelegate(table));
    table->setItemDelegateForColumn(IMPORT_COL_MARKER_ID, new PositiveIntegerCellDelegate(table));

    table->setRowCount(static_cast<int>(points_.size()));
    for (int r = 0; r < static_cast<int>(points_.size()); ++r)
    {
        const ReferencePoint& point = points_[r];

        QTableWidgetItem* useItem = new QTableWidgetItem();
        useItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
        useItem->setCheckState(Qt::Unchecked);
        table->setItem(r, IMPORT_COL_USE, useItem);

        table->setItem(r, IMPORT_COL_NAME,
                       createReadOnlyItem(QString::fromStdString(point.name), false));
        table->setItem(r, IMPORT_COL_X, createReadOnlyItem(QString::number(point.x, 'g', 10), true));
        table->setItem(r, IMPORT_COL_Y, createReadOnlyItem(QString::number(point.y, 'g', 10), true));
        table->setItem(r, IMPORT_COL_Z, createReadOnlyItem(QString::number(point.z, 'g', 10), true));

        for (int c : {IMPORT_COL_POSE_ID, IMPORT_COL_MARKER_ID})
        {
            QTableWidgetItem* idItem = new QTableWidgetItem();
            idItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
            table->setItem(r, c, idItem);
        }

        table->setItem(r, IMPORT_COL_STATUS, createReadOnlyItem("", false));
    }
    table->resizeColumnsToContents();

    connect(table, &QTableWidget::itemChanged,
            this, &ImportReferencePointsDialog::handleTableWidgetItemChanged);

    validateAssignments();
}

//==================================================================================================
ImportReferencePointsDialog::~ImportReferencePointsDialog()
{
    delete pUi_;
}

//==================================================================================================
std::vector<ReferencePointAssignment> ImportReferencePointsDialog::getAssignedPoints() const
{
    std::vector<ReferencePointAssignment> assignments;

    const QTableWidget* table = pUi_->pointsTableWidget;
    for (int r = 0; r < table->rowCount(); ++r)
    {
        if (table->item(r, IMPORT_COL_USE)->checkState() != Qt::Checked)
            continue;

        assignments.push_back({table->item(r, IMPORT_COL_POSE_ID)->text().toInt(),
                               table->item(r, IMPORT_COL_MARKER_ID)->text().toInt(),
                               points_[r]});
    }

    std::sort(assignments.begin(), assignments.end(),
              [](const ReferencePointAssignment& a, const ReferencePointAssignment& b)
              {
                  return std::make_pair(a.poseId, a.markerId) <
                         std::make_pair(b.poseId, b.markerId);
              });

    return assignments;
}

//==================================================================================================
void ImportReferencePointsDialog::handleTableWidgetItemChanged(QTableWidgetItem* item)
{
    if (item->column() == IMPORT_COL_POSE_ID || item->column() == IMPORT_COL_MARKER_ID)
    {
        QTableWidget* table = pUi_->pointsTableWidget;
        const int row       = item->row();
        const bool isFilled = !table->item(row, IMPORT_COL_POSE_ID)->text().isEmpty() &&
                              !table->item(row, IMPORT_COL_MARKER_ID)->text().isEmpty();

        QSignalBlocker blocker(table);
        table->item(row, IMPORT_COL_USE)->setCheckState(isFilled ? Qt::Checked : Qt::Unchecked);
    }

    validateAssignments();
}

//==================================================================================================
void ImportReferencePointsDialog::validateAssignments()
{
    QTableWidget* table = pUi_->pointsTableWidget;
    const int rowCount  = table->rowCount();

    std::vector<QString> rowErrors(rowCount);
    std::map<std::pair<int, int>, std::vector<int>> rowsByPoseAndMarker;
    std::map<int, std::vector<int>> rowsByPose;
    int numTicked = 0;

    for (int r = 0; r < rowCount; ++r)
    {
        if (table->item(r, IMPORT_COL_USE)->checkState() != Qt::Checked)
            continue;
        ++numTicked;

        const QString poseStr   = table->item(r, IMPORT_COL_POSE_ID)->text();
        const QString markerStr = table->item(r, IMPORT_COL_MARKER_ID)->text();
        if (poseStr.isEmpty() || markerStr.isEmpty())
        {
            rowErrors[r] = "Pose ID or Marker ID missing";
            continue;
        }

        const int poseId = poseStr.toInt();
        if (poseId < 1)
        {
            rowErrors[r] = "Pose ID must be 1 or greater";
            continue;
        }

        const int markerId = markerStr.toInt();
        if (std::find(REQUIRED_MARKER_IDS.begin(), REQUIRED_MARKER_IDS.end(), markerId) ==
            REQUIRED_MARKER_IDS.end())
        {
            rowErrors[r] = "Marker ID must be 1, 2, 3 or 4";
            continue;
        }

        rowsByPoseAndMarker[{poseId, markerId}].push_back(r);
        rowsByPose[poseId].push_back(r);
    }

    for (const auto& [poseAndMarker, rows] : rowsByPoseAndMarker)
    {
        if (rows.size() > 1)
        {
            for (int r : rows)
                rowErrors[r] = "Duplicate Pose ID / Marker ID";
        }
    }

    QStringList incompletePoses;
    for (const auto& [poseId, rows] : rowsByPose)
    {
        if (rows.size() < REQUIRED_MARKER_IDS.size())
            incompletePoses << QString::number(poseId);
    }

    //--- update status column
    int numErrors = 0;
    QSignalBlocker blocker(table);
    for (int r = 0; r < rowCount; ++r)
    {
        QTableWidgetItem* statusItem = table->item(r, IMPORT_COL_STATUS);
        const bool isTicked          = table->item(r, IMPORT_COL_USE)->checkState() == Qt::Checked;

        if (!rowErrors[r].isEmpty())
        {
            ++numErrors;
            statusItem->setText(rowErrors[r]);
            statusItem->setForeground(QBrush(Qt::red));
        }
        else
        {
            statusItem->setText(isTicked ? "OK" : "");
            statusItem->setForeground(QBrush());
        }
    }
    blocker.unblock();

    // Pose IDs are unique, sorted and >= 1, thus contiguous from 1 iff the last ID equals the count.
    const bool arePosesContiguous =
      rowsByPose.empty() || rowsByPose.rbegin()->first == static_cast<int>(rowsByPose.size());

    //--- summary
    QString summary = QString("%1 of %2 points selected.").arg(numTicked).arg(rowCount);
    if (numErrors > 0)
        summary += QString(" <span style='color:red;'>%1 row(s) with errors.</span>").arg(numErrors);
    if (!arePosesContiguous)
        summary += "<br/><span style='color:#c87800;'>Pose IDs are not contiguous starting from 1. "
                   "When applying, observations after the first gap will not be submitted.</span>";
    if (!incompletePoses.isEmpty())
        summary += QString("<br/><span style='color:#c87800;'>Pose(s) %1 do not have all markers "
                           "1, 2, 3 and 4. Complete them in the table before applying, otherwise "
                           "they are rejected by the calibration node.</span>")
                     .arg(incompletePoses.join(", "));
    pUi_->summaryLabel->setText(summary);

    pUi_->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(numTicked > 0 && numErrors == 0);
}

} // namespace multisensor_calibration
