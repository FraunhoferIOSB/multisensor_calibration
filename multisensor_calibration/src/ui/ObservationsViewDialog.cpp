/***********************************************************************
 *
 *   Copyright (c) 2022 - 2024 Fraunhofer Institute of Optronics,
 *   System Technologies and Image Exploitation IOSB
 *
 **********************************************************************/

#include "multisensor_calibration/ui/ObservationsViewDialog.h"

// Qt
#include <QAction>
#include <QDialogButtonBox>
#include <QInputDialog>
#include <QItemDelegate>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QRegExpValidator>

// multisensor_calibration
#include "multisensor_calibration/ui/CalibrationGuiBase.h"
#include "ui_ObservationsViewDialog.h"
#include <multisensor_calibration/common/utils.hpp>
#include <multisensor_calibration_interface/srv/add_marker_observations.hpp>

// TODO: Remove static decleration and get marker IDs in item delegate from target configuration, if still applicable.
static std::vector<int> MARKER_IDS_FOR_ITEM_DELEGATE = {1, 2, 3, 4};

using namespace multisensor_calibration_interface::srv;
namespace multisensor_calibration
{

/**
 * @brief Delegate class to create cells with a validator that only accepts positive integers
 */
class PositiveIntegerCellDelegate : public QItemDelegate
{
  public:
    PositiveIntegerCellDelegate() = delete;

    PositiveIntegerCellDelegate(QObject* parent) :
      QItemDelegate(parent)
    {
    }

    virtual ~PositiveIntegerCellDelegate()
    {
    }

  protected:
    QWidget* createEditor(QWidget* parent,
                          const QStyleOptionViewItem& option,
                          const QModelIndex& index) const
    {
        Q_UNUSED(option)
        Q_UNUSED(index)

        QLineEdit* lineEdit = new QLineEdit(parent);
        lineEdit->setLocale(QLocale::English);
        lineEdit->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        QRegExp rx("^\\d*$");
        lineEdit->setValidator(new QRegExpValidator(rx, lineEdit));
        return lineEdit;
    }
};

/**
 * @brief Delegate class to create cells holding the target pose ID.
 * In this the cells are filled with an initial ID gues based on the row index of the cell
 */
class TargetPoseIdCellDelegate : public PositiveIntegerCellDelegate
{
  public:
    TargetPoseIdCellDelegate() = delete;

    TargetPoseIdCellDelegate(QObject* parent) :
      PositiveIntegerCellDelegate(parent)
    {
    }

    virtual ~TargetPoseIdCellDelegate()
    {
    }

  protected:
    void setEditorData(QWidget* editor, const QModelIndex& index) const
    {
        QLineEdit* lineEdit = dynamic_cast<QLineEdit*>(editor);

        //--- add preset value for target pose
        //--- This is done by assuming that the poses are entered in a contigious order starting
        //--- from 1. Thus the pose index is the result of a integer division of the row by
        //--- 4 (number of markers) and adding 1.
        if (lineEdit)
            lineEdit->setText(QString::number((index.row() / 4) + 1));
    }
};

/**
 * @brief Delegate class to create cells holding the target pose ID.
 * In this the cells are filled with an initial ID guess based on the row index of the cell
 */
class MarkerIdCellDelegate : public PositiveIntegerCellDelegate
{
  public:
    MarkerIdCellDelegate() = delete;

    MarkerIdCellDelegate(QObject* parent) :
      PositiveIntegerCellDelegate(parent)
    {
    }

    virtual ~MarkerIdCellDelegate()
    {
    }

  protected:
    void setEditorData(QWidget* editor, const QModelIndex& index) const
    {
        QLineEdit* lineEdit = dynamic_cast<QLineEdit*>(editor);

        //--- add preset value for marker id
        //--- This is done by assuming that the poses are entered in a contigious order starting
        //--- from 1. Thus the pose index is the result of a integer division of the row by
        //--- 4 (number of markers) and adding 1.
        if (lineEdit)
            lineEdit->setText(QString::number(MARKER_IDS_FOR_ITEM_DELEGATE[(index.row() % 4)]));
    }
};

/**
 * @brief Delegate class to create cells with a validator that only accepts floats.
 */
class FloatCellDelegate : public QItemDelegate
{
  public:
    FloatCellDelegate() = delete;

    FloatCellDelegate(QObject* parent) :
      QItemDelegate(parent)
    {
    }

    virtual ~FloatCellDelegate()
    {
    }

  protected:
    QWidget* createEditor(QWidget* parent,
                          const QStyleOptionViewItem& option,
                          const QModelIndex& index) const
    {
        Q_UNUSED(option)
        Q_UNUSED(index)

        QLineEdit* lineEdit = new QLineEdit(parent);
        lineEdit->setLocale(QLocale::English);
        lineEdit->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        QRegExp rx("^[-]?\\d*[.]?\\d*$");
        lineEdit->setValidator(new QRegExpValidator(rx, lineEdit));
        return lineEdit;
    }
};

//==================================================================================================
ObservationsViewDialog::ObservationsViewDialog(CalibrationGuiBase* calibrationGui,
                                               QWidget* parent) :
  QDialog(parent),
  pCalibrationGui_(calibrationGui),
  pUi_(new Ui::ObservationsViewDialog),
  sensorName_("")
{
    pUi_->setupUi(this);

    pUi_->observationsTableWidget->setItemDelegateForColumn(
      0, new TargetPoseIdCellDelegate(pUi_->observationsTableWidget));
    pUi_->observationsTableWidget->setItemDelegateForColumn(
      1, new MarkerIdCellDelegate(pUi_->observationsTableWidget));
    pUi_->observationsTableWidget->setItemDelegateForColumn(
      2, new FloatCellDelegate(pUi_->observationsTableWidget));
    pUi_->observationsTableWidget->setItemDelegateForColumn(
      3, new FloatCellDelegate(pUi_->observationsTableWidget));
    pUi_->observationsTableWidget->setItemDelegateForColumn(
      4, new FloatCellDelegate(pUi_->observationsTableWidget));

    pUi_->observationsTableWidget->setContextMenuPolicy(Qt::CustomContextMenu);

    pUi_->uncommittedChangesLabel->setVisible(false);
    pUi_->buttonBox->setEnabled(false);

    //--- connect signal and slots
    connect(pUi_->buttonBox, &QDialogButtonBox::clicked,
            this, &ObservationsViewDialog::handleButtonBoxClicked);
    connect(pUi_->observationsTableWidget, &QTableWidget::cellChanged,
            this, &ObservationsViewDialog::handleTableWidgetCellChanged);
    connect(pUi_->observationsTableWidget, &QTableWidget::customContextMenuRequested,
            this, &ObservationsViewDialog::handleTableWidgetContextMenuRequest);
}

//==================================================================================================
ObservationsViewDialog::~ObservationsViewDialog()
{
    delete pUi_;
}

//==================================================================================================
void ObservationsViewDialog::setSensorName(const std::string& name)
{
    sensorName_ = name;
}

//==================================================================================================
void ObservationsViewDialog::initializeTfListener(rclcpp::Node* ipNode)
{
    tfBuffer_   = std::make_unique<tf2_ros::Buffer>(ipNode->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
}

//==================================================================================================
void ObservationsViewDialog::handleButtonBoxClicked(QAbstractButton* button)
{
    //--- since there is currently only one button in the button box, there is only one
    //--- functionality.
    Q_UNUSED(button)

    if (!pCalibrationGui_)
        return;

    // number of rows in table (excluding header)
    int tableRowCount = pUi_->observationsTableWidget->rowCount();

    // number of columns in table (excluding header)
    int tableColumnCount = pUi_->observationsTableWidget->columnCount();

    // Function to check if row with given Index is empty.
    auto isRowEmpty = [&](const int& rowIdx) -> bool
    {
        for (int c = 0; c < tableColumnCount; ++c)
        {
            auto pCellItem = pUi_->observationsTableWidget->item(rowIdx, c);
            if (pCellItem && !pCellItem->text().isEmpty())
                return false;
        }

        return true;
    };

    // Function to check if row is complete
    auto isRowComplete = [&](const int& rowIdx) -> bool
    {
        bool isRowComplete = true;

        for (int c = 0; c < tableColumnCount; ++c)
        {
            auto pCellItem = pUi_->observationsTableWidget->item(rowIdx, c);
            isRowComplete &= (pCellItem && !pCellItem->text().isEmpty());
        }

        return isRowComplete;
    };

    //--- check if there are empty and incomplete rows and mark them as to be deleted
    std::vector<int> incompleteRows;
    for (int r = 0; r < tableRowCount; ++r)
    {
        if (!isRowEmpty(r) && !isRowComplete(r))
            incompleteRows.push_back(r);
    }

    //--- remove marked rows in reverse order to preserve indices, in this construct a concise
    //--- error message
    if (!incompleteRows.empty())
    {
        // String holding list of row indices that are to be deleted. Intended for error message.
        QString rowIndicesStr = "";
        for (uint i = 0; i < incompleteRows.size() - 1; ++i)
        {
            if (!rowIndicesStr.isEmpty())
                rowIndicesStr.append(", ");

            rowIndicesStr.append(QString::number(incompleteRows[i]));
        }

        if (!rowIndicesStr.isEmpty())
            rowIndicesStr.append(", and ");

        rowIndicesStr.append(QString::number(incompleteRows.back()));

        QMessageBox::critical(this, this->windowTitle(),
                              "The information entered in row(s) " + rowIndicesStr +
                                " is incomplete. The corresponding observation(s) will not be "
                                "submitted and the row(s) will be removed.");

        //--- remove marked rows
        for (int i = incompleteRows.size() - 1; i >= 0; --i)
            pUi_->observationsTableWidget->removeRow(incompleteRows[i]);
    }

    //--- update table row count
    tableRowCount = pUi_->observationsTableWidget->rowCount();

    /// List of service message that are to be called
    std::vector<AddMarkerObservations::Request::SharedPtr> srvMsgs;

    //--- loop over remaining list and try to construct service messages. To this end, it is
    //--- assumed, that the pose observations are entered in a contiguous order, starting off at
    //--- position one. So it is verified, that the difference between the new pose ID and the
    //--- pose ID of the currently constructed message is not greater than 1.
    int currentPoseIdx = 0;
    for (int r = 0; r < tableRowCount; ++r)
    {
        if (isRowEmpty(r))
            continue;

        // pose index of the current row
        int poseIdx = pUi_->observationsTableWidget->item(r, 0)->text().toInt();

        //--- if the pose index of the current message is 1 less than the pose index of the current
        //--- row, add new message to the list and increment the current pose index.
        if (currentPoseIdx == poseIdx || currentPoseIdx == (poseIdx - 1))
        {
            //--- add new message to the list and increment the current pose index
            if (currentPoseIdx == (poseIdx - 1))
            {
                srvMsgs.push_back(std::make_shared<AddMarkerObservations::Request>());
                currentPoseIdx = poseIdx;
            }

            //--- enter information of the current row into the message
            srvMsgs.back()->observation.marker_ids.push_back(
              pUi_->observationsTableWidget->item(r, 1)->text().toInt());

            srvMsgs.back()->observation.marker_top_left_point.push_back(
              geometry_msgs::msg::Point());
            srvMsgs.back()->observation.marker_top_left_point.back().x =
              pUi_->observationsTableWidget->item(r, 2)->text().toDouble();
            srvMsgs.back()->observation.marker_top_left_point.back().y =
              pUi_->observationsTableWidget->item(r, 3)->text().toDouble();
            srvMsgs.back()->observation.marker_top_left_point.back().z =
              pUi_->observationsTableWidget->item(r, 4)->text().toDouble();
        }
        else
        {
            QMessageBox::critical(this, this->windowTitle(),
                                  "Target poses should be captured and entered in contiguous order "
                                  "and starting of with the pose ID 1. Not all observations that "
                                  "are entered will be submitted.");

            for (int i = tableRowCount - 1; i >= r; --i)
                pUi_->observationsTableWidget->removeRow(i);

            break;
        }
    }

    // Function to do service call
    auto doServiceCall = [&](AddMarkerObservations::Request::SharedPtr srvMsg)
    {
        std::string serviceName = pCalibrationGui_->getCalibratorNodeName() + "/" +
                                  sensorName_ + "/" + ADD_MARKER_OBS_SRV_NAME;

        //--- add reference maker observation
        auto addObservationClient = pCalibrationGui_->nodeSharedPtr()->create_client<AddMarkerObservations>(serviceName);

        auto addObservationResponse = addObservationClient->async_send_request(srvMsg);
        auto retCode                = utils::doWhileWaiting(pCalibrationGui_->executor(), addObservationResponse, [&]()
                                                            { QCoreApplication::processEvents(); }, 100);
        if (retCode != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(pCalibrationGui_->nodePtr()->get_logger(),
                         "[%s] Failed to add observation. Called Service: %s",
                         pCalibrationGui_->getGuiNodeName().c_str(), serviceName.c_str());
        }
    };

    pCalibrationGui_->showProgressDialog(
      "Submitting '" + this->windowTitle() + "' observations...");

    //--- loop over messages and to service calls
    for (AddMarkerObservations::Request::SharedPtr msg : srvMsgs)
    {
        doServiceCall(msg);
    }

    pCalibrationGui_->hideProgressDialog();

    pUi_->uncommittedChangesLabel->setVisible(false);
    pUi_->buttonBox->setEnabled(false);
}

//==================================================================================================
void ObservationsViewDialog::handleTableWidgetCellChanged(int row, int column)
{
    QTableWidgetItem* pItem = pUi_->observationsTableWidget->item(row, column);
    if (pItem)
        pItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // number of rows in table (excluding header)
    int tableRowCount = pUi_->observationsTableWidget->rowCount();

    // number of columns in table (excluding header)
    int tableColumnCount = pUi_->observationsTableWidget->columnCount();

    // Function to check if row with given Index is empty.
    auto isRowEmpty = [&](const int& rowIdx) -> bool
    {
        //--- start from column 2, i.e. the coordinate cells, to evaluate if the row is empty or not
        for (int c = 2; c < tableColumnCount; ++c)
        {
            auto pCellItem = pUi_->observationsTableWidget->item(rowIdx, c);
            if (pCellItem && !pCellItem->text().isEmpty())
                return false;
        }

        return true;
    };

    //--- if last row is not empty, insert row at the bottom
    if (!isRowEmpty(tableRowCount - 1))
        pUi_->observationsTableWidget->insertRow(tableRowCount);
    //--- else, if last row is empty, and if additionally second last row is also empty, remove
    //--- row at the bottom of the table
    else if (tableRowCount > 1 && isRowEmpty(tableRowCount - 2))
        pUi_->observationsTableWidget->removeRow(tableRowCount - 1);

    //--- set info label visibility to true and enable button box
    pUi_->uncommittedChangesLabel->setVisible(true);
    pUi_->buttonBox->setEnabled(true);
}

//==================================================================================================
void ObservationsViewDialog::handleTableWidgetContextMenuRequest(const QPoint& pos)
{
    //--- create context menu
    QMenu* menu     = new QMenu(this);
    QAction* action = new QAction("Set marker coordinates by CSV");

    //--- get currently selected row and add to action
    QVariant currentRowVar = QVariant(pUi_->observationsTableWidget->currentRow());
    action->setData(currentRowVar);

    //--- create lambda function for action
    auto setCoordinatesByCSV = [action, this]()
    {
        //--- get current row from data attached to action
        int currentRow = action->data().toInt();

        //--- get current coordinates as csv
        QString coordinatesCsv = "";
        for (int m = 0; m < 3; ++m)
        {
            auto pCellItem = pUi_->observationsTableWidget->item(currentRow, m + 2);

            if (pCellItem && !pCellItem->text().isEmpty())
                coordinatesCsv.append(pCellItem->text());
            else
                coordinatesCsv.append("0.0");

            if (m < 2)
                coordinatesCsv.append(", ");
        }

        bool isAccepted;
        bool isValid;

        //--- if accepted and input is not empty, check for correct format and add to corresponding
        //--- row
        do
        {
            //--- construct input dialog
            coordinatesCsv = QInputDialog::getText(this, "Marker Coordinates",
                                                   "X, Y, Z:", QLineEdit::Normal,
                                                   coordinatesCsv, &isAccepted);

            if (!isAccepted)
                break;

            //--- construct regular expression, expecting 3 floats separated by ',' with any
            //--- white space in between
            QString floatRegExpStr = "[-]?\\d*[.]?\\d*";
            QRegExp rx("^\\s*" + floatRegExpStr +
                       "\\s*[,]\\s*" + floatRegExpStr +
                       "\\s*[,]\\s*" + floatRegExpStr + "\\s*$");

            //--- if coordinatesCsv matches reg exp, enter into table, else show message box
            if (rx.exactMatch(coordinatesCsv))
            {
                QLineEdit* leItem;
                QStringList coordinateList = coordinatesCsv.split(',');
                for (int i = 0; i < coordinateList.size(); ++i)
                {
                    QLineEdit* leItem = dynamic_cast<QLineEdit*>(
                      this->pUi_->observationsTableWidget->item(currentRow, i + 2));

                    if (leItem)
                    {
                        leItem->setText(coordinateList[i].simplified());
                    }
                    else
                    {
                        QTableWidgetItem* newItem =
                          new QTableWidgetItem(coordinateList[i].simplified());
                        pUi_->observationsTableWidget->setItem(currentRow, i + 2, newItem);
                    }
                }

                //--- add value for target pose
                //--- This is done by assuming that the poses are entered in a contigious order starting
                //--- from 1. Thus the pose index is the result of a integer division of the row by
                //--- 4 (number of markers) and adding 1.
                QString poseStr = QString::number((currentRow / 4) + 1);
                leItem          = dynamic_cast<QLineEdit*>(
                  this->pUi_->observationsTableWidget->item(currentRow, 0));
                if (leItem)
                    leItem->setText(poseStr);
                else
                    pUi_->observationsTableWidget->setItem(
                      currentRow, 0, new QTableWidgetItem(poseStr));

                //--- add preset for marker id
                //--- This is done by assuming that the poses are entered in a contigious order starting
                //--- from 1. Thus the pose index is the result of a integer division of the row by
                //--- 4 (number of markers) and adding 1.
                QString markerStr = QString::number(MARKER_IDS_FOR_ITEM_DELEGATE[(currentRow % 4)]);
                leItem            = dynamic_cast<QLineEdit*>(
                  this->pUi_->observationsTableWidget->item(currentRow, 1));
                if (leItem)
                    leItem->setText(markerStr);
                else
                    pUi_->observationsTableWidget->setItem(
                      currentRow, 1, new QTableWidgetItem(markerStr));

                isValid = true;
            }
            else
            {
                QMessageBox::critical(this, "Marker Coordinates", "Entered coordinates are not in "
                                                                  "correct CSV format.");
                isValid = false;
            }
        } while (!isValid);
    };
    connect(action, &QAction::triggered, this, setCoordinatesByCSV);

    //--- show menu
    menu->addAction(action);
    menu->popup(pUi_->observationsTableWidget->viewport()->mapToGlobal(pos));
}

} // namespace multisensor_calibration
