/***********************************************************************
*
* Copyright (c) Fraunhofer Institute of Optronics,
* System Technologies and Image Exploitation IOSB
*
**********************************************************************/
/***********************************************************************
*
* Table cell delegate whose editor only accepts positive integers.
*
**********************************************************************/

#ifndef MULTISENSORCALIBRATION_UI_POSITIVEINTEGERCELLDELEGATE_H
#define MULTISENSORCALIBRATION_UI_POSITIVEINTEGERCELLDELEGATE_H

// Qt
#include <QItemDelegate>
#include <QLineEdit>
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
#  include <QRegExpValidator>
#else
#  include <QRegularExpressionValidator>
#endif

namespace multisensor_calibration
{

/**
 * @ingroup ui
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

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
        lineEdit->setValidator(new QRegExpValidator(QRegExp("^\\d*$"), lineEdit));
#else
        lineEdit->setValidator(new QRegularExpressionValidator(QRegularExpression("^\\d*$"), lineEdit));
#endif
        return lineEdit;
    }
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_UI_POSITIVEINTEGERCELLDELEGATE_H
