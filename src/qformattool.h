/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022 Jochen Meidow, Fraunhofer IOSB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef QFORMATTOOL_H
#define QFORMATTOOL_H

#include "global.h"

#include <QDialog>

#include <memory>

class QCheckBox;
class QColorDialog;
class QComboBox;
class QDialogButtonBox;
class QFormLayout;
class QSpinBox;


namespace GUI {

class FormatTool : public QDialog {

    Q_OBJECT

public:
    FormatTool( const QString &title, QWidget *);
    FormatTool( const FormatTool &) = delete;
    FormatTool & operator= ( FormatTool ) = delete;
    ~FormatTool() override;

private:
    void createElements();
    void establishConnections();
    void createLayout();

    std::unique_ptr<QFormLayout>  m_layout;
    std::unique_ptr<QColorDialog> m_colorDialog;
    std::unique_ptr<QPushButton>  m_colorPushButton;
    std::unique_ptr<QComboBox>    m_styleCombo;
    std::unique_ptr<QSpinBox>     m_lineWidthSpinBox;
    std::unique_ptr<QSpinBox>     m_markerSizeSpinBox;

    std::unique_ptr<QCheckBox> m_checkBoxDefaultConstrained;
    std::unique_ptr<QCheckBox> m_checkBoxDefaultUnconstrained;
    std::unique_ptr<QCheckBox> m_checkBoxDefaultRequired;
    std::unique_ptr<QCheckBox> m_checkBoxDefaultRedundant;
    std::unique_ptr<QCheckBox> m_checkBoxDefaultStroke;
    std::unique_ptr<QCheckBox> m_checkBoxScribblePen;

    std::unique_ptr<QDialogButtonBox> m_buttonBox;

Q_SIGNALS:
    void signalColorChanged( const QColor &);
    void signalMarkerSizeChanged( int);
    void signalLineWidthChanged( int);
    void signalLineStyleChanged( int);

public Q_SLOTS:
    void showColorDialog();
    void slotUpdateColor( const QColor & col);
    void updateLineWidth( int w) {  Q_EMIT signalLineWidthChanged(w);  }
    void updateLineStyle( int s) {  Q_EMIT signalLineStyleChanged(s);  }
    void updateMarkerSize( int s) { Q_EMIT signalMarkerSizeChanged(s); }

    void slotDiscard()  {   } // TODO
    void slotApply();
};

} // namespace GUI

#endif // QFORMATTOOL_H
