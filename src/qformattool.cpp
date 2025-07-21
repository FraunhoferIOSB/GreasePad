/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2025 Jochen Meidow, Fraunhofer IOSB
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

#include "qformattool.h"
#include "global.h"
#include "mainscene.h"
#include "qconstraints.h"

#include "qlogging.h"
#include "qnamespace.h"
#include "qsegment.h"
#include "qstroke.h"

#include <QCheckBox>
#include <QColor>
#include <QColorDialog>
#include <QComboBox>
#include <QDebug>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QFrame>
#include <QPalette>
#include <QPen>
#include <QPushButton>
#include <QSpinBox>
#include <QStringLiteral>
#include <QWidget>

#include "qtdeprecationdefinitions.h"
#include "qtmetamacros.h"

#include <memory>

namespace GUI {

FormatTool::FormatTool( const QString &title,
                        QWidget *parent)
 : QDialog(parent)
{
    qDebug() << Q_FUNC_INFO;

    setWindowTitle( title );
    setWindowFlags( Qt::Tool );

    //   setMinimumSize(500,500);
    //   resize(1000,1000);

    createElements();
    createLayout();
    establishConnections();
}

FormatTool::~FormatTool()
{
    qDebug() << Q_FUNC_INFO;
}

void FormatTool::createLayout()
{
    m_layout = std::make_unique<QFormLayout>();
    m_layout->addRow( QStringLiteral("Color"),       m_colorPushButton.get()      );
    m_layout->addRow( QStringLiteral("Line width"),  m_lineWidthSpinBox.get() );
    m_layout->addRow( QStringLiteral("Line style"),  m_styleCombo.get()       );
    m_layout->addRow( QStringLiteral("Marker size"), m_markerSizeSpinBox.get());

    auto * line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    m_layout->addRow( line );

    m_layout->addRow( QStringLiteral("segments: constrained"),      m_checkBoxDefaultConstrained.get());
    m_layout->addRow( QStringLiteral("segments: unconstrained"),    m_checkBoxDefaultUnconstrained.get());
    //   m_layout->addRow( line);

    m_layout->addRow( QStringLiteral("constraints: required") ,  m_checkBoxDefaultRequired.get());
    m_layout->addRow( QStringLiteral("constraints: redundant"),  m_checkBoxDefaultRedundant.get());
     //  m_layout->addRow( line);

    m_layout->addRow( QStringLiteral("strokes/tracks"),     m_checkBoxDefaultStroke.get());
    m_layout->addRow( QStringLiteral("scribble pen"),       m_checkBoxScribblePen.get());

    m_layout->addRow( m_buttonBox.get() );

    setLayout( m_layout.get() );
}


void FormatTool::establishConnections()
{
    connect( m_colorDialog.get(), &QColorDialog::currentColorChanged,
             this,                &FormatTool::slotUpdateColor);
    connect( m_colorPushButton.get(), &QPushButton::clicked,
             this,                    &FormatTool::showColorDialog);
    connect( m_lineWidthSpinBox.get(),   static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
             this,                       &FormatTool::updateLineWidth);
    connect( m_markerSizeSpinBox.get(),  static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
             this,                       &FormatTool::updateMarkerSize );
    connect( m_styleCombo.get(),  static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
             this,                &FormatTool::updateLineStyle);

    /* connect( m_buttonBox->button(QDialogButtonBox::Discard),
             &QAbstractButton::pressed,
             this,  & FormatTool::slotDiscard); */
    connect( m_buttonBox->button(QDialogButtonBox::Apply),
             &QAbstractButton::pressed,
             this,  & FormatTool::slotApply);

    //   connect( checkBoxDefaultConstrained.get(),  &QCheckBox::stateChanged,
    //            this,   &QFormatTool::updateDefault);
}


void FormatTool::slotUpdateColor( const QColor & col)
{
    // qDebug() << Q_FUNC_INFO;
    QPalette pal = palette();
    pal.setColor( QPalette::Button, col );
    m_colorPushButton->setPalette( QPalette(col) );
    m_colorPushButton->setText( col.name() );

    Q_EMIT signalColorChanged(col);
}

void FormatTool::showColorDialog()
{
    m_colorDialog->show();
    m_colorDialog->exec();

    QColor const color = m_colorDialog->selectedColor();

    if ( color.isValid() ) {
        QPalette pal = palette();
        pal.setColor( QPalette::Button, color );
        m_colorPushButton->setPalette( QPalette(color) );
        m_colorPushButton->setText( color.name() );
    }

}

void FormatTool::createElements()
{
    m_colorDialog = std::make_unique<QColorDialog>( Qt::black );

    QPalette pal = palette();
    pal.setColor( QPalette::Button, m_colorDialog->currentColor() );

    m_colorPushButton = std::make_unique<QPushButton>();
    m_colorPushButton->setAutoFillBackground( true );
    m_colorPushButton->setFlat(    true );
    m_colorPushButton->setPalette( pal );
    m_colorPushButton->setText(    m_colorDialog->currentColor().name() ) ;

    m_lineWidthSpinBox = std::make_unique<QSpinBox>();
    m_lineWidthSpinBox->setSuffix( QStringLiteral(" Pt.") );
    m_lineWidthSpinBox->setValue( 3 );

    m_styleCombo = std::make_unique<QComboBox>();
    m_styleCombo->addItem( QStringLiteral("NoPen")          );
    m_styleCombo->addItem( QStringLiteral("SolidLine")      );
    m_styleCombo->addItem( QStringLiteral("DashLine")       );
    m_styleCombo->addItem( QStringLiteral("DotLine")        );
    m_styleCombo->addItem( QStringLiteral("DashDotLine")    );
    m_styleCombo->addItem( QStringLiteral("DashDotDotLine") );
    m_styleCombo->setCurrentIndex( Qt::SolidLine );

    m_markerSizeSpinBox = std::make_unique<QSpinBox>();
    m_markerSizeSpinBox->setSuffix( QStringLiteral(" Pt."));
    m_markerSizeSpinBox->setValue( 10 );


    m_checkBoxDefaultConstrained   = std::make_unique<QCheckBox>("new default");
    m_checkBoxDefaultUnconstrained = std::make_unique<QCheckBox>("new default");
    m_checkBoxDefaultRequired      = std::make_unique<QCheckBox>("new default");
    m_checkBoxDefaultRedundant     = std::make_unique<QCheckBox>("new default");
    m_checkBoxDefaultStroke        = std::make_unique<QCheckBox>("new default");
    m_checkBoxScribblePen          = std::make_unique<QCheckBox>("new default");

    m_buttonBox = std::make_unique<QDialogButtonBox>(
                QDialogButtonBox::Apply
               // | QDialogButtonBox::Discard
                );
}

void FormatTool::slotApply()
{
    // qDebug() << Q_FUNC_INFO;

    if ( m_checkBoxDefaultStroke->isChecked() ) {
        QPen p = QEntity::QStroke::defaultPen();
        p.setColor( m_colorDialog->selectedColor() );
        p.setWidth( m_lineWidthSpinBox->value() );
        QEntity::QStroke::setPenDefault( p);
    }

     if ( m_checkBoxDefaultRequired->isChecked() ) {
         QPen p = QConstraint::QConstraintBase::defaultPenReq();

         p.setColor( m_colorDialog->selectedColor() );
         p.setWidth( m_lineWidthSpinBox->value() );
         p.setStyle( Qt::PenStyle( m_styleCombo->currentIndex() ));

         int const s = m_markerSizeSpinBox->value();
         QConstraint::QConstraintBase::setDefaultMarkerSize( s); // TODO(meijoc) req./red.
         QConstraint::QConstraintBase::setDefaultPenReq( p);
     }

     if ( m_checkBoxDefaultRedundant->isChecked() ) {
         QPen p = QConstraint::QConstraintBase::defaultPenRed();

         p.setColor( m_colorDialog->selectedColor() );
         p.setWidth( m_lineWidthSpinBox->value() );
         p.setStyle( Qt::PenStyle( m_styleCombo->currentIndex() ));

         int const s = m_markerSizeSpinBox->value();
         QConstraint::QConstraintBase::setDefaultMarkerSize( s); // TODO(meijoc) req./red.
         QConstraint::QConstraintBase::setDefaultPenRed( p);
     }

     if ( m_checkBoxDefaultConstrained->isChecked() ) {
         QPen p = QEntity::QConstrained::defaultPen();
         p.setColor( m_colorDialog->selectedColor() );
         p.setWidth( m_lineWidthSpinBox->value() );
         p.setStyle( Qt::PenStyle( m_styleCombo->currentIndex() ));
         QEntity::QConstrained::setPenDefault( p );
     }

     if ( m_checkBoxDefaultUnconstrained->isChecked() ) {
         QPen p = QEntity::QUnconstrained::defaultPen();
         p.setColor( m_colorDialog->selectedColor() );
         p.setWidth( m_lineWidthSpinBox->value() );
         p.setStyle( Qt::PenStyle( m_styleCombo->currentIndex() ));
         QEntity::QUnconstrained::setPenDefault( p );
     }

     if ( m_checkBoxScribblePen->isChecked() ) {
         QPen p = MainScene::scribblePen();
         p.setColor( m_colorDialog->selectedColor() );
         p.setWidth( m_lineWidthSpinBox->value() );
         p.setStyle( Qt::PenStyle( m_styleCombo->currentIndex() ));
         MainScene::setScribblePen( p);
     }
}

} // namespace GUI
