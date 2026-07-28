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

#include "helper.h"
#include "logger.h"
#include "mainview.h"

#include "qgraphicsscene.h"
#include "qgraphicsview.h"
#include "qnamespace.h"
#include "qtmetamacros.h"
#include "qtpreprocessorsupport.h"
#include "qtypes.h"

#include <QAction>
#include <QApplication>
#include <QBuffer>
#include <QClipboard>
#include <QList>
#include <QMimeData>
#include <QPainter>
#include <QPalette>
#include <QPdfWriter>
#include <QStringLiteral>
#include <QStyleOptionGraphicsItem>
#include <QSvgGenerator>
#include <QWheelEvent>

#include <memory>

namespace GUI {


bool MainView::s_showBackgroundTiles = false;


MainView::MainView( QGraphicsScene *scene, QWidget *parent ) : QGraphicsView (parent)
{
    setScene( scene );
    setTransformationAnchor( QGraphicsView::AnchorUnderMouse);
    setResizeAnchor( QGraphicsView::AnchorUnderMouse);
    setDragMode(     QGraphicsView::NoDrag);
    setRenderHint(   QPainter::Antialiasing, true);

    // showBackgroundTiles = false;

    scale(3, 3); // ?? TODO
    createActions();
    establishConnections();
}



void MainView::wheelEvent( QWheelEvent *event )
{
    // restricted zooming [0.1, 10]
    const qreal lod = QStyleOptionGraphicsItem::levelOfDetailFromTransform(transform());

    // double s = event->delta() > 0 ? zoom_out : zoom_in;  // old Qt version
    double const s = event->angleDelta().y() > 0 ? zoom_out : zoom_in;

    setTransformationAnchor( QGraphicsView::AnchorUnderMouse );
    // qDebug() << s << lod;
    if ( s > 1.0 && lod < lod_max) {
        scale(s,s);
    }
    else {
        if ( s < 1.0 && lod > lod_min) {
            scale(s,s);
        }
    }
}

void MainView::establishConnections()
{
    // qDebug() << Q_FUNC_INFO;
    connect( actionZoomIn.get(),  &QAction::triggered,
             this,                &MainView::slotZoomIn);
    connect( actionZoomOut.get(),   &QAction::triggered,
             this,                  &MainView::slotZoomOut );
    connect( actionToggleShowBackgroundTiles.get(), &QAction::triggered,
             this,                                  &MainView::slotToggleShowBackgroundTiles);
    connect( actionCopyScreenshotToClipboard.get(), &QAction::triggered,
             this,                                  &MainView::slotCopyScreenshotToClipboard);
    connect( actionCopySvgToClipboard.get(),   &QAction::triggered,
             this,                             &MainView::slotCopySvgToClipboard);
    connect( actionCopyPdfToClipboard.get(),   &QAction::triggered,
             this,                             &MainView::slotCopyPdfToClipboard);

}

void MainView::createActions()
{
     actionZoomIn = makeAction(
        QStringLiteral("Zoom in"),
        QStringLiteral("Zoom in (%1)") // [+]
            .arg(QKeySequence(Qt::Key_Plus).toString(QKeySequence::NativeText)),
        Qt::Key_Plus,
        QPixmap(":/icons/Tango/List-add.svg"),
        true, false, false, false);

    actionZoomOut = makeAction(
        QStringLiteral("Zoom out"),
        QStringLiteral("Zoom out (%1)") // [-]
            .arg(QKeySequence(Qt::Key_Minus).toString(QKeySequence::NativeText)),
        Qt::Key_Minus,
        QPixmap(":/icons/Tango/List-remove.svg"),
        true, false, false, false);

    actionToggleShowBackgroundTiles = makeAction(
        QStringLiteral("Show background tiles"), QStringLiteral("Show background tiles"),
        QKeySequence( QStringLiteral("Ctrl+T") ),
        QPixmap(":/icons/show_checker.svg"),
        true, true, false, false);

    actionCopyScreenshotToClipboard = makeAction(
        QStringLiteral("Copy screenshot (image) to clipboard"), {},
        {}, {},
        true, false, false, false);

    actionCopyPdfToClipboard = makeAction(
        QStringLiteral("Copy PDF to clipboard (application/pdf)"), {},
        {}, {},
        true, false, false, false);

    actionCopySvgToClipboard = makeAction(
        QStringLiteral("Copy SVG to clipboard (image/svg+xml)"), {},
        QKeySequence::Copy,  // Ctrl+C
        QIcon(),
        true, false, false, false);
}


void MainView::slotToggleShowBackgroundTiles()
{
    // qDebug() << Q_FUNC_INFO;
    s_showBackgroundTiles = !s_showBackgroundTiles;
    if ( s_showBackgroundTiles ) {
        QPalette pal = palette();
        pal.setBrush( QPalette::Base, QPixmap( QStringLiteral(":/icons/show_checker.svg") ).scaled(480,480) );
        setPalette( pal);
    }
    else {
        setPalette( QApplication::palette() );
    }
}


bool MainView::slotCopyScreenshotToClipboard()
{
    // qDebug() << Q_FUNC_INFO;

    QPixmap pixmap( width(), height() );
    pixmap.fill( Qt::white );       // works with MS Paint and PowerPoint
    // pixmap.fill( Qt::transparent);  // TODO(joc36395) ??

    QPainter painter;
    painter.begin( & pixmap );
    painter.setRenderHint( QPainter::Antialiasing );
    painter.setBackgroundMode( Qt::TransparentMode );
    render( &painter );
    painter.end();

    QClipboard *cb = QApplication::clipboard();
    cb->setPixmap(pixmap);

    // qDebug() << QApplication::clipboard()->mimeData()->formats();
    const QString msg = QStringLiteral("Image data copied to clipboard (%1)")
                            .arg(QApplication::clipboard()->mimeData()->formats().first()) ;
    Q_EMIT signalShowStatus(msg);
    Logger::log( Logger::Category::IO, msg);

    return true;
}


bool MainView::slotCopyPdfToClipboard()
{
    // qDebug() << Q_FUNC_INFO << QApplication::clipboard()->mimeData()->formats();

    QBuffer buffer;
    if (!buffer.open(QIODevice::ReadWrite)) {
        return false;
    }
    QPdfWriter pdfWriter( &buffer );
    {
        QPainter painter( &pdfWriter );
        painter.setRenderHint( QPainter::Antialiasing );
        render( &painter );
    }
    buffer.seek(0);   // qDebug() << buffer.readAll();

    const QString type = QStringLiteral("application/pdf");
    auto *mime = new QMimeData();    // auto d = QSharedPointer<QMimeData>();
    mime->setData( type, buffer.buffer());
    QApplication::clipboard()->setMimeData( mime, QClipboard::Clipboard );
    // delete mime; // No!
    // QApplication::clipboard()->mimeData()->formats().first()

    const QString msg = QStringLiteral("Figure copied to clipboard (%1)." ).arg( type );
    Q_EMIT signalShowStatus(msg);
    Logger::log( Logger::Category::IO, msg);

    return true;
}


bool MainView::slotCopySvgToClipboard()
{
    // qDebug() << Q_FUNC_INFO;

    QSvgGenerator generator;

    QBuffer b;
    if (!b.open(QBuffer::ReadWrite)) {
        return false;
    }

    generator.setOutputDevice( &b);
    generator.setSize(         QSize( width(), height() )         );
    generator.setViewBox(      QRect(0, 0, width(), height())   );
    generator.setTitle(        QApplication::applicationName()  );
    generator.setDescription(  QStringLiteral("SVG") );

    QPainter painter;
    painter.begin( &generator );
    painter.setRenderHint( QPainter::Antialiasing );
    render( &painter );
    painter.end();

    // b.seek(0);

    const QString type = QStringLiteral("image/svg+xml");
    auto *mime = new QMimeData();
    mime->setData( type, b.buffer() );
    QApplication::clipboard()->setMimeData( mime, QClipboard::Clipboard);
    // delete mime; // No!

    const QString msg = QStringLiteral("Figure copied to clipboard (%1)." ).arg( type);
    Q_EMIT signalShowStatus(msg);
    Logger::log( Logger::Category::IO, msg);

    return true;
}


void MainView::drawForeground( QPainter* painter,
                               const QRectF & rect)
{

    Q_UNUSED( painter )
    Q_UNUSED( rect    )

    QPainter p( viewport() );

    const int h = 10;
    QPoint const Pos(h, viewport()->height() - h);
    const int r = 23;
    const int g = 156;
    const int b = 125;
    p.setPen( QPen( QColor(r,g,b) ) );
    p.drawText( Pos, QApplication::organizationName() );
}

} // namespace GUI
