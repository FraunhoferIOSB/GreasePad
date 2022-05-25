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

#include "mainview.h"

#include <QAction>
#include <QApplication>
#include <QBuffer>
#include <QClipboard>
#include <QMimeData>
#include <QPdfWriter>
#include <QStyleOptionGraphicsItem>
#include <QSvgGenerator>
#include <QWheelEvent>


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

    double s = event->delta() > 0 ? zoom_out : zoom_in;

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
    actionZoomIn = std::make_unique<QAction>( "Zoom in" );

    actionZoomIn->setToolTip(  QString("Zoom in (%1)")
                               .arg(QKeySequence(QKeySequence::ZoomIn).toString(QKeySequence::NativeText))
                               );

    QList<QKeySequence> listIn;
    // listIn << QKeySequence::ZoomIn << Qt::Key_Plus;
    listIn << Qt::Key_Plus;  //  [Ctrl]+[+] : move selected items to top
    actionZoomIn->setShortcuts( listIn);
    actionZoomIn->setIcon(      QPixmap(":/icons/Tango/List-add.svg"));

    QList<QKeySequence> listOut;
    // listOut << QKeySequence::ZoomOut << Qt::Key_Minus;
    listOut << Qt::Key_Minus;  // [Ctrl]+[-] : move selected items to bottom
    actionZoomOut = std::make_unique<QAction>( "Zoom out" );
    actionZoomOut->setToolTip(    QString("Zoom out (%1)")
                                  .arg(QKeySequence(QKeySequence::ZoomOut).toString(QKeySequence::NativeText))
                                );
    actionZoomOut->setShortcuts( listOut );
    actionZoomOut->setIcon(      QPixmap(":/icons/Tango/List-remove.svg"));

    actionToggleShowBackgroundTiles = std::make_unique<QAction>( "Show background tiles" );
    actionToggleShowBackgroundTiles->setShortcut(  QKeySequence( "Ctrl+T" ) );
    actionToggleShowBackgroundTiles->setToolTip(   "Show background tiles" );
    actionToggleShowBackgroundTiles->setCheckable( true );
    actionToggleShowBackgroundTiles->setChecked(   false);
    actionToggleShowBackgroundTiles->setIcon(      QPixmap(":/icons/show_checker.svg"));
    actionToggleShowBackgroundTiles->setIconVisibleInMenu( false );

    actionCopyScreenshotToClipboard = std::make_unique<QAction>( "Copy screenshot (image) to clipboard" );
    actionCopyPdfToClipboard = std::make_unique<QAction>( "Copy PDF to clipboard (application/pdf)" );
    actionCopySvgToClipboard = std::make_unique<QAction>( "Copy SVG to clipboard (image/svg+xml)" );
    actionCopySvgToClipboard->setShortcut(  QKeySequence::Copy );//( "Ctrl+C" ) );
}

void MainView::slotToggleShowBackgroundTiles()
{
    // qDebug() << Q_FUNC_INFO;
    s_showBackgroundTiles = !s_showBackgroundTiles;
    if ( s_showBackgroundTiles ) {
        QPalette pal = palette();
        pal.setBrush( QPalette::Base, QPixmap( ":/icons/show_checker.svg" ).scaled(480,480) );
        setPalette( pal);
    }
    else {
        setPalette( QApplication::palette() );
    }
}

void MainView::slotCopyScreenshotToClipboard()
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
    Q_EMIT signalShowStatus( QString("Image data copied to clipboard (%1)" )
            .arg(QApplication::clipboard()->mimeData()->formats().first()) );
}

void MainView::slotCopyPdfToClipboard()
{
    // qDebug() << Q_FUNC_INFO << QApplication::clipboard()->mimeData()->formats();

    QBuffer b;
    b.open(QBuffer::ReadWrite);

    QPdfWriter pdfwriter( &b );
    QPainter painter( &pdfwriter );
    painter.begin( &pdfwriter );
    painter.setRenderHint( QPainter::Antialiasing );
    render( &painter );
    painter.end();

    b.seek(0);   // qDebug() << b.readAll();

    const QString type = "application.pdf";
    auto *d = new QMimeData();    // auto d = QSharedPointer<QMimeData>();
    d->setData( type, b.buffer());
    QApplication::clipboard()->setMimeData( d, QClipboard::Clipboard );
    // delete  d; // No!
    // QApplication::clipboard()->mimeData()->formats().first()
    Q_EMIT signalShowStatus( QString("Media type '%1' copied to clipboard." ).arg( type ) );
}

void MainView::slotCopySvgToClipboard()
{
    // qDebug() << Q_FUNC_INFO;
    // TODO(meijoc) set width/heigth?

    QSvgGenerator generator;

    QBuffer b;
    b.open(QBuffer::ReadWrite);
    generator.setOutputDevice( &b);
    generator.setSize(         QSize( width(), height() )         );
    generator.setViewBox(      QRect(0, 0, width(), height())   );
    generator.setTitle(        QApplication::applicationName()  );
    generator.setDescription(  "SVG" );

    QPainter painter;
    painter.begin( &generator );
    painter.setRenderHint( QPainter::Antialiasing );
    render( &painter );
    painter.end();

    // b.seek(0);

    const QString type = "image/svg+xml";
    auto *d = new QMimeData();
    d->setData( type, b.buffer() );
    QApplication::clipboard()->setMimeData( d, QClipboard::Clipboard);
    // delete  d; // No!

    Q_EMIT signalShowStatus( QString( "Media type '%1' copied to clipboard." ).arg( type));
}

void MainView::drawForeground( QPainter* painter,
                               const QRectF & rect)
{

    Q_UNUSED( painter )
    Q_UNUSED( rect    )

    QPainter p( viewport() );

    const int h = 10;
    QPoint Pos( h, viewport()->height()-h);
    const int r = 23;
    const int g = 156;
    const int b = 125;
    p.setPen( QPen( QColor(r,g,b) ) );
    p.drawText( Pos, QApplication::organizationName() );
}

} // namespace GUI
