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

#include "mainscene.h"
#include "qtypes.h"
#include "state.h"

#include <QAction>
#include <QApplication>
#include <QDebug>
#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QKeyEvent>
#include <QList>
#include <QMessageBox>
#include <QObject>
#include <QPageLayout>
#include <QPageSize>
#include <QPdfWriter>
#include <QStringLiteral>
#include <QSvgGenerator>

#include "qlogging.h"
#include "qnamespace.h"
#include "qtdeprecationdefinitions.h"
#include "qtmetamacros.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cassert>
#include <memory>
#include <utility>

namespace GUI {

QPen MainScene::s_scribblePen = QPen( Qt::blue, 3, Qt::SolidLine, Qt::RoundCap );

MainScene::MainScene(QObject *parent)
    : QGraphicsScene(parent)
    , m_scribbling(false)
{
    constexpr qreal full_HD_width  = 1920;
    constexpr qreal full_HD_height = 1080;

    m_path_item.setPen( s_scribblePen );

    setSceneRect( 0,0,  full_HD_width, full_HD_height );

    actionExportSaveAs = std::make_unique<QAction>( "Export as..." );
    actionExportSaveAs->setDisabled( false );
    actionExportSaveAs->setToolTip( QStringLiteral( "export entire scene" ) );
}

// MainScene::~MainScene() {  qDebug() << Q_FUNC_INFO;}

bool MainScene::isaStraightStroke(const double T, const QPainterPath &pa)
{
    int const N = pa.elementCount();
    if ( N==1 ) {
        return false;
    }

    Eigen::VectorXd xi(N);
    Eigen::VectorXd yi(N);

    for ( int i=0; i<N; i++ ) {
        xi(i) = pa.elementAt(i).x;
        yi(i) = pa.elementAt(i).y;
    }

    double const x0 = xi.mean();
    double const y0 = yi.mean();
    Eigen::Matrix2d MM;
    MM(0,0) = xi.dot(xi) -N*x0*x0;
    MM(0,1) = xi.dot(yi) -N*x0*y0;
    MM(1,1) = yi.dot(yi) -N*y0*y0;
    MM(1,0) = MM(0,1);

    Eigen::Vector2cd ev = MM.eigenvalues();
    assert(ev.real().maxCoeff() > 0.);
    double const crit = ev.real().minCoeff()/ev.real().maxCoeff();

    return crit < T;
}


void MainScene::mousePressEvent( QGraphicsSceneMouseEvent *event)
{
    // Left button: If not pressed, we enter the selection mode..............
    if ( event->button() != Qt::LeftButton )
    {
        // check, if we have an item under the cursor
        if ( items(event->scenePos()).count() > 0 ) {
            QGraphicsScene::mousePressEvent(event);
        }
        else {
            event->accept();
        }
        return;
    }

    // <Ctrl> + [mouse click]: drag mode ....................................
    if ( event->modifiers() == Qt::Modifier::CTRL )
    {
        QGraphicsView *mView = views().at(0);
        mView->setCursor(   Qt::ClosedHandCursor );
        mView->setDragMode( QGraphicsView::ScrollHandDrag);
        return;
    }

    // start scribbling: record the track and draw the path as point set
    m_scribbling = true;
    m_painter_path = QPainterPath( event->scenePos());
    m_path_item.setPath( m_painter_path );

    m_path_item.setPen( s_scribblePen);
    addItem( &m_path_item );

    QGraphicsScene::mousePressEvent(event);
}


//! track mouse positions
void MainScene::mouseMoveEvent( QGraphicsSceneMouseEvent *event)
{
    if ( m_scribbling )
    {
        // qDebug() << Q_FUNC_INFO;
        m_painter_path.lineTo( event->scenePos());
        m_path_item.setPath( m_painter_path );
    }
    QGraphicsScene::mouseMoveEvent(event);
}

//! stop stracking mouse positions
void MainScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    // qDebug() << Q_FUNC_INFO;

    // stop dragging ...........................................
    if ( !m_scribbling )
    {
        QGraphicsView *mView = views().at(0);
        mView->setCursor(   Qt::ArrowCursor       );
        mView->setDragMode( QGraphicsView::NoDrag );
        return;
    }

    // stop scribbling..........................................
    m_scribbling = false;  // stop tracking mouse positions
    if ( event->button() != Qt::LeftButton )
    {
        return;
    }

    // 2 checks ................................................
    if ( m_path_item.path().elementCount() < min_number_of_points )
    {
        // option:
        QMessageBox msg;
        msg.setIcon(            QMessageBox::Warning );
        msg.setWindowTitle(     QApplication::applicationName() );
        msg.setText(QStringLiteral( "Pen stroke too short (Not enough points)." ));
        msg.setStandardButtons( QMessageBox::Ok );
        msg.exec();

        removeItem( &m_path_item );
        return;
    }

    if ( !isaStraightStroke( threshold_ratio_eigenvalues,
                             m_painter_path) )
    {
        QMessageBox msg;
        msg.setIcon(            QMessageBox::Warning);
        msg.setWindowTitle(     QApplication::applicationName() );
        msg.setText(            QStringLiteral( "Not a straight pen stroke. Try again, please." ));
        msg.setStandardButtons( QMessageBox::Ok );
        msg.exec();

        removeItem( &m_path_item);
        return;
    }

    // remove the path which shows the last stroke
    removeItem( &m_path_item );

    Q_EMIT signalCmdAddStroke( &m_painter_path );
}

void MainScene::keyPressEvent( QKeyEvent *event )
{
    // qDebug() << Q_FUNC_INFO;
    switch ( event->key() ) {
    case Qt::Key_Delete:
        Q_EMIT signalCmdDeleteSelection();
        break;
    case Qt::Key_Space:
        Q_EMIT signalUndoLastAction();
        break;
    default:
        break;
    }
    QGraphicsScene::keyPressEvent(event);
}

void MainScene::export_view_as_pdf( QString &fileName)
{
    qDebug() << Q_FUNC_INFO;

    const QRectF rectView = (views().at(0)->mapToScene(
                            views().at(0)->viewport()->geometry())
                        ).boundingRect();

    const QRectF rectScene = itemsBoundingRect();
    qDebug() << rectView;
    qDebug() << rectScene;
    if ( rectView.contains( rectScene) ) {
        setSceneRect( rectScene );
    }
    else {
        setSceneRect( rectView );
    }


    QPdfWriter pdfwriter( fileName );
    const QPageSize ps(QSize(static_cast<int>(sceneRect().width()),
                       static_cast<int>(sceneRect().height())),
                 QPageSize::Point);
    pdfwriter.setPageMargins( QMarginsF(0., 0., 0., 0.),QPageLayout::Point );
    pdfwriter.setPageSize(    ps);
    pdfwriter.setCreator(     QApplication::applicationName() );
    pdfwriter.setTitle(       QApplication::applicationName() );

    pdfwriter.newPage();
    QPainter painter( &pdfwriter );
    render( &painter );
}


void MainScene::export_view_as_svg( QString &fileName )
{

    // qDebug() << Q_FUNC_INFO;

    const QRectF rectView = ( views().at(0)->mapToScene(
                            views().at(0)->viewport()->geometry())
                        ).boundingRect();

    const QRectF rectScene = itemsBoundingRect();
    if ( rectView.contains( rectScene) ) {
        setSceneRect( rectScene );
    }
    else {
        setSceneRect( rectView );
    }


    QSvgGenerator generator;
    generator.setFileName(fileName);
    generator.setSize(QSize(static_cast<int>(width()), static_cast<int>(height())));
    generator.setViewBox(QRect(0, 0, static_cast<int>(width()), static_cast<int>(height())));
    generator.setTitle(    QApplication::applicationName() );
    generator.setDescription( QStringLiteral( "scalable vector graphics" ) );

    clearSelection();
    // qDebug() << "SVG: items: " << items().size();
    QPainter painter;
    painter.begin( &generator );
    painter.setRenderHint( QPainter::Antialiasing);
    views().at(0)->render( &painter );
    painter.end();
}

void MainScene::removeAllItems()
{
    // remove graphical elements but not on optional background image
    // qDebug() << Q_FUNC_INFO;

    QList<QGraphicsItem*> all = items();
    for (auto *g: std::as_const(all) ) {
        if ( g->type() == QGraphicsPixmapItem::Type) {
            continue;
        }
        if ( g->parentItem()==nullptr ) {
            removeItem( g );
            // delete gi;  // No! Still required for undo.
        }
    }
}


 void MainScene::addGraphicItems( const State *s)
 {
     // qDebug() << Q_FUNC_INFO;
     s->graphicItemsAdd( this);
 }

} // namespace GUI

