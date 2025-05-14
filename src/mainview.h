/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2023 Jochen Meidow, Fraunhofer IOSB
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

#ifndef MAINVIEW_H
#define MAINVIEW_H

#include <QAction>
#include <QGraphicsView>
#include <QDebug>
// #include <QStatusBar>

#include <memory>

namespace GUI {

//! Graphics view
class MainView :public QGraphicsView
{
    Q_OBJECT

public:
     MainView( QGraphicsScene *scene, QWidget *parent); //!< Value constructor
     ~MainView() override { //qDebug() << Q_FUNC_INFO;
     }

     std::unique_ptr<QAction> actionCopyScreenshotToClipboard{}; //!< Copy screenshot to clipboard
     std::unique_ptr<QAction> actionCopySvgToClipboard{};        //!< Copy scalable vector graphics to clipboard
     std::unique_ptr<QAction> actionCopyPdfToClipboard{};        //!< Copy portable document format to clipboard
     std::unique_ptr<QAction> actionToggleShowBackgroundTiles;   //!< Toggle visibility of background tiles
     std::unique_ptr<QAction> actionZoomIn;                      //!< Zoom in  [+]
     std::unique_ptr<QAction> actionZoomOut;                     //!< Zoom out [-]

protected:
     void wheelEvent( QWheelEvent *event) override;                    //!< Zoom via mouse wheel
     void drawForeground( QPainter *painter, const QRectF &) override; //!< Plot the organization name

private:
    const double lod_max = 100.0;
    const double lod_min = 0.01;
    const double zoom_in = 1.25;
    const double zoom_out = 1./zoom_in;

    void createActions();
    void establishConnections();

    void slotToggleShowBackgroundTiles();
    void slotZoomIn()  {  scale( zoom_in,  zoom_in ); }
    void slotZoomOut() {  scale( zoom_out, zoom_out); }
    void slotCopyScreenshotToClipboard();
    void slotCopySvgToClipboard();
    void slotCopyPdfToClipboard();

    static bool s_showBackgroundTiles;

Q_SIGNALS:
    void signalShowStatus( const QString & s); //!< Send message to status bar
};

} // namespace GUI

#endif // MAINVIEW_H
