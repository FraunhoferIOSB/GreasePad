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

#ifndef MAINSCENE_H
#define MAINSCENE_H

#include <QAction>
#include <QDebug>
#include <QGraphicsPathItem>
#include <QGraphicsScene>

#include <memory>

class State;

namespace GUI {

//! Graphics scene
class MainScene : public QGraphicsScene
{
    Q_OBJECT

public:
    MainScene( QObject *parent );  //!< Value constructor
    ~MainScene() override = default; //{ qDebug() << Q_FUNC_INFO; }

protected:
    void mousePressEvent(   QGraphicsSceneMouseEvent *event) override; //!< Handle mouse press event
    void mouseMoveEvent(    QGraphicsSceneMouseEvent *event) override; //!< Handle mouse moce event
    void mouseReleaseEvent( QGraphicsSceneMouseEvent *event) override; //!< Handle mouse release event
    void keyPressEvent(     QKeyEvent *event) override; //!< Handle key press event

public:
    void removeAllItems();       //!< Remove all graphics from the scene but not an optional background imge
    void addGraphicItems( const State *s);  //!< Add graphics from state to scene

    void export_view_as_svg( QString &fileName);  //!< Export the current view as scalabel vector graphics
    void export_view_as_pdf( QString &fileName);  //!< Export the current view in portable document format

    static QPen scribblePen() { return s_scribblePen; }                 //!< Get the scribble pen
    static void setScribblePen( const QPen & p) { s_scribblePen = p; }  //!< Set the scribble pen

private:
    std::unique_ptr<QAction>   actionExportSaveAs;

    bool m_scribbling;              // tracking the point positions
    QGraphicsPathItem m_path_item;  // point set (stroke) as graphics item
    QPainterPath m_painter_path;

    const double threshold_ratio_eigenvalues = 0.01;
    const int    min_number_of_points = 7;

    static bool isaStraightStroke(double T, const QPainterPath &);

    static QPen s_scribblePen;

Q_SIGNALS:
    void signalCmdAddStroke( QPainterPath * ); //!< Command: add a stroke, reasoning, and adjustment
    void signalCmdDeleteSelection();           //!< Command: delete selected items and reasoning
    void signalUndoLastAction();               //!< Command: und last action
};

} // namespace GUI

#endif // MAINSCENE_H
