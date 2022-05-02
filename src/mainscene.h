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

#ifndef MAINSCENE_H
#define MAINSCENE_H

#include <QAction>
#include <QDebug>
#include <QGraphicsPathItem>
#include <QGraphicsScene>

#include <memory>

#include "global.h"

class State;


namespace GUI {

class MainScene : public QGraphicsScene
{
    Q_OBJECT

public:
    MainScene( QObject *parent );
    ~MainScene() override = default; //{ qDebug() << Q_FUNC_INFO; }

    void mousePressEvent(   QGraphicsSceneMouseEvent *event) override;
    void mouseMoveEvent(    QGraphicsSceneMouseEvent *event) override;
    void mouseReleaseEvent( QGraphicsSceneMouseEvent *event) override;
    void keyPressEvent(     QKeyEvent *event) override;

    void removeAllItems();
    void addGraphicItems( const State *s);

    void export_view_as_svg( QString &fileName);
    void export_view_as_pdf( QString &fileName);

    static QPen scribblePen() { return s_scribblePen; }
    static void setScribblePen( const QPen & p) { s_scribblePen = p; }

private:
    std::unique_ptr<QAction>   actionExportSaveAs;

    bool m_scribbling;              // tracking the point positions
    QGraphicsPathItem m_path_item;  // point set (stroke) as graphics item
    QPainterPath m_painter_path;

    const double threshold_ratio_eigenvalues = 0.01;
    const int    min_number_of_points = 7;

    bool isaStraightStroke( double T, const QPainterPath & );

    static QPen s_scribblePen;

Q_SIGNALS:
    void signalCmdAddStroke( QPainterPath * );
    void signalCmdDeleteSelection();
    void signalUndoLastAction();
};

} // namespace GUI

#endif // MAINSCENE_H
