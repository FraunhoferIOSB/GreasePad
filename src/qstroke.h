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

#ifndef QSTROKE_H
#define QSTROKE_H

#include "global.h"

#include <QGraphicsItem>
#include <QPen>


namespace QEntity {

class QStroke :  public QGraphicsPolygonItem
{
public:
    QStroke();
    QStroke( const QPolygonF & p);
    ~QStroke() override = default;

    // enum {Type = UserType +264};
    // int type() const override { return Type; }

    void serialize( QDataStream & out ) const;
    bool deserialize( QDataStream &in );

    void setColor( const QColor & col) {  m_pen.setColor(col); }
    void setLineWidth( const int w ) {    m_pen.setWidth(w); }

    static void toggleShow() { s_show = !s_show; }
    static bool show()  { return s_show; }
    static void setPenDefault( const QPen & p) {  s_defaultPen = p; }
    static QPen defaultPen() {  return s_defaultPen; }

protected:
    void mousePressEvent( QGraphicsSceneMouseEvent *event) override;
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override; //!< plot tracked positions
private:
    QPen m_pen;

    static QPen s_defaultPen;
    static bool s_show;
};

} // namespace QEntity

#endif // QSTROKE_H
