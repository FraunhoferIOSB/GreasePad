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

#ifndef QSTROKE_H
#define QSTROKE_H

#include <QColor>
#include <QGraphicsItem>
#include <QObject>
#include <QPen>
#include <QPolygonF>


namespace QEntity {

//! Graphics: Strokes as point sequence
class QStroke :  public QGraphicsPolygonItem
{
public:
    QStroke();
    explicit QStroke( const QPolygonF & p);
    QStroke( const QStroke &) = delete;    //!< Copy constructor
    QStroke( QStroke&&) = delete;          //!< Move constructor
    QStroke & operator= (const QStroke & ) = delete; //!< Copy assignmemt constructor
    QStroke & operator= (QStroke&&) = delete;        //!< Move assignment constructor

    ~QStroke() override = default;


    void serialize( QDataStream & out ) const;  //!< serialization
    bool deserialize( QDataStream &in );        //!< deserialization

    void setColor( const QColor & col) {  m_pen.setColor(col); } //!< Set color
    void setLineWidth( const int w ) {    m_pen.setWidth(w); }   //!< Set line width, i.e, point size

    static void toggleShow() { s_show = !s_show; }  //!< Toggle visibility
    static bool show()  { return s_show; }          //!< Get status visibility
    static void setPenDefault( const QPen & p) {  s_defaultPen = p; }  //!< Set default pen
    static void setPenSelected( const QPen & p) { s_penSelected = p; } //!< Set pen for selection
    static QPen defaultPen() {  return s_defaultPen; }                 //!< Get current default pen

protected:
    void mousePressEvent( QGraphicsSceneMouseEvent *event) override; //!< Handle mouse press event
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;  //!< Plot point sequence

private:
    QPen m_pen;

    static QPen s_defaultPen;
    static QPen s_penSelected;
    static bool s_show;
};

} // namespace QEntity

#endif // QSTROKE_H
