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

#include "qstroke.h"

#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QPolygonF>
#include <QtPreprocessorSupport>

#include "qnamespace.h"
#include "qobject.h"


namespace QEntity {

QPen QStroke::s_defaultPen = QPen();
QPen QStroke::s_penSelected = QPen();
bool QStroke::s_show = false;

void QStroke::serialize( QDataStream &out ) const
{
    // qDebug() << Q_FUNC_INFO;
    out << m_pen;
    out << polygon(); //  pointSet_;
}

bool QStroke::deserialize( QDataStream &in )
{
    // qDebug() << Q_FUNC_INFO;
    in >> m_pen;
    // in >> pointSet_;
    QPolygonF p;
    in >> p;
    setPolygon(p);
    return in.status()==QDataStream::Ok;  // =0
}


QStroke::QStroke() {
    // qDebug() << Q_FUNC_INFO;
    setVisible( s_show );
    setFlag( ItemIsSelectable, true);
}

QStroke::QStroke(const QPolygonF &p)
    : QGraphicsPolygonItem(p)
    , m_pen(s_defaultPen)
{
    // qDebug() << Q_FUNC_INFO << s_show;
    setVisible( s_show );
    setFlag( ItemIsSelectable, true);
}

void QStroke::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)
    // qDebug() << Q_FUNC_INFO;

    if ( isSelected() ) {
        painter->setPen( s_penSelected );
        painter->drawRect( boundingRect() );
    }
    painter->setPen( m_pen );
    painter->drawPoints( polygon() );

    // ?? QGraphicsPolygonItem::paint(painter,option,widget);
}


void QStroke::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
    // qDebug() << Q_FUNC_INFO ;
    if ( event->button() == Qt::RightButton ) {
        setSelected( !isSelected() );
    }
}


} // namespace QEntity
