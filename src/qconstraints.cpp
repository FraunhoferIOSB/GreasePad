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

#include "conics.h"
#include "global.h"
#include "qconstraints.h"
#include "usegment.h"

#include <QColor>
#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QMenu>
#include <QPainter>
#include <qstyleoption.h>


namespace QConstraint {

bool QConstraintBase::s_show     =  !false; // TODO(meijoc) change later
bool QConstraintBase::s_showColor =  false;
int  QConstraintBase::s_defaultMarkerSize = 10;
QPen QConstraintBase::s_defaultPenReq = QPen();
QPen QConstraintBase::s_defaultPenRed = QPen();
QPen QConstraintBase::s_penSelected   = QPen();



QConstraintBase::QConstraintBase()
{
    qDebug() << Q_FUNC_INFO;
    setVisible( s_show);

    m_pen_req = s_defaultPenReq;
    m_pen_red = s_defaultPenRed;

    m_is_required = false;
    m_is_enforced = false;

    // setHandlesChildEvents(false);  // default is 'false'
    setFlag( ItemIsSelectable, true);
    setFlag( ItemIsMovable,            false);
    setFlag( ItemSendsGeometryChanges, false);
}

void QConstraintBase::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
    if ( event->button() == Qt::RightButton ) {
        setSelected( !isSelected() );  // toggle selection
    }
}


void QConstraintBase::serialize( QDataStream &out )
{
    qDebug() << Q_FUNC_INFO;

    out << m_is_required;
    out << m_is_enforced;

    out << m_pen_req;
    out << m_pen_red;
    out << m_altColor;

    // out << m_markerSize;
    out << markerSize();
    out << pos();
    out << rotation();
}

bool QConstraintBase::deserialize( QDataStream &in )
{
    qDebug() << Q_FUNC_INFO;

    in >> m_is_required;
    in >> m_is_enforced;

    in >> m_pen_req;
    in >> m_pen_red;

    in >> m_altColor;
    qreal s;
    in >> s; // m_markerSize;
    setMarkerSize(s);

    QPointF pos;   in >> pos;
    qreal   rot;   in >> rot;

    setRotation( rot );
    setPos( pos );

    return true;
}



QRectF QOrthogonal::boundingRect() const
{
    return rect().adjusted(-2*m_pen_req.widthF(),
                           -2*m_pen_req.widthF(),
                           2*m_pen_req.widthF(),
                           2*m_pen_req.widthF());
}



void QOrthogonal::paint( QPainter *painter,
                         const QStyleOptionGraphicsItem * option,
                         QWidget * widget)
{

    QPen pen = QPen( m_is_required ? m_pen_req : m_pen_red);
    if ( !enforced() ) {
        pen.setColor( Qt::cyan );
    }

    if ( showColor() ) {
        pen.setColor( m_altColor);
    }

    painter->setPen(pen);
    painter->drawRect( /*square.*/rect() );

    QConstraintBase::paint( painter, option, widget);

}

QOrthogonal::QOrthogonal()
{
    const int s = s_defaultMarkerSize;
    setRect( -s, -s, 2*s, 2*s);
}

void QOrthogonal::setMarkerSize ( const qreal s)
{
    qDebug() << Q_FUNC_INFO;
    setRect( -s, -s, 2*s, 2*s );
}

qreal QOrthogonal::markerSize() const
{
    qDebug() << Q_FUNC_INFO;
    return -rect().x();
}


void QOrthogonal::setGeometry( const uStraightLineSegment &s,
                               const uStraightLineSegment &t)
{
    qDebug() << Q_FUNC_INFO;
    Vector3d xh = s.hl().cross( t.hl() );

    QConstraintBase::setPos( m_sc*xh(0)/xh(2),  m_sc*xh(1)/xh(2));
    //qDebug() << pos();
    QConstraintBase::setRotation( s.phi_deg() );
    QConstraintBase::update();
}

QCopunctual::QCopunctual()
{
    qDebug() << Q_FUNC_INFO;
    const int s = s_defaultMarkerSize;
    setRect( -s, -s, 2*s, 2*s );
}

void QCopunctual::setMarkerSize ( const qreal s)
{
    qDebug() << Q_FUNC_INFO;
    setRect( -s, -s, 2*s, 2*s );
}

qreal QCopunctual::markerSize() const
{
    qDebug() << Q_FUNC_INFO;
    return -rect().x();
}



void QConstraintBase::setAltColor(const QColor & col)
{
    m_altColor = col;
}

QRectF QCopunctual::boundingRect() const
{
    return rect().adjusted( -m_pen_req.width(),
                            -m_pen_req.width(),
                            +m_pen_req.width(),
                            +m_pen_req.width());
}


void QConstraintBase::paint( QPainter * painter,
                             const QStyleOptionGraphicsItem *option,
                             QWidget *widget)
{
    // qDebug() << Q_FUNC_INFO;
    Q_UNUSED(widget)
    Q_UNUSED(option)

    if ( !isSelected() ) {
        return;
    }

    painter->setPen( s_penSelected );
    painter->drawRect( boundingRect());
}

void QCopunctual::paint( QPainter *painter,
                         const QStyleOptionGraphicsItem * option,
                         QWidget * widget)
{

    QPen pen = QPen( m_is_required ? m_pen_req : m_pen_red);

    if ( showColor()) {
        pen.setColor( m_altColor);
    }
    if ( !enforced() ) {
        pen.setColor(Qt::red);
    }

    painter->setPen( pen );

    QRectF R = rect();
    if ( !m_is_required ) {
        R.adjust( -m_pen_req.widthF(), -m_pen_req.widthF(),
                   m_pen_req.widthF(),  m_pen_req.widthF());
    }
    painter->drawEllipse( R );

    QConstraintBase::paint(painter, option, widget);
}

void QCopunctual::setGeometry( const uStraightLineSegment &s,
                               const uStraightLineSegment &t)
{
    qDebug() << Q_FUNC_INFO;
    Vector3d xh = s.hl().cross( t.hl() );
    QConstraintBase::setPos( m_sc*xh(0)/xh(2), m_sc*xh(1)/xh(2) );
}


QParallel::QParallel()
{
    auto s = double( s_defaultMarkerSize );
    a.setLine( +s_sc*s, -s*(1-s_shear),  +s_sc*s, s*(1+s_shear));
    b.setLine( -s_sc*s, -s*(1+s_shear),  -s_sc*s, s*(1-s_shear));
}

void QParallel::setMarkerSize ( const qreal s)
{
    qDebug() << Q_FUNC_INFO;
    a.setLine( +s_sc*s, -s*(1-s_shear),  +s_sc*s, s*(1+s_shear));
    b.setLine( -s_sc*s, -s*(1+s_shear),  -s_sc*s, s*(1-s_shear));
}

qreal QParallel::markerSize() const
{
    qDebug() << Q_FUNC_INFO;
    return a.line().x1()/s_sc;
}


void QParallel::setGeometry( const uStraightLineSegment &s,
                             const uStraightLineSegment &t)
{
    constexpr double N = 4;
    double x = ( s.x()(0) +s.y()(0) +t.x()(0) +t.y()(0) ) / N;
    double y = ( s.x()(1) +s.y()(1) +t.x()(1) +t.y()(1) ) / N;
    setPos( m_sc*x,  m_sc*y);     // position
    setRotation( s.phi_deg() );   // orientation
    update();
}


QRectF QParallel::boundingRect() const
{
    QRectF R = a.boundingRect().united( b.boundingRect() );
    R.adjust( -2*m_pen_req.width(), -2*m_pen_req.width(),
              +2*m_pen_req.width(), +2*m_pen_req.width());
    return R;
}


void QParallel::paint( QPainter *painter,
                       const QStyleOptionGraphicsItem * option,
                       QWidget * widget)
{

    QPen pen = QPen( m_is_required ? m_pen_req : m_pen_red);

    if ( showColor() ) {
        pen.setColor( m_altColor);
    }

    if ( !enforced() ) {
        pen.setColor( Qt::red);
    }
    painter->setPen( pen );
    painter->drawLine( a.line() );
    painter->drawLine( b.line() );

    QConstraintBase::paint( painter, option, widget);
}


QIdentical::QIdentical()
{
    const double s = s_defaultMarkerSize;
    setPolygon( QPolygonF() << QPointF(-s, -s)
                     << QPointF(+s, +s)
                     << QPointF(+s, -s)
                     << QPointF(-s, +s));
}

void QIdentical::setMarkerSize ( const qreal s)
{
    qDebug() << Q_FUNC_INFO;
    setPolygon( QPolygonF() << QPointF(-s, -s)
                << QPointF(+s, +s)
                << QPointF(+s, -s)
                << QPointF(-s, +s));
}


qreal QIdentical::markerSize() const
{
    qDebug() << Q_FUNC_INFO;
    // return -polygon().first().x();
    return -polygon().at(0).x();
}


void QIdentical::setGeometry( const uStraightLineSegment &s,
                              const uStraightLineSegment &t)
{
    Eigen::Matrix<double,2,4> xx;
    xx << s.x(), s.y(), t.x(), t.y();
    Eigen::Matrix4d DD;
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            DD(i,j) = ( xx.col(i) -xx.col(j) ).norm();
        }
        DD(i,i) = DBL_MAX;
    }

    int i;
    int j;
    DD.minCoeff( &i, &j);
    QConstraintBase::setPos( m_sc*(xx(0,i)+xx(0,j))/2, m_sc*(xx(1,i)+xx(1,j))/2  );
    QConstraintBase::setRotation( s.phi_deg() );
}


QRectF QIdentical::boundingRect() const
{
    return polygon().boundingRect();
}


void QIdentical::paint( QPainter *painter,
                        const QStyleOptionGraphicsItem * option,
                        QWidget * widget)
{

    QPen pen = QPen( m_is_required ? m_pen_req : m_pen_red);
    if ( !enforced() ) {
        pen.setColor( Qt::red );
    }
    painter->setPen( pen );
    painter->drawPolygon( polygon() );
    QConstraintBase::paint(painter, option, widget);

}

std::shared_ptr<QConstraintBase> QCopunctual::create() const
{
    return std::make_shared<QCopunctual>();
}

std::shared_ptr<QConstraintBase> QParallel::create() const
{
    return std::make_shared<QParallel>();
}

std::shared_ptr<QConstraintBase> QOrthogonal::create() const
{
    std::shared_ptr<QOrthogonal> p = std::make_shared<QOrthogonal>();
    return std::move(p);
}

std::shared_ptr<QConstraintBase> QIdentical::create() const
{
    return std::make_shared<QIdentical>();
}


void QConstraintBase::setStatus( bool isrequired,
                                 bool isenforced) {
    m_is_required = isrequired;
    m_is_enforced = isenforced;
}

} // namespace QConstraint
