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
    // qDebug() << Q_FUNC_INFO;
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

QConstraintBase::QConstraintBase( const QConstraintBase & other)
{
    // qDebug() << Q_FUNC_INFO << m_sc;
    setVisible( s_show);

    setFlag( ItemIsSelectable, true);
    setFlag( ItemIsMovable,            false);
    setFlag( ItemSendsGeometryChanges, false);

    m_altColor = other.m_altColor;
    m_pen_req = other.m_pen_req;
    m_pen_red = other.m_pen_red;

    m_is_required = other.m_is_required;
    m_is_enforced = other.m_is_enforced ;
}

void QConstraintBase::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
    if ( event->button() == Qt::RightButton ) {
        setSelected( !isSelected() );  // toggle selection
    }
}


void QConstraintBase::serialize( QDataStream &out )
{
    // qDebug() << Q_FUNC_INFO;

    out << m_is_required;
    out << m_is_enforced;

    out << m_pen_req;
    out << m_pen_red;
    out << m_altColor;

    out << markerSize();
    out << pos();
    out << rotation();
}

bool QConstraintBase::deserialize( QDataStream &in )
{
    // qDebug() << Q_FUNC_INFO;

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

void QConstraintBase::setColor( const QColor & col)
{
    m_pen_req.setColor(col);
    m_pen_red.setColor(col);
}

void QConstraintBase::setLineWidth( const int w)
{
    m_pen_req.setWidth(w);
    m_pen_red.setWidth(w);
}


void QConstraintBase::setLineStyle( const int s)
{
    m_pen_req.setStyle( Qt::PenStyle(s) );
    m_pen_red.setStyle( Qt::PenStyle(s) );
}


std::shared_ptr<QConstraintBase> QConstraintBase::clone() const
{
   // qDebug() << Q_FUNC_INFO;
   std::shared_ptr<QConstraintBase> ptr = doClone();
   auto & r = *ptr; // .get();
   assert( typeid( r ) == typeid(*this)
           && "QConstraintBase: doClone() incorrectly overridden" );
   return ptr;
}

void QConstraintBase::setAltColor(const QColor & col)
{
    m_altColor = col;
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


void QConstraintBase::setStatus( bool isrequired,
                                 bool isenforced) {
    m_is_required = isrequired;
    m_is_enforced = isenforced;
}



QRectF QAligned::boundingRect() const
{
    return a.boundingRect().adjusted(
                -2*m_pen_req.width(), -2*m_pen_req.width(),
                +2*m_pen_req.width(), +2*m_pen_req.width()
                );
}



void QAligned::paint( QPainter *painter,
                       const QStyleOptionGraphicsItem * option,
                       QWidget * widget)
{
    QPen pen = QPen( m_is_required ? m_pen_req : m_pen_red);
    if ( !enforced() ) {
        pen.setColor( Qt::red );
    }

    if ( showColor() ) {
        pen.setColor( m_altColor);
    }

    painter->setPen( pen );
    painter->drawLine( a.line() );
    QConstraintBase::paint( painter, option, widget);
}




qreal QAligned::markerSize() const
{
    return a.line().length()/2;
}



void QAligned::setMarkerSize ( const qreal s)
{
    a.setLine( -s, 0,  +s, 0);  // x1,y1; x2,y2
}



QAligned::QAligned()
{
    a.setLine( -s_defaultMarkerSize, 0,  +s_defaultMarkerSize, 0);
}



QAligned::QAligned( const QAligned & other)
    : QConstraintBase( other)
{
    // qDebug() << Q_FUNC_INFO;
}



std::shared_ptr<QConstraintBase> QAligned::doClone() const
{
    auto T = std::shared_ptr<QAligned>( new QAligned(*this));
    T->setMarkerSize( this->markerSize() );
    return std::move(T);
}


void QAligned::setGeometry( QVector<std::shared_ptr<const uStraightLineSegment>> &s,
                            const Eigen::VectorXi &idx)
{
    assert( idx.size()==1 );
    int i = idx(0);
    double x = 0.5*( s.at(i)->x()(0)+s.at(i)->y()(0));
    double y = 0.5*( s.at(i)->x()(1)+s.at(i)->y()(1));
    setPos( m_sc*x, m_sc*y);
    setRotation( s.at(i)->phi_deg() );   // orientation
    update();
}




std::shared_ptr<QConstraintBase> QAligned::create()
{
    return std::shared_ptr<QAligned>( new QAligned());
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

    painter->setPen( pen );
    painter->drawRect( rect() );

    QConstraintBase::paint( painter, option, widget);
}





QOrthogonal::QOrthogonal()
{
    setRect( -s_defaultMarkerSize,
             -s_defaultMarkerSize,
             2*s_defaultMarkerSize,
             2*s_defaultMarkerSize );
}

void QOrthogonal::setMarkerSize ( const qreal s)
{
    // qDebug() << Q_FUNC_INFO;
    setRect( -s, -s, 2*s, 2*s );
}



qreal QOrthogonal::markerSize() const
{
    // qDebug() << Q_FUNC_INFO;
    return rect().width()/2;
}


void QOrthogonal::setGeometry( QVector<std::shared_ptr<const uStraightLineSegment>> &s,
                               const Eigen::VectorXi &idx)
{
    // qDebug() << Q_FUNC_INFO;
    assert( idx.size()==2 );
    int i = idx(0);
    int j = idx(1);

    Vector3d xh = s.at(i)->hl().cross(  s.at(j)->hl()   );

    QConstraintBase::setPos( m_sc*xh(0)/xh(2),  m_sc*xh(1)/xh(2));
    QConstraintBase::setRotation( s.at(i)->phi_deg() );
    QConstraintBase::update();
}

QOrthogonal::QOrthogonal( const QOrthogonal & other)
    : QConstraintBase( other)
{
    // qDebug() << Q_FUNC_INFO;
}

std::shared_ptr<QConstraintBase> QOrthogonal::create()
{
    return std::shared_ptr<QOrthogonal>( new QOrthogonal());
}


std::shared_ptr<QConstraintBase> QOrthogonal::doClone() const
{
    // qDebug() << Q_FUNC_INFO;
    auto T = std::shared_ptr<QOrthogonal>( new QOrthogonal(*this));
    T->setMarkerSize( this->markerSize() );
    return std::move(T);
}

QCopunctual::QCopunctual()
{
    // qDebug() << Q_FUNC_INFO;
    setRect( -s_defaultMarkerSize,
             -s_defaultMarkerSize,
             2*s_defaultMarkerSize,
             2*s_defaultMarkerSize  );
}

QCopunctual::QCopunctual( const QCopunctual & other)
    : QConstraintBase( other )
{
    // qDebug() << Q_FUNC_INFO;
}


void QCopunctual::setMarkerSize ( const qreal s)
{
    // qDebug() << Q_FUNC_INFO;
    setRect( -s, -s, 2*s, 2*s );
}

std::shared_ptr<QConstraintBase> QCopunctual::create()
{
    return std::shared_ptr<QCopunctual>( new QCopunctual() );
}

void QCopunctual::setGeometry( QVector<std::shared_ptr<const uStraightLineSegment>> &s,
                               const Eigen::VectorXi &idx)
{
    assert( idx.size()==3 );
    int i = idx(0);
    int j = idx(1);
    // int k = idx(2);
    Vector3d xh = s.at(i)->hl().cross(   s.at(j)->hl()  );
    QConstraintBase::setPos( m_sc*xh(0)/xh(2), m_sc*xh(1)/xh(2) );
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
        // enlarge
        R.adjust( -m_pen_req.widthF(), -m_pen_req.widthF(),
                   m_pen_req.widthF(),  m_pen_req.widthF());
    }
    painter->drawEllipse( R );

    QConstraintBase::paint(painter, option, widget);
}

QRectF QCopunctual::boundingRect() const
{
    return rect().adjusted( -m_pen_req.width(),
                            -m_pen_req.width(),
                            +m_pen_req.width(),
                            +m_pen_req.width());
}

std::shared_ptr<QConstraintBase> QCopunctual::doClone() const
{
    auto T = std::shared_ptr<QCopunctual>( new QCopunctual(*this) );
    T->setMarkerSize( this->markerSize() );
    return std::move(T);
}

qreal QCopunctual::markerSize() const
{
    // qDebug() << Q_FUNC_INFO;
    return rect().width()/2;
}


QParallel::QParallel( const QParallel & other)
    : QConstraintBase( other )
{
    // qDebug() << Q_FUNC_INFO;
}


std::shared_ptr<QConstraintBase> QParallel::doClone() const
{
    auto T = std::shared_ptr<QParallel>( new QParallel(*this) );
    T->setMarkerSize( this->markerSize() );
    return std::move(T);
}




QParallel::QParallel()
{
    auto s = double( s_defaultMarkerSize );
    a.setLine( +s_sc*s, -s*(1-s_shear),  +s_sc*s, s*(1+s_shear));
    b.setLine( -s_sc*s, -s*(1+s_shear),  -s_sc*s, s*(1-s_shear));
}

void QParallel::setMarkerSize ( const qreal s)
{
    // qDebug() << Q_FUNC_INFO;
    a.setLine( +s_sc*s, -s*(1-s_shear),  +s_sc*s, s*(1+s_shear));
    b.setLine( -s_sc*s, -s*(1+s_shear),  -s_sc*s, s*(1-s_shear));
}

qreal QParallel::markerSize() const
{
    // qDebug() << Q_FUNC_INFO;
    return a.line().x1()/s_sc;
}



void QParallel::setGeometry( QVector<std::shared_ptr<const uStraightLineSegment>> &s,
                             const Eigen::VectorXi &idx)
{
    assert( idx.size()==2 );
    int i = idx(0);
    int j = idx(1);
    constexpr double N = 4;
    double x = ( s.at(i)->x()(0) +s.at(i)->y()(0) +s.at(j)->x()(0) +s.at(j)->y()(0) ) / N;
    double y = ( s.at(i)->x()(1) +s.at(i)->y()(1) +s.at(j)->x()(1) +s.at(j)->y()(1) ) / N;
    setPos( m_sc*x,  m_sc*y);            // position
    setRotation( s.at(i)->phi_deg() );   // orientation
    update();
}



QRectF QParallel::boundingRect() const
{
    QRectF R = a.boundingRect().united( b.boundingRect() );
    return R.adjusted(
                -2*m_pen_req.width(),
                -2*m_pen_req.width(),
                +2*m_pen_req.width(),
                +2*m_pen_req.width());
}

std::shared_ptr<QConstraintBase> QParallel::create()
{
    return std::shared_ptr<QParallel>( new QParallel() );
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
    // qDebug() << Q_FUNC_INFO;
    setPolygon( QPolygonF() << QPointF(-s, -s)
                << QPointF(+s, +s)
                << QPointF(+s, -s)
                << QPointF(-s, +s));
}


qreal QIdentical::markerSize() const
{
    // qDebug() << Q_FUNC_INFO;
    // return -polygon().first().x();
    return -polygon().at(0).x();
}



void QIdentical::setGeometry( QVector<std::shared_ptr<const uStraightLineSegment>> &s,
                              const Eigen::VectorXi &idx)
{
    assert( idx.size()==2 );
    int i = idx(0);
    int j = idx(1);

    Eigen::Matrix<double,2,4> xx;
    xx << s.at(i)->x(), s.at(i)->y(),s.at(j)->x(), s.at(j)->y();
    Eigen::Matrix4d DD;
    for (int l=0; l<4; l++) {
        for (int m=0; m<4; m++) {
            DD(l,m) = ( xx.col(l) -xx.col(m) ).norm();
        }
        DD(l,l) = DBL_MAX;
    }

    int ii;
    int jj;
    DD.minCoeff( &ii, &jj);
    QConstraintBase::setPos( m_sc*(xx(0,ii)+xx(0,jj))/2, m_sc*(xx(1,ii)+xx(1,jj))/2  );
    QConstraintBase::setRotation( s.at(i)->phi_deg() );
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



QIdentical::QIdentical( const QIdentical & other)
    : QConstraintBase(other)
{
    // qDebug() << Q_FUNC_INFO;
}



std::shared_ptr<QConstraintBase> QIdentical::create()
{
    return std::shared_ptr<QIdentical>( new QIdentical() );
}



std::shared_ptr<QConstraintBase> QIdentical::doClone() const
{
    auto T = std::shared_ptr<QIdentical>( new QIdentical(*this) );
    T->setMarkerSize( this->markerSize() );
    return std::move(T);
}

} // namespace QConstraint
