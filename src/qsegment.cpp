/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2026 Jochen Meidow, Fraunhofer IOSB
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


#include "geometry/conics.h"
#include "matfun.h"
#include "qsegment.h"
#include "upoint.h"
#include "ustraightline.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <QDebug>
#include <QGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>

#include "qassert.h"
#include "qnamespace.h"
#include "qsharedpointer.h"
#include "qtdeprecationdefinitions.h"
#include "qtpreprocessorsupport.h"
#include "qtypes.h"

#include <algorithm>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <utility>


using Eigen::EigenSolver;
using Eigen::Index;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Uncertain::uStraightLine;
using Uncertain::uPoint;

using Matfun::cof3;


namespace QEntity {

bool QConstrained::s_show   = true;
bool QConstrained::s_showColor = false;
bool QUnconstrained::s_show = false;

bool QSegment::s_showUncertainty = false;

QPen QSegment::s_penSelected = QPen();

namespace {
QPen & myQPen();

//! Initialized pen upon first call to the function
QPen & myQPen()
{
    static QPen p;
    return p;
}

} // namespace


QPen QUnconstrained::s_defaultPen = myQPen();
QPen   QConstrained::s_defaultPen = QPen();

QSegment::QSegment( QGraphicsItem * parent )
    : QGraphicsItem(parent)
{
    // qDebug() << Q_FUNC_INFO;

    setFlag( ItemIsSelectable,         true);
    setFlag( ItemIsMovable,            false);
    setFlag( ItemSendsGeometryChanges, false);
    setFlag( ItemClipsChildrenToShape, true );
}

QSegment::QSegment(const uPoint &ux, const uPoint &uy)
{
    // qDebug() << Q_FUNC_INFO;
    const Vector3d xh = ux.v().normalized();
    const Vector3d yh = uy.v().normalized();
    Q_ASSERT_X( xh.cross(yh).norm() > FLT_EPSILON,
                Q_FUNC_INFO,
                "identical end-points");

    setFlag( ItemIsSelectable,         true);
    setFlag( ItemIsMovable,            false);
    setFlag( ItemSendsGeometryChanges, false);
    setFlag( ItemClipsChildrenToShape, true);
    setShape( ux, uy );

    const qreal halfWidth = getSelectionOffset( ux, uy);
    createSelectionPolygon( halfWidth);
}


void QSegment::paint( QPainter *painter,
                      const QStyleOptionGraphicsItem *option,
                      QWidget *widget)
{
    Q_UNUSED( widget )
    Q_UNUSED( option )
    // qDebug() << Q_FUNC_INFO << boundingRect() << showUncertainty_;
    // ??? painter->setClipRect( boundingRect());

    painter->drawLine(line_);

    if ( s_showUncertainty ) {
        painter->drawPolygon(  ellipse_.first );
        painter->drawPolygon(  ellipse_.second);
        painter->drawPolyline( branch_.first  );
        painter->drawPolyline( branch_.second );
    }

    if ( isSelected() ) {
        painter->setPen( s_penSelected );
        painter->drawPolygon( selectionPolygon_ );
    }
}


QRectF QSegment::boundingRect() const
{
    // qDebug() << Q_FUNC_INFO;

    QRectF result = selectionPolygon_.boundingRect();
    result.adjust( -pen_.width(),
                   -pen_.width(),
                   +pen_.width(),
                   +pen_.width() );

    if ( !result.isValid() ) {
        result = QRectF( line_.p1(),line_.p2() ).normalized();
    }

    return result;
}


QPainterPath QSegment::shape() const
{
    QPainterPath ret;
    ret.addPolygon( selectionPolygon_ );

    return ret;
}


QPolygonF QSegment::toPoly(std::pair<Eigen::VectorXd, Eigen::VectorXd> p)
{
    const Index N = p.first.size();
    QPolygonF poly( N );
    for ( int i=0; i<N; i++) {
        poly[i] = QPointF( p.first(i), p.second(i));
    }

    return poly;
}


double QSegment::getSelectionOffset(const uPoint &ux, const uPoint &uy)
{
    double m = 0;

    const uPoint u1 = ux.euclidean();
    const EigenSolver<Eigen::Matrix2d> eig1(u1.Cov().topLeftCorner(2, 2));
    m = std::max( m, eig1.eigenvalues().real().cwiseAbs().maxCoeff());

    const uPoint u2 = uy.euclidean();
    const EigenSolver<Eigen::Matrix2d> eig2(u2.Cov().topLeftCorner(2, 2));
    m = std::max( m, eig2.eigenvalues().real().cwiseAbs().maxCoeff() );

    return m_scale*sqrt(m_quantile*m);  // chi2inv(0.9, 2)
}


void QSegment::setShape( const uPoint &ux,
                         const uPoint &uy)
{
    // qDebug() << Q_FUNC_INFO;
    // const double k2 = 4.6; // ch2inv(0.9,2)
    constexpr int nSupport = 64;

    const Vector3d xh = ux.v().normalized();
    const Vector3d yh = uy.v().normalized();

    // check .........................................
    Q_ASSERT_X( xh.cross(yh).norm() >  FLT_EPSILON,
                Q_FUNC_INFO,
                "identical end-points");
    if ( std::fabs( xh(2) )>0.  &&  std::fabs( yh(2) )>0. ) {
        line_.setLine( m_scale*xh(0)/xh(2), m_scale*xh(1)/xh(2),
                       m_scale*yh(0)/yh(2), m_scale*yh(1)/yh(2) );
    }

    /*if ( k2<=0.) {
        ellipse_.first.clear();
        ellipse_.second.clear();
        branch_.first.clear();
        branch_.second.clear();   //update();
        return;
    }*/

    // unconditioning .........................................
    const Matrix3d TT = Vector3d(m_scale, m_scale, 1).asDiagonal();
    const uPoint x2 = ux.transformed(TT);
    const uPoint y2 = uy.transformed(TT);

    Q_ASSERT( x2.v().norm() > FLT_EPSILON );
    Q_ASSERT( y2.v().norm() > FLT_EPSILON );
    Q_ASSERT( x2.v().cross( y2.v() ).norm()  > FLT_EPSILON );

    // two ellipses ...............................................
    uPoint ux2 = x2.euclidean();
    Matrix3d CC = cof3( m_quantile*ux2.Cov() -ux2.v()*ux2.v().adjoint() );
    const Geometry::Ellipse ell_x(CC);

    ux2 = y2.euclidean();
    CC = cof3( m_quantile*ux2.Cov() -ux2.v()*ux2.v().adjoint() );
    const Geometry::Ellipse ell_y(CC);

    ellipse_.first  = toPoly( ell_x.poly( nSupport ) );
    ellipse_.second = toPoly( ell_y.poly( nSupport ) );

    // hyperbola ..................................................
    uStraightLine ul(x2.cross(y2));
    ul = ul.euclidean();
    CC = m_quantile*ul.Cov() -ul.v()*ul.v().adjoint();
    const Geometry::Hyperbola hyp( CC );

    // Two polar lines ............................................
    const Vector3d lx = ell_y.polar( x2.v() ).normalized();
    const Vector3d ly = ell_x.polar( y2.v() ).normalized();

    // Four tangent points ........................................
    std::pair<Vector3d,Vector3d> pair1 = hyp.intersect( lx );
    std::pair<Vector3d,Vector3d> pair2 = hyp.intersect( ly );

    // Hyperbola: Axis and point of symmetry ......................
    Vector3d symaxis = hyp.centerline();
    const Vector3d x0 = hyp.center();
    assert( std::fabs(x0(2))>0. );
    Vector3d m;              // perpendicular m, passing the center
    m(0) = -symaxis(1);
    m(1) = +symaxis(0);
    m(2) = -x0.head(2).dot( m.head(2) )/ x0(2);

    // checks before Euclidean normalization
    assert( std::fabs( symaxis.head(2).norm() ) >0. );
    assert( std::fabs(       m.head(2).norm() ) >0. );
    assert( std::fabs( pair1.first(2)  ) >0. );
    assert( std::fabs( pair1.second(2) ) >0. );
    assert( std::fabs( pair2.first(2)  ) >0. );
    assert( std::fabs( pair2.second(2) ) >0. );

    // Euclidean normalizations for efficient distance computations
    symaxis /= symaxis.head(2).norm();
    m /= m.head(2).norm();
    pair1.first  /= pair1.first(2);
    pair1.second /= pair1.second(2);
    pair2.first  /= pair2.first(2);
    pair2.second /= pair2.second(2);

    // Euclidean distances of tangent points to axis a.
    Vector4d lr;
    lr(0) = symaxis.dot( pair1.first );
    lr(1) = symaxis.dot( pair1.second);
    lr(2) = symaxis.dot( pair2.first );
    lr(3) = symaxis.dot( pair2.second);

    // orthogonal distances of the 4 tangent points to m.
    Vector4d ud;
    ud(0) = m.dot( pair1.first);
    ud(1) = m.dot( pair1.second);
    ud(2) = m.dot( pair2.first);
    ud(3) = m.dot( pair2.second);

    Vector4d d = Vector4d::Zero();
    for (int i=0; i<ud.size(); i++) {
        if ( lr(i) < 0 && ud(i) < 0) {
            d(2) = -ud(i);
        }
        else
            if ( lr(i) < 0 && ud(i) > 0) {
                d(3) = -ud(i);
            }
            else {
                if ( lr(i) > 0 && ud(i) < 0) {
                    d(0) = -ud(i);
                }
                else {
                    if ( lr(i) > 0 && ud(i) > 0) {
                        d(1) = -ud(i);
                    }
                }
            }
    }

    // hyperbola: branches to plot
    const auto ab = hyp.lengthsSemiAxes();
    const double aa = ab.first * ab.first;
    const double bb = ab.second * ab.second;

    // transformation branches (motion) ..........................
    QTransform t;
    t.translate( x0(0)/x0(2), x0(1)/x0(2) );
    t.rotate( hyp.angle_deg(), Qt::ZAxis );

    // first branch of hyperbola .................................
    VectorXd xi = VectorXd::LinSpaced( nSupport, d(0),d(1) );
    VectorXd yi = xi;
    branch_.first.clear();
    for ( Eigen::Index i=0; i<xi.size(); i++ )
    {
        yi(i) = +sqrt( xi(i)*xi(i) *aa/(bb) +aa);
        branch_.first << QPointF( xi(i),yi(i) );
    }
    branch_.first = t.map( branch_.first );

    // second branch of hyperbola .................................
    xi = VectorXd::LinSpaced( nSupport, d(3), d(2) );
    branch_.second.clear();
    yi =  -((xi.array().square() *aa/(bb)).array() +aa ).sqrt();
    for ( Index i=0; i<xi.size(); i++ ) {
        branch_.second << QPointF( xi(i), yi(i) );
    }
    branch_.second = t.map( branch_.second );
}


void QConstrained::paint( QPainter *painter,
                          const QStyleOptionGraphicsItem *option,
                          QWidget *widget)
{
    Q_UNUSED(widget)
    Q_UNUSED(option)
    // qDebug() << Q_FUNC_INFO;
    // ??? TODO painter->setClipRect( boundingRect());
    QPen p = pen();
    if ( s_showColor ) {
        p.setColor( altColor);
    }
    painter->setPen( p);
    QSegment::paint(painter,option,widget);
}


QUnconstrained::QUnconstrained( QGraphicsItem * /* parent */ )
{
    // qDebug() << Q_FUNC_INFO;
    setVisible( QUnconstrained::show() );
    setPen( s_defaultPen );
}


QConstrained::QConstrained()
    : altColor(Qt::black)
{
    // qDebug() << Q_FUNC_INFO;
    setVisible( QConstrained::show() );
    setPen(s_defaultPen);
}


QConstrained::QConstrained(const uPoint &ux, const uPoint &uy)
    : QSegment(ux, uy)
    , altColor(Qt::black)
{
    setVisible( QConstrained::show() );
    setPen( s_defaultPen );
}


void QUnconstrained::paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget)
{
    Q_UNUSED(widget)
    Q_UNUSED(option)
    // qDebug() << Q_FUNC_INFO;
    // ??? TODO painter->setClipRect( boundingRect());
    painter->setPen( pen() );
    QSegment::paint(painter,option,widget);
}

void QConstrained::setAltColor(const QColor & col)
{
    altColor = col;
}


void QSegment::serialize( QDataStream & out ) const
{
    // qDebug() << Q_FUNC_INFO;

    out << pen_;
    out << line_;
    out << branch_.first;
    out << branch_.second;
    out << ellipse_.first;
    out << ellipse_.second;
    out << selectionPolygon_;
}

bool QSegment::deserialize( QDataStream &in )
{
    // qDebug() << Q_FUNC_INFO;

    in >> pen_;
    in >> line_;
    in >> branch_.first;
    in >> branch_.second;
    in >> ellipse_.first;
    in >> ellipse_.second;
    in >> selectionPolygon_;

    return in.status()==0;
}



void QSegment::mousePressEvent( QGraphicsSceneMouseEvent * event )
{

    // qDebug() << Q_FUNC_INFO;
    if ( event->button() == Qt::RightButton ) {
        setSelected( !isSelected());
    }

}


void QSegment::createSelectionPolygon( const qreal halfWidth)
{
    // qDebug() << Q_FUNC_INFO;

    const QRectF rect(-line().length() / 2 - halfWidth,
                      -halfWidth,
                      line().length() + 2 * halfWidth,
                      2 * halfWidth);
    const QPolygonF poly(rect);

    QTransform t;
    t.translate( line().center().x(),
                 line().center().y() );
    t.rotate(   -line().angle());

    selectionPolygon_.clear();
    selectionPolygon_ = t.map( poly);
}


QUnconstrained::QUnconstrained( const uPoint & ux,
                                const uPoint & uy)
    : QSegment(ux,uy)
{
     setVisible( QUnconstrained::show() );
     setPen( s_defaultPen );
}


} // namespace QEntity
