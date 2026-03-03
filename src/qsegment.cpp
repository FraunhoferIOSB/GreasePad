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
#include "qsegment.h"
#include "uncertain/upoint.h"
#include "uncertain/ustraightline.h"

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
    // set QLineF
    const Vector3d xh = ux.v().normalized();
    const Vector3d yh = uy.v().normalized();
    assert( xh.cross(yh).norm() > FLT_EPSILON && "identical end-points");
    if ( std::fabs(xh(2)) > 0 && std::fabs(yh(2)) > 0 ) {
        line_.setLine( m_scale*xh(0)/xh(2), m_scale*xh(1)/xh(2),
                       m_scale*yh(0)/yh(2), m_scale*yh(1)/yh(2) );
    }

    // unconditioning
    const Matrix3d TT = Vector3d(m_scale, m_scale, 1).asDiagonal();
    const uPoint ux2 = ux.transformed(TT);
    const uPoint uy2 = uy.transformed(TT);

    assert( ux2.v().norm() > FLT_EPSILON );
    assert( uy2.v().norm() > FLT_EPSILON );
    assert( ux2.v().cross(uy2.v()).norm() > FLT_EPSILON );

    // Points: two ellipses
    constexpr int nSupport = 64;
    const Matrix3d CCx = ux2.conicMatrix(m_quantile);
    const Matrix3d CCy = uy2.conicMatrix(m_quantile);
    const Geometry::Ellipse ell_x(CCx);
    const Geometry::Ellipse ell_y(CCy);
    ellipse_.first  = toPoly( ell_x.poly( nSupport ) );
    ellipse_.second = toPoly( ell_y.poly( nSupport ) );

    // Straight line: hyperbola
    const uStraightLine ul(ux2.cross(uy2));
    const Matrix3d CCl = ul.conicMatrix(m_quantile);
    const Geometry::Hyperbola hyp( CCl );

    // Two polar lines
    const Vector3d lx = (CCy*ux2.v()).normalized();
    const Vector3d ly = (CCx*uy2.v()).normalized();

    // Four tangent points
    std::pair<Vector3d,Vector3d> pair1 = hyp.intersect( lx );
    std::pair<Vector3d,Vector3d> pair2 = hyp.intersect( ly );

    // Hyperbola: Axis and point of symmetry
    Vector3d symaxis = hyp.centerline();
    const Vector3d x0 = hyp.center();
    assert( std::fabs(x0(2)) > 0 );

    // hyperbola: branches to plot
    const auto ab = hyp.lengthsSemiAxes();
    const double aa = ab.first * ab.first;
    const double bb = ab.second * ab.second;

    // perpendicular m, passing the center
    Vector3d m;
    m(0) = -symaxis(1);
    m(1) = +symaxis(0);
    m(2) = -x0.head(2).dot( m.head(2) )/ x0(2);

    // checks before Euclidean normalization
    assert( std::fabs( symaxis.head(2).norm() ) > 0 );
    assert( std::fabs(       m.head(2).norm() ) > 0 );
    assert( std::fabs( pair1.first(2)  ) > 0 );
    assert( std::fabs( pair1.second(2) ) > 0 );
    assert( std::fabs( pair2.first(2)  ) > 0 );
    assert( std::fabs( pair2.second(2) ) > 0 );

    // Euclidean normalizations for efficient distance computations
    symaxis /= symaxis.head(2).norm();
    m /= m.head(2).norm();
    pair1.first  /= pair1.first(2);
    pair1.second /= pair1.second(2);
    pair2.first  /= pair2.first(2);
    pair2.second /= pair2.second(2);

    // Euclidean distances of tangent points to axis
    const Vector4d lr {
        symaxis.dot( pair1.first ),
        symaxis.dot( pair1.second),
        symaxis.dot( pair2.first ),
        symaxis.dot( pair2.second)
    };

    // orthogonal distances of the 4 tangent points
    const Vector4d ud {
        m.dot( pair1.first),
        m.dot( pair1.second),
        m.dot( pair2.first),
        m.dot( pair2.second)
    };

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

    // first branch of hyperbola .................................
    branch_.first.clear();
    VectorXd xi = VectorXd::LinSpaced( nSupport, d(0),d(1) );
    VectorXd yi = (xi.array().square() *aa/bb +aa).sqrt();
    for (Index i=0; i<xi.size(); i++) {
        branch_.first << QPointF(xi(i),yi(i));
    }

    // second branch of hyperbola .................................
    branch_.second.clear();
    xi = VectorXd::LinSpaced( nSupport, d(3), d(2) );
    yi = -(xi.array().square() *aa/bb +aa).sqrt();
    for (Index i=0; i<xi.size(); i++) {
        branch_.second << QPointF(xi(i),yi(i));
    }

    // transformation branches (motion)
    QTransform t;
    t.translate( x0(0)/x0(2), x0(1)/x0(2) );
    t.rotate( hyp.angle_deg(), Qt::ZAxis );
    branch_.first  = t.map( branch_.first );
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
