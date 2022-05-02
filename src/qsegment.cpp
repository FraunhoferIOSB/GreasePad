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
#include "qsegment.h"
#include "upoint.h"
#include "ustraightline.h"

#include <QDebug>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>

namespace QEntity {

using Uncertain::uStraightLine;
using Uncertain::uPoint;

bool QConstrained::s_show   = true;
bool QConstrained::s_showColor = false;
bool QUnconstrained::s_show = false;

bool QSegment::s_showUncertainty = false;

QPen & myQPen();
QPen & myQPen()
{
    // Initialized upon first call to the function
    static QPen p;
    return p;
}

QPen QUnconstrained::s_defaultPen = myQPen();
QPen   QConstrained::s_defaultPen = QPen();

static const double T_ZERO = 1e-7;

QSegment::QSegment() {
    // qDebug() << Q_FUNC_INFO;

    setFlag( ItemIsSelectable,         true);
    setFlag( ItemIsMovable,            false);
    setFlag( ItemSendsGeometryChanges, false);
    setFlag( ItemClipsChildrenToShape, true );
}

QSegment::QSegment( const uPoint & ux,
                    const uPoint & uy)
{
    // qDebug() << Q_FUNC_INFO;
    Vector3d xh = ux.v().normalized();
    Vector3d yh = uy.v().normalized();
    Q_ASSERT_X( xh.cross(yh).norm() >  T_ZERO,
                Q_FUNC_INFO,
                "identical end-points");   // TODO(meijoc)


    setFlag( ItemIsSelectable,         true);
    setFlag( ItemIsMovable,            false);
    setFlag( ItemSendsGeometryChanges, false);
    setFlag( ItemClipsChildrenToShape, true );
    setShape( ux, uy );

    qreal halfWidth = getSelectionOffset( ux, uy);
    createSelectionPolygon( halfWidth);
}

/*QSegment & QSegment::operator= ( const QSegment &other )
{
    // qDebug() << Q_FUNC_INFO;
    if ( &other != this ) {
        ellipse_      = other.ellipse_;
        branch_  = other.branch_;
        line_   = other.line_;
    }
    return *this;
}*/

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
        painter->setPen( QPen( Qt::blue, 2, Qt::DashLine, Qt::RoundCap) );
        painter->drawPolygon( minBBox );
    }

}


QRectF QSegment::boundingRect() const
{
    // qDebug() << Q_FUNC_INFO;
    //    QRectF result = ellipse_.first.boundingRect().united(
    //                ellipse_.second.boundingRect()).normalized();
    QRectF result = minBBox.boundingRect();
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
    ret.addPolygon( minBBox );
    return ret;
}



QPolygonF QSegment::toPoly(std::pair<Eigen::VectorXd, Eigen::VectorXd> p)
{
    int N = static_cast<int>( p.first.size() );
    QPolygonF poly( N );
    for ( int i=0; i<N; i++) {
        poly[i] = QPointF( p.first(i), p.second(i));
    }
    return poly;
}

double QSegment::getSelectionOffset( const uPoint & ux,
                                     const uPoint & uy) const
{
    double m = 0;

    uPoint u1 = ux.euclidean();
    Eigen::EigenSolver<Eigen::Matrix2d> eig1( u1.Cov().topLeftCorner(2,2) );
    m = std::max( m, eig1.eigenvalues().real().cwiseAbs().maxCoeff() );

    uPoint u2 = uy.euclidean();
    Eigen::EigenSolver<Eigen::Matrix2d> eig2( u2.Cov().topLeftCorner(2,2) );
    m = std::max( m, eig2.eigenvalues().real().cwiseAbs().maxCoeff() );

    return 1000*sqrt(4.6052*m);  // chi2inv(0.9, 2)
}

void QSegment::setShape( const uPoint &ux,
                         const uPoint &uy)
{
    // qDebug() << Q_FUNC_INFO;
    const double k2 = 4.6; // ch2inv(0.9,2)   TODO(meijoc)

    Vector3d xh = ux.v().normalized();
    Vector3d yh = uy.v().normalized();

    // check .........................................
    Q_ASSERT_X( xh.cross(yh).norm() >  T_ZERO,
                Q_FUNC_INFO,
                "identical end-points");   // TODO(meijoc)
    if ( std::fabs( xh(2) )>0.  &&  std::fabs( yh(2) )>0. ) {
        line_.setLine( 1000*xh(0)/xh(2), 1000*xh(1)/xh(2),
                       1000*yh(0)/yh(2), 1000*yh(1)/yh(2) );
    }

    if ( k2<=0.) {
        ellipse_.first.clear();
        ellipse_.second.clear();
        branch_.first.clear();
        branch_.second.clear();   //update();
        return;
    }

    // unconditioning .........................................
    const Matrix3d TT = ( Matrix3d() << 1000,0,0, 0,1000,0, 0,0,1 ).finished();
    uPoint x2 = ux.transformed(TT);
    uPoint y2 = uy.transformed(TT);

    Q_ASSERT( x2.v().norm() > T_ZERO );
    Q_ASSERT( y2.v().norm() > T_ZERO );
    Q_ASSERT( x2.v().cross( y2.v() ).norm()  > T_ZERO );

    // two ellipses ...............................................
    Conic::Ellipse ell_x( x2, k2);
    Conic::Ellipse ell_y( y2, k2);
    ellipse_.first  = toPoly( ell_x.poly( 64 ) );
    ellipse_.second = toPoly( ell_y.poly( 64 ) );

    // hyperbola ..................................................
    Conic::Hyperbola hyp( uStraightLine( x2,y2 ),  k2);

    // Two polar lines ............................................
    Vector3d lx = ell_y.polar( x2.v() ).normalized();
    Vector3d ly = ell_x.polar( y2.v() ).normalized();

    // Four tangent points ........................................
    std::pair<Vector3d,Vector3d> pair1 = hyp.intersect( lx );
    std::pair<Vector3d,Vector3d> pair2 = hyp.intersect( ly );

    // Hyperbola: Axis and point of symmetry ......................
    Vector3d symaxis = hyp.centerline();
    const Vector3d x0 = hyp.center();
    Vector3d m;              // perpendicular m, passing the center
    m(0) = -symaxis(1);
    m(1) = +symaxis(0);
    assert( std::fabs(x0(2))>0. );
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
    Eigen::Vector4d lr;
    lr(0) = symaxis.dot( pair1.first );
    lr(1) = symaxis.dot( pair1.second);
    lr(2) = symaxis.dot( pair2.first );
    lr(3) = symaxis.dot( pair2.second);

    // orthogonal distances of the 4 tangent points to m.
    Eigen::Vector4d ud;
    ud(0) = m.dot( pair1.first);
    ud(1) = m.dot( pair1.second);
    ud(2) = m.dot( pair2.first);
    ud(3) = m.dot( pair2.second);

    Eigen::Vector4d d = Eigen::Vector4d::Zero();
    for (int i=0; i<4; i++) {
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
    auto ab = hyp.lengthsSemiAxes();
    double aa = ab.first*ab.first;
    double bb = ab.second*ab.second;

    // transformation branches (motion) ..........................
    QTransform t;
    t.translate( x0(0)/x0(2), x0(1)/x0(2) );
    t.rotate( hyp.angle_deg() );

    // first branch of hyperbola .................................
    Eigen::VectorXd xi = Eigen::VectorXd::LinSpaced( 64, d(0),d(1) );
    Eigen::VectorXd yi = xi;
    branch_.first.clear();
    for ( Eigen::Index i=0; i<xi.size(); i++ )
    {
        yi(i) = +sqrt( xi(i)*xi(i) *aa/(bb) +aa);
        branch_.first << QPointF( xi(i),yi(i) );
    }
    branch_.first = t.map( branch_.first );

    // second branch of hyperbola .................................
    xi = Eigen::VectorXd::LinSpaced( 64, d(3), d(2) );
    branch_.second.clear();
    yi =  -((xi.array().square() *aa/(bb)).array() +aa ).sqrt();
    for ( Eigen::Index i=0; i<xi.size(); i++ ) {
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


QUnconstrained::QUnconstrained()
{
    // qDebug() << Q_FUNC_INFO;
    setVisible( QUnconstrained::show() );
    setPen( s_defaultPen );
}

QConstrained::QConstrained()
{
    // qDebug() << Q_FUNC_INFO;
    setVisible( QConstrained::show() );
    altColor = Qt::black;
    setPen( s_defaultPen );
}

QConstrained::QConstrained( const uPoint & ux,
                            const uPoint & uy)
    : QSegment(ux,uy) {
    setVisible( QConstrained::show() );
    setPen( s_defaultPen );
    altColor = Qt::black;
}

void QUnconstrained::paint( QPainter *painter,
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
    out << minBBox;
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
    in >> minBBox;

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

    QRectF rect(
                -line().length()/2-halfWidth,
                -halfWidth,
                line().length()+2*halfWidth,
                2*halfWidth );
    QPolygonF poly(rect);

    QTransform t;
    t.translate( line().center().x(),
                 line().center().y() );
    t.rotate(   -line().angle());

    minBBox.clear();
    minBBox = t.map( poly);
}

QUnconstrained::QUnconstrained( const uPoint & ux,
                                const uPoint & uy)
    : QSegment(ux,uy)
{
     setVisible( QUnconstrained::show() );
     setPen( s_defaultPen );
}

} // namespace QEntity
