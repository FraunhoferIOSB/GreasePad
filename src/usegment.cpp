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


#include "matfun.h"
#include "udistance.h"
#include "uncertain.h"
#include "upoint.h"
#include "usegment.h"
#include "ustraightline.h"

#include <Eigen/Core>

#include <QDebug>

#include "qsharedpointer.h"

#include <cassert>
#include <cfloat>
#include <cmath>
#include <memory>

using Eigen::Matrix3d;
using Eigen::VectorXi;
using Eigen::Vector3d;
using Eigen::VectorXd;

using Matfun::sign;


namespace Uncertain {

//! Get uncertain endpoint x=S(l)*m in homogeneous coordinates
uPoint uStraightLineSegment::ux() const
{
    const Matrix<double, 6,6> Sigma_lm = m_Cov_tt.topLeftCorner(6,6);
    const Matrix3d Sl = skew( hl() );

    Matrix<double,3,6> JJ;
    JJ << -skew( hm() ), Sl;

    // x = Sl*m:
    return { Sl*hm(), JJ*Sigma_lm*JJ.adjoint() };
}


//! Get uncertain endpoint y=S(l)*n in homogeneous coordinates
uPoint uStraightLineSegment::uy() const
{
    const Matrix3d Sl = skew( hl() );

    Matrix<double,3,9> JJ;
    JJ << -skew( hn() ),  Matrix3d::Zero(), Sl;

    return { Sl*hn(), JJ*Cov_tt()*JJ.adjoint() };
}


//! Construction of an uncertain straight line segment via two uncorrelated endpoints
uStraightLineSegment::uStraightLineSegment( const uPoint & ux,
                                            const uPoint & uy)
    : m_bounding_box( ux.bbox().united( uy.bbox()) )
{
    static const Matrix3d CC = Vector3d(1,1,0).asDiagonal();

    // (1) 9-parameter vector t = [l; m; n];
    const Matrix3d Sx = skew( ux.v() );
    const Matrix3d Sy = skew( uy.v() );
    const Matrix3d UUx = Sx*CC*Sx;
    const Matrix3d UUy = Sy*CC*Sy;

    const Vector3d l = Sx*uy.v();
    assert( l.norm() > FLT_EPSILON && "identical points.");

    const double xh = ux.v()(2);
    const double yh = uy.v()(2);

    m_t.segment( 0, 3) = l;
    m_t.segment( 3, 3) = +sign(yh)*UUx*uy.v();
    m_t.segment( 6, 3) = -sign(xh)*UUy*ux.v();

    // (2) 9x9 covariance matrix
    //Matrix3d JJlx = -Sy;
    //Matrix3d JJly = Sx;

    const Matrix3d JJmx = -sign(yh) * (Sx*CC*Sy + skew((CC*l).eval()) );
    const Matrix3d JJmy = +sign(yh) * UUx;

    const Matrix3d JJnx = -sign(xh)*UUy;
    const Matrix3d JJny = +sign(xh)*( Sy*CC*Sx -skew( (CC*l).eval() ));

    /* JJ = [ ...
        JJlx, JJly; ...
        JJmx, JJmy; ...
        JJnx, JJny];  */

    Matrix<double,9,6> JJ;
    JJ << -Sy, Sx,
        JJmx, JJmy,
        JJnx, JJny;

    Matrix<double,6,6> Sigma_xy;
    Sigma_xy.setZero();
    Sigma_xy.topLeftCorner(3,3)     = ux.Cov();
    Sigma_xy.bottomRightCorner(3,3) = uy.Cov();

    m_Cov_tt = JJ*Sigma_xy*JJ.transpose();
}


//! Check if the 'this' and the uncertain straight line segment 'ut' intersect (deterministic)
bool uStraightLineSegment::intersects( const uStraightLineSegment &ut) const
{
    const double d1 = ut.hx().dot( hl() );
    const double d2 = ut.hy().dot( hl() );
    if ( d1*d2 >= 0  ) {   // same sign?
        return false;
    }

    const double d3 = hx().dot( ut.hl() );
    const double d4 = hy().dot( ut.hl() );

    return !(d3*d4 >=0 );
}


//! Check if one of the endpoints of the uncertain straight line segment 'other' is incident.
bool uStraightLineSegment::touches( const uStraightLineSegment & other,
                                    const double T_dist,
                                    const double T_in) const
{
    if ( other.touchedBy( ux(), T_dist, T_in) ) {
        return true;
    }
    if ( other.touchedBy( uy(), T_dist, T_in) ) {
        return true;
    }

    return false;
}


//! Check if the uncertain point 'ux' is incident.
bool uStraightLineSegment::touchedBy( const uPoint & ux,
                                      const double T_dist,
                                      const double T_in) const
{
    if ( !ux.isIncidentWith(ul(), T_in) ) { // dist(x,l)
        return false;
    }

    const uDistance ud1 = ux.distanceAlgebraicTo( um() );  // dist(x,m)
    if ( !ud1.isLessThanZero(T_dist) ) {
        return false;
    }

    const uDistance ud2 = ux.distanceAlgebraicTo( un() );  // dist(x,n)
    if ( !ud2.isGreaterThanZero(T_dist) ) {
        return false;
    }

    return true;
}


//! Get the endpoints connecting uncertain straight line l = S(x)*y
uStraightLine uStraightLineSegment::ul() const
{
    const Matrix3d Sigma_ll = m_Cov_tt.topLeftCorner(3,3);

    return { m_t.head(3), Sigma_ll };
}


//! Get the delimiting uncertain straight line m in homogeneous coordinates.
uStraightLine uStraightLineSegment::um() const
{
    const Matrix3d Sigma_mm = m_Cov_tt.block(3,3,3,3);

    return { m_t.segment(3,3), Sigma_mm};
}


//! Get the delimiting uncertain straight line n in homogeneous coordinates.
uStraightLine uStraightLineSegment::un() const
{
    const Matrix3d Sigma_nn = m_Cov_tt.bottomRightCorner(3,3);

    return { m_t.tail(3), Sigma_nn};
}


//! Get the endpoint x in homogeneous coordinates
Vector3d uStraightLineSegment::hx() const
{
    const Vector3d l = m_t.head(3);
    const Vector3d m = m_t.segment(3,3);

    return skew( l )*m;
}


//! Get the endpoint y in homogeneous coordinates
Vector3d uStraightLineSegment::hy() const
{
    const Vector3d l = m_t.head(3);
    const Vector3d n = m_t.tail(3);

    return skew( l )*n;
}


//! Move endpoint x along l to intersection point of m and l.
bool uStraightLineSegment::move_x_to(const Vector3d &m)
{
    const Vector3d l = hl().normalized();
    const Vector3d z = l.cross( m.normalized() );     // intersection point z
    if ( z.norm() < FLT_EPSILON ) {
        return false; // l==m
    }

    const Vector2d dx = x() - z.head(2) / z(2);
    const Matrix3d HH {
        { 1, 0, dx(0)},
        { 0, 1, dx(1)},
        { 0, 0,    1 } };
    Matrix9d TT = Matrix9d::Identity();
    TT.block(3,3,3,3) = HH.adjoint();  // m
    transform( TT);

    return true;
}


//! Move endpoint y along l to intersection point of n and l.
bool uStraightLineSegment::move_y_to( const Vector3d & n )
{
    // intersection point z ............................
    const Vector3d l = hl().normalized();
    const Vector3d z = l.cross( n.normalized() );
    if ( z.norm() < FLT_EPSILON) {
        return false; // l==n
    }

    const Vector2d dx = y() - z.head(2) / z(2);
    const Matrix3d HH {
        { 1,  0, dx(0)},
        { 0,  1, dx(1)},
        { 0,  0,    1 } };
    Matrix9d TT = Matrix9d::Identity();
    TT.block(6,6,3,3) = HH.adjoint();  // n
    transform( TT);

    return true;
}


//! Transform uncertain straight line segment via 9x9 transformation matrix for t = [l',m',n']'.
void uStraightLineSegment::transform( const Matrix9d & TT)
{
         m_t = TT*m_t;
    m_Cov_tt = TT*m_Cov_tt*TT.adjoint();
}

/* Matlab:  [~,idx] = sort(x);
VectorXi uStraightLineSegment::indices_of_sorting( const VectorXd &v)
{
    //    size_t N = v.size();
    //    std::vector<int> idx( N );
    //    std::generate( idx.begin(), idx.end(),
    //                   [n=N-1] () mutable { return n--; } );
    //    auto comparator = [&v](unsigned int a, unsigned int b){ return v[a] < v[b]; };
    //    std::sort( idx.begin(), idx.end(), comparator);

    //    return idx;

    Eigen::Index N = v.size();
    VectorXi idx = VectorXi::LinSpaced( N,0,static_cast<int>(N-1) );  // [0,1,...,N-1]
    auto comparator = [&v](unsigned int a, unsigned int b){ return v[a] < v[b]; };
    std::sort( idx.begin(), idx.end(), comparator);
    return idx;
}*/

namespace {

QDataStream & operator>> (QDataStream & in, Aabb<double> & bbox)
{
    double x_min = NAN;
    double x_max = NAN;
    double y_min = NAN;
    double y_max = NAN;
    in >> x_min >> x_max >> y_min >> y_max;
    bbox = Aabb<double>(
        Vector2d(x_min, y_min),
        Vector2d(x_max, y_max)   );

    return in;
}


//! Overloaded >>operator for 9-vectors
QDataStream & operator>> ( QDataStream & in, Eigen::Matrix<double,9,1> & v)
{
    for ( double & val : v ) {
        in >> val;
    }

    return in;
}


QDataStream & operator<< (QDataStream & out, const Aabb<double> & bbox)
{
    out
        << bbox.min(0) << bbox.max(0)
        << bbox.min(1) << bbox.max(1);

    return out;
}


//! Overloaded >>operator for 9x9 matrices
QDataStream & operator>> ( QDataStream & in, Eigen::Matrix<double,9,9> & MM)
{
    for ( int r=0; r<MM.rows(); r++) {
        for ( int c=0; c<MM.cols(); c++) {
            in >> MM(r,c);
        }
    }

    return in;
}


//! Overloaded <<operator for 9-vectors
QDataStream & operator<< ( QDataStream & out, const Eigen::Matrix<double,9,1> &v)
{
    //qDebug() << Q_FUNC_INFO;
    for ( double val : v) {
        out << val;
    }

    return out;
}


//! Overloaded <<operator for 9x9 matrices
QDataStream & operator<< ( QDataStream & out, const Eigen::Matrix<double,9,9> &MM)
{
    //qDebug() << Q_FUNC_INFO;
    for ( int r=0; r<MM.rows(); r++) {
        for (int c=0; c<MM.cols(); c++) {
            out << MM(r,c);
        }
    }

    return out;
}

} // namespace


//! Serialization of the uncertain straight line segment t=[l',m',n']' and its bounding box
void uStraightLineSegment::serialize( QDataStream & out ) const
{
    //qDebug() << Q_FUNC_INFO;
    out << m_t;
    out << m_Cov_tt;
    out << m_bounding_box;
}


//! Deserialization of uncertain straight line segment and its bounding box
bool uStraightLineSegment::deserialize( QDataStream & in )
{
    // qDebug() << Q_FUNC_INFO;
    in >> m_t;
    in >> m_Cov_tt;
    in >> m_bounding_box;

    return in.status() == QDataStream::Ok;
}


//! Create uncertain straight line segment (for deserialization)
std::shared_ptr<uStraightLineSegment>
uStraightLineSegment::create()
{
     return std::shared_ptr<uStraightLineSegment>(
                    new uStraightLineSegment()
                    );
     // TODO(meijoc)
     // return std::make_shared<uStraightLineSegment>();  //  why not?
}

} // namespace Uncertain
