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

#include "upoint.h"
#include "usegment.h"

#include <math.h>
#include "ustraightline.h"

#include <QDebug>

using Eigen::Matrix3d;
using Eigen::VectorXi;
using Eigen::Vector3d;
using Eigen::VectorXd;

static const double T_ZERO = 1e-7;



namespace Uncertain {

class uPoint;


//! Input operator
QDataStream & operator<< (QDataStream & out, const Aabb & bbox);

//! Output operator
QDataStream & operator>> (QDataStream & in, Aabb & bbox);


//! Diag([1,1,0])
Matrix3d uStraightLineSegment::CC()
{
    static Matrix3d tmp = ( Matrix3d() << 1,0,0,0,1,0,0,0,0 ).finished();
    return tmp;
}


//! Get uncertain endpoint x in homogeneous coordinates
uPoint uStraightLineSegment::ux() const
{
    Matrix<double, 3,6> JJ;
    Matrix<double, 6,6> Cov = m_Cov_tt.topLeftCorner(6,6);
    Matrix3d Sl = skew( hl() );

    JJ.leftCols(3) = -skew( hm() );
    JJ.rightCols(3) = Sl;

    // x = Sl*m:
    return { Sl*hm(), JJ*Cov*JJ.adjoint() };
}


//! Get uncertain endpoint y in homogeneous coordinates
uPoint uStraightLineSegment::uy() const
{
    Matrix<double,3,9> JJ;
    Matrix3d Sl = skew( hl() );

    JJ << -skew( hn() ),  Matrix3d::Zero(), Sl;

    return { Sl*hn(), JJ*Cov_tt()*JJ.adjoint() };
}



//! Construction of an uncertain straight line segment via two uncorrelated endpoints
uStraightLineSegment::uStraightLineSegment( const uPoint & ux,
                                            const uPoint & uy)
    : m_bounding_box( ux.bbox().united( uy.bbox()) )
{
    // (1) 9-parameter vector t = [l; m; n];
    const Matrix3d Sx = skew( ux.v() );
    const Matrix3d Sy = skew( uy.v() );
    const Matrix3d UUx = Sx*CC()*Sx;
    const Matrix3d UUy = Sy*CC()*Sy;

    const Vector3d l = Sx*uy.v();
    Q_ASSERT( fabs( l.norm() ) > T_ZERO );
    assert( fabs( l.norm() ) > T_ZERO && "identical points.");

    double xh = ux.v()(2);
    double yh = uy.v()(2);
    // Vector9d t;
    m_t.segment( 0, 3) = l;
    m_t.segment( 3, 3) = +sign(yh)*UUx*uy.v();
    m_t.segment( 6, 3) = -sign(xh)*UUy*ux.v();

    // (2) 9x9 covariance matrix
    //Matrix3d JJlx = -Sy;
    //Matrix3d JJly = Sx;

    Matrix3d JJmx = -sign(yh)*(  Sx*CC()*Sy  + skew( CC()*l ));
    Matrix3d JJmy = +sign(yh)*UUx;

    Matrix3d JJnx = -sign(xh)*UUy;
    Matrix3d JJny = +sign(xh)*( Sy*CC()*Sx -skew( CC()*l ));

    /* JJ = [ ...
        JJlx, JJly; ...
        JJmx, JJmy; ...
        JJnx, JJny];  */

    Matrix<double, 9,6> JJ;
    JJ.block(0,0,3,3) = -Sy;   // = JJlx
    JJ.block(0,3,3,3) = Sx;    // = JJly;
    JJ.block(3,0,3,3) = JJmx;
    JJ.block(3,3,3,3) = JJmy;
    JJ.block(6,0,3,3) = JJnx;
    JJ.block(6,3,3,3) = JJny;

    Matrix<double,6,6> Cov_pp;
    Cov_pp.setZero();
    Cov_pp.block(0,0,3,3) = ux.Cov();
    Cov_pp.block(3,3,3,3) = uy.Cov();

    m_Cov_tt = JJ*Cov_pp*JJ.transpose();
    // return { t,  JJ*Cov_pp*JJ.transpose()};
}


//! Check if the 'this' and the uncertain straight line segment 'ut' intersect (deterministic)
bool uStraightLineSegment::intersects( const uStraightLineSegment & ut) const
{
    double d1 = ut.hx().dot( hl() );
    double d2 = ut.hy().dot( hl() );
    if ( sameSign(d1,d2) ) {
        return false;
    }

    double d3 = hx().dot( ut.hl() );
    double d4 = hy().dot( ut.hl() );

    // redundant boolean literal in conditional return statement:
    /* if ( sameSign( d3,d4) )
         return false;
    return true; */

    return !sameSign(d3,d4);
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
    if ( !ux.isIncidentWith( ul(), T_in ) ) {      // dist(x,l)
        return false;
    }

    uDistance ud1 = ux.distanceAlgebraicTo( um() );  // dist(x,m)
    if ( !ud1.isLessThanZero(T_dist) ) {
        return false;
    }

    uDistance ud2 = ux.distanceAlgebraicTo( un() );  // dist(x,n)
    if ( !ud2.isGreaterThanZero(T_dist) ) {
        return false;
    }

    return true;
}

//! Get the endpoints connecting uncertain straight line l = S(x)*y
uStraightLine uStraightLineSegment::ul() const
{
    Matrix3d Cov_ll = m_Cov_tt.topLeftCorner(3,3);
    return { m_t.head(3), Cov_ll };
}


//! Get the delimiting uncertain straight line m in homogeneous coordinates.
uStraightLine uStraightLineSegment::um() const
{
  Matrix3d Cov_mm = m_Cov_tt.block(3,3,3,3);
  return { m_t.segment(3,3), Cov_mm};
}


//! Get the delimiting uncertain straight line n in homogeneous coordinates.
uStraightLine uStraightLineSegment::un() const
{
    Matrix3d Cov_nn = m_Cov_tt.bottomRightCorner(3,3);
    return { m_t.tail(3), Cov_nn};
}


//! Get the endpoint x in homogeneous coordinates
Vector3d uStraightLineSegment::hx() const
{
    Vector3d l = m_t.head(3);
    Vector3d m = m_t.segment(3,3);
    return skew( l )*m;
}


//! Get the endpoint y in homogeneous coordinates
Vector3d uStraightLineSegment::hy() const
{
    Vector3d l = m_t.head(3);
    Vector3d n = m_t.tail(3);
    return skew( l )*n;
}


//! Check if the uncertain straight line segment is vertical.
bool uStraightLineSegment::isVertical( const double T) const
{
    return ul().isVertical( T);
}

//! Check if the uncertain straight line segment is horizontal.
bool uStraightLineSegment::isHorizontal( const double T) const
{
    return ul().isHorizontal( T);
}

//! Check if the uncertain straight line segment is diagonal.
bool uStraightLineSegment::isDiagonal( const double T) const
{
    return ul().isDiagonal( T);
}


//! Check if uncertain straight line segment is orthogonal to uncertaint straight line segment 'ut'
bool uStraightLineSegment::isOrthogonalTo( const uStraightLine & um,
                                           const double T_q) const
{
    return ul().isOrthogonalTo( um, T_q );
}

//! Check if uncertain straight line segment is parallel to uncertaint straight line segment 'ut'
bool uStraightLineSegment::isParallelTo( const uStraightLine & um,
                                         const double T) const
{
    return ul().isParallelTo( um, T );
}

//! Check if uncertain straight lines 'this' and 'ut' are identical
bool uStraightLineSegment::straightLineIsIdenticalTo( const uStraightLine & um,
                                                      const double T) const
{
    return ul().isIdenticalTo( um, T );
}

//! Move endpoint x along l to intersection point of m and l.
bool uStraightLineSegment::move_x_to( const Vector3d & m)
{
    Vector3d l = hl().normalized();
    Vector3d z = l.cross( m.normalized() );     // intersection point z
    if ( z.norm()<T_ZERO ) {
        return false; // l==m
    }
    // Q_ASSERT_X( z.norm() > T_ZERO, "move x", "l == m.");

    Vector2d dx = x() -z.head(2)/z(2);
    Matrix3d HH = ( Matrix3d() << 1,0,dx(0), 0,1,dx(1), 0,0,1).finished();
    Matrix9d TT = Matrix9d::Identity();
    TT.block(3,3,3,3) = HH.adjoint();  // m
    transform( TT);
    return true;
}

//! Move endpoint y along l to intersection point of n and l.
bool uStraightLineSegment::move_y_to( const Vector3d & n )
{
    // intersection point z ............................
    Vector3d l = hl().normalized();
    Vector3d z = l.cross( n.normalized() );
    if ( z.norm() < T_ZERO) {
        return false; // l==n
    }
    // Q_ASSERT_X( z.norm() > T_ZERO,  "move y", "l == n.");

    Vector2d dx = y() -z.head(2)/z(2);
    Matrix3d HH = (Matrix3d() << 1,0,dx(0), 0,1,dx(1), 0,0,1).finished();
    Matrix9d TT = Matrix9d::Identity();
    TT.block(6,6,3,3) = HH.adjoint();  // n
    transform( TT);
    return true;
}


//! Check if the uncertain straight line of 'this' is copunctual with the two uncertaint straight lines of 'us' and 'ut'
bool uStraightLineSegment::isCopunctualWith( const uStraightLine & um,
                                             const uStraightLine & un,
                                             const double T) const
{
    return ul().isCopunctualWith( um, un, T);
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


QDataStream & operator<< (QDataStream & out, const Aabb & bbox)
{
    out << bbox.x_min() << bbox.x_max()
        << bbox.y_min() << bbox.y_max();
    return out;
}

QDataStream & operator>> (QDataStream & in, Aabb & bbox)
{
    double x_min = NAN;
    double x_max = NAN;
    double y_min = NAN;
    double y_max = NAN;
    in >> x_min >> x_max >> y_min >> y_max;
    bbox = Aabb( x_min, x_max, y_min, y_max);

    return in;
}


QDataStream & operator>> ( QDataStream & in, Eigen::Matrix<double,9,1> & v);

//! Overloaded >>operator for 9-vectors
QDataStream & operator>> ( QDataStream & in, Eigen::Matrix<double,9,1> & v)
{
    for ( int i=0; i<9; i++) {  in >> v[i]; }
    return in;
}

QDataStream & operator>> ( QDataStream & in, Eigen::Matrix<double,9,9> & MM);

//! Overloaded >>operator for 9x9 matrices
QDataStream & operator>> ( QDataStream & in, Eigen::Matrix<double,9,9> & MM)
{
    for ( int i=0; i<9; i++) {
        for ( int j=0; j<9; j++) {
            in >> MM(i,j);
        }
    }
    return in;
}


QDataStream & operator<< ( QDataStream & out, const Eigen::Matrix<double,9,1> &v);

//! Overloaded <<operator for 9-vectors
QDataStream & operator<< ( QDataStream & out, const Eigen::Matrix<double,9,1> &v)
{
    //qDebug() << Q_FUNC_INFO;
    for ( int i=0; i<9; i++) {  out << v[i];  }
    return out;
}
QDataStream & operator<< ( QDataStream & out, const Eigen::Matrix<double,9,9> &MM);

//! Overloaded <<operator for 9x9 matrices
QDataStream & operator<< ( QDataStream & out, const Eigen::Matrix<double,9,9> &MM)
{
    qDebug() << Q_FUNC_INFO;
    for ( int i=0; i<9; i++) {
        for (int j=0; j<9; j++) {
            out << MM(i,j);
        }
    }
    return out;
}


//! Serialization of the uncertain straight line segment t=[l',m',n']' and its bounding box
void uStraightLineSegment::serialize( QDataStream & out ) const
{
    qDebug() << Q_FUNC_INFO;
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

    return in.status()==0;
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
