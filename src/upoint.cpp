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
#include "statistics/iscov.h"
#include "udistance.h"
#include "uncertain.h"
#include "upoint.h"
#include "ustraightline.h"

#include <Eigen/Core>

#include <QDebug>

#include "qassert.h"
#include "qtdeprecationdefinitions.h"

#include <cassert>
#include <cmath>
#include <cstdlib>


using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Vector2d;

using Matfun::sign;

using Stats::isCovMat;


namespace Uncertain {


//! Construction of uncertain point via 3-vector x and its covariance matrix
uPoint::uPoint( const Vector3d & x, const Matrix3d & Sigma_xx)
    : BasicEntity2D (x, Sigma_xx)
{
    // qDebug() << Q_FUNC_INFO;
}


//! Get axis-aligned bounding box
Aabb<double> uPoint::bbox() const
{
    const double uu = v(0);
    const double vv = v(1);
    const double ww = v(2);

    const Matrix<double,2,3> JJ {
        { 1/ww,      0,   -uu/(ww*ww)},
        {    0,   1/ww,   -vv/(ww*ww)}  };
    const Matrix2d Cov_xx = JJ * Cov() * JJ.transpose();

    const double x = v(0) / v(2);
    const double y = v(1) / v(2);

    const double x_min = x - sqrt(Cov_xx(0, 0));
    const double x_max = x + sqrt(Cov_xx(0, 0));
    const double y_min = y - sqrt(Cov_xx(1, 1));
    const double y_max = y + sqrt(Cov_xx(1, 1));

    return Aabb<double>{
        Vector2d(x_min, y_min),
        Vector2d(x_max, y_max)
    };
}


//! Get Euclidean distance to uncertain straight line 'ul'
uDistance uPoint::distanceEuclideanTo( const uStraightLine & ul) const
{
    const Vector3d l = ul.v();
    const Vector3d x = v();
    const double n = l.head(2).norm(); // ||l_h||
    const double d_val = v().dot(l) / (abs( v(2))*n);
    const double nax2 = n*fabs(x(2));

    /* JJx(0) = l(0)/(abs(x(2))*n);
    JJx(1) = l(1)/(abs(x(2))*n);
    JJx(2) = l(2)/(abs(x(2))*n) -x.dot(l)*sign(x(2))/( abs(x(2))*abs(x(2)) *n);

    JJl(0) = x(0)/(abs(x(2))*n) -x.dot(l)*l(0) / (abs(x(2)) *n*n*n);
    JJl(1) = x(1)/(abs(x(2))*n) -x.dot(l)*l(1) / (abs(x(2)) *n*n*n);
    JJl(2) = x(2)/(abs(x(2))*n);*/

    Vector3d JJx;
    JJx(0) = l(0)/nax2;
    JJx(1) = l(1)/nax2;
    JJx(2) = (-l(0)*x(0) -l(1)*x(1))/( x(2)*nax2 );

    Vector3d JJl;
    JJl(0) = ( (-l(0)*l(1)*x(1) -l(0)*l(2)*x(2) +l(1)*l(1)*x(0) ) *fabs(x(2))) / ( x(2)*x(2)*n*n*n);
    JJl(1) = ( ( l(0)*l(0)*x(1) -l(0)*l(1)*x(0) -l(1)*l(2)*x(2) ) *fabs(x(2))) / ( x(2)*x(2)*n*n*n);
    JJl(2) = x(2)/nax2;

    const double d_var = JJx.dot( Cov() * JJx) + JJl.dot(ul.Cov() * JJl);

    return {d_val,d_var};
}


//! Get uncertain point in homogeneous coordinates, Euclidean normalized
uPoint uPoint::euclidean() const
{
    const double uu = v(0); // w'=u/w
    const double vv = v(1); // v'=v/w
    const double ww = v(2); // w'=w/w=1
    const Matrix3d JJ {
        { 1/ww,   0, -uu/(ww*ww) },
        { 0,   1/ww, -vv/(ww*ww) },
        { 0,      0,        0    } };

    return { v()/v(2), JJ*Cov()*JJ.adjoint() };
}


//! Get uncertain point, transformed via 3x3 transformation matrix
uPoint uPoint::transformed( const Matrix3d & TT) const
{
    return {TT*v(), TT*Cov()*TT.adjoint() };
}


//! Check if uncertain straight line 'ul' is incident.
bool uPoint::isIncidentWith( const uStraightLine & ul,
                             const double T) const
{
    const double d = v().dot(ul.v());                                       //  d = x'*l
    const double var_d = ul.v().dot(Cov() * ul.v()) + v().dot( ul.Cov() * v()); // uncorrelated
    Q_ASSERT( var_d>0. );

    return (d*d) / var_d < T;
}


//! Get algebraic distance of 'this' and straight line of 'ul'
uDistance uPoint::distanceAlgebraicTo( const uStraightLine & ul ) const
{
    const double d = sign( v(2) ) * v().dot(ul.v());
    const Vector3d JJx = ul.v().adjoint();
    const Vector3d JJl = sign( v(2) ) * v().adjoint();
    // JJ = [l', sign(x(3))*x'];

    const double var_d = JJx.dot( Cov() * JJx) + JJl.dot(ul.Cov() * JJl); // uncorrelated

    return {d, var_d};
}


//! Get uncertain point in homogeneous coordinates, spherically normalized
uPoint uPoint::sphericalNormalized() const
{
    uPoint ux(*this);
    ux.normalizeSpherical();
    return ux;  // Compiler invokes the copy constructor.
}


//! cross product of two vectors representing points, l=cross(x,y)
uStraightLine uPoint::cross( const uPoint & other) const
{
    // qDebug() << Q_FUNC_INFO;

    const Matrix3d Sx = skew( this->v() );
    const Matrix3d Sy = skew( other.v());

    const Vector3d m_val = Sx*other.v();         // cross(x,y)
    const Matrix3d m_cov = -Sy*this->Cov()*Sy -Sx*other.Cov()*Sx;  // S' = -S

    Q_ASSERT_X( m_val.norm()>0, Q_FUNC_INFO, "identical points");
    assert( isCovMat(m_cov) );

    return {m_val, m_cov };
}

} // namespace Uncertain
