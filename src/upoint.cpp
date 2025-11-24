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

#include "aabb.h"
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

namespace Uncertain {

//! Construction of uncertain point vai 3-vector x and its covariance matrix
uPoint::uPoint(const Vector3d &x,
               const Matrix3d &Sigma_xx)
    : BasicEntity2D (x, Sigma_xx)
{
    // qDebug() << Q_FUNC_INFO;
    Q_ASSERT_X( isCovMat( Cov() ),
                Q_FUNC_INFO,
                "invalid covariance matrix");
}


//! Get axis-aligned bounding box
Aabb uPoint::bbox() const
{
    double const uu = v(0);
    double const vv = v(1);
    double const ww = v(2);
    Eigen::Matrix<double,2,3> JJ;
    JJ.col(0) << 1/ww, 0;
    JJ.col(1) << 0, 1/ww;
    JJ.col(2) << -uu/(ww*ww), -vv/(ww*ww);
    Eigen::Matrix2d Cov_xx = JJ * Cov() * JJ.transpose();

    double const x = v(0) / v(2);
    double const y = v(1) / v(2);

    double const x_min = x - sqrt(Cov_xx(0, 0));
    double const x_max = x + sqrt(Cov_xx(0, 0));
    double const y_min = y - sqrt(Cov_xx(1, 1));
    double const y_max = y + sqrt(Cov_xx(1, 1));

    return Aabb{ x_min, x_max, y_min, y_max} ;
}

//! Get Euclidean distance to uncertain straight line 'ul'
uDistance uPoint::distanceEuclideanTo( const uStraightLine & ul) const
{
    const Vector3d l = ul.v();
    const Vector3d x = v();
    const double n = l.head(2).norm(); // ||l_h||
    const double d = v().dot(l) / (abs( v(2))*n);

    Vector3d JJx;
    Vector3d JJl;
    JJx(0) = l(0)/(abs(x(2))*n);
    JJx(1) = l(1)/(abs(x(2))*n);
    JJx(2) = l(2)/(abs(x(2))*n) -x.dot(l)*sign(x(2))/( abs(x(2))*abs(x(2)) *n);

    JJl(0) = x(0)/(abs(x(2))*n) -x.dot(l)*l(0) / (abs(x(2)) *n*n*n);
    JJl(1) = x(1)/(abs(x(2))*n) -x.dot(l)*l(1) / (abs(x(2)) *n*n*n);
    JJl(2) = x(2)/(abs(x(2))*n);

    double const var_d = JJx.dot( Cov() * JJx) + JJl.dot(ul.Cov() * JJl);

    return {d,var_d};
}

//! Get uncertain point in homogeneous coordinates, Euclidean normalized
uPoint uPoint::euclidean() const
{
    const double uu = v(0); // w'=u/w
    const double vv = v(1); // v'=v/w
    const double ww = v(2); // w'=w/w=1
    Matrix3d JJ;
    JJ.row(0) << 1/ww,   0, -uu/(ww*ww);
    JJ.row(1) <<   0, 1/ww, -vv/(ww*ww);
    JJ.row(2) <<   0,   0,        0;

    return { v()/v(2), JJ*Cov()*JJ.adjoint() };
}

//! Get uncertain point, transformed via 3x3 transformation matrix
uPoint uPoint::transformed( const Matrix3d & TT) const
{
    // uPoint ux(*this);
    // ux.transform(TT);
    // return ux;
    return {TT*v(), TT*Cov()*TT.adjoint() };
}


//! Check if uncertain straight line 'ul' is incident.
bool uPoint::isIncidentWith( const uStraightLine & ul,
                             const double T) const
{
    double const d = v().dot(ul.v());                                            //  d = x'*l
    double const var_d = ul.v().dot(Cov() * ul.v()) + v().dot( ul.Cov() * v()); // uncorrelated
    Q_ASSERT( var_d>0. );
    double const T_in = (d * d) / var_d;
    return (T_in < T);
}

//! Get algebraic distance of 'this' and straight line of 'ul'
uDistance uPoint::distanceAlgebraicTo( const uStraightLine & ul ) const
{
    double const d = sign( v(2) ) * v().dot(ul.v());
    Vector3d const JJx = ul.v().adjoint();
    Vector3d const JJl = sign( v(2) ) * v().adjoint();
    // JJ = [l', sign(x(3))*x'];

    double const var_d = JJx.dot( Cov() * JJx) + JJl.dot(ul.Cov() * JJl); // uncorrelated

    return {d, var_d};
}

//! Get uncertain point in homogeneous coordinates, spherically normalized
uPoint uPoint::sphericalNormalized() const
{
    // uPoint ux =  *this ;
    uPoint ux(*this);
    //uPoint ux = uPoint(*this);
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
