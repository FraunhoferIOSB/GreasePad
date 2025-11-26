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

#include "conics.h"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <QDebug>

#include "qassert.h"
#include "qlogging.h"

#include <cassert>
#include <cmath>
#include <utility>

#include "matfun.h"

namespace Conic {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Matfun::skew;


ConicBase::ConicBase(Matrix3d other)
    : CC(std::move(other))
{
    constexpr double T_sym = 1e-6;
    assert( ( CC -CC.adjoint() ).norm() < T_sym );
}



Ellipse::Ellipse(const Matrix3d & CC) : ConicBase(CC)
{
    // check if matrix represents an ellipse
    assert( C().topLeftCorner(2,2).determinant() > 0.0 );  // PCV Table 5.8
}

Hyperbola::Hyperbola(const Matrix3d & CC) : ConicBase(CC)
{
    // check if matrix represents a hyperbola
    assert( C().topLeftCorner(2,2).determinant() < 0.0); // PCV Table 5.8
}


Vector3d Hyperbola::centerline() const
{
    const Vector3d x0 = center();
    const double phi_ = angle_rad();
    const double nx = -std::sin(phi_);
    const double ny =  std::cos(phi_);

    assert( fabs(x0(2)) > 0 );
    return { nx, ny, -nx*x0(0)/x0(2) -ny*x0(1)/x0(2) };
}

double Hyperbola::angle_rad() const
{
    return atan2( 2*C().coeff(0,1), C().coeff(0,0)-C().coeff(1,1)) / 2;
}

double Hyperbola::angle_deg() const
{
    constexpr double rho = 180./3.14159;
    return rho * angle_rad();
}

std::pair<double,double>
Hyperbola::lengthsSemiAxes() const
{
    auto ev = eigenvalues();
    double const Delta = C().determinant();
    double const D = C().topLeftCorner(2, 2).determinant();
    assert( -Delta/( ev.first*D ) >= 0. );
    assert( +Delta/( ev.second*D) >= 0. );
    double const a = std::sqrt(-Delta / (ev.first * D));
    double const b = std::sqrt(+Delta / (ev.second * D));

    return {a,b};
}



bool ConicBase::isCentral() const
{
    return CC.topLeftCorner(2,2).determinant() !=0.;
}




std::pair<double, double> Hyperbola::eigenvalues() const
{
    double const p = -C().topLeftCorner(2, 2).trace();
    double const q = C().topLeftCorner(2, 2).determinant();

    double const ev0 = -p / 2 - std::sqrt(p * p / 4 - q);
    double const ev1 = -p / 2 + std::sqrt(p * p / 4 - q);

    return {ev0,ev1};
}


Vector3d Hyperbola::center() const
{
    Vector3d xh;
    if ( isCentral() ) {
        Matrix2d const C33 = C().topLeftCorner(2, 2);
        Vector2d const ch0 = C().topRightCorner(2, 1);
        Vector2d x0 = -C33.ldlt().solve(ch0);
        xh << x0(0), x0(1), 1.0;
    }
    else {
        qDebug() << "! no central conic";  // TODO(meijoc)
        xh << 0,0,0;
    }
    return xh;
}


std::pair<Vector3d,Vector3d>
ConicBase::intersect( const Vector3d &l) const
{
    Matrix3d const MM = skew(l);
    Matrix3d BB = MM.adjoint()*CC*MM;

    int idx = 0;       // [den,idx] = max( abs(l) );
    double const den = l.array().abs().maxCoeff(&idx);

    // minors ...............................................
    double alpha = 0;
    switch (idx) {
    case 0:
        alpha = BB(1,1)*BB(2,2) -BB(2,1)*BB(1,2);
        break;
    case 1:
        alpha = BB(0,0)*BB(2,2) -BB(2,0)*BB(0,2);
        break;
    case 2:
        alpha = BB(0,0)*BB(1,1) -BB(1,0)*BB(0,1);
        break;
    default:
        Q_ASSERT_X( false, "ConicBase::intersect",
                    "intersection of conic and straight line: index out of range");
    }

    // intersection points .......................................
    Q_ASSERT( alpha <= 0 );
    Q_ASSERT(   den >  0 );
    Matrix3d DD = BB +std::sqrt(-alpha)/den*MM;
    int r = 0;
    int c = 0;
    DD.array().abs().maxCoeff( &r, &c);
    Vector3d const p = DD.row(r);
    Vector3d const q = DD.col(c);

    return {p,q};
}


std::pair<Eigen::VectorXd, Eigen::VectorXd>
Ellipse::poly( const int N) const
{
    Matrix2d const Chh = C().topLeftCorner(2, 2);
    Vector2d const ch0 = C().topRightCorner(2, 1);
    Vector2d x0 = -Chh.ldlt().solve(ch0);  // centre point
    double const c00q = C().coeff(2, 2) - ch0.dot(Chh.ldlt().solve(ch0));

    assert( std::fabs(c00q)>0. );

    Eigen::EigenSolver<Matrix2d> const eig(-Chh / c00q, true);
    Vector2d ev = eig.eigenvalues().real();
    Matrix2d const RR = eig.eigenvectors().real();

    if ( ev(0)<0.0 ) {
        ev *= -1;
    }

    const double two_pi = 2*3.14159;
    Eigen::VectorXd t = Eigen::VectorXd::LinSpaced( N, 0, two_pi);

    Eigen::MatrixXd  x(2,N);
    x.row(0) = t.array().sin()/std::sqrt(ev(0));
    x.row(1) = t.array().cos()/std::sqrt(ev(1));

    x = RR*x;                    // rotation
    x.row(0).array() += x0(0);   // translation
    x.row(1).array() += x0(1);

    return { x.row(0), x.row(1)};
}

} // namespace Conic
