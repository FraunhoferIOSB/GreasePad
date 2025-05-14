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
#include "qlogging.h"
#include "upoint.h"
#include "ustraightline.h"

#include <QDebug>
#include <cassert>
#include <cmath>
#include <utility>

namespace Conic {

using Uncertain::uPoint;
using Uncertain::uStraightLine;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;

ConicBase::ConicBase( const Matrix3d& other) : Matrix3d(other)
{
    // Q_ASSERT( isSymmetric()==true);
    assert( isSymmetric()==true );
}

Matrix3d ConicBase::skew(const Vector3d &x)
{
    return (Matrix3d() << 0.,-x(2),x(1), x(2),0.,-x(0), -x(1),x(0),0.).finished();
}

bool ConicBase::isSymmetric() const
{
   constexpr double T_sym = 1e-6;

   return (*this -this->adjoint()).norm() < T_sym ;
}


ConicBase & ConicBase::operator=( const Matrix3d & other )
{
    // qDebug() << Q_FUNC_INFO;
    this->Matrix3d::operator= (other);
    Q_ASSERT( isSymmetric()==true );
    assert ( isSymmetric()==true );
    return *this;
}

Matrix3d ConicBase::cof3(const Matrix3d &MM)
{
    Matrix3d Cof;

    Cof(0,0) = MM(1,1)*MM(2,2) -MM(2,1)*MM(1,2);
    Cof(0,1) = -( MM(1,0)*MM(2,2) -MM(2,0)*MM(1,2) );
    Cof(0,2) = MM(1,0)*MM(2,1) -MM(2,0)*MM(1,1);

    Cof(1,0) = -( MM(0,1)*MM(2,2) -MM(2,1)*MM(0,2) );
    Cof(1,1) = MM(0,0)*MM(2,2) -MM(2,0)*MM(0,2);
    Cof(1,2) = -(MM(0,0)*MM(2,1) -MM(2,0)*MM(0,1));

    Cof(2,0) = MM(0,1)*MM(1,2) -MM(1,1)*MM(0,2);
    Cof(2,1) = -(MM(0,0)*MM(1,2) -MM(1,0)*MM(0,2));
    Cof(2,2) = MM(0,0)*MM(1,1) -MM(1,0)*MM(0,1);

    return Cof;
}

Ellipse::Ellipse(const uPoint &ux, const double k2)
{
    uPoint const uy = ux.euclidean();

    // cofactor matrix is adjugate due to symmetry
    *this = cof3( k2*uy.Cov() -uy.v()*uy.v().adjoint() );
}

Hyperbola::Hyperbola(const uStraightLine &ul, const double k2)
{
    uStraightLine const um = ul.euclidean();
    *this = k2*um.Cov() -um.v()*um.v().adjoint();
}


Vector3d Hyperbola::centerline() const
{
    const Vector3d x0 = center();
    const double phi_ = angle_rad();
    const double nx = -std::sin(phi_);
    const double ny =  std::cos(phi_);

    return { nx, ny, -nx*x0(0)/x0(2) -ny*x0(1)/x0(2) };
}

double Hyperbola::angle_rad() const
{
    return 0.5*atan2( 2.0*coeff(0,1), coeff(0,0)-coeff(1,1));
}

double Hyperbola::angle_deg() const
{
    constexpr double rho = 180./3.14159;
    return rho * 0.5 * atan2( 2.0*coeff(0,1), coeff(0,0)-coeff(1,1));
}

std::pair<double,double>
Hyperbola::lengthsSemiAxes() const
{
    auto ev = eigenvalues();
    double const Delta = determinant();
    double const D = topLeftCorner(2, 2).determinant();
    assert( -Delta/( ev.first*D ) >= 0. );
    assert( +Delta/( ev.second*D) >= 0. );
    double const a = std::sqrt(-Delta / (ev.first * D));
    double const b = std::sqrt(+Delta / (ev.second * D));

    return {a,b};
}


Vector3d ConicBase::polar( const Vector3d &x ) const
{
    return (*this*x); // i.e., l = C*x
}

bool ConicBase::isCentral() const
{
    return topLeftCorner(2,2).determinant() !=0.;
}

bool ConicBase::isProper() const
{
    return determinant() !=0.;
}

bool ConicBase::isEllipse() const
{
    return topLeftCorner(2,2).determinant()>0.;
}

bool ConicBase::isHyperbola() const
{
    return topLeftCorner(2,2).determinant()<0.;
}

bool ConicBase::isParabola() const
{
    return topLeftCorner(2,2).determinant()==0.;
}



std::pair<double, double> Hyperbola::eigenvalues() const
{
    double const p = -topLeftCorner(2, 2).trace();
    double const q = topLeftCorner(2, 2).determinant();

    Q_ASSERT_X( q<0.0, Q_FUNC_INFO, "no hyperbola");
    assert( q<0.0 );
    double const ev0 = -p / 2 - std::sqrt(p * p / 4 - q);
    double const ev1 = -p / 2 + std::sqrt(p * p / 4 - q);

    return {ev0,ev1};
}


Vector3d ConicBase::center() const
{
    Vector3d xh;
    if ( isCentral() ) {
        Matrix2d const C33 = topLeftCorner(2, 2);
        Vector2d const ch0 = topRightCorner(2, 1);
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
    Matrix3d BB = MM.adjoint()*(*this)*MM;

    int idx = 0;       // [den,idx] = max( abs(l) );
    double const den = l.array().abs().maxCoeff(&idx);

    // minors ...............................................
    double alpha=0;
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


void ConicBase::transform( const Matrix3d &HH )
{
    Matrix3d TT = cof3(HH);  // cf. PCV (6.57)
    *this = TT*(*this)*TT.transpose();
}



void Ellipse::scale( const double s )
{
    Q_ASSERT( s>=0.0 );
    Vector3d c = center();
    c /= c(2);
    const Matrix3d HH = ( Matrix3d() <<
                          s,   0.0,  c(0)-s*c(0),
                          0.0,   s,  c(1)-s*c(1),
                          0.0, 0.0,          1.0 ).finished();
    this->transform(HH);
}


std::pair<Eigen::VectorXd, Eigen::VectorXd>
Ellipse::poly( const int N) const
{
    Matrix2d const Chh = topLeftCorner(2, 2);
    Vector2d const ch0 = topRightCorner(2, 1);
    Vector2d x0 = -Chh.ldlt().solve(ch0);  // centre point
    double const c00q = coeff(2, 2) - ch0.dot(Chh.ldlt().solve(ch0));

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
