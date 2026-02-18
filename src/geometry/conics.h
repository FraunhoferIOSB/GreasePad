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

#ifndef CONICS_H
#define CONICS_H

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <cassert>
#include <cmath>
#include <utility>

#include "geometry/skew.h"


//! conics, rotations, bounding boxes, cross product
namespace Geometry {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;


//! Base class for conics
class Conic
{
public:
    explicit Conic(const Matrix3d & other) //!< Value constructor
        : CC(other)
    {
        constexpr double T_sym = 1e-6;
        assert( ( CC -CC.adjoint() ).norm() < T_sym );
    }

    [[nodiscard]] Matrix3d C() const { return CC;}  //!< getter

    //! check if conic has a central point
    [[nodiscard]] bool isCentral() const
    {
        constexpr double T_zero = 1e-6;
        const double det = CC.topLeftCorner(2,2).determinant();
        return std::fabs(det) > T_zero;
    }

    //! two intersection points with a straight line
    [[nodiscard]] std::pair<Vector3d,Vector3d> intersect( const Vector3d & l ) const
    {
        const Matrix3d MM = skew(l);
        const Matrix3d BB = MM.adjoint()*CC*MM;

        int idx = 0;       // [den,idx] = max( abs(l) );
        const double denom = l.array().abs().maxCoeff(&idx);

        // minors
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
            assert( false && "intersection of conic and straight line: index out of range");
        }

        // intersection points
        assert( alpha <= 0 );
        assert( denom >  0 );
        const Matrix3d DD = BB +std::sqrt(-alpha)/denom*MM;
        int r = 0;
        int c = 0;
        DD.array().abs().maxCoeff( &r, &c);

        return {DD.row(r), DD.col(c)};
    }

private:
    Matrix3d CC;  // symmetric and homogeneous
};


//! Ellipse
class Ellipse : public Conic
{
public:
    explicit Ellipse(const Matrix3d &CC ) //!< Value constructor (uncertain point)
        : Conic(CC)
    {
        // check if matrix represents an ellipse
        assert( C().topLeftCorner(2,2).determinant() > 0.0 );  // PCV Table 5.8
    }

    //! get N points on ellipse
    [[nodiscard]] std::pair<VectorXd,VectorXd> poly( const int N ) const
    {
        const Matrix2d Chh = C().topLeftCorner(2,2);
        const Vector2d ch0 = C().topRightCorner(2,1);
        const Vector2d  x0 = -Chh.ldlt().solve(ch0);  // centre point
        const double  c00q = C().coeff(2,2) -ch0.dot(Chh.ldlt().solve(ch0));
        assert( std::fabs(c00q)>0. );

        const Eigen::EigenSolver<Matrix2d> eig(-Chh / c00q, true);
        const Matrix2d RR = eig.eigenvectors().real();
        Vector2d ev = eig.eigenvalues().real();
        if ( ev(0) < 0 ) {
            ev = -ev;
        }

        constexpr double two_pi = 2*3.14159265358979323846;
        const VectorXd t = VectorXd::LinSpaced( N, 0, two_pi);

        MatrixXd xx(2,N);
        xx.row(0) = t.array().sin()/std::sqrt(ev(0));
        xx.row(1) = t.array().cos()/std::sqrt(ev(1));

        xx = RR*xx;           // rotation
        xx.colwise() += x0;   // translation

        return { xx.row(0), xx.row(1)};  // (x,y)
    }

    //! polar l (straight line) for point x, i.e., l=C*x
    [[nodiscard]] Vector3d polar( const Vector3d & x ) const { return C()*x; }
};


//! Hyperbola
class Hyperbola : public Conic
{
public:
    explicit Hyperbola(const Matrix3d &CC)
        : Conic(CC)
    {
        // check if matrix CC represents a hyperbola
        assert( C().topLeftCorner(2,2).determinant() < 0.0 ); // PCV Table 5.8
    }

    //! centerline of hyperbola, i.e., axis of symmetry
    [[nodiscard]] Vector3d centerline() const
    {
        const Vector3d x0 = center();
        const double phi = angle_rad();
        const double nx  = -std::sin(phi);
        const double ny  =  std::cos(phi);
        assert( std::fabs(x0(2)) > 0 );
        return { nx, ny, -nx*x0(0)/x0(2) -ny*x0(1)/x0(2) };
    }

    //! lengths of the two semiaxes
    [[nodiscard]] std::pair<double,double> lengthsSemiAxes() const
    {
        const Vector2d ev = eigenvalues();
        const double Delta = C().determinant();
        const double D = C().topLeftCorner(2,2).determinant();
        assert( -Delta/( ev(0)*D) >= 0. );
        assert( +Delta/( ev(1)*D) >= 0. );
        const double a = std::sqrt(-Delta / (ev(0) * D));
        const double b = std::sqrt(+Delta / (ev(1) * D));

        return {a,b};
    }

    //! angle between straight line and x-axis in radians.
    [[nodiscard]] double angle_rad() const
    {
        return 0.5*atan2( 2*C().coeff(0,1), C().coeff(0,0)-C().coeff(1,1));
    }

    //! angle between straight line and x-axis in degrees.
    [[nodiscard]] double angle_deg() const
    {
        constexpr double pi = 3.14159265358979323846;
        constexpr double rho = 180./pi;
        return rho*angle_rad();
    }

    //! center point of hyperbola (point of symmetry)
    [[nodiscard]] Vector3d center() const
    {
        if ( !isCentral() ) {
            return {0,0,0};
        }

        const Matrix2d C33 = C().topLeftCorner(2,2);
        const Vector2d ch0 = C().topRightCorner(2,1);
        const Vector2d x0 = -C33.ldlt().solve(ch0);

        return {x0(0),x0(1),1};
    }

private:
    [[nodiscard]] Vector2d eigenvalues() const
    {
        const double p = -C().topLeftCorner(2,2).trace();
        const double q =  C().topLeftCorner(2,2).determinant();

        const double radicant = p*p/4 -q;
        assert( radicant >=0 );

        const double ev0 = -p/2 -std::sqrt(radicant);
        const double ev1 = -p/2 +std::sqrt(radicant);

        return {ev0,ev1};
    }

};

} // namespace Geometry

#endif // CONICS_H
