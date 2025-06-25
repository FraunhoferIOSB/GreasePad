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

#ifndef CONICS_H
#define CONICS_H

#include <Eigen/Core>
#include <utility>

namespace Uncertain {
class uPoint;
class uStraightLine;
} // namespace Uncertain


//! Namespace Conic
namespace Conic {

using Eigen::Matrix3d;
using Eigen::Vector3d;

//! Base class for conics
class ConicBase : public Matrix3d
{
protected:
    ConicBase() = default;                   //!< Standard constructor
    ~ConicBase() = default;

public:
    ConicBase (const ConicBase &) = delete;  //!< Copy constructor
    ConicBase ( ConicBase &&) = delete;      //!< Move constructor
    ConicBase & operator = ( const ConicBase &) = delete ;  //!< Copy assignment operator
    ConicBase & operator= ( ConicBase &&) = delete;         //!< Move assignment operator
    explicit ConicBase( const Matrix3d & other);               //!< Value constructor
    ConicBase & operator= ( const Matrix3d & other);  //!< Assignment operator

    [[nodiscard]] bool isCentral()   const;  //!< Check if conic has a central point

    [[nodiscard]] Vector3d center() const;   //!< Get center point of conic
    [[nodiscard]] Vector3d polar( const Vector3d & x ) const;                          //!< Compute polar l (straight line) for point x, i.e., l=C*x
    [[nodiscard]] std::pair<Vector3d,Vector3d> intersect( const Vector3d & l ) const;  //!< Two intersection points with a straight line

protected:
    static Matrix3d cof3(const Matrix3d &MM);     //!< 3x3 cofactor matrix, i.e., transposed adjunct

private:
    static Matrix3d skew(const Vector3d &x);
    [[nodiscard]]  bool isSymmetric() const;  // for debugging only
};


//! Ellipse
class Ellipse : public ConicBase
{
public:
    explicit Ellipse( const Uncertain::uPoint &ux, double k2=1.0 ); //!< Value constructor (uncertain point)

    [[nodiscard]] std::pair<Eigen::VectorXd,Eigen::VectorXd> poly( int N ) const;  //!< Get N points on ellipse

    using ConicBase::operator=;
};


//! Hyperbola
class Hyperbola : public ConicBase
{
public:
    explicit Hyperbola( const Uncertain::uStraightLine &l, double k2=1.0 ); //!< Value constructor (uncertain straight line)
    using ConicBase::operator=;

    [[nodiscard]] Vector3d centerline() const;  //!< Get straight line
    [[nodiscard]] std::pair<double,double> lengthsSemiAxes() const;  //!< Get lengths/2 of axes
    [[nodiscard]] double angle_rad() const;     //!< Get angle between straight line and x-axis in radians.
    [[nodiscard]] double angle_deg() const;     //!< Get angle between straight line and x-axis in degerees.

private:
    [[nodiscard]] std::pair<double,double> eigenvalues() const;
};

} // namespace Conic


#endif // CONICS_H
