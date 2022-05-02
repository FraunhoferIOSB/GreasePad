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

#ifndef CONICS_H
#define CONICS_H

#include <Eigen/Dense>

namespace Uncertain {
class uPoint;
class uStraightLine;
}

using Eigen::Matrix3d;
using Eigen::Vector3d;

namespace Conic {


class ConicBase : public Matrix3d
{
protected:
    ConicBase() = default;
    virtual ~ConicBase() = default;

public:
    ConicBase (const ConicBase &) = delete;
    ConicBase ( ConicBase &&) = delete;
    ConicBase & operator = ( const ConicBase &) = delete ;
    ConicBase & operator= ( ConicBase &&) = delete;
    ConicBase( const Matrix3d& other) : Matrix3d(other)  {
        // Q_ASSERT( isSymmetric()==true);
        assert( isSymmetric()==true );
    }
    ConicBase & operator= ( const Matrix3d & other);

    bool isEllipse()   const;  // obsolete
    bool isHyperbola() const;  // obsolete
    bool isParabola()  const;  // obsolete
    bool isCentral()   const;
    bool isProper()    const;  // obsolete

    Vector3d center() const;
    Vector3d polar( const Vector3d & x ) const;
    std::pair<Vector3d,Vector3d> intersect( const Vector3d & l ) const;

protected:
    void transform( const Matrix3d & HH );
    Matrix3d cof3( const Matrix3d & MM ) const;   //!< 3x3 cofactor matrix

private:
    Matrix3d skew( const Vector3d & x ) const;
    bool isSymmetric() const;
};



class Ellipse : public ConicBase
{
public:
    Ellipse( const Uncertain::uPoint &ux, double k2=1.0 );

    std::pair<Eigen::VectorXd,Eigen::VectorXd> poly( int N ) const;
    void scale( double s ); // obsolete

    using ConicBase::operator=;
};



class Hyperbola : public ConicBase
{
public:
    Hyperbola( const Uncertain::uStraightLine &l, double k2=1.0 );
    using ConicBase::operator=;

    Vector3d centerline() const;
    std::pair<double,double> lengthsSemiAxes() const;
    double angle_rad() const;
    double angle_deg() const;

private:
    std::pair<double,double> eigenvalues() const;
};

} // namespace Conic


#endif // CONICS_H
