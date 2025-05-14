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

#ifndef USTRAIGHTLINE_H
#define USTRAIGHTLINE_H

#include "uncertain.h"
#include <cmath>

// #include <QDebug>  // Q_REQUIRED_RESULT

using Eigen::VectorXd;

namespace Uncertain {

class uPoint;

Matrix3d skew( const Vector3d & x);

//! Uncertain straight line
class uStraightLine : public BasicEntity2D
{
public:
    uStraightLine() = default;
    uStraightLine( const Vector3d & l,
                   const Matrix3d & Cov_ll);
    uStraightLine( const VectorXd & xi,
                   const VectorXd & yi);
    uStraightLine( const uPoint & ux,
                   const uPoint & uy);
    // uStraightLine( const uStraightLine &other) = default;
    // uStraightLine( const BasicEntity2D &other) : BasicEntity2D(other) {}
    // ~uStraightLine() override = default;

    /* nodiscard */ uStraightLine euclidean() const;
    /* nodiscard */ uStraightLine sphericalNormalized() const;

    //! Angle between this straight line and the x-axis in radians
    double angle_rad()  const { return atan2( m_val(1),m_val(0) ); }

    //! Signed distance between (0,0) and this straight line
    double signedDistanceToOrigin() const { return v().z()/v().head(2).norm(); }

    uPoint project( const uPoint & ux) const;
    uPoint cross(   const uStraightLine & um ) const;  // not required
    double acute(   const uStraightLine & um ) const;

    // unary relations ..................................................
    bool isVertical(   double T) const;
    bool isHorizontal( double T) const;
    bool isDiagonal(   double T) const;

    // binary relations ..................................................
    bool isOrthogonalTo(const uStraightLine & um, double T) const;
    bool isParallelTo(  const uStraightLine & um, double T) const;

    // ternary relation ..................................................
    bool isCopunctualWith( const uStraightLine & um,
                           const uStraightLine & un,
                           double T) const;

private:
    using Matrix6d    = Eigen::Matrix<double,6,6>;
    using RowVector6d = Eigen::Matrix<double,1,6>;

    static Matrix3d cof3(const Matrix3d &MM); // compute 3x3 cofactor matrix

    static Matrix3d CC();
    static Matrix3d S3();
};

} // namespace Uncertain

#endif // USTRAIGHTLINE_H
