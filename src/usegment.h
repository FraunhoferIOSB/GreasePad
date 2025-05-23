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

#ifndef USEGMENT_H
#define USEGMENT_H

#include "aabb.h"

#include <QDataStream> // Qt
#include <Eigen/Dense> // Eigen
#include <cmath>
#include <memory> // C++

namespace Uncertain {

class uPoint;
class uStraightLine;
using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

//! Uncertain straight line segment
class uStraightLineSegment
{
private:
    using Vector9d = Matrix<double,9,1>;  //  type alias (typedef)
    using Matrix9d = Matrix<double,9,9>;  //  type alias (typedef)
    uStraightLineSegment() = default;

public:
    uStraightLineSegment( const uPoint & ux,
                          const uPoint & uy);

    // deserialization with protected constructor
    static std::shared_ptr<uStraightLineSegment> create();  // TODO obsolete?

    void serialize( QDataStream & out) const;
    bool deserialize( QDataStream & in);

    // relations ....................................................
    bool intersects( const uStraightLineSegment & ut_) const;
    bool touches(    const uStraightLineSegment & other,
                     double T_dist,
                     double T_in) const;
    bool touchedBy(  const uPoint &,
                     double T_dist,
                     double T_in) const;

    // unary
    bool isVertical  ( double T) const;
    bool isHorizontal( double T) const;
    bool isDiagonal  ( double T) const;

    // binary
    bool isOrthogonalTo( const uStraightLine & um,    double T) const;
    bool isParallelTo(   const uStraightLine & um,    double T) const;
    bool straightLineIsIdenticalTo( const uStraightLine & um,  double T) const;

    // ternary
    bool isCopunctualWith( const uStraightLine & um,
                           const uStraightLine & un,
                           double T) const;

    uPoint ux() const;
    uPoint uy() const;
    uStraightLine ul() const;
    uStraightLine um() const;
    uStraightLine un() const;
    Vector3d hx() const;
    Vector3d hy() const;

    //! Get the endpoints connecting straight line l = S(x)*y
    Vector3d hl() const { return m_t.head(3); }

    //! Get delimiting straight line m in homogeneous coordinates
    Vector3d hm() const { return m_t.segment(3,3); }

    //! Get delimiting straight line n in homogeneous coordinates
    Vector3d hn() const { return m_t.tail(3); }

    //! Get the 9-vector t = [l',m',n']'
    VectorXd t()  const { return m_t;}

    //! Get 9x9 covariance matrix of vector t
    MatrixXd Cov_tt() const { return m_Cov_tt; }

    //! Get endpoint x in Euclidean coordinates
    Vector2d x() const { return hx().head(2)/hx()(2); }

    //! Get endpoint y in Euclidean coordinates
    Vector2d y() const { return hy().head(2)/hy()(2); }

    //! Get angle between straight line l and x-axis in degree
    double phi_deg() const { return 180./3.14159*atan2( m_t(1),m_t(0) ); }

    //! Get axis-aligned bounding box
    Aabb bounding_box() const { return m_bounding_box; }

    /* nodiscard */ bool move_x_to( const Vector3d & m );
    /* nodiscard */ bool move_y_to( const Vector3d & n );
    void transform( const Matrix9d & TT );

private:
    Vector9d m_t;              // 9-vector t=[l',m',n']'
    Matrix9d m_Cov_tt;
    Aabb     m_bounding_box;   // not constant, due to merge operation, .united(...)

    static Matrix3d CC();   // Diag([1,1,0])

    template <typename T>
    inline int sign(T val) {  return (T(0) <= val) - (val < T(0));  }  // sign(0):=+1

    template <typename T>
    inline bool sameSign( T a, T b ) const {  return a*b >= 0.; }   // for debugging and assertion
};

} // namespace Uncertain

#endif // USEGMENT_H
