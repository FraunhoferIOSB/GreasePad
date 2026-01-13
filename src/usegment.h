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

#ifndef USEGMENT_H
#define USEGMENT_H

#include "geometry/aabb.h"

#include <QDataStream> // Qt

#include <Eigen/Core>
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

using Geometry::Aabb;


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
    static std::shared_ptr<uStraightLineSegment> create();

    void serialize( QDataStream & out) const;
    bool deserialize( QDataStream & in);

    // relations ....................................................
    [[nodiscard]] bool intersects( const uStraightLineSegment & ut_) const;
    [[nodiscard]] bool touches(    const uStraightLineSegment & other,
                                   double T_dist,
                                   double T_in) const;
    [[nodiscard]] bool touchedBy(  const uPoint &,
                                   double T_dist,
                                   double T_in) const;

    [[nodiscard]] uPoint ux() const;
    [[nodiscard]] uPoint uy() const;
    [[nodiscard]] uStraightLine ul() const;
    [[nodiscard]] uStraightLine um() const;
    [[nodiscard]] uStraightLine un() const;
    [[nodiscard]] Vector3d hx() const;
    [[nodiscard]] Vector3d hy() const;

    //! Get the endpoints connecting straight line l = S(x)*y
    [[nodiscard]] Vector3d hl() const { return m_t.head(3); }

    //! Get delimiting straight line m in homogeneous coordinates
    [[nodiscard]] Vector3d hm() const { return m_t.segment(3,3); }

    //! Get delimiting straight line n in homogeneous coordinates
    [[nodiscard]] Vector3d hn() const { return m_t.tail(3); }

    //! Get the 9-vector t = [l',m',n']'
    [[nodiscard]] VectorXd t()  const { return m_t;}

    //! Get 9x9 covariance matrix of vector t
    [[nodiscard]] MatrixXd Cov_tt() const { return m_Cov_tt; }

    //! Get endpoint x in Euclidean coordinates
    [[nodiscard]] Vector2d x() const { return hx().head(2)/hx()(2); }

    //! Get endpoint y in Euclidean coordinates
    [[nodiscard]] Vector2d y() const { return hy().head(2)/hy()(2); }

    //! Get angle between straight line l and x-axis in degree
    [[nodiscard]] double phi_deg() const {
        constexpr double pi  = 3.141592653589793;
        constexpr double rho = 180./pi;
        return rho*atan2( m_t(1),m_t(0) );
    }

    //! Get axis-aligned bounding box
    [[nodiscard]] Aabb bounding_box() const { return m_bounding_box; }

    [[nodiscard]] bool move_x_to( const Vector3d & m );
    [[nodiscard]] bool move_y_to( const Vector3d & n );
    void transform( const Matrix9d & TT );

private:
    Vector9d m_t;              // 9-vector t=[l',m',n']'
    Matrix9d m_Cov_tt;
    Aabb     m_bounding_box;   // not constant, due to merge operation, .united(...)

    static Matrix3d CC();   // Diag([1,1,0])
};

} // namespace Uncertain

#endif // USEGMENT_H
