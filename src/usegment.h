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

#ifndef USEGMENT_H
#define USEGMENT_H

#include "aabb.h"

#include <Eigen/Dense>   // Eigen
#include <memory>        // C++
#include <QDataStream>   // Qt


namespace Uncertain {

class uPoint;
class uStraightLine;
using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

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
    bool isOrthogonalTo( const uStraightLineSegment & ut,
                         double T) const;
    bool isParallelTo( const uStraightLineSegment & ut,
                       double t_para) const;
    bool straightLineIsIdenticalTo( const uStraightLineSegment & ut,
                                       double t_ident) const;
    bool isCopunctualWith( const uStraightLineSegment & us,
                           const uStraightLineSegment & ut,
                           double T_det) const;

           uPoint ux() const;
           uPoint uy() const;
    uStraightLine ul() const;
    uStraightLine um() const;
    uStraightLine un() const;
        Vector3d hx() const;
        Vector3d hy() const;

        //! Get the endpoints connecting straight line l = S(x)*y
        Vector3d hl() const { return t_.head(3); }

        //! Get delimiting straight line m in homogeneous coordinates
        Vector3d hm() const { return t_.segment(3,3); }

        //! Get delimiting straight line n in homogeneous coordinates
        Vector3d hn() const { return t_.tail(3); }

        //! Get the 9-vector t = [l',m',n']'
        VectorXd t()  const { return t_;}

        //! Get 9x9 covariance matrix of vector t
        MatrixXd Cov_tt() const { return Cov_tt_; }

        //! Get endpoint x in Euclidean coordinates
        Vector2d x() const { return hx().head(2)/hx()(2); }

        //! Get endpoint y in Euclidean coordinates
        Vector2d y() const { return hy().head(2)/hy()(2); }

        //! Get angle between straight line l and x-axis in degree
          double phi_deg() const { return 180./3.14159*atan2( t_(1),t_(0) ); }

          //! Get axis-aligned bounding box
            aabb bounding_box() const { return bounding_box_; }

    /* nodiscard */ bool move_x_to( const Vector3d & m );
    /* nodiscard */ bool move_y_to( const Vector3d & n );
    void transform( const Matrix9d & TT );

private:
    Vector9d t_;              // 9-vector t=[l',m',n']'
    Matrix9d Cov_tt_;
    aabb     bounding_box_;   // not constant, due to merge operation, .united(...)

    static Matrix3d CC();   // Diag([1,1,0])

    template <typename T>
    inline int sign(T val) {  return (T(0) < val) - (val < T(0));  }

    template <typename T>
    inline bool sameSign( T a, T b ) const {  return a*b >= 0.; }   // for debugging and assertion
};

} // namespace Uncertain

#endif // USEGMENT_H
