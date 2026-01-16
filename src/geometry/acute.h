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

#ifndef ACUTE_H
#define ACUTE_H


#include <Eigen/Core>

#include <cassert>
#include <cmath>

#include "skew.h"

namespace Geometry {

using Eigen::Matrix3d;
using Eigen::Vector3d;


//! acute angle between two straight lines in radians
[[nodiscard,maybe_unused]] static inline double acute( const Vector3d & l,
                                                       const Vector3d & m)
{
    constexpr double pi   = 3.141592653589793;

    static const Matrix3d S3 = skew( Vector3d(0,0,1) );
    static const Matrix3d G3 = Vector3d(1,1,0).asDiagonal();

    // PCV (7.34); std::atan2(0,0) := 0
    const double gamma = std::atan2( l.dot(S3*m), l.dot(G3*m) );
    const double beta  = std::fmod(  gamma+2*pi, pi );
    const double alpha = std::fmin(  beta, pi-beta  );

    assert( alpha >= 0.0 && alpha <= pi/2 );

    return alpha;
}

} // namespace Geometry

#endif // ACUTE_H
