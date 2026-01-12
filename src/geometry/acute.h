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
#include <cfloat>
#include <cmath>
#include <cstdlib>


namespace Geometry {

using Eigen::Vector2d;
using Eigen::Vector3d;


//! acute angle between two straight lines in radians
static inline double acute( const Vector3d l, const Vector3d m)
{
    constexpr double pi   = 3.141592653589793;
    constexpr double pi_2 = 3.141592653589793/2;

    Vector2d lh = l.head(2);
    Vector2d mh = m.head(2);
    assert( lh.norm() > FLT_EPSILON );
    assert( mh.norm() > FLT_EPSILON );
    lh.normalize();
    mh.normalize();

    assert( std::fabs(lh.dot(mh)) <= 1.0 );  // a'*b in [-1,+1]
    double alpha = std::acos( lh.dot(mh) );  // [0,pi]
    if ( alpha > pi_2) {
        alpha = pi -alpha;
    }
    assert( alpha >= 0.0    );
    assert( alpha <= pi_2 );

    return alpha;
}

} // namespace Geometry

#endif // ACUTE_H
