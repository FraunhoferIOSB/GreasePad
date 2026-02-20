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


#ifndef MINROT_H
#define MINROT_H

#include <cassert>

#include <Eigen/Core>


namespace Geometry {

using Eigen::Matrix;
using Eigen::Vector;

template <typename T, int N>
[[nodiscard]] static inline Matrix<T,N,N>
Rot_ab( const Vector<T,N> &a, const Vector<T,N> &b)
{
    constexpr double T_zero = 1e-5;
    assert( std::fabs( a.norm()-1.) < T_zero );
    assert( std::fabs( b.norm()-1.) < T_zero );

    const T denom = T(1)+a.dot(b);
    if ( std::fabs(denom) < T_zero ) {
        // case a==-b
        return Matrix<T,N,N>::Identity(N,N) +T(2)*b*a.adjoint();
    }

    return Matrix<T,N,N>::Identity(N,N)
           +T(2)*b*a.adjoint()
           -(a+b)*(a+b).adjoint()/denom;
}

} // namespace Geometry

#endif // MINROT_H
