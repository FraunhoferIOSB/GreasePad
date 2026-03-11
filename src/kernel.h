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


#ifndef KERNEL_H
#define KERNEL_H

#include <Eigen/Core>

#include <cassert>

namespace Matfun {

using Eigen::Matrix;
using Eigen::Vector;


//! Basis vectors for the nullspace of a row vector
//!
//! Förstner & Wrobel (2016) Photogrammetric Computer Vision, eq. (A.120)
template<int N>
[[nodiscard]] static Matrix<double,N,N-1>
null( const Vector<double,N> & xs )
{
    constexpr double T_zero = 1e-6;
    assert( std::fabs( xs.norm()-1. ) < T_zero && "vector norm not 1");

    Vector<double,N-1> x0 = xs.head(N-1);
    double xN = xs(N-1);
    if ( xN < 0 ) {
        x0 = -x0;
        xN = -xN;
    }

    const Matrix<double,N,N-1> JJ = (Matrix<double,N,N-1>() <<
        Matrix<double,N-1,N-1>::Identity(N-1,N-1) -x0*x0.transpose()/(1.+xN),
        -x0.transpose() ).finished();

    const Vector<double,N-1> check = JJ.adjoint()*xs;
    assert( check.norm() < T_zero && "not a zero vector");

    return JJ;
}

} // namespace Matfun

#endif // KERNEL_H
