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


#ifndef FIND_H
#define FIND_H

#include <Eigen/Core>
#include <Eigen/SparseCore>


namespace Matfun {

using Eigen::ArrayXi;
using Eigen::Dynamic;
using Eigen::Index;
using Eigen::Vector;
using Eigen::SparseVector;


//! Find indices of 1-elements in vector
[[nodiscard, maybe_unused]] static ArrayXi
find( const Vector<bool,Dynamic> & cond)
{
    ArrayXi idx( cond.count() );
    for (int i=0, k=0; i<cond.size(); i++) {
        if ( cond(i) ) {
            idx(k++) = i;
        }
    }

    return idx;
}


//! Find indices of nonzeros elements in sparse vector
template <typename T>
[[nodiscard]] static Vector<Index,Dynamic>
find( const SparseVector<T> & v)
{
    Vector<Index,Dynamic> idx( v.nonZeros() );

    Index i=0;
    for ( typename SparseVector<T>::InnerIterator it(v); it; ++it) {
        idx(i++) = it.index();
    }

    return idx;
}

} // namespace Matfun


// //! Matlab's find(x,1,'first')
// template <typename T>
// [[nodiscard]] static Index indexOf(const Eigen::Vector<T,Eigen::Dynamic> &v, const T x)
// {
//     for ( Index i=0; i<v.size(); i++) {
//         if ( v(i)==x ) {
//             return i;
//         }
//     }
//     return -1;
//
//     /* Eigen 3.4.0
//     auto it = std::find( v.begin(), v.end(), i);
//     if ( it==v.end() ) {
//         return -1;
//     }
//     return std::distance( v.begin(), it); */
// }


#endif // FIND_H
