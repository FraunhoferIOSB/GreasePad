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


#ifndef UNIQUE_H
#define UNIQUE_H

#include <Eigen/Core>

#include <vector>


namespace Matfun {

using Eigen::Array;
using Eigen::Vector;
using Eigen::Dynamic;

//! unique values in Eigen::Vector
template <typename T>
[[nodiscard,maybe_unused]] static Vector<T, Dynamic> unique( Vector<T,Dynamic> u)
{
    std::sort( u.begin(), u.end() );
    // remove consecutive (adjacent) duplicates:
    const auto last = std::unique( u.begin(), u.end() );
    const auto n = std::distance( u.begin(), last);
    return u.head(n);
}


//! unique values in nx1 Eigen::Array
template <typename T>
[[nodiscard,maybe_unused]] static Array<T, Dynamic,1> unique( Array<T,Dynamic,1> u)
{
    std::sort( u.begin(), u.end() );
    // remove consecutive (adjacent) duplicates:
    const auto last = std::unique( u.begin(), u.end() );
    const auto n = std::distance( u.begin(), last);
    return u.head(n);
}


//! unique values in std::vector
template <typename T>
[[nodiscard,maybe_unused]] static std::vector<T> unique( std::vector<T> u)
{
    std::sort( u.begin(), u.end() );
    // remove consecutive (adjacent) duplicates:
    const auto last = std::unique( u.begin(), u.end() );
    u.erase( last, u.end());
    return u;
}

} // namespace Matfun

#endif // UNIQUE_H
