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


#ifndef SKEW_H
#define SKEW_H


#include <Eigen/Core>

namespace Geometry {

//! skew-symmetric matrix S(x) from vector x
template <typename T>
[[maybe_unused]] inline static Eigen::Matrix<T,3,3> skew(const Eigen::Vector<T,3> &x)
{
    return (Eigen::Matrix<T,3,3>() << 0.,-x(2),x(1), x(2),0.,-x(0), -x(1),x(0),0.).finished();
}

} // namespace Geometry

#endif // SKEW_H
