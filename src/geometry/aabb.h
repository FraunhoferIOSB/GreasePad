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

#ifndef AABB_H
#define AABB_H

#include <cassert>
#include <cmath>


namespace Geometry {

//! Axis-aligned bounding box
class Aabb {

public:
    explicit Aabb( double x_min, double x_max,
                   double y_min, double y_max )  //!< Value constructor
        : m_x_min(x_min), m_x_max(x_max), m_y_min(y_min), m_y_max(y_max)
    {
        assert( m_x_min <= m_x_max );  // option: swap
        assert( m_y_min <= m_y_max );
    }
    Aabb() = default;

    [[nodiscard]] double x_min() const { return m_x_min; } //!< Get minimum x-value
    [[nodiscard]] double x_max() const { return m_x_max; } //!< Get maximum x-value
    [[nodiscard]] double y_min() const { return m_y_min; } //!< Get minimum y-value
    [[nodiscard]] double y_max() const { return m_y_max; } //!< Get maximum y-value

    [[nodiscard]] bool overlaps( const Aabb & other) const   //!< Check if the other box overlaps
    {
        return (   ( std::fmin( m_x_max, other.m_x_max) > std::fmax( m_x_min, other.m_x_min) )
                && ( std::fmin( m_y_max, other.m_y_max) > std::fmax( m_y_min, other.m_y_min) )
                );
    }
    [[nodiscard]] Aabb united( const Aabb & other) const     //!< Get united box of this and the other box
    {
        return Aabb( std::fmin( m_x_min, other.m_x_min),
                     std::fmax( m_x_max, other.m_x_max),
                     std::fmin( m_y_min, other.m_y_min),
                     std::fmax( m_y_max, other.m_y_max) );
    }

private:
    double m_x_min = 0;
    double m_x_max = 0;
    double m_y_min = 0;
    double m_y_max = 0;
};

} // namespace Geometry

#endif // AABB_H
