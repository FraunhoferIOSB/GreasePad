/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2025 Jochen Meidow, Fraunhofer IOSB
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

#include "aabb.h"

#include <Eigen/Dense>  // Eigen
#include <cassert>
#include <cmath>

Aabb::Aabb( const double x_min,  const double x_max,
            const double y_min,  const double y_max)
: m_x_min(x_min), m_x_max(x_max), m_y_min(y_min), m_y_max(y_max)
{
   assert( m_x_min <= m_x_max );  // option: swap
   assert( m_y_min <= m_y_max );
}


bool Aabb::overlaps( const Aabb & other) const
{
    return (    ( std::fmin( m_x_max, other.m_x_max) > std::fmax( m_x_min, other.m_x_min) )
             && ( std::fmin( m_y_max, other.m_y_max) > std::fmax( m_y_min, other.m_y_min) )
           );
}

Aabb Aabb::united( const Aabb & other) const
{
    /* return { std::fmin( m_x_min, other.m_x_min),
            std::fmax( m_x_max, other.m_x_max),
            std::fmin( m_y_min, other.m_y_min),
            std::fmax( m_y_max, other.m_y_max) };*/
    return Aabb( std::fmin( m_x_min, other.m_x_min),
                std::fmax( m_x_max, other.m_x_max),
                std::fmin( m_y_min, other.m_y_min),
                std::fmax( m_y_max, other.m_y_max) );
}
