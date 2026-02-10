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

#include <Eigen/Core>

namespace Geometry {

using Eigen::Dynamic;
using Eigen::Index;
using Eigen::Vector;


//! Axis-aligned bounding box
template <typename T>
class Aabb {

public:
    //! Value constructor
    explicit Aabb( Vector<T,Dynamic> min, Vector<T,Dynamic> max)
        : m_min(min), m_max(max)
    {
        assert( m_min.size() == m_max.size() );
        assert( ( m_min.array() <= m_max.array() ).all() );
    }
    Aabb() = default;

    //! Get i-th minimum value
    [[nodiscard]] T min(const int i) const {
        assert( i>=0 && i<m_min.size() );
        return m_min(i);
    }

    //! Get i-th maximum value
    [[nodiscard]] T max(const int i) const {
        assert( i>=0 && i<m_max.size() );
        return m_max(i);
    }

    //! Check if the other box overlaps
    [[nodiscard]] bool overlaps( const Aabb & other) const
    {
        return ( m_max.cwiseMin(other.m_max).array()
               > m_min.cwiseMax(other.m_min).array() ).all();
    }

    //! Get united/merged box of this and the other box
    [[nodiscard]] Aabb united( const Aabb & other) const
    {
        return Aabb {
            m_min.cwiseMin(other.m_min),
            m_max.cwiseMax(other.m_max)
        };
    }

    //! Get dimension of bounding box
    [[nodiscard]] Index dim() const {return m_min.size();}

private:
    Vector<T,Dynamic> m_min;
    Vector<T,Dynamic> m_max;
};

} // namespace Geometry

#endif // AABB_H
