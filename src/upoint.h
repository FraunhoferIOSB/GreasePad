/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022 Jochen Meidow, Fraunhofer IOSB
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

#ifndef UPOINT_H
#define UPOINT_H

#include "uncertain.h"

class aabb;    // axis-aligned bounding box

namespace Uncertain {

class uStraightLine;


class uPoint : public BasicEntity2D
{
public:
    uPoint() = delete; // default;
    uPoint( const Vector3d & x,
            const Matrix3d & Cov_xx);
    // uPoint( const uPoint &ux ) = default;
    ~uPoint() override = default;

    aabb bbox() const;
    /* nodiscard */ uPoint euclidean() const;
    /* nodiscard */ uPoint sphericalNormalized() const;
    /* nodiscard */ uPoint transformed( const Matrix3d & TT) const;

    uDistance distanceAlgebraicTo( const uStraightLine & ul) const;
    uDistance distanceEuclideanTo( const uStraightLine & ul) const;

    bool isIncidentWith( const uStraightLine & ul, double T) const;

private:
    template <typename T>
    inline int sign(T val) const { return (T(0) < val) - (val < T(0));  }
};

} // namespace Uncertain

#endif // UPOINT_H
