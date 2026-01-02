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

#ifndef UPOINT_H
#define UPOINT_H

#include "geometry/aabb.h"     // axis-aligned bounding box
#include "uncertain.h"


namespace Uncertain {

class uStraightLine;

using Geometry::Aabb;


//! Uncertain point
class uPoint : public BasicEntity2D
{
public:
    uPoint() = delete;
    uPoint( const Vector3d & x,
            const Matrix3d & Sigma_xx);

    [[nodiscard]] Aabb bbox() const;
    [[nodiscard]] uPoint euclidean() const;
    [[nodiscard]] uPoint sphericalNormalized() const;
    [[nodiscard]] uPoint transformed( const Matrix3d & TT) const;

    [[nodiscard]] uStraightLine cross( const uPoint &) const;

    [[nodiscard]] uDistance distanceAlgebraicTo( const uStraightLine & ul) const;
    [[nodiscard]] uDistance distanceEuclideanTo( const uStraightLine & ul) const;

    [[nodiscard]] bool isIncidentWith( const uStraightLine & ul, double T) const;
};

} // namespace Uncertain

#endif // UPOINT_H
