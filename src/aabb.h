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

#ifndef AABB_H
#define AABB_H

#include <QDataStream>

class aabb {

public:
    // aabb() = default;
    // ~aabb() = default;
    aabb( double x_min=0, double x_max=0,
          double y_min=0, double y_max=0 );

    void serialize(   QDataStream & out ) const;
    bool deserialize( QDataStream & in  );

    // aabb & operator= (const aabb & rhs) = default;

    double x_min() const { return x_min_; }
    double x_max() const { return x_max_; }
    double y_min() const { return y_min_; }
    double y_max() const { return y_max_; }

    bool intersects( const aabb & other) const;
    /* nodiscard */ aabb united( const aabb & other) const;

private:
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
};

#endif // AABB_H
