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

#include "aabb.h"

#include <Eigen/Dense>  // Eigen
#include <QDataStream>  // Qt

aabb::aabb( const double x_min,
            const double x_max,
            const double y_min,
            const double y_max)
: x_min_(x_min)
{
   x_max_ = x_max;
   y_min_ = y_min;
   y_max_ = y_max;

   assert( x_min_ <= x_max_ );  // option: swap
   assert( y_min_ <= y_max_ );
}


bool aabb::intersects( const aabb & other) const
{
    return (    ( std::fmin( x_max_, other.x_max_) > std::fmax( x_min_, other.x_min_) )
             && ( std::fmin( y_max_, other.y_max_) > std::fmax( y_min_, other.y_min_) )
           );
}

aabb aabb::united( const aabb & other) const
{
   return { std::fmin( x_min_, other.x_min_),
            std::fmax( x_max_, other.x_max_),
            std::fmin( y_min_, other.y_min_),
            std::fmax( y_max_, other.y_max_)};
}


void aabb::serialize( QDataStream &out ) const
{
    // qDebug() << Q_FUNC_INFO;
    out << x_min_ << x_max_ << y_min_ << y_max_;
}

bool aabb::deserialize( QDataStream &in )
{
    // qDebug() << Q_FUNC_INFO;
    in >> x_min_ >> x_max_ >> y_min_ >> y_max_;
    return in.status()==0;
}

