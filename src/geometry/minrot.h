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


#ifndef MINROT_H
#define MINROT_H

#include <cassert>

#include <Eigen/Core>


namespace Geometry {

using Eigen::MatrixXd;
using Eigen::VectorXd;


[[maybe_unused]] static MatrixXd Rot_ab( const VectorXd &a, const VectorXd &b)
{
    assert( a.size()==b.size() );
#ifdef QT_DEBUG
    Q_ASSERT( std::fabs( a.norm()-1.) < 1e-5 );
    Q_ASSERT( std::fabs( b.norm()-1.) < 1e-5 );
#endif
    return MatrixXd::Identity( a.size(),a.size())
           +2*b*a.adjoint()
           -(a+b)*(a+b).adjoint()/(1.+a.dot(b));
}

} // namespace Geometry

#endif // MINROT_H
