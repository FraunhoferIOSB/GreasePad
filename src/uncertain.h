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

#ifndef UNCERTAIN_H
#define UNCERTAIN_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cassert>
#include <utility>

#include "geometry/skew.h"
#include "statistics/iscov.h"


//! Uncertain geometric entities
namespace Uncertain {

using Eigen::Matrix3d;
using Eigen::Vector3d;

using Geometry::skew;

using Stats::isCovMat;


//! Base class for uncertain geometric entities, represented by 3-vectors
class BasicEntity2D
{

public:
    //! Construct entity with homogeneous 3-vector and its covariance matrix
    BasicEntity2D( Vector3d z, Matrix3d Sigma_zz)
        : m_val(std::move(z)), m_cov(std::move(Sigma_zz))
    {
        assert( m_val.size()==m_cov.cols() );
        assert( isCovMat( m_cov) );
    }
    BasicEntity2D ( const BasicEntity2D &) = default;              //!< Copy constructor
    BasicEntity2D( BasicEntity2D &&) = default; //delete;                     //!< Move constructor
    BasicEntity2D & operator= ( const BasicEntity2D &&) = delete;  //!< Move assignment

    ~BasicEntity2D() = default;

    void normalizeSpherical();

    //! Get covariance matrix
    [[nodiscard]] Matrix3d Cov() const { return m_cov; }

    //! Get homogeneous 3-vector representing the entity
    [[nodiscard]] Vector3d v()   const { return m_val; }

    //! Get i-th element of homogeneous 3-vector
    [[nodiscard]] double v( const int i)  const {
        assert( i>=0 && i<3);
        return m_val(i);
    }

    [[nodiscard]] bool isIdenticalTo( const BasicEntity2D &s, double T) const;

protected:
    BasicEntity2D() = default;                                     //!< Default constructor
    BasicEntity2D & operator= ( const BasicEntity2D &) = default;  //!< Copy assignment

private:
    Vector3d m_val; //!< homogeneous 3-vector representing the entity
    Matrix3d m_cov; //!< homogeneous 3x3 covariance matrix
};

} // namespace Uncertain

#endif // UNCERTAIN_H
