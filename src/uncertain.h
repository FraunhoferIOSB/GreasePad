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
#include <cmath>
#include <utility>

#include "geometry/skew.h"


//! Uncertain geometric entities
namespace Uncertain {

using Eigen::Matrix3d;
using Eigen::Vector3d;

using Geometry::skew;


//! Uncertain distance
class uDistance
{
public:
    //! Construct uncertain distance with distance and its variance
    uDistance( const double d, const double var_d) : m_d(d), m_var_d(var_d) {}

    //! Get variance of distance
    [[nodiscard]] double var_d() const { return m_var_d; }

    //! Get distance
    [[nodiscard]] double d() const { return m_d; }

    //! Check if distance is greater than zero.
    [[nodiscard]] bool isGreaterThanZero( const double T) const {
        if (m_var_d <= 0.0) {return false;}
        return m_d / std::sqrt(m_var_d) > -T;
    }

    //! Check if distance is less than zero.
    [[nodiscard]] bool isLessThanZero(    const double T) const {
        if (m_var_d <= 0.0) {return false;}
        return m_d / std::sqrt(m_var_d) < +T;
    }

private:
    double     m_d;
    double m_var_d;
};


//! Base class for uncertain geometric entities, represented by 3-vectors
class BasicEntity2D
{

public:
    //! Construct entity with homogeneous 3-vector and its covariance matrix
    BasicEntity2D( Eigen::Vector3d z, Eigen::Matrix3d Cov_zz) : m_val(std::move(z)), m_cov(std::move(Cov_zz)) {}
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

    /* template <typename T>
    int sign(T val) const { return (T(0) <= val) - (val < T(0));  }  // sign(0):=+1*/

private:
    Vector3d m_val; //!< homogeneous 3-vector representing the entity
    Matrix3d m_cov; //!< homogeneous 3x3 covariance matrix

    //! Nullspace of row 3-vector
    //static Eigen::Matrix<double, 3, 2> null(const Eigen::Vector3d &xs);
};

} // namespace Uncertain

#endif // UNCERTAIN_H
