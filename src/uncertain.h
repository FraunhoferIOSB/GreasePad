/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2023 Jochen Meidow, Fraunhofer IOSB
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

#include <Eigen/Dense>

#include <cfloat>
#include <cmath>
#include <utility>

//! Uncertain geometric entities
namespace Uncertain {

using Eigen::Matrix3d;
using Eigen::Vector3d;

//! Check if matrix is covariance matrix
bool isCovMat( const Eigen::MatrixXd &MM);


//! Uncertain distance
class uDistance
{
public:
    //! Construct uncertain distance with distance and its variance
    uDistance( const double d, const double var_d) : m_d(d), m_var_d(var_d) {}

    //! Get variance of distance
    double var_d() const { return m_var_d; }

    //! Get distance
    double d() const { return m_d; }

    //! Check if distance is greater than zero.
    bool isGreaterThanZero( const double T) const { return m_d/sqrt(m_var_d) > -T;  }

    //! Check if distance is less than zero.
    bool isLessThanZero(    const double T) const { return m_d/sqrt(m_var_d) < +T;  }

private:
    const double     m_d;
    const double m_var_d;
};


//! Base class for uncertain geometric entities, represented by 3-vectors
class BasicEntity2D
{
protected:
    BasicEntity2D() = default;                                     //!< Default constructor
    BasicEntity2D & operator= ( const BasicEntity2D &) = default;  //!< Copy assignment

public:
    //! Construct entity with homogeneous 3-vector and its covariance matrix
    BasicEntity2D( Eigen::Vector3d z, Eigen::Matrix3d Cov_zz) : m_val(std::move(z)), m_cov(std::move(Cov_zz)) {}
    BasicEntity2D ( const BasicEntity2D &) = default;              //!< Copy constructor
    BasicEntity2D( BasicEntity2D &&) = delete;                     //!< Move constructor

    ~BasicEntity2D() = default;

    void normalizeSpherical();
    void transform( const Eigen::Matrix3d &TT);    // point or line transformation

    //! Get covariance matrix
    Matrix3d Cov() const { return m_cov; }

    //! Get homogeneous 3-vector representing the entity
    Vector3d v()   const { return m_val; }

    bool isIdenticalTo( const BasicEntity2D &s, double T) const;

protected:
    Vector3d m_val; //!< homogeneous 3-vector representing the entity
    Matrix3d m_cov; //!< homogeneous 3x3 covariance matrix

    template <typename T>
    inline int sign(T val) const { return (T(0) <= val) - (val < T(0));  }  // sign(0):=+1

private:
    //! Nullspace of row 3-vector
    static Eigen::Matrix<double, 3, 2> null(const Eigen::Vector3d &xs);
};

class uPoint;

//! Estimation of two points delimiting an uncertain straight line segment
std::pair<uPoint,uPoint> uEndPoints( const Eigen::VectorXd &,
                                     const Eigen::VectorXd &);

} // namespace Uncertain

#endif // UNCERTAIN_H
