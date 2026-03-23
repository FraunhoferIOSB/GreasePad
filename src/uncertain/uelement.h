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


#ifndef UELEMENT_H
#define UELEMENT_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cassert>
#include <utility>

#include "geometry/skew.h"
#include "kernel.h"
#include "matfun.h"
#include "statistics/iscov.h"


//! Uncertain geometric entities
namespace Uncertain {

using Eigen::Matrix;
using Eigen::Vector;

using Geometry::skew;

using Matfun::null;
using Matfun::sign;

using Stats::isCovMat;


//! Base class for uncertain geometric elements, represented by homogeneous N-vectors
template <int N>
class uElement
{

public:
    //! Construct geometric element with N-vector and its covariance matrix
    uElement( Vector<double,N> z, Matrix<double,N,N> Sigma_zz)
        : m_val(std::move(z)), m_cov(std::move(Sigma_zz))
    {
        assert( m_val.size()==m_cov.cols() );
        assert( isCovMat( m_cov) );
    }
    uElement ( const uElement &) = default;  //!< Copy constructor
    uElement( uElement &&) = default;        //!< Move constructor
    uElement & operator= ( const uElement &&) = delete;  //!< Move assignment

    ~uElement() = default;

    void normalizeSpherical();

    //! Get covariance matrix
    [[nodiscard]] Matrix<double,N,N> Cov() const {return m_cov;}

    //! Get homogeneous N-vector representing the geometric element
    [[nodiscard]] Vector<double,N> v() const {return m_val;}

    //! Get i-th element of the homogeneous N-vector
    [[nodiscard]] double v( const int i) const {return m_val(i);}

    [[nodiscard]] bool isIdenticalTo( const uElement &s, double T) const;

protected:
    uElement() = default;  //!< default constructor
    uElement & operator= ( const uElement &) = default;  //!< copy assignment

private:
    Vector<double,N> m_val; //!< homogeneous N-vector representing the element
    Matrix<double,N,N> m_cov; //!< homogeneous NxN covariance matrix
};


//! Spherically normalize the entity
template <int N>
void uElement<N>::normalizeSpherical()
{
    static const Matrix<double,N,N> Id = Matrix<double,N,N>::Identity();
    assert( m_val.norm() > 0 );
    const Matrix<double,N,N> Jac = (Id-m_val*m_val.transpose()/m_val.squaredNorm())/m_val.norm();
    m_cov = Jac*m_cov*Jac.transpose();
    m_val.normalize();  // x = x /norm(x)
}


//! Check if uncertain element is identical with other uncertain element
template<int N>
bool uElement<N>::isIdenticalTo(const uElement & us,
                                const double T) const
{
    uElement a(*this);
    uElement b(us);

    // fix ambiguities
    a.normalizeSpherical();
    b.normalizeSpherical();

    int idx = 0;  // visitor
    a.v().cwiseAbs().maxCoeff( &idx );  // [~,idx] = max( abs(a) )
    a.m_val *= sign( a.v(idx) );
    b.m_val *= sign( b.v(idx) );

    const Matrix<double,N,N-1> Jac = null(a.v());     // (A.120)
    const Vector<double,N-1> d = Jac.transpose()*(a.v()-b.v() );  // (10.141)
    const Matrix<double,N-1,N-1> Sigma_dd = Jac.transpose()*(a.Cov()+b.Cov())*Jac;

    return d.dot( Sigma_dd.ldlt().solve(d) ) < T;
}

} // namespace Uncertain


#endif // UELEMENT_H
