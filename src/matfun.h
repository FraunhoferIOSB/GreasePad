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

#ifndef MATFUN_H
#define MATFUN_H


#include <Eigen/Core>
#include <Eigen/OrderingMethods>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>

#include <algorithm>
#include <cassert>
#include <cmath>


namespace Matfun {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::SparseMatrix;
using Eigen::SparseVector;
using Eigen::Matrix3d;
using Eigen::Index;
using Eigen::Vector;
using Eigen::Dynamic;
using Eigen::placeholders::last;


//! Nullspace of row vector
[[nodiscard,maybe_unused]] static MatrixXd null( const VectorXd & xs )
{
    // cf. PCV, eq. (A.120)
    constexpr double T_zero = 1e-6;
    assert( std::fabs( xs.norm()-1. ) < T_zero && "vector norm not 1");

    const Index N = xs.size();
    VectorXd x0 = xs.head(N-1);
    double   xN = xs(last);
    if ( xN < 0.0 ) {
        x0 = -x0;
        xN = -xN;
    }

    const MatrixXd JJ = ( MatrixXd(N,N-1)
         << MatrixXd::Identity(N-1,N-1) -x0*x0.adjoint()/(1.+xN),
            -x0.adjoint() ).finished();

    const VectorXd check = JJ.adjoint()*xs;
    assert( check.norm() < T_zero && "not a zero vector");

    return JJ;
}


//! check if the matrix AA is rank-deficient
[[maybe_unused]] static bool is_rank_deficient( SparseMatrix<double,Eigen::ColMajor> & AA, const double threshold )
{
    // Eigen::ColPivHouseholderQR<MatrixXd> qr(AA);
    // qr.setThreshold( T );
    Eigen::SparseQR<SparseMatrix<double>,Eigen::COLAMDOrdering<int>> qr;
    AA.makeCompressed();
    qr.compute(AA);
    qr.setPivotThreshold( threshold );

    return ( qr.rank() < std::min( AA.rows(), AA.cols()) );
}


//! 3x3 cofactor matrix, i.e., transposed adjugate
[[nodiscard,maybe_unused]] static Matrix3d cof3(const Matrix3d &MM)
{
    const Matrix3d Cof{
        {  +MM(1,1)*MM(2,2) -MM(2,1)*MM(1,2),
           -MM(1,0)*MM(2,2) +MM(2,0)*MM(1,2),
           +MM(1,0)*MM(2,1) -MM(2,0)*MM(1,1) },
        {  -MM(0,1)*MM(2,2) +MM(2,1)*MM(0,2),
           +MM(0,0)*MM(2,2) -MM(2,0)*MM(0,2),
           -MM(0,0)*MM(2,1) +MM(2,0)*MM(0,1) },
        {  +MM(0,1)*MM(1,2) -MM(1,1)*MM(0,2),
           -MM(0,0)*MM(1,2) +MM(1,0)*MM(0,2),
           +MM(0,0)*MM(1,1) -MM(1,0)*MM(0,1) }};

    return Cof;
}


// //! Matlab's find(x,1,'first')
// template <typename T>
// [[nodiscard]] static Index indexOf(const Eigen::Vector<T,Eigen::Dynamic> &v, const T x)
// {
//     for ( Index i=0; i<v.size(); i++) {
//         if ( v(i)==x ) {
//             return i;
//         }
//     }
//     return -1;
//
//     /* Eigen 3.4.0
//     auto it = std::find( v.begin(), v.end(), i);
//     if ( it==v.end() ) {
//         return -1;
//     }
//     return std::distance( v.begin(), it); */
// }


//! signum function with sign(0):=+1
template <typename T>
constexpr int sign(T val) noexcept {
    return (T(0) <= val) - (val < T(0));
}


//! Matlab's find
[[nodiscard,maybe_unused]] static Vector<Index,Dynamic> find( const Vector<bool,Dynamic> & cond)
{
    Vector<Index,Dynamic> idx( cond.count() );
    for (Index i=0, k=0; i< cond.size(); i++) {
        if ( cond(i) ) {
            idx(k++) = i;
        }
    }
    return idx;
}

//! find indices of sparse vectors
template <typename T>
[[nodiscard,maybe_unused]] static Vector<Index,Dynamic> spfind( const SparseVector<T> & v)
{
    Vector<Index,Dynamic> idx( v.nonZeros() );

    Index i=0;
    for ( typename SparseVector<T>::InnerIterator it(v); it; ++it) {
        idx(i++) = it.index();
    }

    return idx;
}

} // namespace Matfun

#endif // MATFUN_H
