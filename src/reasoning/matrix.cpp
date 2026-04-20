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

#include "matrix.h"

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <cassert>
#include <vector>


namespace Graph {

using Eigen::Index;


//! check if coeff(r,c)==1
bool IncidenceMatrix::isSet( const Index r, const Index c) const
{
    assert( coeff(r,c)<2 );
    return coeff(r,c)==1;
}


bool IncidenceMatrix::isSet( const Index r, const last_t /*unused*/) const
{
    return coeff(r,cols()-1)==1;
}


bool IncidenceMatrix::isSet( const last_t /*unused*/, const Index c) const
{
    return coeff(rows()-1,c)==1;
}


//! Biadjacency matrix B = [O, A;  A',O]
SparseMatrix<int> IncidenceMatrix::biadjacency() const
{
    const Index C = cols();
    const Index R = rows();

    // vector of triplets (i,j,value)
    std::vector<Eigen::Triplet<int, Index> > tripletList;
    for ( Index k=0; k<outerSize(); ++k) {
        for (SparseMatrix<int>::InnerIterator it(*this,k); it; ++it)
        {
            tripletList.emplace_back( it.row(),    it.col()+R, it.value() );
            tripletList.emplace_back( it.col()+R, it.row(),    it.value() );
        }
    }

    // create sparse matrix
    SparseMatrix<int> AA;
    AA.resize( C+R, C+R );
    AA.setFromTriplets( tripletList.begin(), tripletList.end() );

    return AA;
}


//! remove column c
void IncidenceMatrix::remove_column( const Index c)
{
    assert( c>=0 && c<cols() );

    const Index C = cols()-1;

    SparseMatrix<int> RR(C+1,C);
    for (Index i=0; i<c; i++) {
        RR.insert(i,i) = 1;
    }
    for (Index i=c; i<C; i++) {
        RR.insert(i+1,i) = 1;
    }
    *this = (*this)*RR;
}


//! remove row r
void IncidenceMatrix::remove_row( const Index r)
{
    assert( r>=0 && r<rows() );

    const Index R = rows()-1;

    SparseMatrix<int> LL(R,R+1);
    for (Index i=0; i<r; i++) {
        LL.insert(i,i) = 1;
    }
    for (Index i=r; i<R; i++) {
        LL.insert(i,i+1) = 1;
    }

    *this = LL*(*this);
}


//! remove r-th column and c-th row
void IncidenceMatrix::reduce( const Index r, const Index c)
{
    assert( r>=0 && r<rows() );
    assert( c>=0 && c<cols() );

    // selection matrix LL (left, rows)
    SparseMatrix<int> LL( rows()-1,rows());
    for ( Index i=0; i<r; i++) {
        LL.insert(i,i) = 1;
    }
    for ( Index i=r; i<LL.rows(); i++) {
        LL.insert(i,i+1) = 1;
    }

    // selection matrix RR (right, cols)
    SparseMatrix<int> RR( cols(),cols()-1);
    for ( Index i=0; i<c; i++) {
        RR.insert(i,i) = 1;
    }
    for ( Index i=c; i<RR.cols(); i++) {
        RR.insert(i+1,i) = 1;
    }

    *this = LL*(*this)*RR;
}

} // namespace Graph
