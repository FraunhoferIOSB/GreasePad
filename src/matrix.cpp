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

#include <QDataStream>
#include <QtCompilerDetection>
#include <qassert.h>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <vector>


using Eigen::Index;
using VectorXidx = Eigen::Vector<Index,Eigen::Dynamic>;


namespace Graph {


//! check if coeff(r,c)==1
bool IncidenceMatrix::isSet( const Index r, const Index c) const
{
    Q_ASSERT_X( r>=0 && r<rows(), Q_FUNC_INFO, "row index out of range" );
    Q_ASSERT_X( c>=0 && c<cols(), Q_FUNC_INFO, "column index out of range" );
    Q_ASSERT( coeff(r,c)<2 );
    return coeff(r,c)==1;
}


//! Biadjacency matrix A = [O, B;  B',O]
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
void IncidenceMatrix::remove_column( const Index c) {

    // qDebug() << Q_FUNC_INFO;

    Q_ASSERT( c>=0 &&  c<cols() );

    const Index C = cols()-1;

    SparseMatrix<int> TT(C+1,C);
    for (Index i=0; i<c; i++) {
        TT.insert(i,i) = 1;
    }
    for (Index i=c; i<C; i++) {
        TT.insert(i+1,i) = 1;
    }
    *this = (*this)*TT;
}


//! remove row r
void IncidenceMatrix::remove_row( const Index r)
{
    // qDebug() << QString("remove row #%1").arg(r);

    Q_ASSERT( r>=0 && r<rows() );

    const Index R = rows()-1;

    SparseMatrix<int> SS(R,R+1);
    for (Index i=0; i<r; i++) {
        SS.insert(i,i) = 1;  // i.e., true
    }
    for (Index i=r; i<R; i++) {
        SS.insert(i,i+1) = 1;
    }

    *this = SS*(*this);
}


//! remove i-th column and i-th row
void IncidenceMatrix::reduce( const Index i)
{
    Q_ASSERT( i>=0 && i<rows() && i<cols() );
    // qDebug() << QString("reduce %1").arg(i);

    // selection matrix S
    SparseMatrix<int> SS( rows()-1,rows());
    for ( Index j=0; j<i; j++) {
        SS.insert(j,j) = 1;
    }
    for ( Index j=i; j<SS.rows(); j++) {
        SS.insert(j,j+1) = 1;
    }

    *this = SS*(*this)*SS.transpose();
}

} // namespace Graph
