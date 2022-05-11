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

#include "global.h"
#include "matrix.h"

#include<QDataStream>

namespace Graph {

// using Eigen::IOFormat;
// using Eigen::DontAlignCols;
using Eigen::Triplet;
using Eigen::Index;

// const IOFormat IncidenceMatrix::fmt = IOFormat( 4, DontAlignCols, ", ", "\n", "| ", "|", "", ".\n");



 /* void IncidenceMatrix::serialize( QDataStream &out ) const
{
    // qDebug() << Q_FUNC_INFO

    out <<  uint( rows() );
    out <<  uint( cols() );
    out <<  uint( nonZeros() );

    for( Index c=0; c<outerSize(); ++c) {
        SparseMatrix<int,ColMajor>::InnerIterator it( *this, c);
        for( ; it; ++it) {
            out << int(it.row()) << int(it.col());
        }
    }
}


bool IncidenceMatrix::deserialize( QDataStream &in )
{
    // qDebug() << Q_FUNC_INFO;

    uint nrows;
    uint ncols;
    uint nnz;
    int r;
    int c;
    in >> nrows >> ncols >> nnz;

    if ( nnz> UINT_MAX  ) {
        return false;
    }

    std::vector< Triplet<int> > tripletList;
    tripletList.reserve( nnz );
    for ( uint i=0; i<nnz; i++ ) {
        in >> r >> c;
        tripletList.emplace_back( Triplet<int>(r, c, 1) );
    }
    if ( in.status() != 0 ) {
        return false;
    }

    resize( int(nrows), int(ncols) );
    setFromTriplets( tripletList.begin(),
                     tripletList.end() );

    return true;
}*/


bool IncidenceMatrix::isSet( const Index r, const Index c) const
{
    Q_ASSERT_X( r>=0 && r<rows(), Q_FUNC_INFO, "row index out of range" );
    Q_ASSERT_X( c>=0 && c<cols(), Q_FUNC_INFO, "column index out of range" );
    Q_ASSERT( coeff(r,c)<2 );
    return coeff(r,c)==1;
}

SparseMatrix<int> IncidenceMatrix::biadjacency() const {

    // compile A = [O, B;  B',O]
    Index C_ = cols();
    Index R_ = rows();

    SparseMatrix<int> AA;
    AA.resize(C_+R_,C_+R_);
    std::vector<Triplet<int, Index> > tripletList;
    for ( Index k = 0; k < outerSize(); ++k) {
        for (SparseMatrix<int>::InnerIterator it(*this,k); it; ++it)
        {
            tripletList.emplace_back( Triplet<int, Index>( it.row(),   it.col() +R_,  it.value() ));
            tripletList.emplace_back( Triplet<int, Index>( it.col() +R_,   it.row(),  it.value() ));
        }
    }
    AA.setFromTriplets( tripletList.begin(), tripletList.end() );

    return AA;
}


VectorXi IncidenceMatrix::findInColumn( const int c ) const
{
    Eigen::Index nnz = innerVector(c).nonZeros();

    VectorXi idx( nnz );
    int i=0;
    for ( SparseMatrix<int>::InnerIterator it(*this,c); it; ++it) {
        idx(i++) = it.index();
    }

    return idx;
}



void IncidenceMatrix::remove_column( const int c) {

    // qDebug() << Q_FUNC_INFO;

    Q_ASSERT( c>=0);
    Q_ASSERT( c<cols() );
    Index C =  cols()-1;

    SparseMatrix<int> TT(C+1,C);
    Index c2;
    for (c2=0; c2<c; c2++) {
        TT.insert(c2,c2) = 1;
    }
    for (c2=c; c2<C; c2++) {
        TT.insert(c2+1,c2) = 1;
    }
    *this = (*this)*TT;
}

void IncidenceMatrix::remove_row(const int r)
{
    // qDebug() << QString("remove row #%1").arg(r);

    Q_ASSERT( r>=0 );
    Q_ASSERT( r<rows() );

    Index R = rows()-1;
    SparseMatrix<int> SS(R,R+1);
    Index r2;
    for (r2=0; r2<r; r2++) {
        SS.insert(r2,r2) = 1;  // i.e., true
    }
    for (r2=r; r2<R; r2++) {
        SS.insert(r2,r2+1) = 1;
    }

    *this = SS*(*this);
}


// remove i-th column and i-th row.....................................
void IncidenceMatrix::reduce(const int i)
{
    // qDebug() << QString("reduce %1").arg(i);

    // selection matrix S
    SparseMatrix<int> SS( rows()-1,rows());
    for ( int c=0; c<i; c++) {
        SS.insert(c,c) = 1;
    }
    for ( Index c=i; c<SS.rows(); c++) {
        SS.insert(c,c+1) = 1;
    }

    *this = SS*(*this)*SS.transpose();
}

} // namespace Graph
