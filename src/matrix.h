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

#ifndef MATRIX_H
#define MATRIX_H

#include <Eigen/Sparse>

namespace Graph {

using Eigen::SparseMatrix;
using Eigen::ColMajor;
using Eigen::VectorXi;
// using Eigen::IOFormat;

//! Sparse incidence matrix to encode relationships
class IncidenceMatrix : public SparseMatrix<int, ColMajor, int>
{
public:
    //! Copy assignment operator
    template<typename OtherDerived>
    IncidenceMatrix& operator=( const Eigen::SparseMatrixBase <OtherDerived>& other)
    {
        this->Eigen::SparseMatrix<int, ColMajor,int>::operator=(other);
        return *this;
    }

    VectorXi findInColumn( int c ) const;    //!< Matlab: find(A(:,c))
    bool isSet( Index r, Index c) const;     //!< Check if r and c are related
    SparseMatrix<int> biadjacency() const;   //!< Create biadjacency matrix [O, A; A', O]

    void   set( Index r, Index c)  { coeffRef(r,c) = 1; }  //!< Set relation (row r, column c)
    void unset( Index r, Index c)  { coeffRef(r,c) = 0; }  //!< Delete relation (row r, column c)

    void remove_row(    int r );  //!< Remove r-th row
    void remove_column( int c );  //!< Remove c-th column
    void reduce( int i);          //!< Remove i-th column and i-th row

    //! Augment matrix by one row and one column
    void augment() { conservativeResize( rows()+1, cols()+1); }

private:
    // static const IOFormat fmt;
};

} // namespace Graph

#endif // MATRIX_H
