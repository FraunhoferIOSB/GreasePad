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

#ifndef MATRIX_H
#define MATRIX_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>



namespace Graph {

using Eigen::ColMajor;
using Eigen::SparseMatrix;
using Eigen::VectorXi;
using Eigen::placeholders::last_t;


//! Sparse incidence matrix to encode relationships
class IncidenceMatrix : public SparseMatrix<int, ColMajor, int>
{
public:
    //! Copy assignment operator
    template<typename OtherDerived>
    IncidenceMatrix& operator=( const SparseMatrixBase <OtherDerived>& other)
    {
        this->SparseMatrix<int, ColMajor,int>::operator=(other);
        return *this;
    }

    [[nodiscard]] bool isSet( Index r, Index c) const;     //!< Check if r and c are related
    [[nodiscard]] bool isSet( Index r, last_t last) const;     //!< Check if r and c are related
    [[nodiscard]] bool isSet( last_t last, Index c) const;     //!< Check if r and c are related

    [[nodiscard]] SparseMatrix<int> biadjacency() const;   //!< Create biadjacency matrix [O, A; A', O]

    void set( const Index r, const Index c) { coeffRef(r,c) = 1; }  //!< Set relation (row r, column c)
    void set( const Index r, const last_t /*unused*/) { coeffRef(r,cols()-1) = 1; }  //!< Set relation (row r, column c)
    void set( const last_t /*unused*/, const Index c) { coeffRef(rows()-1,c) = 1; }  //!< Set relation (row r, column c)

    void unset( const Index r, const Index c)  { coeffRef(r,c) = 0; }  //!< Delete relation (row r, column c)
    void unset( const Index r, const last_t /*unused*/)  { coeffRef(r,cols()-1) = 0; }  //!< Delete relation (row r, column c)
    void unset( const last_t /*unused*/, const Index c)  { coeffRef(rows()-1,c) = 0; }  //!< Delete relation (row r, column c)

    void remove_row(    Index r );  //!< Remove r-th row
    void remove_column( Index c );  //!< Remove c-th column
    void reduce( Index i);          //!< Remove i-th column and i-th row

    //! Augment matrix by one row and one column
    void augment() { conservativeResize( rows()+1, cols()+1); }
};

} // namespace Graph

#endif // MATRIX_H
