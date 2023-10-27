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

#ifndef CONNCOMP_H
#define CONNCOMP_H

#include <Eigen/Sparse>

//! Sparse incidence matrix and connected components
namespace Graph {

using Eigen::SparseMatrix;
using Eigen::ColMajor;
using Eigen::Index;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::VectorXi;

//! Connected components (vector with indices/labels)
class ConnComp
{
public:
    ConnComp( const SparseMatrix<int, ColMajor> & BB);  //!< Value constructor with sparse matrix

    VectorXi mapHead( int cc, int n) const;  //!< Get linear indices of the elements in 1...n with label 'cc'.
    VectorXi mapTail( int cc, int n) const;  //!< Get linear indices of the elements in n-1...end with label 'cc'.
    int label( Index i) const;     //!< Get label/index) of i-th element
    VectorXi head( int n) const;   //!< Get labels/indices of first n elements
    VectorXi tail( int n) const;   //!< Get labels/indices of last n elements

    //! Get number of connected components
    int number() const {  return m_comp.size()>0 ? m_comp.maxCoeff()+1 : 0; }

private:
    void dfs( const SparseMatrix<int,ColMajor> & CC,
              int c, Index v);

    VectorXi m_comp;                     // index vector components, 0-based
    Matrix<bool,Dynamic,1> m_visited;    // for each vertex: visited?
};

} // namespace Graph

#endif // CONNCOMP_H
