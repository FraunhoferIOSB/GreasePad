/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2025 Jochen Meidow, Fraunhofer IOSB
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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include "matrix.h"


//! Sparse incidence matrix and connected components
namespace Graph {

using Eigen::SparseMatrix;
using Eigen::ColMajor;
using Eigen::Index;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::VectorXi;
using Eigen::VectorXidx;


[[nodiscard,maybe_unused]] static VectorXi conncomp(const SparseMatrix<int, ColMajor> &AA)
{

    static VectorXi m_comp;    // index vector components, 0-based
    static Eigen::Vector<bool,Dynamic> m_visited;    // for each vertex: visited?

    struct s {
        static void depthFirstSearch( const SparseMatrix<int,ColMajor> & CC, int c, Index v)
        {
            m_visited(v) = true;
            m_comp(v) = c;

            for ( SparseMatrix<int,ColMajor>::InnerIterator it(CC,v); it; ++it) {
                if ( !m_visited(it.index()) ) {
                    depthFirstSearch( CC, c, it.row() );
                }
            }
        }
    };

    const Index V = AA.rows(); // number of vertices
    constexpr int INVALID_INDEX = -1;
    m_visited.setConstant( V,false);
    m_comp.setConstant( V,INVALID_INDEX);
    int num_connected_components = 0;
    for ( Index v=0; v<V; v++ ) {
        if ( !m_visited(v) ) {
            s::depthFirstSearch( AA, num_connected_components++, v );
        }
    }

    return m_comp;
}


/* Connected components (vector with indices/labels)
class ConnComp
{
public:
    explicit ConnComp( const SparseMatrix<int, ColMajor> & BB);  //!< Value constructor with sparse matrix

    // [[nodiscard]] VectorXidx mapHead( int cc, Index n) const;  //!< Get linear indices of the elements in 1...n with label 'cc'.
    // [[nodiscard]] VectorXidx mapTail(int cc, Index n) const;  //!< Get linear indices of the elements in n-1...end with label 'cc'.
    [[nodiscard]] int label( Index i) const;     //!< Get label/index of i-th element
    [[nodiscard]] VectorXi head( Index n) const;   //!< Get labels/indices of first n elements
    [[nodiscard]] VectorXi tail( Index n) const;   //!< Get labels/indices of last n elements

    //! Get number of connected components
    [[nodiscard]] int number() const {  return m_comp.size()>0 ? m_comp.maxCoeff()+1 : 0; }

private:
    void dfs( const SparseMatrix<int,ColMajor> & CC,
              int c, Index v);

    VectorXi m_comp;                     // index vector components, 0-based
    Matrix<bool,Dynamic,1> m_visited;    // for each vertex: visited?
};*/

} // namespace Graph

#endif // CONNCOMP_H
