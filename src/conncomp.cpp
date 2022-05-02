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

#include "conncomp.h"
#include "global.h"

namespace Graph {

ConnComp::ConnComp( const SparseMatrix<int, ColMajor> &BB)
{
    // qDebug() << Q_FUNC_INFO;
    assert(   BB.rows()==BB.cols() );
    // Q_ASSERT( BB.rows()==BB.cols() );

    const Index V =  BB.rows(); // number of vertices
    m_visited.setConstant( V,false);
    m_comp.setConstant( V,-1);

    int num_connected_components = 0;
    for ( Index v=0; v<V; v++ ) {
        if ( !m_visited(v) ) {
            dfs( BB, num_connected_components++, v );
        }
    }
}



VectorXi ConnComp::head(const int n) const
{
   assert( n>=0 );
   if ( n==0) {
       return VectorXi(0);
   }
   return m_comp.head(n);
}

VectorXi ConnComp::tail( const int n) const
{
   assert( n>=0 );
   if ( n==0) {
       return VectorXi(0);
   }
   return m_comp.tail(n);
}

/*! Label of i-th element */
int ConnComp::label( const Index i) const
{
    assert( i>=0 &&  i<m_comp.size() ) ;
    return m_comp(i);
}


VectorXi ConnComp::mapHead( const int cc, const int n) const
{
    // (1) Vector length: How many elements with value cc? .........
    int sz=0;
    for ( int s=0; s<n; s++ ) {
        if ( m_comp(s)==cc ) {
            sz++;
        }
    }

    // (2) Linear indices of these elements ........................
    VectorXi map_(sz);
    for ( int k=0, s=0; s<n; s++ ) {
        if ( m_comp(s)==cc ) {
            map_( k++ ) = s;
        }
    }

    return map_;
}


VectorXi ConnComp::mapTail( const int cc, const int n) const
{
    // (1) Vector length: How many elements with value cc? .........
    int sz=0;
    for ( Index s=m_comp.size()-n; s<m_comp.size(); s++ ) {
        if ( m_comp(s)==cc ) {
            sz++;
        }
    }

    // (2) Linear indices of these elements ........................
    VectorXi map_(sz);
    Index offset = m_comp.size() -n;
    for ( int k=0,  s=0; s<n; s++  ) {
        if ( m_comp( s+offset )==cc ) {
            map_( k++ ) = s;
        }
    }

    return map_;
}


void ConnComp::dfs( const SparseMatrix<int,ColMajor> &CC,
                    const int c,
                    const Index v)
{
    m_visited(v) = true;
    m_comp(v) = c;

    for ( SparseMatrix<int,ColMajor>::InnerIterator it(CC,v); it; ++it) {
        if ( !m_visited(it.index()) ) {
            dfs( CC, c, it.row() );
        }
    }
}

} // namespace Graph
