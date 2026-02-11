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


#ifndef QUNCERTAIN_H
#define QUNCERTAIN_H

#include "uncertain/usegment.h"

#include <QDataStream>

using Uncertain::uStraightLineSegment;

using Eigen::Vector;
using Eigen::Matrix;
using Eigen::MatrixBase;
using Eigen::Dynamic;


namespace {

//! Overloaded operator>> for matrices
template <typename T>
QDataStream & operator>> ( QDataStream & in, MatrixBase<T> & MM)
{
    for ( int r=0; r<MM.rows(); r++) {
        for ( int c=0; c<MM.cols(); c++) {
            in >> MM(r,c);
        }
    }

    return in;
}


//! Overloaded operator<< for matrices
template <typename T>
QDataStream & operator<< ( QDataStream & out, const MatrixBase<T> &MM)
{
    //qDebug() << Q_FUNC_INFO;
    for ( int r=0; r<MM.rows(); r++) {
        for (int c=0; c<MM.cols(); c++) {
            out << MM(r,c);
        }
    }

    return out;
}


//! Overloaded operator<< for vectors
template <typename T>
QDataStream & operator<< ( QDataStream & out, const Vector<T,Dynamic> &v)
{
    //qDebug() << Q_FUNC_INFO;
    for (const T val : v) {
        out << val;
    }

    return out;
}


//! Overloaded operator>> for vectors
template <typename T, int N>
QDataStream & operator>> ( QDataStream & in, Vector<T,N> & v)
{
    for (T & val : v ) {
        in >> val;
    }

    return in;
}


//! Deserialization of uncertain straight line segment and its bounding box
QDataStream & operator>> ( QDataStream & in, uStraightLineSegment & us)
{
    // qDebug() << Q_FUNC_INFO;
    Vector<double,9> t;
    Matrix<double,9,9> Sigma_tt;
    in >> t;
    in >> Sigma_tt;
    us = uStraightLineSegment(t, Sigma_tt);

    return in;
}


QDataStream & operator<< ( QDataStream & out, const uStraightLineSegment & us)
{
    //qDebug() << Q_FUNC_INFO;
    out << us.t();
    out << us.Cov_tt();

    return out;
}

} // namespace

#endif // QUNCERTAIN_H
