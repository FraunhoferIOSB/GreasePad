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

#include <Eigen/Core>


namespace Uncertain {

using Eigen::MatrixBase;
using Eigen::Dynamic;
using Eigen::Vector;


//! Overloaded operator>> for matrices
template <typename T>
QDataStream & operator>> ( QDataStream & in, MatrixBase<T> & MM);

//! Overloaded operator<< for matrices
template <typename T>
QDataStream & operator<< ( QDataStream & out, const MatrixBase<T> &MM);

//! Overloaded operator<< for vectors
template <typename T>
QDataStream & operator<< ( QDataStream & out, const Vector<T,Dynamic> &v);

//! Overloaded operator>> for vectors
template <typename T, int N>
QDataStream & operator>> ( QDataStream & in, Vector<T,N> & v);


//! Deserialization of uncertain straight line segment and its bounding box
QDataStream & operator>> ( QDataStream & in, uStraightLineSegment & us);
QDataStream & operator<< ( QDataStream & out, const uStraightLineSegment & us);

} // namespace



#endif // QUNCERTAIN_H
