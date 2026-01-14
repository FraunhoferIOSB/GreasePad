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

#include "constraints.h"

#include <Eigen/Core>

#include <QDebug>
#include <QStringLiteral>

#include "qassert.h"

#include <cassert>
#include <cmath>
#include <cstdlib>

#include "geometry/minrot.h"
#include "geometry/skew.h"
#include "matfun.h"

using Geometry::skew;
using Geometry::Rot_ab;

using Eigen::Vector3d;
using Eigen::Vector2cd;
using Eigen::Vector2d;
using Eigen::Matrix;
using Eigen::Matrix3d;
using VectorXidx = Eigen::Vector<Eigen::Index,Eigen::Dynamic>;

using Matfun::null;
using Matfun::cof3;
using Matfun::sign;


namespace Constraint {

ConstraintBase::ConstraintBase()
    : m_status(UNEVAL)
    , m_enforced(false)
{
    // qDebug().noquote() << Q_FUNC_INFO;
}

/*std::shared_ptr<ConstraintBase> ConstraintBase::clone() const
{
   // qDebug() << Q_FUNC_INFO;
   std::shared_ptr<ConstraintBase> ptr = doClone();
   auto & r = *ptr;  // .get();
   assert( typeid(r) == typeid(*this)
           && "ConstraintBase: doClone() incorrectly overridden" );
   return ptr;
}*/



MatrixXd Orthogonal::Jacobian( const VectorXidx & idxx,
                               const VectorXd &l0,
                               const VectorXd &l) const
{
    const Vector3d a0 = l0.segment( 3*idxx(0), 3);
    const Vector3d b0 = l0.segment( 3*idxx(1), 3);

    const Vector3d a = l.segment( 3*idxx(0), 3);
    const Vector3d b = l.segment( 3*idxx(1), 3);

    static const Matrix3d CC = Vector3d(1,1,0).asDiagonal();

    const Matrix<double,1,2> JJa = b0.adjoint()*CC*Rot_ab(a0,a)*null(a0);
    const Matrix<double,1,2> JJb = a0.adjoint()*CC*Rot_ab(b0,b)*null(b0);

    return (Matrix<double,1,4>() << JJa, JJb).finished();
}


VectorXd Orthogonal::contradict( const VectorXidx &idx,
                                 const VectorXd &l0) const
{
    const Vector3d a = l0.segment( 3*idx(0), 3);
    const Vector3d b = l0.segment( 3*idx(1), 3);

    static const Matrix3d CC = Vector3d(1,1,0).asDiagonal();

    return a.adjoint()*CC*b; // .dot(...) returns scalar
}


MatrixXd Copunctual::Jacobian(const VectorXidx &idx, const VectorXd &l0, const VectorXd &l) const
{
    const Vector3d a0 = l0.segment( 3*idx(0), 3);
    const Vector3d b0 = l0.segment( 3*idx(1), 3);
    const Vector3d c0 = l0.segment( 3*idx(2), 3);

    const Vector3d a = l.segment( 3*idx(0), 3);
    const Vector3d b = l.segment( 3*idx(1), 3);
    const Vector3d c = l.segment( 3*idx(2), 3);

    const Matrix3d MM = (Matrix3d() << a0,b0,c0 ).finished(); // [a0,b0,c0]
    const Matrix3d AA = cof3(MM).adjoint(); // adj(MM)

    constexpr int SIX = 6;
    const MatrixXd JJ = ( Matrix<double,1,SIX>()
                             << AA.row(0)*Rot_ab(a0,a)*null(a0),
                                AA.row(1)*Rot_ab(b0,b)*null(b0),
                                AA.row(2)*Rot_ab(c0,c)*null(c0) ).finished();

    return JJ;
}


VectorXd Copunctual::contradict(const VectorXidx &idx, const VectorXd &l0) const
{
    const Vector3d a = l0.segment( 3*idx(0), 3);
    const Vector3d b = l0.segment( 3*idx(1), 3);
    const Vector3d c = l0.segment( 3*idx(2), 3);

    const Matrix3d MM = (Matrix3d() << a,b,c).finished();  // [a0,b0,c0]

    return Vector<double,1>( MM.determinant() );
}


MatrixXd Identical::Jacobian( const VectorXidx & idx,
                              const VectorXd & l0,
                              const VectorXd & l) const
{
    Vector3d a0 = l0.segment( 3*idx(0),3 );
    Vector3d b0 = l0.segment(3 * idx(1), 3);

    Vector3d const a = l.segment(3 * idx(0), 3);
    Vector3d const b = l.segment( 3*idx(1),3 );

    Eigen::FullPivLU<MatrixXd> LU;  // identical
    Matrix<double,3,2> JJ;   // identical

    int idx1 = 0;
    int idx2 = 0;
    a0.cwiseAbs().maxCoeff( &idx1 );
    b0.cwiseAbs().maxCoeff( &idx2 );

    Q_ASSERT( a0(idx1)*b0(idx2) >= 0 );  // same sign

    LU.compute(a0.adjoint());
    JJ = LU.kernel();               //  JJ = null( a');
    // d2 = JJ.adjoint()*(a -b);       //  (10.141)

    Matrix<double, 2, 3> const JJa = null(a0).adjoint() * Rot_ab(a, a0);
    Matrix<double, 2, 3> const JJb = null(b0).adjoint() * Rot_ab(b, b0);

    MatrixXd Tmp(2,4);
    Tmp << JJ.adjoint()*JJa.adjoint(), -JJ.adjoint()*JJb.adjoint();

    return Tmp;
}

VectorXd Identical::contradict( const VectorXidx & idx,
                                const VectorXd & l0) const
{
    const Vector3d a0 = l0.segment( 3*idx(0),3 );
    const Vector3d b0 = l0.segment( 3*idx(1),3 );

    // check sign ............................................
    int idx1 = 0;
    int idx2 = 0;
    a0.head(2).cwiseAbs().maxCoeff( &idx1 );
    b0.head(2).cwiseAbs().maxCoeff( &idx2 );

    Q_ASSERT( a0(idx1)*b0(idx2) >= 0 ); // same sign

    const Matrix<double, 3, 2> JJ = null(a0);
    const Vector2d d2 = JJ.adjoint() * (a0 - b0); //  (10.141)

    return d2;
}



MatrixXd Parallel::Jacobian(const VectorXidx &idx, const VectorXd &l0, const VectorXd &l) const
{
    const Vector3d a0 = l0.segment( 3*idx(0), 3);
    const Vector3d b0 = l0.segment( 3*idx(1), 3);

    const Vector3d a = l.segment( 3*idx(0), 3);
    const Vector3d b = l.segment( 3*idx(1), 3);

    static const Matrix3d S3 = skew( Vector3d(0,0,1) );

    const Matrix<double,1,2> JJa = -b0.adjoint() * S3 * Rot_ab(a0, a) * null(a0);
    const Matrix<double,1,2> JJb =  a0.adjoint() * S3 * Rot_ab(b0, b) * null(b0);

    return (Matrix<double,1,4>() << JJa, JJb).finished();
}


VectorXd Parallel::contradict(const VectorXidx &idx, const VectorXd &l0) const
{
    const Vector3d a = l0.segment( 3*idx(0), 3);
    const Vector3d b = l0.segment( 3*idx(1), 3);

    static const Matrix3d S3 = skew( Vector3d(0,0,1) );

    return a.adjoint()*S3*b;
}


MatrixXd Vertical::Jacobian(const VectorXidx &idx, const VectorXd &l0, const VectorXd &l) const
{
    static const Vector3d e2(0,1,0);

    const Vector3d a0 = l0.segment(  3*idx(0), 3);
    const Vector3d a   =  l.segment( 3*idx(0), 3);

    // .dot(...) returns a scalar, not a 1-vector
    return e2.adjoint()* Rot_ab(a0,a)*null(a0);
}


VectorXd Vertical::contradict( const VectorXidx &idx,
                               const VectorXd &l0) const
{
    VectorXd tmp(1);
    tmp << l0.segment( (3*idx(0))+1, 1);  // 2nd element
    return tmp;
}

VectorXd Diagonal::contradict( const VectorXidx &idx,
                               const VectorXd &l0) const
{
    Vector3d l = l0.segment( 3*idx(0), 3);
    VectorXd tmp(1);  // scalar as vector
    tmp << abs(l(0)) - abs( l(1));   // abs(a)-abs(b)
    return tmp;
}


MatrixXd Diagonal::Jacobian( const VectorXidx &idx,
                             const VectorXd &l0,
                             const VectorXd &l) const
{
    // abs(a)-abs(b) = 0,  l=[a,b,c]',  J = [ sign(a), -sign(b), 0]
    Vector3d a0 = l0.segment( 3*idx(0), 3);
    Vector3d const a = l.segment(3 * idx(0), 3);
    Eigen::RowVector3d const JJ = (Eigen::RowVector3d() << sign(a0(0)), -sign(a0(1)), 0).finished();
    return JJ * Rot_ab(a0,a) * null(a0);
}


VectorXd Horizontal::contradict( const VectorXidx &idx,
                                 const VectorXd &l0) const
{
    VectorXd tmp(1);  // scalar as vector
    tmp << l0.segment( 3*idx(0), 1); // 1st element of 3-vector
    return tmp;
}


MatrixXd Horizontal::Jacobian( const VectorXidx &idx,
                               const VectorXd &l0,
                               const VectorXd &l) const
{
    const Vector3d a0 = l0.segment(3*idx(0), 3);
    const Vector3d a  = l.segment( 3*idx(0), 3);

    static const Vector3d e1(1,0,0);

    return e1.adjoint()*Rot_ab(a0,a)*null(a0);
}


} // namespace Constraint
