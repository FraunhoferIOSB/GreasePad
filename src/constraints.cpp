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

#include "constraints.h"

#include <Eigen/Core>
#include <QDebug>
#include "qassert.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <memory>

namespace Constraint {

using Eigen::Vector3d;
using Eigen::Vector2cd;
using Eigen::Matrix;

#ifdef QT_DEBUG
static const double T_ZERO = 1e-5;
#endif

MatrixXd ConstraintBase::Rot_ab( const VectorXd &a,
                                 const VectorXd &b)
{
    Q_ASSERT( a.size()==b.size() );
#ifdef QT_DEBUG
    Q_ASSERT( std::fabs( a.norm()-1.) < T_ZERO );
    Q_ASSERT( std::fabs( b.norm()-1.) < T_ZERO );
#endif
    return MatrixXd::Identity( a.size(),a.size())
            +2*b*a.adjoint()
            -(a+b)*(a+b).adjoint()/(1.+a.dot(b));
}

MatrixXd ConstraintBase::null( const VectorXd & xs )
{
    // cf. PCV, eq. (A.120)

    //if ( fabs(xs.norm()-1.) > T_ZERO )
    //    qDebug() << xs;

#ifdef QT_DEBUG
    QString const what = QStringLiteral("norm(x) = %1").arg( QString::number(xs.norm()) );
    Q_ASSERT_X(fabs(xs.norm() - 1.) <= T_ZERO, "null(x)", what.toStdString().data());
#endif

    Eigen::Index  const N = xs.size();

    VectorXd x0 = xs.head(N-1);
    double   xN = xs(N-1);
    if ( xN < 0.0 ) {
        x0 = -x0;
        xN = -xN;
    }

    MatrixXd JJ( N, N-1);
    JJ.topRows(N-1)  = MatrixXd::Identity(N-1,N-1) -x0*x0.adjoint()/(1.+xN);
    JJ.bottomRows(1) = -x0.adjoint();

    VectorXd const check = JJ.adjoint() * xs;
#ifdef QT_DEBUG
    Q_ASSERT_X( check.norm() <= T_ZERO, Q_FUNC_INFO, "not a zero vector");
#endif
    return JJ;
}

ConstraintBase::ConstraintBase()
    : m_status(UNEVAL)
    , m_enforced(false)
{
    // qDebug().noquote() << Q_FUNC_INFO;
}

std::shared_ptr<ConstraintBase> ConstraintBase::clone() const
{
   // qDebug() << Q_FUNC_INFO;
   std::shared_ptr<ConstraintBase> ptr = doClone();
   auto & r = *ptr;  // .get();
   assert( typeid(r) == typeid(*this)
           && "ConstraintBase: doClone() incorrectly overridden" );
   return ptr;
}



MatrixXd Orthogonal::Jacobian( const VectorXi & idxx,
                               const VectorXd &l0,
                               const VectorXd &l) const
{
    Vector3d const a0 = l0.segment(3 * idxx(0), 3);
    Vector3d const b0 = l0.segment(3 * idxx(1), 3);

    Vector3d const a = l.segment(3 * idxx(0), 3);
    Vector3d const b = l.segment(3 * idxx(1), 3);

    Matrix<double,1,2> JJa;
    Matrix<double,1,2> JJb;

    JJa = b0.adjoint()*CC()*Rot_ab(a0,a)*null(a0);
    JJb = a0.adjoint()*CC()*Rot_ab(b0,b)*null(b0);

    MatrixXd Tmp(1,4);
    Tmp << JJa, JJb;
    return Tmp;
}

VectorXd Orthogonal::contradict( const VectorXi &idx,
                                 const VectorXd &l0) const
{
    Vector3d const a = l0.segment(3 * idx(0), 3);
    Vector3d const b = l0.segment(3 * idx(1), 3);

    VectorXd tmp(1);
    tmp << a.dot( CC()*b );

    return tmp;
}


//! Diag([1,1,0])
Matrix3d Orthogonal::CC()
{
    const static Matrix3d tmp =  (Matrix3d() << 1,0,0,  0,1,0, 0,0,0).finished();
    return tmp;
}


std::shared_ptr<ConstraintBase> Orthogonal::doClone() const
{
    auto T = std::make_shared<Orthogonal>();
    T->setStatus( this->status() );
    T->setEnforced( this->enforced());
    return T;
}

MatrixXd Copunctual::Jacobian(const VectorXi &idx, const VectorXd &l0, const VectorXd &l) const
{
    Vector3d const a0 = l0.segment(3 * idx(0), 3);
    Vector3d const b0 = l0.segment(3 * idx(1), 3);
    Vector3d const c0 = l0.segment(3 * idx(2), 3);

    Vector3d const a = l.segment(3 * idx(0), 3);
    Vector3d const b = l.segment(3 * idx(1), 3);
    Vector3d const c = l.segment(3 * idx(2), 3);

    Matrix3d const MM = (Matrix3d() << a0,b0,c0 ).finished(); // [a0,b0,c0]
    Matrix3d Adju = cof3(MM).adjoint(); // adjugate(MM)

    MatrixXd Tmp(1,6);
    Tmp << Adju.row(0)*Rot_ab(a0,a)*null(a0),
           Adju.row(1)*Rot_ab(b0,b)*null(b0),
           Adju.row(2)*Rot_ab(c0,c)*null(c0);

    return Tmp;
}

VectorXd Copunctual::contradict(const VectorXi &idx, const VectorXd &l0) const
{
    Vector3d const a = l0.segment(3 * idx(0), 3);
    Vector3d const b = l0.segment(3 * idx(1), 3);
    Vector3d const c = l0.segment(3 * idx(2), 3);

    Matrix3d const MM = (Matrix3d() << a,b,c).finished();  // [a0,b0,c0]

    VectorXd tmp(1);
    tmp << MM.determinant();
    return tmp;
}


//! 3x3 cofactor matrix, i.e., transposed adjugate
Matrix3d Copunctual::cof3(const Matrix3d &MM)
{
    Matrix3d Cof;
    Cof(0,0) = +MM(1,1)*MM(2,2) -MM(2,1)*MM(1,2);
    Cof(0,1) = -MM(1,0)*MM(2,2) +MM(2,0)*MM(1,2);
    Cof(0,2) = +MM(1,0)*MM(2,1) -MM(2,0)*MM(1,1);

    Cof(1,0) = -MM(0,1)*MM(2,2) +MM(2,1)*MM(0,2);
    Cof(1,1) = +MM(0,0)*MM(2,2) -MM(2,0)*MM(0,2);
    Cof(1,2) = -MM(0,0)*MM(2,1) +MM(2,0)*MM(0,1);

    Cof(2,0) = +MM(0,1)*MM(1,2) -MM(1,1)*MM(0,2);
    Cof(2,1) = -MM(0,0)*MM(1,2) +MM(1,0)*MM(0,2);
    Cof(2,2) = +MM(0,0)*MM(1,1) -MM(1,0)*MM(0,1);

    return Cof;
}

std::shared_ptr<ConstraintBase> Copunctual::doClone() const
{
    auto T = std::make_shared<Copunctual>();
    T->setStatus(   this->status()   );
    T->setEnforced( this->enforced() );
    return T;
}

MatrixXd Identical::Jacobian( const VectorXi & idx,
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

    Q_ASSERT( sign( a0(idx1) )==sign( b0(idx2) ) );
    Q_ASSERT( sameSign( a0(idx1),  b0(idx2) ) );

    LU.compute(a0.adjoint());
    JJ = LU.kernel();               //  JJ = null( a');
    // d2 = JJ.adjoint()*(a -b);       //  (10.141)

    Matrix<double, 2, 3> const JJa = null(a0).adjoint() * Rot_ab(a, a0);
    Matrix<double, 2, 3> const JJb = null(b0).adjoint() * Rot_ab(b, b0);

    MatrixXd Tmp(2,4);
    Tmp << JJ.adjoint()*JJa.adjoint(), -JJ.adjoint()*JJb.adjoint();
    return Tmp;
}

VectorXd Identical::contradict( const VectorXi & idx,
                                const VectorXd & l0) const
{
    Vector3d a0 = l0.segment( 3*idx(0),3 );
    Vector3d b0 = l0.segment( 3*idx(1),3 );
    Eigen::FullPivLU<MatrixXd> const LU;

    // check sign ............................................
    int idx1 = 0;
    int idx2 = 0;
    a0.head(2).cwiseAbs().maxCoeff( &idx1 );
    b0.head(2).cwiseAbs().maxCoeff( &idx2 );

    Q_ASSERT( sameSign(a0(idx1), b0(idx2)) );
    // LU.compute( a0.adjoint() );
    // Matrix<double,3,2> JJ = LU.kernel();  //  JJ = null( a0');
    Matrix<double, 3, 2> const JJ = null(a0);
    Eigen::Vector2d const d2 = JJ.adjoint() * (a0 - b0); //  (10.141)

    return d2;
}

std::shared_ptr<ConstraintBase> Identical::doClone() const
{
    auto T = std::make_shared<Identical>();
    T->setStatus(   this->status()  );
    T->setEnforced( this->enforced());
    return T;
}

MatrixXd Parallel::Jacobian(const VectorXi &idx, const VectorXd &l0, const VectorXd &l) const
{
    Vector3d const a0 = l0.segment(3 * idx(0), 3);
    Vector3d const b0 = l0.segment(3 * idx(1), 3);

    Vector3d const a = l.segment(3 * idx(0), 3);
    Vector3d const b =  l.segment( 3*idx(1),3 );

    // Matrix<double,1,2> JJa;
    // Matrix<double,1,2> JJb;

    Matrix<double, 1, 2> const JJa = -b0.adjoint() * S3() * Rot_ab(a0, a) * null(a0);
    Matrix<double, 1, 2> const JJb = a0.adjoint() * S3() * Rot_ab(b0, b) * null(b0);

    MatrixXd Tmp(1,4);
    Tmp << JJa, JJb;
    return Tmp;
}

VectorXd Parallel::contradict(const VectorXi &idx, const VectorXd &l0) const
{
    Vector3d const a = l0.segment(3 * idx(0), 3);
    Vector3d const b = l0.segment( 3*idx(1),3 );

    VectorXd tmp(1,1);
    tmp << a.dot( S3()*b );

    return tmp;
}




std::shared_ptr<ConstraintBase> Parallel::doClone() const
{
    auto T = std::make_shared<Parallel>();
    T->setStatus(   this->status()  );
    T->setEnforced( this->enforced());
    return T;
}


//! skew(e_3),  e_3=[0,0,1]'
Matrix3d Parallel::S3()
{
    const static Matrix3d tmp = (Matrix3d() << 0,-1,0, +1,0,0, 0,0,0).finished();
    return tmp;
}


std::shared_ptr<ConstraintBase> Vertical::doClone() const
{
    auto T = std::make_shared<Vertical>();
    T->setStatus( this->status() );
    T->setEnforced( this->enforced());
    return T;
}


//! e_2 = [0,1,0]'
Vector3d Vertical::e2()
{
    const static Vector3d tmp = (Vector3d() << 0,1,0).finished();
    return tmp;
}

MatrixXd Vertical::Jacobian(const VectorXi &idx, const VectorXd &l0, const VectorXd &l) const
{
    Vector3d const a0 = l0.segment(3 * idx(0), 3);
    Vector3d const a   =  l.segment( 3*idx(0), 3);

    // returns a scalar, not a 1-vector:
    //    return e2().dot(  Rot_ab(a0,a)*null(a0) );
    return e2().adjoint()* Rot_ab(a0,a)*null(a0);
}

VectorXd Vertical::contradict( const VectorXi &idx,
                               const VectorXd &l0) const
{
    VectorXd tmp(1);
    tmp << l0.segment( 3*idx(0)+1, 1);  // 2nd element
    return tmp;
}

VectorXd Diagonal::contradict( const VectorXi &idx,
                               const VectorXd &l0) const
{
    Vector3d l = l0.segment( 3*idx(0), 3);
    VectorXd tmp(1);  // scalar as vector
    tmp << abs(l(0)) - abs( l(1));   // abs(a)-abs(b)
    return tmp;
}


std::shared_ptr<ConstraintBase> Diagonal::doClone() const
{
    auto T = std::make_shared<Diagonal>();
    T->setStatus( this->status() );
    T->setEnforced( this->enforced());
    return T;
}


MatrixXd Diagonal::Jacobian( const VectorXi &idx,
                             const VectorXd &l0,
                             const VectorXd &l) const
{
    // abs(a)-abs(b) = 0,  l=[a,b,c]',  J = [ sign(a), -sign(b), 0]
    Vector3d a0 = l0.segment( 3*idx(0), 3);
    Vector3d const a = l.segment(3 * idx(0), 3);
    Eigen::RowVector3d const JJ = (Eigen::RowVector3d() << sign(a0(0)), -sign(a0(1)), 0).finished();
    return JJ * Rot_ab(a0,a) * null(a0);
}


std::shared_ptr<ConstraintBase> Horizontal::doClone() const
{
    auto T = std::make_shared<Horizontal>();
    T->setStatus( this->status() );
    T->setEnforced( this->enforced());
    return T;
}

VectorXd Horizontal::contradict( const VectorXi &idx,
                                 const VectorXd &l0) const
{
    VectorXd tmp(1);  // scalar as vector
    tmp << l0.segment( 3*idx(0), 1); // 1st element of 3-vector
    return tmp;
}


//! e_1 = [1,0,0]'
Vector3d Horizontal::e1()
{
    const static Vector3d tmp = (Vector3d() << 1,0,0).finished();
    return tmp;
}

MatrixXd Horizontal::Jacobian( const VectorXi &idx,
                               const VectorXd &l0,
                               const VectorXd &l) const
{
    Vector3d const a0 = l0.segment(3 * idx(0), 3);
    Vector3d const a = l.segment(3 * idx(0), 3);

    return e1().adjoint()*Rot_ab(a0,a)*null(a0);
}


} // namespace Constraint
