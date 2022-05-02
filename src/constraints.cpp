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

#include "constraints.h"
#include "matrix.h"

#include <QDebug>

namespace Constraint {

using Eigen::Vector3d;
using Eigen::Vector2cd;
using Eigen::Matrix;

static const double T_ZERO = 1e-5;





MatrixXd ConstraintBase::Rot_ab( const VectorXd &a,
                                 const VectorXd &b)
{
    Q_ASSERT( a.size()==b.size());
    Q_ASSERT( std::fabs( a.norm()-1.) < T_ZERO );
    Q_ASSERT( std::fabs( b.norm()-1.) < T_ZERO );

    return MatrixXd::Identity( a.size(),a.size())
            +2*b*a.adjoint()
            -(a+b)*(a+b).adjoint()/(1.+a.dot(b));
}

MatrixXd ConstraintBase::null( const VectorXd & xs )
{
    // cf. PCV, eq. (A.120)

    //if ( fabs(xs.norm()-1.) > T_ZERO )
    //    qDebug() << xs;

#ifdef Q_DEBUG
    QString what = QStringLiteral("norm(x) = %1").arg( QString::number(xs.norm()) );
    Q_ASSERT_X( fabs(xs.norm()-1.) <= T_ZERO,
                "null(x)",
                what.toStdString().data() ) ;
#endif

    Eigen::Index  N = xs.size();

    VectorXd x0 = xs.head(N-1);
    double   xN = xs(N-1);
    if ( xN < 0.0 ) {
        x0 = -x0;
        xN = -xN;
    }

    MatrixXd JJ( N, N-1);
    JJ.topRows(N-1)  = MatrixXd::Identity(N-1,N-1) -x0*x0.adjoint()/(1.+xN);
    JJ.bottomRows(1) = -x0.adjoint();

    VectorXd check = JJ.adjoint()*xs;
    Q_ASSERT_X( check.norm() <= T_ZERO, Q_FUNC_INFO, "not a zero vector");

    return JJ;
}

// Diag([1,1,0])
Matrix3d Orthogonal::CC()
{
    const static Matrix3d tmp =  (Matrix3d() << 1,0,0,  0,1,0, 0,0,0).finished();
    return tmp;
}

// S(e3)
Matrix3d Parallel::S3()
{
    const static Matrix3d tmp = (Matrix3d() << 0,-1,0, +1,0,0, 0,0,0).finished();
    return tmp;
}


ConstraintBase::ConstraintBase() {
    m_status   = UNEVAL;
    m_enforced = false;
}

MatrixXd Orthogonal::Jacobian( const VectorXi & idxx,
                               const VectorXd &l0,
                               const VectorXd &l) const
{
    Vector3d a0 =  l0.segment( 3*idxx(0), 3);
    Vector3d b0 =  l0.segment( 3*idxx(1), 3);

    Vector3d a =  l.segment( 3*idxx(0), 3);
    Vector3d b =  l.segment( 3*idxx(1), 3);

    Matrix<double,1,2> JJa;
    Matrix<double,1,2> JJb;
    MatrixXd JJl;

    JJl = null(a0).adjoint() * Rot_ab(a,a0);
    JJa = b.adjoint()*CC()* JJl.adjoint();

    JJl = null(b0).adjoint() * Rot_ab(b,b0);
    JJb = a0.adjoint()*CC()*JJl.adjoint();

    MatrixXd Tmp(1,4);
    Tmp << JJa, JJb;
    return Tmp;
}

VectorXd Orthogonal::contradict( const VectorXi &idxx,
                                 const VectorXd &l0) const
{
    Vector3d a =  l0.segment (3*idxx(0), 3);
    Vector3d b =  l0.segment (3*idxx(1), 3);

    VectorXd tmp(1);
    tmp << a.dot( CC()*b );

    return tmp;
}


MatrixXd Copunctual::Jacobian( const VectorXi & idxx,
                               const VectorXd & l0,
                               const VectorXd & l) const
{
    Vector3d a0 = l0.segment(3*idxx(0),3);
    Vector3d b0 = l0.segment(3*idxx(1),3);
    Vector3d c0 = l0.segment(3*idxx(2),3);

    Vector3d a = l.segment(3*idxx(0),3);
    Vector3d b = l.segment(3*idxx(1),3);
    Vector3d c = l.segment(3*idxx(2),3);

    Matrix3d MM = (Matrix3d() << a0,b0,c0).finished();
    Matrix3d Adju = cof3(MM).adjoint(); // adjugate(MM)

    Matrix<double,2,3> JJa = null(a0).adjoint() * Rot_ab(a,a0);
    Matrix<double,2,3> JJb = null(b0).adjoint() * Rot_ab(b,b0);
    Matrix<double,2,3> JJc = null(c0).adjoint() * Rot_ab(c,c0);

    constexpr int six = 6;
    MatrixXd Tmp(1,six);
    Tmp << Adju.row(0)*JJa.adjoint(),
            Adju.row(1)*JJb.adjoint(),
            Adju.row(2)*JJc.adjoint();

    return Tmp;
}



VectorXd Copunctual::contradict( const VectorXi & idxx,
                                 const VectorXd & l0) const
{
    Vector3d a = l0.segment( 3*idxx(0), 3 );
    Vector3d b = l0.segment( 3*idxx(1), 3 );
    Vector3d c = l0.segment( 3*idxx(2), 3 );

    Matrix3d MM = (Matrix3d() << a,b,c).finished();

    VectorXd tmp(1);
    tmp << MM.determinant();
    return tmp;
}



MatrixXd Identical::Jacobian( const VectorXi & idxx,
                              const VectorXd & l0,
                              const VectorXd & l) const
{
    Vector3d a0 = l0.segment( 3*idxx(0),3 );
    Vector3d b0 = l0.segment( 3*idxx(1),3 );

    Vector3d a = l.segment( 3*idxx(0),3 );
    Vector3d b = l.segment( 3*idxx(1),3 );

    Matrix<double,2,3> JJa = null(a0).adjoint() * Rot_ab(a,a0);
    Matrix<double,2,3> JJb = null(b0).adjoint() * Rot_ab(b,b0);

    Eigen::FullPivLU<MatrixXd> LU;  // identical
    Matrix<double,3,2> JJ;   // identical

    int idx1;
    int idx2;
    a0.cwiseAbs().maxCoeff( &idx1 );
    b0.cwiseAbs().maxCoeff( &idx2 );

    Q_ASSERT( sign( a0(idx1) )==sign( b0(idx2) ) );
    Q_ASSERT( sameSign( a0(idx1),  b0(idx2) ) );

    LU.compute(a0.adjoint());
    JJ = LU.kernel();               //  JJ = null( a');
    // d2 = JJ.adjoint()*(a -b);       //  (10.141)

    MatrixXd Tmp(2,4);
    Tmp << JJ.adjoint()*JJa.adjoint(), -JJ.adjoint()*JJb.adjoint();
    return Tmp;
}

VectorXd Identical::contradict( const VectorXi & idxx,
                                const VectorXd & l0) const
{
    Vector3d a = l0.segment( 3*idxx(0),3 );
    Vector3d b = l0.segment( 3*idxx(1),3 );
    Eigen::FullPivLU<MatrixXd> LU;

    // check sign ............................................
    int idx1;
    int idx2;
    a.head(2).cwiseAbs().maxCoeff( &idx1 );
    b.head(2).cwiseAbs().maxCoeff( &idx2 );

    Q_ASSERT( sameSign(a(idx1), b(idx2)) );
    LU.compute( a.adjoint() );
    Matrix<double,3,2> JJ = LU.kernel();  //  JJ = null( a');
    Eigen::Vector2d d2 = JJ.adjoint()*(a -b);    //  (10.141)

    return d2;
}


MatrixXd Parallel::Jacobian( const VectorXi & idxx,
                             const VectorXd & l0,
                             const VectorXd & l) const
{
    Vector3d a0 = l0.segment( 3*idxx(0),3 );
    Vector3d b0 = l0.segment( 3*idxx(1),3 );

    Vector3d a =  l.segment( 3*idxx(0),3 );
    Vector3d b =  l.segment( 3*idxx(1),3 );

    Matrix<double,1,2> JJa;
    Matrix<double,1,2> JJb;

    MatrixXd JJaa = null(a0).adjoint() * Rot_ab(a,a0);
    MatrixXd JJbb = null(b0).adjoint() * Rot_ab(b,b0);

    JJa = -b0.adjoint()*S3()*JJaa.adjoint();
    JJb =  a0.adjoint()*S3()*JJbb.adjoint();

    MatrixXd Tmp(1,4);
    Tmp << JJa, JJb;
    return Tmp;
}


VectorXd Parallel::contradict( const VectorXi & idxx,
                               const VectorXd & l0) const
{
    Vector3d a = l0.segment( 3*idxx(0),3 );
    Vector3d b = l0.segment( 3*idxx(1),3 );

    VectorXd tmp(1,1);
    tmp  <<  a.dot( S3()*b );

    return tmp;
}



Matrix3d Copunctual::cof3( const Matrix3d & MM ) const
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


void ConstraintBase::serialize( QDataStream & out) const
{
    // qDebug() <<  Q_FUNC_INFO << type_name();
    out << type_name();
    out << status();    // { UNEVAL=0 | REQUIRED | OBSOLETE };
    out << enforced();
}

std::shared_ptr<ConstraintBase>
ConstraintBase::deserialize( QDataStream &in )
{
    char* type_name;
    in >> type_name;
    // qDebug() << Q_FUNC_INFO << type_name;
    if ( in.status()!=0) {
        return nullptr;
    }

    std::shared_ptr<ConstraintBase> c;
    if ( std::strcmp( type_name, "orthogonal")==0 ){
        c = Orthogonal::create();
    }
    if ( std::strcmp( type_name, "parallel")==0 ){
        c = Parallel::create();
    }
    if ( std::strcmp( type_name, "copunctual")==0 ){
        c = Copunctual::create();
    }

    int status;
    in >> status;
    bool enforced;
    in >> enforced;
    if ( in.status()!=0) {
        return nullptr;
    }
    c->setStatus( static_cast<Status>(status) );
    c->setEnforced( enforced );

    return c;
}


std::shared_ptr<ConstraintBase> Parallel::clone() const
{
    auto T = std::make_shared<Parallel>();
    T->setStatus(   this->status()  );
    T->setEnforced( this->enforced());
    return std::move(T);
}

std::shared_ptr<ConstraintBase> Identical::clone() const
{
    auto T = std::make_shared<Identical>();
    T->setStatus(   this->status()  );
    T->setEnforced( this->enforced());
    return std::move(T);
}

std::shared_ptr<ConstraintBase> Copunctual::clone() const
{
    auto T = std::make_shared<Copunctual>();
    T->setStatus(   this->status()   );
    T->setEnforced( this->enforced() );
    return std::move(T);
}


std::shared_ptr<ConstraintBase> Orthogonal::clone() const
{
    auto T = std::make_shared<Orthogonal>();
    T->setStatus( this->status() );
    T->setEnforced( this->enforced());
    return std::move(T);
}

} // namespace Constraint
