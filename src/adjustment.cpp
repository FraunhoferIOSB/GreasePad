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

#include "adjustment.h"
#include "constraints.h"
#include "global.h"
#include "matrix.h"

using Graph::IncidenceMatrix;
using Constraint::ConstraintBase;

static const double T_ZERO = 1e-5;


int AdjustmentFramework::indexOf( const Eigen::VectorXi & v,
                                  const int x) const
{
    //    for ( Index j=0; j<v.size(); j++) {
    //        if ( v(j)==x ) {
    //            return int(j);
    //        }
    //    }

    // Eigen 3.4.0
    auto it = std::find( v.begin(), v.end(), x);
    if ( it==v.end() ) {
        return -1;
    }
    return std::distance( v.begin(), it);
}


MatrixXd AdjustmentFramework::Rot_ab( const VectorXd &a,
                                      const VectorXd &b) const
{
    Q_ASSERT( a.size()==b.size());
    Q_ASSERT( fabs( a.norm()-1.) < T_ZERO );
    Q_ASSERT( fabs( b.norm()-1.) < T_ZERO );

    return MatrixXd::Identity( a.size(),a.size())
            +2*b*a.adjoint()
            -(a+b)*(a+b).adjoint()/(1.+a.dot(b));
}

MatrixXd AdjustmentFramework::null( const VectorXd &xs ) const
{
    // cf. PCV, eq. (A.120)

    //if ( fabs(xs.norm()-1.) > T_ZERO )
    //    qDebug() << xs;

#ifdef Q_DEBUG
    QString what = QStringLiteral("norm(x) = %1").arg( QString::number(xs.norm()) );
    Q_ASSERT_X( std::fabs(xs.norm()-1.) <= T_ZERO,
                Q_FUNC_INFO,
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
    Q_ASSERT_X( check.norm() <= T_ZERO, "nullspace", "not a zero vector");

    return JJ;
}

std::pair<VectorXd,MatrixXd >
AdjustmentFramework::getEntity( const Index s,
                                const int len) const
{
    // qDebug() << Q_FUNC_INFO;

    Index offset = len*s;
    // vector must be the null space of the covariance matrix
    MatrixXd RR = Rot_ab( l_.segment(offset,len),  l0_.segment(offset,len));

    return {  l0_.segment(offset,len),
              RR*Cov_ll_.block(offset,offset,len,len)*RR.adjoint() };
}


void AdjustmentFramework::update( const Index start,
                                  const VectorXd &x)
{
    const Index idx3 = 3*start;

    // (1) via normalization ...................................................
    // l0 := N( l0 + null(l0') * ^Delta l_r )
    // m.segment(idx3,3) += util::null( m.segment(idx3,3) )*x.segment(2*start,2);
    // m.segment(idx3,3).normalize();

    // (2) via retraction ......................................................
    Eigen::Vector3d v = null( l0_.segment(idx3,3) )*x.segment(2*start,2);
    double nv = v.norm();
    if ( nv<=0.0 ) {
        return;
    }

    Eigen::Vector3d p = l0_.segment( idx3,3);
    assert( nv>0.0 );
    l0_.segment( idx3,3) = cos( nv)*p +sin(nv)*v/nv;  // nv>0
    assert( l0_.segment( idx3,3).hasNaN()==false );
}

bool AdjustmentFramework::enforce_constraints( const QList<std::shared_ptr<Constraint::ConstraintBase> > *constr,
                                               const IncidenceMatrix * bi,
                                               // const int E,
                                               const Eigen::RowVectorXi & maps,
                                               const Eigen::RowVectorXi & mapc )
{
    if ( mapc.size() < 1 ) {
        return true;
    }

    const Index C = mapc.size();
    const Index S = maps.size();

    if ( verbose ) {
        qDebug().noquote() << QString("Trying to enforce %1 constraint%2...")
                              .arg( C). arg( C==1 ? "" : "s" );
    }

    reset(); // set adjusted observations  l0 := l

    // number of equations (not constraints),
    // e.g., two equations fpr parallelism
    int E = 0;
    for ( Index c=0; c<mapc.size(); c++ ) {
        if ( constr->at( mapc(c) )->required() ) {
            E += constr->at( mapc(c) )->dof();
        }
    }
    // assert( E==E2 ); // TODO

    VectorXd redl;
    VectorXd cg;
    double rcn = 0.;

    // allocation ......................................................
    SparseMatrix<double,Eigen::ColMajor> BBr( E, 2*S );
    SparseMatrix<double,Eigen::ColMajor> rCov_ll( 2*S, 2*S );
    rCov_ll.reserve(4*S);                  // S 2x2 blocks
    VectorXd lr = VectorXd::Zero( 2*S );   // reduced coordinates observ.
    VectorXd g0 = VectorXd::Zero( E );     // contradictions

    // iterative adjustment ............................................
    int it = 0;  // verbose output
    for ( it=0; it < nIterMax(); it++ )
    {
        if ( verbose ) {
            qDebug().noquote() << QString("  iteration #%1...").arg(it+1);
        }
        a_Jacobian( constr, bi, BBr, g0, maps, mapc);

        // reduced coordinates: vector and covariance matrix .............
        for ( Index s=0; s<S; s++ )
        {
            Index offset3 = 3*s;
            Index offset2 = 2*s;

            Eigen::Matrix<double,3,2> NN = null( l0_segment(offset3,3) );

            // (i) reduced coordinates of observations
            lr.segment(offset2,2) = NN.adjoint() * l_segment(offset3,3);

            // (ii) covariance matrix, reduced coordinates
            Eigen::Matrix3d RR = Rot_ab( l_segment(offset3,3),
                                  l0_segment(offset3,3) );
            Eigen::Matrix<double,2,3> JJ = NN.adjoint()*RR;
            Eigen::Matrix2d Cov_rr = JJ*Cov_ll_block( offset3,3)*JJ.adjoint();

            // rCov_ll.block(offset2, offset2, 2, 2) = Cov_rr; // not for sparse matrices
            for ( int i=0; i<2; i++ ) {
                for ( int j=0; j<2; j++ ) {
                    rCov_ll.coeffRef( offset2 +i,offset2 +j) = Cov_rr(i,j);
                }
            }
        }

        // check rank and condition .....................................

        // rank-revealing decomposition
        Eigen::FullPivLU<MatrixXd> lu_decomp2( BBr*rCov_ll*BBr.adjoint() );
        rcn = lu_decomp2.rcond();
        if ( rcn < threshold_ReciprocalConditionNumber() ) {
            // redundant or contradictory constraint
            if ( verbose ) {
                qDebug().noquote() << QString("-> ill-conditioned. Reciprocal condition number = %1").arg(rcn);
            }
            return false;
        }

        Eigen::FullPivLU<MatrixXd> lu_decomp1( BBr );
        lu_decomp1.setThreshold( threshold_rankEstimate() );
        if ( lu_decomp1.rank() < BBr.rows() ) {
            // redundant or contradictory constraint
            if ( verbose ) {
                qDebug().noquote() << QString("-> Rank deficiency. Reciprocal condition number = %1").arg(rcn);
            }
            return false;
        }


        // contradictions
        cg = -g0  -BBr*lr;

        // estimated update of reduced coordinates observations
        redl = rCov_ll*BBr.adjoint()*lu_decomp2.inverse()*cg +lr;

        // updates adjusted observations, via retraction
        for ( Index s=0; s<maps.size(); s++ ) {
            update( s, redl );
        }

        if ( redl.norm() < threshold_convergence() ) {
            break;
        }

    } // loop iterations

    // check convergence ........................................
    if ( redl.norm() >= threshold_convergence() )
    {
        // redundant or contradictory
        if ( verbose ) {
            qDebug().noquote() << QString("-> Not converged after %1 iteration%2. Reciprocal condition number = %3")
                                  .arg( nIterMax() )
                                  .arg( nIterMax()==1 ? "" : "s")
                                  .arg( rcn );
        }
        return false;
    }
    if ( verbose ) {
        qDebug().noquote() << QString("-> Converged after %1 iteration%2. Reciprocal condition number = %3")
                              .arg( it)
                              .arg( it+1==1 ? "" : "s")
                              .arg( rcn );
    }

    // check constraints .........................................
    a_check_constraints( constr, bi, maps, mapc);

    return true;
}


void AdjustmentFramework::a_Jacobian(
        const QList<std::shared_ptr<ConstraintBase> > * constr,
        const IncidenceMatrix * Bi,
        SparseMatrix<double, Eigen::ColMajor> & BBr,
        VectorXd & g0,
        const Eigen::RowVectorXi & maps,
        const Eigen::RowVectorXi & mapc ) const
{
    int R = 0; // counter for number of equations

    for ( Index c=0; c<mapc.size(); c++ )
    {
        //qDebug().noquote() << QStringLiteral("constraint #%1 (status = %2)").arg( c+1).arg(
           //                       constr_.at(mapc_(c))->status());

        // !! not required ==> obsolete or(!) unevaluated
        const auto & con = constr->at( mapc(c) );
        if ( con->status() != ConstraintBase::REQUIRED )  { // observe the "!="
            continue;
        }

        // (first) location of idx(i) in vector 'maps',
        //     Matlab: [~,idx] = ismember(idx,maps)
        auto idx = Bi->findInColumn( mapc(c) );
        for ( Index i=0; i<idx.size(); i++ ) {
            idx(i) = indexOf( maps, idx(i) );
        }

        auto JJ = con->Jacobian( idx, l0(), l() );
        int dof   = con->dof();
        for ( int i=0; i< con->arity(); i++ )
        {
            // dof rows for this constraint
            Q_ASSERT( JJ.rows()==dof );
            for ( Index r=0; r<JJ.rows(); r++ ) {
                BBr.coeffRef( R+r, 2*idx(i)   ) = JJ(r,2*i  );  // Two columns for each entity
                BBr.coeffRef( R+r, 2*idx(i)+1 ) = JJ(r,2*i+1);
            }
        }
        g0.segment(R,dof) = con->contradict( idx, l0() );
        R += dof;
    }

}

void AdjustmentFramework::a_check_constraints(
        const QList<std::shared_ptr<Constraint::ConstraintBase> > * constr,
        const IncidenceMatrix * bi,
        const Eigen::RowVectorXi & maps,
        const Eigen::RowVectorXi & mapc) const
{
    double d;       // distance to be checked, d = 0?
    const Index C = mapc.size();

    // check intrinsic constraints ..............................
#ifdef QT_DEBUG
    const Index S = maps.size();
    for (int s=0; s<S; s++) {
        d = 1.0 -l0_segment(3*s,3).norm();
        if ( std::fabs( d ) > threshold_numericalCheck() ) {
            // QApplication::beep();
            qDebug().noquote() <<  QString("intrinsic constraint %1  check = %2").arg(s).arg(d);
        }
    }
#endif

    // check all(!) geometric constraints..........................
    constexpr int width = 25;
    for ( int c=0; c<C; c++ )
    {
        auto & con = constr->at( mapc(c) );
        if ( con->unevaluated() ) {
            continue;
        }

        auto idx = bi->findInColumn( mapc(c) );
        for ( Index i=0; i<idx.size(); i++ ) {
            idx(i) = indexOf( maps, idx(i) );
        }

        d =  con->contradict( idx, l0() ).norm();
        con->setEnforced(  fabs(d) < threshold_numericalCheck()  );

        if ( verbose ) {
            QString msg = QString("%1:  check = %2").arg(
                        QString::fromLatin1( con->type_name()), width).arg(d);

            qDebug().noquote() << (con->enforced() ? green : red)
                               << msg << black;
        }

    }
}

