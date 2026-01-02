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

#include "adjustment.h"
#include "constraints.h"
#include "geometry/minrot.h"
#include "global.h"
#include "matfun.h"
#include "matrix.h"

#include <QString>
#include <QStringLiteral>

#include "qassert.h"
#include "qcontainerfwd.h"
#include "qlogging.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <numeric>
#include <utility>

#include <Eigen/Core>
#include <Eigen/SparseCore>


using Graph::IncidenceMatrix;
using Constraint::ConstraintBase;

using Eigen::Index;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::VectorXidx;

using Geometry::Rot_ab;

using Matfun::null;
using Matfun::is_rank_deficient;
using Matfun::indexOf;


using TextColor::black;
using TextColor::blue;
using TextColor::green;
using TextColor::red;


std::pair<VectorXd,MatrixXd >
AdjustmentFramework::getEntity( const Index s) const
{
    const Index offset = 3*s;
    // vector must be the null space of the covariance matrix
    const MatrixXd RR = Rot_ab( l_.segment(offset,3),  l0_.segment(offset,3));

    return {  l0_.segment(offset,3),
              RR*Cov_ll_.block(offset,offset,3,3)*RR.adjoint() };
}


//! update of parameters (observations) via retraction
void AdjustmentFramework::update( const VectorXd &x)
{
    constexpr double kNormEps = 1e-8;

    for (Index s=0; s<x.size()/2; s++) {
        const Index idx3 = 3*s;

        // (1) via normalization ...................................................
        // l0 := N( l0 + null(l0') * ^Delta l_r )
        // m.segment(idx3,3) += util::null( m.segment(idx3,3) )*x.segment(2*start,2);
        // m.segment(idx3,3).normalize();

        // (2) via retraction ......................................................
        const Eigen::Vector3d v = null( l0_.segment(idx3, 3) ) * x.segment(2 * s, 2);
        const double nv = v.norm();
        if ( nv > kNormEps ) {
            const Eigen::Vector3d p = l0_.segment(idx3, 3);
            l0_.segment( idx3,3) = cos( nv)*p +sin(nv)*v/nv;
        }
    }
}

bool AdjustmentFramework::enforce_constraints( const QVector<std::shared_ptr<Constraint::ConstraintBase> > *constr,
                                               const IncidenceMatrix * Bi,
                                               const VectorXidx & maps,
                                               const VectorXidx & mapc )
{
    const Index C = mapc.size();
    const Index S = maps.size();

    if ( C < 1 ) {
        return true;
    }

    // number of required constraints, inclusive hypothesis to be tested
    /* int numReq = 0;
    for ( Index c=0; c<C; c++ ) {
        if ( constr->at( mapc(c) )->required() ) {
            numReq++;
        }
    } */

    /* int numReq= 0;
    for ( auto c : mapc  ) {
         numReq += constr->at( c )->required() ? 1 : 0;
    } */

    const Index numReq = std::count_if(
        mapc.begin(), mapc.end(),
        [&constr](Index i){ return constr->at(i)->required();} );

    if ( verbose ) {
        qDebug().noquote() << QStringLiteral("%1 constraint%2 recognized...")
                              .arg( C). arg( C==1 ? "" : "s" );
        qDebug().noquote() << QStringLiteral("%1 constraint%2 required?...")
                              .arg( numReq). arg( numReq==1 ? "" : "s" );
    }

    // number of required equations (not constraints!),
    // e.g., two equations for parallelism
    /* int E = 0;
    for ( Index c=0; c<mapc.size(); c++ ) {
        if ( constr->at( mapc(c) )->required() ) {
            E += constr->at( mapc(c) )->dof();
        }
    }*/
    const int E = std::accumulate(
        mapc.begin(),  mapc.end(),    0,
        [&constr](int acc, int i){
            return acc += constr->at(i)->required() ? constr->at(i)->dof() : 0;} );


    l0_ = l_; // reset(); // set adjusted observations  l0 := l


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
            qDebug().noquote() << QStringLiteral("  iteration #%1...").arg(it+1);
        }
        Jacobian( constr, Bi, BBr, g0, maps, mapc);
        reduce ( lr, rCov_ll);

        // check rank and condition .....................................

        if ( is_rank_deficient( BBr, threshold_rankEstimate()) ) {
            // redundant or contradictory constraint
            if ( verbose ) {
                qDebug().noquote() << QStringLiteral("-> Rank deficiency");
            }
            return false;
        }


        // rank-revealing decomposition
        Eigen::FullPivLU<MatrixXd> const lu_decomp(BBr * rCov_ll * BBr.adjoint());
        rcn = lu_decomp.rcond();
        if ( rcn < threshold_ReciprocalConditionNumber() ) {
            // redundant or contradictory constraint
            if ( verbose ) {
                qDebug().noquote() << red
                                   << QStringLiteral("-> ill-conditioned. Reciprocal condition number = %1").arg(rcn)
                                   << black;
            }
            return false;
        }



        // contradictions
        cg = -g0  -BBr*lr;

        // estimated update of reduced coordinates observations
        redl = rCov_ll*BBr.adjoint()*lu_decomp.inverse()*cg +lr;

        // updates adjusted observations, via retraction
        update( redl );

        if ( redl.norm() < threshold_convergence() ) {
            break;
        }

    } // loop iterations

    // check convergence ........................................
    if ( redl.norm() >= threshold_convergence() )  {
        // redundant or contradictory
        if ( verbose ) {
            qDebug().noquote() << red << QStringLiteral("-> Not converged after %1 iteration%2. Reciprocal condition number = %3")
                                  .arg( nIterMax() )
                                  .arg( nIterMax()==1 ? "" : "s")
                                  .arg( rcn ) << black;
        }
        return false;
    }
    if ( verbose ) {
        qDebug().noquote() << green << QStringLiteral("-> Converged after %1 iteration%2. Reciprocal condition number = %3")
                              .arg( it)
                              .arg( it+1==1 ? "" : "s")
                              .arg( rcn ) << black;
    }

    // check constraints .........................................
    check_constraints( constr, Bi, maps, mapc);

    return true;
}



void AdjustmentFramework::reduce (
    Eigen::VectorXd & lr,
    SparseMatrix<double, Eigen::ColMajor> & rCov_ll) const
{
    // reduced coordinates: vector and covariance matrix .............
    const Index S = l0_.size()/3;
    for ( Index s=0; s<S; s++ )
    {
        Index const offset3 = 3 * s;
        Index const offset2 = 2 * s;

        Eigen::Matrix<double, 3, 2> const NN = null( l0_.segment(offset3, 3) );

        // (i) reduced coordinates of observations
        lr.segment(offset2,2) = NN.adjoint() * l_.segment(offset3,3);

        // (ii) covariance matrix, reduced coordinates
        Eigen::Matrix3d const RR = Rot_ab(l_.segment(offset3, 3), l0_.segment(offset3, 3) );
        Eigen::Matrix<double, 2, 3> const JJ = NN.adjoint() * RR;
        Eigen::Matrix2d Cov_rr = JJ*Cov_ll_.block( offset3,offset3,3,3)*JJ.adjoint();

        // rCov_ll.block(offset2, offset2, 2, 2) = Cov_rr; // not for sparse matrices
        for ( int i=0; i<2; i++ ) {
            for ( int j=0; j<2; j++ ) {
                rCov_ll.coeffRef( offset2 +i,offset2 +j) = Cov_rr(i,j);
            }
        }
    }
}


void AdjustmentFramework::Jacobian(
        const QVector<std::shared_ptr<Constraint::ConstraintBase> > *constr,
        const IncidenceMatrix * Bi,
        SparseMatrix<double, Eigen::ColMajor> & BBr,
        VectorXd & g0,
        const VectorXidx & maps,
        const VectorXidx & mapc ) const
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
            idx(i) = indexOf<Index>( maps, idx(i) );
        }

        auto JJ = con->Jacobian( idx, l0_, l_ );
        int const dof = con->dof();
        for ( Index i=0; i< con->arity(); i++ )   {
            // dof rows for this constraint
            Q_ASSERT( JJ.rows()==dof );
            for ( Index r=0; r<JJ.rows(); r++ ) {
                BBr.coeffRef( R+r,  2*idx(i)   )  = JJ(r, 2*i   );  // Two columns for each entity
                BBr.coeffRef( R+r, (2*idx(i))+1 ) = JJ(r,(2*i)+1);
            }
        }
        g0.segment(R,dof) = con->contradict( idx, l0_ );
        R += dof;
    }

}

//! check constraints (required and non-required)
void AdjustmentFramework::check_constraints(
        const QVector<std::shared_ptr<Constraint::ConstraintBase> > *constr,
        const IncidenceMatrix * bi,
        const Eigen::VectorXidx & maps,
        const Eigen::VectorXidx & mapc) const
{
    // double d = NAN;       // distance to be checked, d = 0?
    const Index C = mapc.size();

    // check intrinsic constraints ..............................
#ifdef QT_DEBUG
    const Index S = maps.size();
    for (Index s=0; s<S; s++) {
        if (verbose) {
            qDebug().noquote()
                    << QStringLiteral("straight line #%1: [").arg(s+1, 2)
                    << l0_.segment(3*s,3).x() << ","
                    << l0_.segment(3*s,3).y() << ","
                    << l0_.segment(3*s,3).z() << "], \tnorm ="
                    << l0_.segment(3*s,3).norm();
        }
        const double d = 1.0 -l0_.segment(3*s,3).norm();
        if ( std::fabs( d ) > threshold_numericalCheck() ) {
            // QApplication::beep();
            qDebug().noquote() <<  red << QStringLiteral("intrinsic constraint %1  check = %2").arg(s).arg(d) << black;
        }
    }
#endif

    // check all(!) geometric constraints..........................
    //constexpr int width = 25;
    for ( int c=0; c<C; c++ )
    {
        const auto & con = constr->at( mapc(c) );
        if ( con->unevaluated() ) {
            continue;
        }

        auto idx = bi->findInColumn( mapc(c) );
        for ( Index i=0; i<idx.size(); i++ ) {
            idx(i) = indexOf<Index>( maps, idx(i) );
        }

        const double d =  con->contradict( idx, l0_ ).norm();
        con->setEnforced(  fabs(d) < threshold_numericalCheck()  );

#ifdef QT_DEBUG
        if ( verbose ) {
            QString const msg1 = QStringLiteral("%1:  ").arg(QString::fromLatin1(con->type_name()), 12);
            QString const msg2 = QStringLiteral("check = %2, \t").arg(d);
            QDebug deb = qDebug().noquote();
            deb << QStringLiteral("constraint #%1:  ").arg(c+1,3);
            deb << (con->required() ? green : blue) << msg1 << black;
            deb << (con->enforced() ? black : red)  << msg2 << black;

            auto idxx = bi->findInColumn( mapc(c) );
            for ( Index i=0; i<idxx.size(); i++ ) {
                const Index idxxx = indexOf<Index>(maps, idxx(i));
                deb << idxxx+1;
            }
        }
#endif
    }
}

