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

#include "qcontainerfwd.h"
#include "qlogging.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <numeric>
#include <sstream>
#include <utility>

#include <Eigen/Core>
#include <Eigen/SparseCore>


using Graph::IncidenceMatrix;
using Constraint::ConstraintBase;

using Eigen::ColMajor;
using Eigen::Index;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;

using Geometry::Rot_ab;

using Matfun::null;
using Matfun::is_rank_deficient;
using Matfun::indexOf;
using Matfun::spfind;

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
        // m.segment(idx3,3) += null( m.segment(idx3,3) )*x.segment(2*start,2);
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

bool AdjustmentFramework::enforce_constraints( const QVector<std::shared_ptr<Constraint::ConstraintBase> > & constr,
                                               const IncidenceMatrix & Bi,
                                               const VectorXidx & maps,
                                               const VectorXidx & mapc )
{
    const Index numOfConstraints = mapc.size();
    const Index numOfSegments    = maps.size();

    if ( numOfConstraints < 1 ) {
        return true;
    }

    QString suffix;

    // number of required constraints, inclusive hypothesis to be tested
    const Index numOfRequiredConstraints = std::count_if( mapc.begin(), mapc.end(),
        [&constr](Index i){ return constr.at(i)->required();} );

    if ( verbose ) {
        suffix = numOfConstraints==1 ? "" : "s";
        qDebug().noquote() << QStringLiteral("%1 constraint%2 recognized...")
                              .arg(numOfConstraints). arg(suffix);
        suffix = numOfRequiredConstraints==1 ? "" : "s";
        qDebug().noquote() << QStringLiteral("%1 constraint%2 required?...")
                              .arg(numOfRequiredConstraints).arg(suffix);
    }

    // number of required equations (not constraints!),
    // e.g., two equations for parallelism
    const int numOfRequiredEquations = std::accumulate(
        mapc.begin(),  mapc.end(),    0,
        [&constr](int acc, int i){
            return acc += constr.at(i)->required() ? constr.at(i)->dof() : 0;} );


    l0_ = l_; // set adjusted observations  l0 := l

    double reciprocalConditionNumber = 0.;
    double normUpdateReducedObserv   = 0.;

    // allocation ......................................................
    SparseMatrix<double,ColMajor> BBr( numOfRequiredEquations, 2*numOfSegments    );
    SparseMatrix<double,ColMajor> rCov_ll( 2*numOfSegments   , 2*numOfSegments    );
    rCov_ll.reserve(4*numOfSegments   );                  // S 2x2 blocks
    VectorXd lr = VectorXd::Zero( 2*numOfSegments    );   // reduced coordinates observ.
    VectorXd g0 = VectorXd::Zero( numOfRequiredEquations );     // contradictions

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
        const Eigen::FullPivLU<MatrixXd> lu_decomp(BBr * rCov_ll * BBr.adjoint());
        reciprocalConditionNumber = lu_decomp.rcond();
        if ( reciprocalConditionNumber < threshold_ReciprocalConditionNumber() ) {
            // redundant or contradictory constraint
            if ( verbose ) {
                qDebug().noquote() << red
                                   << QStringLiteral("-> ill-conditioned. Reciprocal condition number = %1").arg(reciprocalConditionNumber)
                                   << black;
            }
            return false;
        }

        // contradictions
        const VectorXd cg = -g0  -BBr*lr;

        // estimated update of reduced coordinates observations
        const VectorXd redl = rCov_ll*BBr.adjoint()*lu_decomp.solve(cg) +lr;

        // updates adjusted observations, via retraction
        update( redl );

        normUpdateReducedObserv = redl.norm();
        if ( normUpdateReducedObserv < threshold_convergence() ) {
            break;
        }

    } // loop iterations

    // check convergence ........................................
    if ( normUpdateReducedObserv >= threshold_convergence() )  {
        // redundant or contradictory
        if ( verbose ) {
            suffix = nIterMax()==1 ? "" : "s";
            qDebug().noquote() << red << QStringLiteral("-> Not converged after %1 iteration%2. Reciprocal condition number = %3")
                                  .arg(nIterMax()).arg(suffix)
                                  .arg(reciprocalConditionNumber) << black;
        }
        return false;
    }
    if ( verbose ) {
        suffix = it+1==1 ? "" : "s";
        qDebug().noquote() << green << QStringLiteral("-> Converged after %1 iteration%2. Reciprocal condition number = %3")
                              .arg(it).arg(suffix).arg( reciprocalConditionNumber ) << black;
    }

    // check constraints .........................................
    check_constraints( constr, Bi, maps, mapc);

    return true;
}



void AdjustmentFramework::reduce (
    Eigen::VectorXd & lr,
    SparseMatrix<double,ColMajor> & rCov_ll) const
{
    // reduced coordinates: vector and covariance matrix .............
    const Index S = l0_.size()/3;
    for ( Index s=0; s<S; s++ )
    {
        const Index offset3 = 3 * s;
        const Index offset2 = 2 * s;

        const Eigen::Matrix<double, 3, 2> NN = null( l0_.segment(offset3, 3) );

        // (i) reduced coordinates of observations
        lr.segment(offset2,2) = NN.adjoint() * l_.segment(offset3,3);

        // (ii) covariance matrix, reduced coordinates
        const Eigen::Matrix3d RR = Rot_ab(l_.segment(offset3, 3), l0_.segment(offset3, 3) );
        const Eigen::Matrix<double, 2, 3> JJ = NN.adjoint() * RR;
        const Eigen::Matrix2d Cov_rr = JJ*Cov_ll_.block( offset3,offset3,3,3)*JJ.adjoint();

        // rCov_ll.block(offset2, offset2, 2, 2) = Cov_rr; // not for sparse matrices
        for ( int i=0; i<2; i++ ) {
            for ( int j=0; j<2; j++ ) {
                rCov_ll.coeffRef( offset2 +i,offset2 +j) = Cov_rr(i,j);
            }
        }
    }
}


void AdjustmentFramework::Jacobian(
        const QVector<std::shared_ptr<Constraint::ConstraintBase> > & constr,
        const IncidenceMatrix &  Bi,
        SparseMatrix<double,ColMajor> & BBr,
        VectorXd & g0,
        const VectorXidx & maps,
        const VectorXidx & mapc ) const
{
    int R = 0; // counter for number of equations

    for ( const auto & c : mapc)
    {
        //qDebug().noquote() << QStringLiteral("constraint #%1 (status = %2)").arg( c+1).arg(
           //                       constr_.at(mapc_(c))->status());

        // !! not required ==> obsolete or(!) unevaluated
        const auto & con = constr.at( c );
        if ( con->status() != ConstraintBase::REQUIRED )  { // observe the "!="
            continue;
        }

        // (first) location of idx(i) in vector 'maps',
        //     Matlab: [~,idx] = ismember(idx,maps)
        auto idx = spfind<int>( Bi.col(c) );
        for ( Index i=0; i<idx.size(); i++ ) {
            idx(i) = indexOf<Index>( maps, idx(i) );
        }

        auto JJ = con->Jacobian( idx, l0_, l_ );
        int const dof = con->dof();
        assert( JJ.rows()==dof );
        for ( Index i=0; i< con->arity(); i++ ) {
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
    const QVector<std::shared_ptr<ConstraintBase> > & constr,
    const IncidenceMatrix & bi,
    const VectorXidx & maps,
    const VectorXidx & mapc) const
{
    // double d = NAN;       // distance to be checked, d = 0?
    const Index numOfConstraints = mapc.size();


    // check intrinsic constraints ..............................
#ifdef QT_DEBUG
    const Index numOfSegments = maps.size();
    for (Index s=0; s<numOfSegments; s++) {
        if (verbose) {
            const Eigen::IOFormat fmt(4,0, ", ", "", "[", "]");
            std::stringstream ss;
            ss << l0_.segment(3*s,3).transpose().format(fmt);
            qDebug().noquote()
                << QStringLiteral("straight line #%1: ").arg(s+1, 2)
                << QString::fromStdString(ss.str())
                << ",\tnorm(l) = " << l0_.segment(3*s,3).norm();
        }
        const double d = 1.0 -l0_.segment(3*s,3).norm();
        if ( std::fabs( d ) > threshold_numericalCheck() ) {
            // QApplication::beep();
            qDebug().noquote() <<  red << QStringLiteral("intrinsic constraint %1  check = %2").arg(s).arg(d) << black;
        }
    }
#endif

    // check all(!) geometric constraints
    for ( int c=0; c<numOfConstraints; c++ )
    {
        const auto & con = constr.at( mapc(c) );
        if ( con->unevaluated() ) {
            continue;
        }

        VectorXidx idx = spfind<int>( bi.col(mapc(c)) );
        for ( Index i=0; i<idx.size(); i++ ) {
            idx(i) = indexOf<Index>( maps, idx(i) );
        }

        const double d =  con->contradict( idx, l0_ ).norm();
        con->setEnforced(  fabs(d) < threshold_numericalCheck()  );

#ifdef QT_DEBUG
        if ( verbose ) {
            const QString msg1 = QStringLiteral("%1:  ").arg(QString::fromLatin1(con->type_name()), 12);
            const QString msg2 = QStringLiteral("check = %2, \t").arg(d);
            QDebug deb = qDebug().noquote();
            deb << QStringLiteral("constraint #%1:  ").arg(c+1,3);
            deb << (con->required() ? green : blue) << msg1 << black;
            deb << (con->enforced() ? black : red)  << msg2 << black;

            const VectorXidx idxx = spfind<int>( bi.col(mapc(c)) );
            for ( auto i : idxx ) {
                deb << indexOf<Index>(maps, i) +1;
            }
        }
#endif
    }
}
