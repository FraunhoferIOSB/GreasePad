/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2023 Jochen Meidow, Fraunhofer IOSB
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

#ifndef ADJUSTMENT_H
#define ADJUSTMENT_H

#include "matrix.h"

#include <Eigen/Dense>

#include <memory> // C++

#include <QList>         // Qt
#include <utility>

namespace Constraint {
class ConstraintBase;
}
using Constraint::ConstraintBase;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;
using Eigen::Index;
using Eigen::RowVectorXi;

//! Adjustment framework: obervations, Jacobians, optimization...
class AdjustmentFramework
{
public:
    AdjustmentFramework() = delete;
    AdjustmentFramework( const AdjustmentFramework &) = delete;

    //! Value constructor: vector of observations and covariance matrix
    AdjustmentFramework( const std::pair<VectorXd, SparseMatrix<double> > & p)
        : l_(p.first), Cov_ll_(p.second)
    {
        l0_ = l_;  // initilization approx./estimated observations
    }

    AdjustmentFramework ( AdjustmentFramework &&) = delete;
    ~AdjustmentFramework() = default;
    void operator= ( const AdjustmentFramework &)  = delete;
    AdjustmentFramework & operator= ( AdjustmentFramework &&) = delete;

    //! Enforce the constraints of a subtask (adjustment)
    bool enforce_constraints( const QVector<std::shared_ptr<ConstraintBase> > *constr,
                              const Graph::IncidenceMatrix *Bi,
                              const RowVectorXi &maps,
                              const RowVectorXi &mapc);

    //! Get s-th entity, i.e., segment, represented by vector of length len
    std::pair<VectorXd, MatrixXd> getEntity( Index s, int len) const;

private:
    VectorXd l0_segment(   Index offset, int len) const { return l0_.segment(offset,len);}
    VectorXd l_segment(    Index offset, int len) const { return l_.segment(offset,len);}
    MatrixXd Cov_ll_block( Index offset, int n) const   { return Cov_ll_.block( offset,offset,n,n); }

    VectorXd l0() const { return l0_ ;}
    VectorXd l() const  { return l_; }

       int nIterMax() const { return nIterMax_; }
    double threshold_convergence() const { return threshold_.convergence; }
    double threshold_ReciprocalConditionNumber() const { return threshold_.ReciprocalConditionNumber; }
    double threshold_rankEstimate()   const { return threshold_.rankEstimate;   }
    double threshold_numericalCheck() const { return threshold_.numericalCheck; }

    void reset() { l0_ = l_;}
    void update( Index start, const VectorXd &x );

    void Jacobian( const QVector<std::shared_ptr<ConstraintBase> > *constr,
                   const Graph::IncidenceMatrix *Bi,
                   SparseMatrix<double, Eigen::ColMajor> & BBr,
                   VectorXd & g0,
                   const RowVectorXi & maps,
                   const RowVectorXi & mapc) const;

    void check_constraints( const QVector<std::shared_ptr<ConstraintBase> > *constr,
                            const Graph::IncidenceMatrix *bi,
                            const RowVectorXi & maps,
                            const RowVectorXi & mapc ) const;

    bool verbose = true;
    VectorXd l0_;             // vector of approximate/estimated observations
    const VectorXd l_;        // vector of observations.
    const MatrixXd Cov_ll_;   // covariance matrix of vector of observations

    const int nIterMax_ = 10;                 // maximal number of iterations
    struct Threshold
    {
        const double convergence     = 1e-7;  // Threshold convergence
        const double ReciprocalConditionNumber = 1e-4;  // condition number
        const double rankEstimate    = 1e-6;  // rank estimation
        const double numericalCheck  = 1e-5;  // numerical check constraints
    } threshold_;

    //! Rotation matrix for minimal rotation
    static MatrixXd Rot_ab(const VectorXd &a, const VectorXd &b);
    //! Nullspace of row vector
    static MatrixXd null(const VectorXd &xs);

    static Index indexOf(const Eigen::VectorXi &v, int x);
};

#endif // ADJUSTMENT_H
