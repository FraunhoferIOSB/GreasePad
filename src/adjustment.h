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

#ifndef ADJUSTMENT_H
#define ADJUSTMENT_H

#include "constraints.h"
#include "matrix.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include <memory> // C++
#include <utility>

#include <QList>         // Qt

#include "qcontainerfwd.h"



//! Adjustment framework: obervations, Jacobians, optimization...
class AdjustmentFramework
{
public:
    AdjustmentFramework() = delete;
    AdjustmentFramework( const AdjustmentFramework &) = delete;

    //! Value constructor: vector of observations and covariance matrix
    explicit AdjustmentFramework( const std::pair<Eigen::VectorXd, Eigen::SparseMatrix<double> > & p)
        : l_(p.first), Cov_ll_(p.second)
    {
        // initilization: approx. adjusted observations := observations
        l0_ = l_;
    }

    AdjustmentFramework ( AdjustmentFramework &&) = delete;
    ~AdjustmentFramework() = default;
    void operator= ( const AdjustmentFramework &)  = delete;
    AdjustmentFramework & operator= ( AdjustmentFramework &&) = delete;

    //! Enforce the constraints of a subtask (adjustment)
    bool enforce_constraints( const QVector<std::shared_ptr<Constraint::ConstraintBase> > *constr,
                              const Graph::IncidenceMatrix *Bi,
                              const Eigen::RowVectorXi &maps,
                              const Eigen::RowVectorXi &mapc);

    //! Get s-th entity, i.e., segment, represented by vector of length len
    [[nodiscard]] std::pair<Eigen::VectorXd, Eigen::MatrixXd> getEntity( Eigen::Index s, int len) const;

private:
    [[nodiscard]] Eigen::VectorXd l0_segment(   Eigen::Index offset, int len) const { return l0_.segment(offset,len);}
    [[nodiscard]] Eigen::VectorXd l_segment(    Eigen::Index offset, int len) const { return l_.segment(offset,len);}
    [[nodiscard]] Eigen::MatrixXd Cov_ll_block( Eigen::Index offset, int n) const   { return Cov_ll_.block( offset,offset,n,n); }

    [[nodiscard]] Eigen::VectorXd l0() const { return l0_ ;}
    [[nodiscard]] Eigen::VectorXd l() const  { return l_; }

    [[nodiscard]]    int nIterMax() const { return nIterMax_; }
    [[nodiscard]] double threshold_convergence() const { return convergence; }
    [[nodiscard]] double threshold_ReciprocalConditionNumber() const { return ReciprocalConditionNumber; }
    [[nodiscard]] double threshold_rankEstimate()   const { return rankEstimate;   }
    [[nodiscard]] double threshold_numericalCheck() const { return numericalCheck; }

    void reset() { l0_ = l_;}
    void update( Eigen::Index start, const Eigen::VectorXd &x );

    void Jacobian( const QVector<std::shared_ptr<Constraint::ConstraintBase> > *constr,
                   const Graph::IncidenceMatrix *Bi,
                   Eigen::SparseMatrix<double, Eigen::ColMajor> & BBr,
                   Eigen::VectorXd & g0,
                   const Eigen::RowVectorXi & maps,
                   const Eigen::RowVectorXi & mapc) const;

    void check_constraints( const QVector<std::shared_ptr<Constraint::ConstraintBase> > *constr,
                            const Graph::IncidenceMatrix *bi,
                            const Eigen::RowVectorXi & maps,
                            const Eigen::RowVectorXi & mapc ) const;

    bool verbose = true;

    Eigen::VectorXd l0_;             // vector of approximate/estimated observations
    const Eigen::VectorXd l_;        // vector of observations.
    const Eigen::MatrixXd Cov_ll_;   // covariance matrix of vector of observations

    const int    nIterMax_ = 10;          // maximal number of iterations
    const double convergence     = 1e-7;  // Threshold convergence
    const double ReciprocalConditionNumber = 1e-4;  // condition number
    const double rankEstimate    = 1e-6;  // rank estimation
    const double numericalCheck  = 1e-5;  // numerical check constraints


    //! Rotation matrix for minimal rotation
    static Eigen::MatrixXd Rot_ab(const Eigen::VectorXd &a, const Eigen::VectorXd &b);
    //! Nullspace of row vector
    static Eigen::MatrixXd null(const Eigen::VectorXd &xs);

    static Eigen::Index indexOf(const Eigen::VectorXi &v, int x);
};

#endif // ADJUSTMENT_H
