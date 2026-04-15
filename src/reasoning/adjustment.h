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

#ifndef ADJUSTMENT_H
#define ADJUSTMENT_H


#include <Eigen/Core>
#include <Eigen/SparseCore>

#include <memory>
#include <utility>

#include "qcontainerfwd.h"

#include "geometry/minrot.h"


namespace Constraint {class ConstraintBase;} // namespace Constraint
namespace Graph {class IncidenceMatrix;} // namespace Graph

enum class Attribute : int;

//! Adjustment framework: observations, Jacobians, optimization...
class AdjustmentFramework
{
private:
    using VectorXidx = Eigen::Vector<Eigen::Index,Eigen::Dynamic>;

public:
    AdjustmentFramework() = delete;
    AdjustmentFramework( const AdjustmentFramework &) = delete;

    //! Value constructor: vector of observations and covariance matrix
    explicit AdjustmentFramework(const std::pair<Eigen::VectorXd, Eigen::SparseMatrix<double> > & p);
    AdjustmentFramework ( AdjustmentFramework &&) = delete;
    ~AdjustmentFramework() = default;
    void operator= ( const AdjustmentFramework &)  = delete;
    AdjustmentFramework & operator= ( AdjustmentFramework &&) = delete;

    //! Enforce the constraints of a subtask (adjustment)
    bool enforceConstraints(const QVector<std::shared_ptr<Constraint::ConstraintBase> > &constr,
                             const Graph::IncidenceMatrix &relsub,
                             const Eigen::ArrayXi &mapc,
                            Eigen::Array<Attribute,Eigen::Dynamic,1> & status,
                            Eigen::Array<bool,Eigen::Dynamic,1> & enforced);

    //! Get i-th uncertain entity represented by N-vector and covariance matrix
    template<typename T,int N>
    std::pair< Eigen::Vector<T,N>, Eigen::Matrix<T,N,N> >
    getEntity( Eigen::Index i) const;

private:
    [[nodiscard]] static int nIterMax() { return nIterMax_; }
    [[nodiscard]] static double threshold_convergence() { return convergence; }
    [[nodiscard]] static double threshold_ReciprocalConditionNumber() { return ReciprocalConditionNumber; }
    [[nodiscard]] static double threshold_rankEstimate() { return rankEstimate;   }
    [[nodiscard]] static double threshold_numericalCheck() { return numericalCheck; }

    void update( const Eigen::VectorXd & x );  // update of adjusted observatons in l0_

    void Jacobian(const QVector<std::shared_ptr<Constraint::ConstraintBase> > & constr,
                  const Graph::IncidenceMatrix & relsub,
                  Eigen::SparseMatrix<double, Eigen::ColMajor> & BBr,
                  const Eigen::ArrayXi & mapc,
                  const Eigen::Array<Attribute,Eigen::Dynamic,1> & status,
                  Eigen::VectorXd & g0  );

    //! compute reduced coordinates
    void reduce();

    void checkConstraints( const QVector<std::shared_ptr<Constraint::ConstraintBase> > & constr,
                           const Graph::IncidenceMatrix & relsub,
                           const Eigen::ArrayXi & mapc,
                           const Eigen::Array<Attribute,Eigen::Dynamic,1> & status,
                           Eigen::Array<bool,Eigen::Dynamic,1> & enforced                          ) const;

    bool verbose = true;

    // observations
    Eigen::VectorXd l0_;             // vector of approximate/estimated observations
    const Eigen::VectorXd l_;        // vector of observations.
    const Eigen::SparseMatrix<double> Sigma_ll_;   // covariance matrix of vector of observations

    // reduced coordinates obervations
    Eigen::VectorXd lr;
    Eigen::SparseMatrix<double,Eigen::ColMajor> rSigma_ll;


    static constexpr int    nIterMax_ = 10;          // maximal number of iterations
    static constexpr double convergence     = 1e-7;  // Threshold convergence
    static constexpr double ReciprocalConditionNumber = 1e-4;  // condition number
    static constexpr double rankEstimate    = 1e-6;  // rank estimation
    static constexpr double numericalCheck  = 1e-5;  // numerical check of constraints
};


//! Get i-th uncertain entity represented by N-vector and covariance matrix
template<typename T,int N>
std::pair< Eigen::Vector<T,N>, Eigen::Matrix<T,N,N> >
AdjustmentFramework::getEntity( const Eigen::Index i) const
{
    const Eigen::Index offset = N*i;  // start position
    const Eigen::Matrix<T,N,N> RR = Geometry::Rot_ab<T,N>(
        l_.segment<N>(offset), l0_.segment<N>(offset) );

    return {l0_.segment<N>(offset),
            RR*Sigma_ll_.block(offset,offset,N,N)*RR.transpose()};
}


#endif // ADJUSTMENT_H
