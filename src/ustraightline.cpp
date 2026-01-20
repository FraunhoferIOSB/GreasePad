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


#include <cassert>
#include <cfloat>
#include <cmath>

#include "matfun.h"
#include "statistics/iscov.h"
#include "uncertain.h"
#include "upoint.h"
#include "ustraightline.h"

#include <Eigen/Core>

#include <QDebug>

#include "qassert.h"
#include "qtdeprecationdefinitions.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using Matrix6d    = Eigen::Matrix<double,6,6>;
using RowVector6d = Eigen::Matrix<double,1,6>;
using Matrix9d    = Eigen::Matrix<double,9,9>;
using RowVector9d = Eigen::Matrix<double,1,9>;

using Matfun::sign;
using Matfun::cof3;

using Stats::isCovMat;


namespace Uncertain {


//! Uncertain straight line, defined by a 3-vector and its covariance matrix
uStraightLine::uStraightLine(const Vector3d & l,
                             const Matrix3d & Sigma_ll)
    : BasicEntity2D( l, Sigma_ll )
{
    /* check: null(Sigma_ll)
    Eigen::FullPivLU<MatrixXd> lu(Sigma_ll);
    MatrixXd null_space = lu.kernel();

    std::cout << "null = " << null_space.format(row) << '\n' << std::flush;
    std::cout << "   l = " << l.format(row) << '\n' << std::flush;
    std::cout << l(0)*l(0) +l(1)*l(1) << '\n' << std::flush;*/
}


//! Uncertain straight line approximating the point set {x_i,y_i}
uStraightLine uStraightLine::estim( const VectorXd & xi,
                                    const VectorXd & yi)
{
    // centroid
    const double x0 = xi.mean();
    const double y0 = yi.mean();

    // 2nd moments
    const auto numPtsF = static_cast<double>(xi.rows());

    Matrix2d MM;
    MM(0,0) = xi.dot(xi)  -numPtsF*x0*x0;
    MM(0,1) = xi.dot(yi)  -numPtsF*x0*y0;
    MM(1,1) = yi.dot(yi)  -numPtsF*y0*y0;
    MM(1,0) = MM(0,1);

    // SVD .....................................................
    const Eigen::JacobiSVD<Matrix2d> svd( MM, Eigen::ComputeFullU );//| ComputeThinV);
    const Vector2d sv = svd.singularValues();
    const Matrix2d UU = svd.matrixU();
    const Vector2d n  = UU.col(1);

    // straight line  l = [n; -n'*x0];  .........................
    const Vector3d l( n(0), n(1), -(n(0)*x0) -(n(1)*y0) );

    // check: point-line incidence, x'l = 0
    Q_ASSERT( std::fabs( (x0*l(0)) +(y0*l(1)) +l(2) ) < FLT_EPSILON);

    // estimated variance factor (10.167) .........................
    Q_ASSERT(sv(0) > sv(1));
    const double evar0 = sv(1) / (numPtsF - 2.);

    Q_ASSERT_X( std::fabs(sv(0)) > FLT_EPSILON, Q_FUNC_INFO,
               "Division by zero or near-zero singular value sv(0)");
    Q_ASSERT_X( numPtsF>0, Q_FUNC_INFO,
               "Division by zero: I (number of points) must be nonzero");

    // cov. mat. for m = [0 1 0], (10.166)
    // Sigma_mm = diag( [1/lambda(2), 0, 1/(I*wq)]);
    const Matrix3d Sigma_mm  = numPtsF*Vector3d(1./sv(0), 0.0, 1./numPtsF).asDiagonal();

    // variance propagation
    Matrix3d HH = Matrix3d::Identity(3,3);
    HH.topLeftCorner(2,2) = UU;
    HH(0,2) = x0;
    HH(1,2) = y0;      //  HH = [UU, x0; 0 0 1]
    const Matrix3d Sigma_ll = evar0*(HH.inverse()).transpose()*Sigma_mm* HH.inverse();

    Q_ASSERT_X( isCovMat(Sigma_ll ), Q_FUNC_INFO, "invalid covariance matrix");
    assert( isCovMat(Sigma_ll) );

    return {l, Sigma_ll};
}


//! Project the uncertain point 'ux' onto uncertain straight line 'this'
uPoint uStraightLine::project(const uPoint &ux) const
{
    static const Matrix3d CC = Vector3d(1,1,0).asDiagonal();

    const Vector3d l = v();
    const Vector3d x = ux.v();
    const Vector3d z = skew(l)*skew(x)*CC*l;

    // Jacobians
    const Matrix3d AA = -skew(l)*skew((CC*l).eval()) ;
    const Matrix3d BB = skew(l) * skew(x)*CC + skew(( skew((CC*l).eval()) * x).eval());

    const Matrix3d Cov_zz = AA * ux.Cov() *AA.transpose()
            +BB*Cov()*BB.transpose();

    return { z, Cov_zz};
}


//! Get Euclidean normalized version of this uncertain straight line
uStraightLine uStraightLine::euclidean() const
{
    const Vector2d lh = v().head(2);
    const double l0 = v(2);
    const double n = lh.norm();
    Q_ASSERT_X( n>0, Q_FUNC_INFO, "normal is 0-vector");

    Matrix3d JJ = Matrix3d::Identity();
    JJ.topLeftCorner(2,2)   -=  lh*lh.adjoint()/(n*n);
    JJ.bottomLeftCorner(1,2) = -l0*lh.adjoint()/(n*n);
    JJ /= n;

    return {v()/n, JJ*Cov()*JJ.adjoint()};
}


//! Get spherically normalized version of this uncertain straight line.
uStraightLine uStraightLine::sphericalNormalized() const
{
    uStraightLine p{*this};
    p.normalizeSpherical();
    return {p};
}


//! Check if the uncertain straight line is vertical.
bool uStraightLine::isVertical( const double T_q) const
{
    // l=[a,b,c], b=0 ?
    const double d = v(1);
    const RowVector3d JJ( 0, 1, 0);
    const double var_d = JJ.dot( Cov()*JJ.adjoint() );

    return d*d/var_d < T_q;
}


//! Check if the uncertain straight line is horizontal.
bool uStraightLine::isHorizontal( const double T_q) const
{
    // l=[a,b,c], a=0 ?
    const double d = v(0);
    const RowVector3d JJ( 1, 0, 0);
    const double var_d = JJ.dot( Cov()*JJ.adjoint() );

    return d*d/var_d < T_q;
}


//! Check if the uncertain straight line is diagonal.
bool uStraightLine::isDiagonal( const double T_q) const
{
    // l=[a,b,c],   abs(a)-abs(b)=0  ?
    const double d = std::fabs(v(0)) -std::fabs(v(1));
    const RowVector3d JJ( sign(v(0)), -sign(v(1)), 0);
    const double var_d = JJ.dot( Cov()*JJ.adjoint() );

    return d*d/var_d < T_q;
}


//! Check if uncertain straight line 'um' is orthogonal to this.
bool uStraightLine::isOrthogonalTo( const uStraightLine & um,
                                    const double T_q) const
{
    static const Matrix3d CC = Vector3d(1,1,0).asDiagonal();

    RowVector6d JJ;
    JJ << um.v().adjoint()*CC, v().adjoint()*CC;

    Matrix6d Sigma_lm = Matrix6d::Zero();
    Sigma_lm.topLeftCorner(3,3) = Cov();
    Sigma_lm.bottomRightCorner(3,3) = um.Cov();

    const double d = v().dot( CC*um.v() );
    const double var_d = JJ.dot( Sigma_lm*JJ.adjoint() );

    return d*d/var_d < T_q;
}


//! Check if the three uncertain straight lines 'um', 'un', and 'this' are copunctual.
bool uStraightLine::isCopunctualWith( const uStraightLine & um,
                                      const uStraightLine & un,
                                      const double T_d) const
{
    // M = [l,m,n]
    const Matrix3d MM = (Matrix3d() << v(), um.v(), un.v()).finished();

    const double d = MM.determinant(); // distance, dof = 1

    Matrix9d Sigma_lmn = Matrix9d::Zero();
    Sigma_lmn.topLeftCorner(3,3)     = Cov();
    Sigma_lmn.block(3,3,3,3)         = um.Cov();
    Sigma_lmn.bottomRightCorner(3,3) = un.Cov();

    const RowVector9d JJ = cof3(MM).reshaped();
    const double var_d = JJ.dot( Sigma_lmn*JJ.adjoint() ); // error propagation

    return d*d/var_d < T_d;
}


//! Check if the uncertain straight line 'um' is parallel to 'this'.
bool uStraightLine::isParallelTo( const uStraightLine & um,
                                  const double T_q) const
{
    static const Matrix3d S3 = skew( Vector3d(0,0,1) );

    const double d = -v().dot( S3*um.v() ); // (7.24)

    RowVector6d JJ;
    JJ <<  -um.v().adjoint()*S3.adjoint(), -v().adjoint()*S3;

    Matrix6d Sigma_lm = Matrix6d::Zero();
    Sigma_lm.topLeftCorner(3,3)     = Cov();
    Sigma_lm.bottomRightCorner(3,3) = um.Cov();

    const double var_d = JJ.dot( Sigma_lm*JJ.adjoint() );

    return d*d/var_d < T_q;
}


} // namespace Uncertain
