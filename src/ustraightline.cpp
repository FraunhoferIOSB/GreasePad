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

#include <cassert>
#include <cfloat>
//#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <cstdlib>

#include "uncertain.h"
#include "upoint.h"
#include "ustraightline.h"

#include <Eigen/Core>

#include <QDebug>

#include "qassert.h"
#include "qmath.h"
#include "qtdeprecationdefinitions.h"


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;


namespace Uncertain {

//! Diag([1,1,0])
Matrix3d uStraightLine::CC()
{
    const static Matrix3d tmp = (Matrix3d() << 1,0,0,0,1,0,0,0,0).finished();
    return tmp;
}

//! skew([1,0,0])
Matrix3d uStraightLine::S3()
{
  const static Matrix3d tmp = (Matrix3d() << 0,-1,0,1,0,0,0,0,0).finished();
  return tmp;
}


//! Uncertain straight line, defined by a 3-vector and its covariance matrix
uStraightLine::uStraightLine(const Vector3d & l,
                             const Matrix3d & Sigma_ll)
    : BasicEntity2D( l, Sigma_ll )
{
    // TODO(meijoc)  check: null(Cov) = l ?
    assert( l.size()==Sigma_ll.cols() );
    // assert( isCovMat( Cov_ll) );
}


//! Uncertain straight line approximating the point set {x_i,y_i}
// uStraightLine::uStraightLine( const VectorXd & xi,  const VectorXd & yi)
uStraightLine uStraightLine::estim( const VectorXd & xi,
                                   const VectorXd & yi)
{
    // qDebug() << Q_FUNC_INFO;

    // centroid ............................................
    double const x0 = xi.mean();
    double const y0 = yi.mean();    // qDebug() << "mean = (" << x0 << "," << y0 << ")";

    // 2nd moments .........................................
    const auto I = static_cast<double>(xi.rows());  //qDebug() << "I = " << I;

    Eigen::Matrix2d MM;
    MM(0,0) = xi.dot(xi)  -I*x0*x0;
    MM(0,1) = xi.dot(yi)  -I*x0*y0;
    MM(1,1) = yi.dot(yi)  -I*y0*y0;
    MM(1,0) = MM(0,1);

    // SVD .....................................................
    const Eigen::JacobiSVD<Eigen::Matrix2d> svd( MM, Eigen::ComputeFullU );//| ComputeThinV);
    Eigen::Vector2d sv = svd.singularValues();
    Eigen::Matrix2d UU = svd.matrixU();
    Eigen::Vector2d n  = UU.col(1);

    // straight line  l = [n; -n'*x0];  .........................
    /* m_val */ const Vector3d v = (Vector3d() << n(0), n(1), -(n(0)*x0) -(n(1)*y0)).finished();

    // check: point-line incidence, x'l = 0
    Q_ASSERT( std::fabs( (x0*v(0)) +(y0*v(1)) +v(2) ) < FLT_EPSILON);

    // estimated variance factor (10.167) .........................
    Q_ASSERT(sv(0) > sv(1));
    double const evar0 = sv(1) / (I - 2);
    // qDebug() << "estd_0 = " << sqrt(evar0);

    // cov. mat. for l = [0 1 0], (10.166) ........................
    // Cov_ll = diag( [1/lambda(2), 0, 1/(I*wq)]);

    Vector3d const t = (Vector3d() << 1. / sv(0), 0, 1. / I).finished();
    Matrix3d const Sigma_ll  = I*t.asDiagonal(); // TODO(meijoc)

    // variance propagation .......................................
    Matrix3d HH = Matrix3d::Identity(3,3);
    HH.topLeftCorner(2,2) = UU;
    HH(0,2) = x0;
    HH(1,2) = y0;      //  HH = [UU, x0; 0 0 1]
    /* m_cov */ const Matrix3d Cov = evar0*(HH.inverse()).transpose()*Sigma_ll* HH.inverse();

    // TODO(meijoc): warning
    Q_ASSERT_X( isCovMat(Cov ), Q_FUNC_INFO, "invalid covariance matrix");
    assert( isCovMat(Cov ) );
    return{ v, Cov};
}

//! Project the uncertain point 'ux' onto uncertain straight line 'this'
uPoint uStraightLine::project(const uPoint &ux) const
{
    Vector3d const l = v();
    Vector3d const x = ux.v();
    Vector3d const z = skew(l)*skew(x)*CC()*l;

    // Jacobians
    Matrix3d AA = -skew(l)*skew(CC()*l);
    Matrix3d BB = skew(l) * skew(x) * CC() + skew(skew(CC() * l) * x);

    Matrix3d const Cov_zz = AA * ux.Cov() *AA.transpose()
            +BB*Cov()*BB.transpose();

    return { z, Cov_zz};
}



double uStraightLine::acute(const uStraightLine &um) const
{
    Eigen::Vector2d a = v().head(2);
    Eigen::Vector2d b = um.v().head(2);
    assert( a.norm() > FLT_EPSILON );
    assert( b.norm() > FLT_EPSILON );
    a.normalize();
    b.normalize();

    assert( fabs(a.dot(b)) <= 1.0 );  // a'*b in [-1,+1]
    double alpha = acos( a.dot(b) );  // [0,pi]
    if ( alpha > M_PI_2) {            // M_PI_2 = pi/2
        alpha = M_PI -alpha;
    }
    assert( alpha >= 0.0    );
    assert( alpha <= M_PI_2 );
    return alpha;
}

//! Uncertain straight line connecting two uncorrelated uncertain points
// uStraightLine::uStraightLine( const uPoint & ux, const uPoint & uy)
/* uStraightLine uStraightLine::cross( const uPoint & ux,
                                   const uPoint & uy)
{
    // qDebug() << Q_FUNC_INFO;
    return ux.cross(uy);

    const Matrix3d Sx = skew(ux.v());
    const Matrix3d Sy = skew(uy.v());
    const Vector3d m_val = Sx*uy.v();         // cross(x,y)

    Q_ASSERT_X( m_val.norm()>0, Q_FUNC_INFO, "identical points");
    const Matrix3d m_cov = -Sy*ux.Cov()*Sy -Sx*uy.Cov()*Sx;  // S' = -S
    return {m_val, m_cov };
}*/


//! Get Euclidean normalized version of this uncertain straight line
uStraightLine uStraightLine::euclidean() const
{
    Eigen::Vector2d const lh = v().head(2);
    double const l0 = v(2);
    Matrix3d JJ = Matrix3d::Identity();
    double const n = lh.norm();

    Q_ASSERT_X( n>0, Q_FUNC_INFO, "normal is 0-vector");
    JJ.topLeftCorner(2,2)   -=  lh*lh.adjoint()/(n*n);
    JJ.bottomLeftCorner(1,2) = -l0*lh.adjoint()/(n*n);
    JJ /= n;

    const Matrix3d Sigma = JJ * Cov() * JJ.adjoint();
    return { v()/n,  Sigma };
}


//! Get spherically normalized version of this uncertain straight line.
uStraightLine uStraightLine::sphericalNormalized() const
{
    uStraightLine p{*this};
    p.normalizeSpherical();
    return {p};
}

//! Check if the uncertain straight line is vertical.
bool uStraightLine::isVertical( double T_q) const
{
    // l=[a,b,c], b=0 ?
    double const d = v(1);
    Eigen::RowVector3d const JJ = (Eigen::RowVector3d() << 0, 1, 0).finished();
    double const var_d = JJ * Cov() * JJ.adjoint();
    return d*d/var_d < T_q;
}

//! Check if the uncertain straight line is horizontal.
bool uStraightLine::isHorizontal(double T_q) const
{
    // l=[a,b,c], a=0 ?
    double const d = v(0);
    Eigen::RowVector3d const JJ = (Eigen::RowVector3d() << 1, 0, 0).finished();
    double const var_d = JJ * Cov() * JJ.adjoint();
    return d*d/var_d < T_q;
}

//! Check if the uncertain straight line is diagonal.
bool uStraightLine::isDiagonal(double T_q) const
{
    // l=[a,b,c],   abs(a)-abs(b)=0  ?

    double const d = abs(v(0)) - abs(v(1));
    Eigen::RowVector3d const JJ = (Eigen::RowVector3d() << sign(v(0)), -sign(v(1)), 0)
                                      .finished();
    double const var_d = JJ * Cov() * JJ.adjoint();
    return d*d/var_d < T_q;
}


//! Check if uncertain straight line 'um' is orthogonal to this.
bool uStraightLine::isOrthogonalTo( const uStraightLine & um,
                                    const double T_q) const
{
    RowVector6d JJ;
    JJ.leftCols(3)  = um.v().adjoint()*CC();
    JJ.rightCols(3) =    v().adjoint()*CC();

    Matrix6d Cov_lm = Matrix6d::Zero();
    Cov_lm.topLeftCorner(3,3) = Cov();
    Cov_lm.bottomRightCorner(3,3) = um.Cov();

    double const d = v().dot(CC() * um.v());
    double const var_d = JJ * Cov_lm * JJ.adjoint();
    // double T_d  = d*d/var_d;

    return d*d/var_d <  T_q;
}

//! Check if the three uncertain straight lines 'um', 'un', and 'this' are copunctual.
bool uStraightLine::isCopunctualWith( const uStraightLine & um,
                                      const uStraightLine & un,
                                      const double T_d) const
{
    Matrix3d const MM = (Matrix3d() << v(), um.v(), un.v()).finished();

    double const d = MM.determinant(); // distance, dof = 1
    Eigen::Matrix<double,9,9> Sigma_lmn;
    Sigma_lmn.fill(0);
    Sigma_lmn.block(0,0,3,3) = Cov();
    Sigma_lmn.block(3,3,3,3) = um.Cov();
    Sigma_lmn.bottomRightCorner(3,3) = un.Cov();

    Matrix3d cofMM = cof3( MM);
    Eigen::Matrix<double,1,9> JJ;
    JJ.segment( 0, 3) = cofMM.col(0);
    JJ.segment( 3, 3) = cofMM.col(1);
    JJ.segment( 6, 3) = cofMM.col(2);
    double const var_d = JJ * Sigma_lmn * JJ.adjoint(); // error propagation

    return d*d/var_d < T_d;                  // test statistic T
}


//! Check if the uncertain straight line 'um' is parallel to 'this'.
bool uStraightLine::isParallelTo( const uStraightLine & um,
                                  const double T_q) const
{
    double const d = -v().dot(S3() * um.v()); // (7.24)

    RowVector6d JJ;
    JJ.leftCols(3) = -um.v().adjoint()*S3().adjoint();  // adj(S3)=-S3 ?
    JJ.rightCols(3) =  -v().adjoint()*S3();

    Matrix6d Cov_lm = Matrix6d::Zero();
    Cov_lm.topLeftCorner(3,3)     = Cov();
    Cov_lm.bottomRightCorner(3,3) = um.Cov();

    double const var_d = JJ * Cov_lm * JJ.adjoint();

    return d*d/var_d < T_q;
}


//! Coefficent matrix of 3x3 Matrix MM, i.e., transposed adjugate of MM
Matrix3d uStraightLine::cof3(const Matrix3d &MM)
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


} // namespace Uncertain
