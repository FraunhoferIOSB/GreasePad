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

#include "upoint.h"
#include "usegment.h"
#include "ustraightline.h"

#include <QDebug>

namespace Uncertain {

static const double T_ZERO = 1e-7;

//! Type-safe sign of parameter value
template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

//! Nullspace of row vector
Matrix<double,3,2> BasicEntity2D::null( const Vector3d &xs ) const
{
    // cf. PCV, eq. (A.120)

    //if ( fabs(xs.norm()-1.) > T_ZERO )
    //    qDebug() << xs;

#ifdef QT_DEBUG
    QString what = QStringLiteral("norm(x) = %1")
            .arg( QString::number(xs.norm()) );
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
    Q_ASSERT_X( check.norm() <= T_ZERO,
                Q_FUNC_INFO,
                "not a zero vector");

    return JJ;
}


//! Transform uncertain entity via 3x3 transformation matrix
void BasicEntity2D::transform( const Matrix3d & TT )
{
    m_val = TT*m_val;
    m_cov = TT*m_cov*TT.adjoint();
    this->normalizeSpherical();
}


//! Check if matrix MM is a proper covariance matrix
bool isCovMat( const MatrixXd & MM )
{
    // qDebug() << Q_FUNC_INFO;
    Eigen::SelfAdjointEigenSolver<MatrixXd> eig( MM, Eigen::ComputeEigenvectors);
    Eigen::VectorXcd ev = eig.eigenvalues();

#ifdef QT_DEBUG
    if ( (ev.real().array() < -DBL_EPSILON ).any() ) {
        for ( Eigen::Index i=0; i< ev.size(); i++) {
            qDebug().noquote() << QStringLiteral( "(%1,%2)")
                        .arg( ev(i).real() )
                        .arg( ev(i).imag() );
        }
    }
#endif

    return (ev.real().array() >= -DBL_EPSILON ).all();
}

//! Spherically normalize the entity
void BasicEntity2D::normalizeSpherical()
{
    // qDebug() << Q_FUNC_INFO;

    const Matrix3d I = Matrix3d::Identity();
    assert( m_val.norm()>0.0 );
    Matrix3d Jac = (I - m_val*m_val.adjoint()/m_val.squaredNorm()) / m_val.norm();
    m_cov = Jac*m_cov*Jac.adjoint();

    m_val.normalize();  // x = x /norm(x)
}

//! Check if uncertainty entity is identical with uncertain entity 'us'
bool BasicEntity2D::isIdenticalTo( const BasicEntity2D & us,
                                   const double T) const
{
    BasicEntity2D a(*this);
    BasicEntity2D b(us);      // copies

    // fix ambiguities
    a.normalizeSpherical();
    b.normalizeSpherical();

    int idx;                           // visitor
    a.v().cwiseAbs().maxCoeff( &idx ); // [~,idx] = max( abs(a) )
    a.m_val *= sign( a.v()(idx) );      // a = a*sign( a(idx) );
    b.m_val *= sign( b.v()(idx) );      // b = b*sign( b(idx) );

    Matrix<double,3,2> JJ = null( a.v() );    // (A.120)
    Vector2d d = JJ.adjoint()*( a.v() -b.v() );     // (10.141)
    Eigen::Matrix2d Cov_dd = JJ.adjoint()*( a.Cov()+b.Cov())*JJ;

    // return d.adjoint()*Cov_dd.inverse()*d < T;    // dof = 2
    return d.dot(Cov_dd.inverse()*d) < T;    // dof = 2
}



std::pair<uPoint,uPoint> uEndPoints( const Eigen::VectorXd & xi,
                                     const Eigen::VectorXd & yi)
{
    Q_ASSERT( xi.size()>0 );
    Q_ASSERT( yi.size()==xi.size() );

    const uStraightLine l(xi,yi);
    const double phi  = l.angle_rad();
    const VectorXd zi = sin(phi)*xi -cos(phi)*yi;

    int idx;

    Vector3d x1;
    zi.minCoeff( &idx);
    x1 << xi(idx), yi(idx), 1;

    Vector3d x2;
    zi.maxCoeff( &idx);
    x2 << xi(idx), yi(idx), 1;

    const Matrix3d Zeros = Matrix3d::Zero(3,3);
    Matrix3d Cov_xx = Matrix3d::Zero();

    uDistance ud1 = uPoint(x1, Zeros).distanceEuclideanTo(l);
    Cov_xx.diagonal() << ud1.var_d(), ud1.var_d(), 0.;      // isotropic
    uPoint first_ = l.project( uPoint( x1, Cov_xx) ) ;

    uDistance ud2 = uPoint(x2,Zeros).distanceEuclideanTo(l);
    Cov_xx.diagonal() << ud2.var_d(), ud2.var_d(), 0.;
    uPoint second_ = l.project( uPoint( x2, Cov_xx) );

    return { first_, second_};
}

//! Skew-symmetric cross product matrix S(x): a x b = S(a)*b
Matrix3d skew( const Vector3d & x)
{
return (Matrix3d() << 0.,-x(2),x(1), x(2),0.,-x(0), -x(1),x(0),0.).finished();
}

} // namespace Uncertain
