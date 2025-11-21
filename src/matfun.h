#ifndef MATFUN_H
#define MATFUN_H


#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/SparseCore>

#include <cfloat>
#include <cmath>

#include <QString>
#include <QStringLiteral>
#include <qassert.h>
#include <qtdeprecationdefinitions.h>


namespace Matfun {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::SparseMatrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;


//! Nullspace of row vector
static MatrixXd null( const VectorXd & xs )
{
    // cf. PCV, eq. (A.120)

#ifdef QT_DEBUG
    QString const what = QStringLiteral("norm(x) = %1").arg( QString::number(xs.norm()) );
    Q_ASSERT_X(fabs(xs.norm() - 1.) <= FLT_EPSILON, "null(x)", what.toStdString().data());
#endif

    Eigen::Index  const N = xs.size();

    VectorXd x0 = xs.head(N-1);
    double   xN = xs(N-1);
    if ( xN < 0.0 ) {
        x0 = -x0;
        xN = -xN;
    }

    MatrixXd JJ( N, N-1);
    JJ.topRows(N-1)  = MatrixXd::Identity(N-1,N-1) -x0*x0.adjoint()/(1.+xN);
    JJ.bottomRows(1) = -x0.adjoint();

    VectorXd const check = JJ.adjoint() * xs;
#ifdef QT_DEBUG
    Q_ASSERT_X( check.norm() <= FLT_EPSILON, Q_FUNC_INFO, "not a zero vector");
#endif
    return JJ;
}



static MatrixXd Rot_ab( const VectorXd &a, const VectorXd &b)
{
    Q_ASSERT( a.size()==b.size() );
#ifdef QT_DEBUG
    Q_ASSERT( std::fabs( a.norm()-1.) < FLT_EPSILON );
    Q_ASSERT( std::fabs( b.norm()-1.) < FLT_EPSILON );
#endif
    return MatrixXd::Identity( a.size(),a.size())
           +2*b*a.adjoint()
           -(a+b)*(a+b).adjoint()/(1.+a.dot(b));
}


//! check if the matrix AA is rank-deficient
static bool is_rank_deficient( Eigen::SparseMatrix<double,Eigen::ColMajor> & AA, const double T )
{
    Eigen::ColPivHouseholderQR<MatrixXd> qr(AA);
    qr.setThreshold( T );
    return ( qr.rank() < AA.rows() );
}


static Matrix3d skew(const Vector3d &x)
{
    return (Matrix3d() << 0.,-x(2),x(1), x(2),0.,-x(0), -x(1),x(0),0.).finished();
}

} // namespace Matfun


#endif // MATFUN_H
