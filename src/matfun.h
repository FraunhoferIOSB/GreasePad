#ifndef MATFUN_H
#define MATFUN_H


#include <Eigen/Core>
// #include <Eigen/QR>
#include <Eigen/OrderingMethods>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>

#include <cfloat>
#include <cmath>

#include <QDebug>
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
using Eigen::Index;
using Eigen::Matrix;
using Eigen::Vector;
using Eigen::VectorXi;
using Eigen::Dynamic;


//! Nullspace of row vector
[[nodiscard,maybe_unused]] static MatrixXd null( const VectorXd & xs )
{
    // cf. PCV, eq. (A.120)

#ifdef QT_DEBUG
    const QString what = QStringLiteral("norm(x) = %1").arg( QString::number(xs.norm()) );
    Q_ASSERT_X(fabs(xs.norm() - 1.) <= FLT_EPSILON, "null(x)", what.toStdString().data());
#endif

    const Index N = xs.size();

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



[[maybe_unused]] static MatrixXd Rot_ab( const VectorXd &a, const VectorXd &b)
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
[[maybe_unused]] static bool is_rank_deficient( SparseMatrix<double,Eigen::ColMajor> & AA, const double threshold )
{
    // Eigen::ColPivHouseholderQR<MatrixXd> qr(AA);
    // qr.setThreshold( T );
    Eigen::SparseQR<SparseMatrix<double>,Eigen::COLAMDOrdering<int>> qr;
    AA.makeCompressed();
    qr.compute(AA);
    qr.setPivotThreshold( threshold );

    return ( qr.rank() < AA.rows() );
}


//! skew-symmetric matrix S(x) from vector x
template <typename T>
[[maybe_unused]] static Matrix<T,3,3> skew(const Vector<T,3> &x)
{
    return (Matrix<T,3,3>() << 0.,-x(2),x(1), x(2),0.,-x(0), -x(1),x(0),0.).finished();
}


//! 3x3 cofactor matrix, i.e., transposed adjugate
[[maybe_unused]] static Matrix3d cof3(const Matrix3d &MM)
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

//! Matlab's find(x,1,'first')
template <typename T>
[[nodiscard]] static Index indexOf(const Eigen::Vector<T,Eigen::Dynamic> &v, const T x)
{
    for ( Index i=0; i<v.size(); i++) {
        if ( v(i)==x ) {
            return i;
        }
    }
    return -1;

    /* Eigen 3.4.0
    auto it = std::find( v.begin(), v.end(), i);
    if ( it==v.end() ) {
        return -1;
    }
    return std::distance( v.begin(), it); */
}


//! sign(0):=+1
template <typename T>
int sign(T val) { return (T(0) <= val) - (val < T(0));  }


//! Matlab's find
[[nodiscard,maybe_unused]] static Vector<Index,Dynamic> find( const Vector<bool,Dynamic> & cond)
{
    Eigen::Vector<Index,Dynamic> idx( cond.count() );
    for (int i=0, k=0; i< cond.size(); i++) {
        if ( cond(i) ) {
            idx(k++) = i;
        }
    }
    return idx;
}


//! Unique values in vector
template <typename T>
 [[nodiscard,maybe_unused]] static Vector<T, Dynamic> unique( const Vector<T,Dynamic> &x)
{
    auto u = x;
    std::sort( u.begin(), u.end() );
    auto last = std::unique( u.begin(), u.end() );  // remove consecutive (adjacent) duplicates
    auto n = std::distance( u.begin(), last);
    return u.head(n);
}


/* [[nodiscard,maybe_unused]] static VectorXi unique(const VectorXi &x)
{
    // qDebug() << Q_FUNC_INFO;
    Q_ASSERT( (x.array()>=0).all() );

    constexpr int INVALID_INDEX = -1;

    if (x.size()==0) {
        return VectorXi(0);
    }

    const int m = x.maxCoeff();
    VectorXi t = VectorXi::Zero(m + 1);
    for ( auto xi : x ) {
        t( xi ) = 1;
    }
    const int s = t.sum();

    VectorXi u = VectorXi::Constant(s,INVALID_INDEX);
    for (int uniqueIndex=0, valueIndex=0; valueIndex<m+1; valueIndex++) {
        if ( t(valueIndex)>0 ) {
            u( uniqueIndex++) = valueIndex;
        }
    }

    Q_ASSERT( (u.array()>=0).all() );

    return u;
} */


} // namespace Matfun


#endif // MATFUN_H
