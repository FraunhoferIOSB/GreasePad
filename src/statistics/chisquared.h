#ifndef CHISQUARED_H
#define CHISQUARED_H

#include "statistics/gamma.h"
#include "statistics/normal.h"
#include "statistics/prob.h"

#include <cassert>
#include <cmath>


namespace Stats {

//! Chi-squared distribution
class ChiSquared
{
public:
    explicit ChiSquared( int nu );                    //!< Value Constructor
    ChiSquared (const ChiSquared &  other) = delete;   //!< Copy constructor
    ChiSquared (const ChiSquared && other) = delete;  //!< Move constructor
    ChiSquared & operator = (const ChiSquared & other) = delete;  //!< Assignment operator
    ChiSquared & operator = (const ChiSquared && other) = delete;  //!< Move assignment operator

    ~ChiSquared() = default;

    [[nodiscard]] double pdf( double x) const;    //!< Probability density function
    [[nodiscard]] Prob cdf( double x) const;    //!< Cumulative distribution function
    [[nodiscard]] double icdf( Prob P) const;   //!< Inverse cumulative distribution function
    [[nodiscard]] double mean()   const { return nu;   }    //!< Mean
    [[nodiscard]] double var()    const { return 2*nu; }    //!< Variance
    [[nodiscard]] double mode()   const { return std::fmax( nu-2, 0.); } //!< mode
    [[nodiscard]] double rnd()    const;           //!< Random number
    [[nodiscard]] int dof() const { return nu; }   //!< Get degrees of freedom

private:
    static double GammaFctHalfInt( double x);
    constexpr static unsigned int factorial( unsigned int n);

    const int nu; // degrees of freedom
};



inline ChiSquared::ChiSquared( const int nu ) : nu(nu) {
    assert( nu>=0 && "degrees of freedom negative");
}


/*!
 * \brief probability density function
 * \param x real number [0,inf)
 * \return value of the probability density function (pdf)
 */
inline double ChiSquared::pdf( const double x) const
{
    if ( x<0 ) {
        return 0.;
    }

    // K.-R. Koch (261.1)
    return  1. / ( pow(2,0.5*nu) * GammaFctHalfInt(0.5*nu) )
           * pow(x, 0.5*nu-1) * exp(-0.5*x);

    //qDebug().noquote() << QString("chi2pdf(%1,%2) = %3").arg(x).arg(nu).arg(f);
}


/*!
 * \brief cumulative density function
 * \param x real number
 * \return value of cumulative density function
 */
inline Prob ChiSquared::cdf( const double x) const
{
    if ( x<0 ) {
        return Prob(0);
    }

    // K.-R. Koch, (261.6).
    constexpr int num_iter_max = 20;
    constexpr double threshold = 1e-6;
    double sum  = 0.;
    double prod = 1.;

    for (int i=1; i<num_iter_max; i++) {
        prod *= static_cast<double>( nu+2*i );
        const double summand = pow(x, i) / prod;
        sum += summand;
        if ( summand < threshold ) {
            break;
        }
    }

    return Prob(
        pow( 0.5*x, 0.5*nu) * exp(-0.5*x) / GammaFctHalfInt( 0.5*(nu+2) )*(1. +sum )
        );

    //qDebug().noquote() << QString("chi2cdf(%1, %2) = %3").arg(x).arg(nu).arg(F);
}



/*!
 * \brief Quantile of the chi-squared distribution.
 * \param P probability [0,1]
 * \return Quantile x so that cdf(x,nu) = P.
 */
inline double ChiSquared::icdf( const Prob P) const
{
    const double alpha = 1.-P();

    // approx., K.-R. Koch (261.11)
    // const StandardNormal normal;
    const double xa = Stats::StandardNormal::icdf(P);
    const auto df = static_cast<double>(nu);
    const double t = 2./(9.*df);
    double q = df* pow(  xa*sqrt( t )+1.-t,  3 );

    // iterative refinement of q, K.-R. Koch (261.13)
    constexpr int numIterMax = 10;
    constexpr double threshold = 1e-7;

    for ( int i=0; i<numIterMax; i++) {
        const double alpha_n = 1. - cdf(q)();
        const double summand = (alpha_n - alpha) / pdf(q);
        q += summand;
        if ( fabs(summand) < threshold ) {
            break;
        }
    }

    //qDebug().noquote() << QString("chi2inv(%1,%2) = %3").arg(P).arg(nu).arg(q);
    return q;
}


inline double ChiSquared::rnd() const
{
    const Stats::Gamma gamma( nu/2., 2.);
    return gamma.rnd();
}


/*!
 * \brief Gamma function for arguments being multiples of 0.5
 * \param x parameter
 * \return GammaFct(x)
 */
inline double ChiSquared::GammaFctHalfInt( const double x)
{
    assert( trunc(2*x)==2*x && "argument not a multiple of 1/2" );

    // K.-R. Koch, (243.5), (243.6).
    constexpr double sqrt_pi = 1.772453850905516;  // = sqrt(3.14159);

    // case 1, 2, 3, ...
    if ( trunc(x)==x ) {
        return factorial( static_cast<unsigned int>(x)-1); // x \in N, x>0
    }

    // else:  case 0.5, 1.5, 2.5, ...
    const int p = static_cast<int>( x-0.5 );
    int pr = 1; // product
    for ( int i=1; i<=2*p-1; i+=2 ) {
        pr = pr*i;
    }

    return  pr*sqrt_pi/static_cast<double>( pow(2,p) );
}


/*!
 * \brief factorial, compile-time enabled evaluation
 * \param n non-negative integer [0,17]
 * \return factorial n!
*/
constexpr unsigned int ChiSquared::factorial( const unsigned int n)
{
    // prevent silliness and overflow:
    assert( n<18 );     // 17! =  3.5569e+14

    unsigned int x = 1;   // 0! = 1
    for ( unsigned int i=2; i<=n; ++i) {
        x *= i;
    }
    return x;
}


} // namespace Stats

#endif // CHISQUARED_H
