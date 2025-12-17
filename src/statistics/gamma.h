#ifndef GAMMA_H
#define GAMMA_H

#include <cassert>
#include <cmath>
#include <limits>

#include "statistics/normal.h"
#include "statistics/prob.h"
#include "statistics/uniform.h"


namespace Stats {

//! Gamma distribution
class Gamma
{
public:
    Gamma( double alpha, double beta);        //!< Value constructor
    Gamma (const Gamma & other) = delete;   //!< Copy constructor
    Gamma (const Gamma && other) = delete;  //!< Move constructor
    Gamma & operator = (const Gamma & other) = delete;  //!< Assignment operator
    Gamma & operator = (const Gamma && other) = delete;  //!< Move assignment operator

    ~Gamma() = default;

    [[nodiscard]] double pdf(  double x ) const;   //!< Probability density function
    [[nodiscard]] Prob cdf(  double x ) const;   //!< Cumulative distribution function
    [[nodiscard]] double icdf( Prob P ) const;   //!< Inverse cumulative distribution function
    [[nodiscard]] double mean()   const { return alpha/beta;  }
    [[nodiscard]] double var()    const { return alpha/(beta*beta); }
    [[nodiscard]] double mode()   const;   //!< Mode of distribution
    [[nodiscard]] double rnd()    const;   //!< Random number
    [[nodiscard]] double shape() const { return alpha;   }  //!< Get value of shape parameter
    [[nodiscard]] double rate()  const { return beta;    }  //!< Get rate (inverse scale)
    [[nodiscard]] double scale() const { return 1./beta; }  //!< Get value of scale parameter (inverse rate)

private:
    const double alpha;  //  > 0 shape
    const double beta;   //  > 0 rate
};


inline Gamma::Gamma( const double alpha, const double beta)
    : alpha(alpha), beta(beta)
{
    assert( alpha > 0 && "parameter alpha not positive" );
    assert( beta  > 0 && "parameter beta not positive" );
}


inline double Gamma::mode() const
{
    if ( alpha >= 1 ) {
        return (alpha -1.)/beta;
    }
    return std::numeric_limits<double>::quiet_NaN();
}


inline double Gamma::rnd() const
{
    // Marsaglia's simple transformation-rejection method
    const Stats::ContinuousUniform uniform(0,1);

    const double d = alpha -1./3.;
    double v = NAN;

    while ( true ) {
        const double x = Stats::StandardNormal::rnd();   // ~N(0,1)
        const double u = uniform.rnd();  // ~U(0,1)
        v = pow( 1.+x/sqrt(9.*d), 3.);
        if  ( v>0  &&  log(u) < 0.5*x*x +d -d*v +d*log(v) ) {
            break;
        }
    }
    return d*v*beta;
}


inline double Gamma::icdf( const Prob P ) const
{
    constexpr int numIterMax = 100;
    constexpr double threshold = 1e-7;

    double y_old = mean();

    for ( int i=0; i<numIterMax; i++) {
        const double h = ( cdf(y_old)() -P() ) / pdf(y_old);
        const double y_new = y_old -h;
        if ( fabs( h ) < threshold ) {
            return y_new;
        }
        y_old = y_new;
    }

    // TO DO handle case not converged.
    return y_old;
}


inline Prob Gamma::cdf( const double x) const
{
    // K.R. Koch (243.11)
    assert( x>0 );
    constexpr int numIterMax = 100;
    constexpr double threshold = 1e-6;

    double sum = 0.;
    double den = 1.;

    for ( int j=1; j<numIterMax; j++) {
        den *= alpha+j;                    // (alpha+1)*(alpha+2)* ...*(alpha+j)
        double const Delta = pow(beta * x, j) / den;
        sum += Delta;
        if ( Delta < threshold ) {
            break;
        }
    }
    return Prob(
        (1.+sum) * pow(beta,alpha) * pow(x,alpha) * exp(-beta*x)/ std::tgamma(alpha+1.)
        );
}


inline double Gamma::pdf( const double x ) const
{
    assert( x>=0 );
    return pow( beta, alpha)/ std::tgamma(alpha) * pow( x, alpha-1.0 ) * exp( -beta*x);
}


} // namespace Stats

#endif // GAMMA_H
