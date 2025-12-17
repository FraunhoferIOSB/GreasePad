#ifndef NORMAL_H
#define NORMAL_H

#include "statistics/prob.h"
#include "statistics/uniform.h"

#include <cfloat>
#include <cmath>

namespace Stats {

//! Standard normal distribution
class StandardNormal
{
public:
    StandardNormal() = default;
    StandardNormal (const StandardNormal & other) = delete;   //!< Copy constructor
    StandardNormal (const StandardNormal && other) = delete;  //!< Move constructor
    StandardNormal & operator = (const StandardNormal & other) = delete;  //!< Assignment operator
    StandardNormal & operator = (const StandardNormal && other) = delete;  //!< Move assignment operator

    ~StandardNormal() = default;

    [[nodiscard]] static double pdf( double x );    //!< Probability density function
    [[nodiscard]] Prob cdf( double x ) const;    //!< Cumulative distribution function
    [[nodiscard]] static double icdf( Prob P);     //!< Inverse cumulative distribution function
    [[nodiscard]] static double mean() { return 0.; }  //!< Mean
    [[nodiscard]] static double var()  { return 1.; }  //!< Variance
    [[nodiscard]] static double mode() { return 0.; }  //!< Mode of distribution
    [[nodiscard]] static double rnd();                 //!< Random number

private:
    constexpr static const double s_normalizing_constant =  0.398942280401433; // = 1/sqrt(2*pi);
};


// M. Abramowitz and I.E. Stegun (ed.),
// Handbook of Mathematical Functions With Formulas, Graphs and Mathematical Tables.
// National Bureau of Standards, Washington (1964).
// Abramowitz and Stegun Approximations 26.2.23
inline double StandardNormal::icdf( const Prob P )
{
    if ( P()<0 ) {
        return -DBL_MAX;
    }

    double const alpha0 = 1.-P();
    // K-R Koch, (241.8), alpha > 0.5
    const double alpha = (alpha0 < 0.5) ? 1. - alpha0 : alpha0;

    double const t = sqrt( log( 1./ ( (1.-alpha)*(1.-alpha) )) );

    constexpr double c0 =  2.653962002601684482;
    constexpr double c1 =  1.561533700212080345;
    constexpr double c2 =  0.061146735765196993;
    constexpr double d1 =  1.904875182836498708;
    constexpr double d2 =  0.454055536444233510;
    constexpr double d3 =  0.009547745327068945;
    double xa = t-(c0+c1*t+c2*t*t)/(1. +d1*t +d2*t*t +d3*t*t*t);

    if ( alpha0 > 0.5 ) {
        xa = -xa;
    }

    // qDebug().noquote() << QString("norminv(%1) = %2.").arg(P).arg(xa);
    return xa;
}

/*!
 * \brief probability density function
 * \param x real-valued parameter
 * \return value of probability density function
 */
inline double StandardNormal::pdf( const double x)
{
    return s_normalizing_constant*exp(-0.5*x*x);
}

/*!
 * \brief cumulative density function (cdf) via polynomial approximation.
 * \param x real number
 * \return value of the cdf
 */
inline Prob StandardNormal::cdf( const double x) const
{
    //  Koch (241.5), 1e-5
    // N.R. p. 221? 1e-7

    if ( x<0 ) {
        return Prob( cdf(-x).complement() );  // 1-cdf()
    }

    constexpr double p = 0.33267;
    constexpr double a1 =  0.4361836;
    constexpr double a2 = -0.1201676;
    constexpr double a3 =  0.9372980;
    const double t = 1.0 / (1.0+p*x);

    return Prob( 1.0-exp(-0.5*x*x)*(a1 +(a2 +a3*t)*t)*t *s_normalizing_constant );
}


inline double StandardNormal::rnd()
{
    // Marsaglia's polar method
    double s = NAN;
    double u = NAN;
    double v = NAN;

    constexpr double threshold = 1e-6;

    const ContinuousUniform uniform(-1,1);
    do {
        u = uniform.rnd();
        v = uniform.rnd();
        s = u*u +v*v;
    } while ( s >=1 || fabs(s)<threshold );

    return u*std::sqrt( -2*log(s)/s );
}

} // namespace Stats

#endif // NORMAL_H
