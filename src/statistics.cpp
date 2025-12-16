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

#include "statistics.h"

#include <QDebug>

#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace Stats {


// M. Abramowitz and I.E. Stegun (ed.),
// Handbook of Mathematical Functions With Formulas, Graphs and Mathematical Tables.
// National Bureau of Standards, Washington (1964).
// Abramowitz and Stegun Approximations 26.2.23

double StandardNormal::icdf( const Prob P ) const
{
    if (P() < DBL_EPSILON) {
        return -DBL_MAX;
    }

    double const alpha0 = 1.0-P();
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
double StandardNormal::pdf( const double x) const
{
    return s_normalizing_constant*exp(-0.5*x*x);
}

/*!
 * \brief cumulative density function (cdf) via polynomial approximation.
 * \param x real number
 * \return value of the cdf
 */
Prob StandardNormal::cdf( const double x) const
{
    //  Koch (241.5), 1e-5
    // N.R. p. 221? 1e-7

    if ( x<0.0 ) {
       return Prob( cdf(-x).complement() );  // 1-cdf()
    }

    constexpr double p = 0.33267;
    constexpr double a1 =  0.4361836;
    constexpr double a2 = -0.1201676;
    constexpr double a3 =  0.9372980;
    const double t = 1.0 / (1.0+p*x);

    return Prob( 1.0-exp(-0.5*x*x)*(a1 +(a2 +a3*t)*t)*t *s_normalizing_constant );
}

double StandardNormal::rnd() const
{
    // Marsaglia's polar method
    double s = NAN;
    double u = NAN;
    double v = NAN;

    // std::srand(time(NULL));  // #include <ctime>
    do {
        u = 2.0*std::rand()/static_cast<double>(RAND_MAX) -1.0;
        v = 2.0*std::rand()/static_cast<double>(RAND_MAX) -1.0;
        s = u*u +v*v;
    } while ( s >=1 || std::fabs(s)<1e-6 );

    return u*std::sqrt( -2.0*std::log(s)/s );
}


/*!
 * \brief Gamma function for arguments being multiples of 0.5
 * \param x parameter
 * \return GammaFct(x)
 */
double ChiSquared::GammaFctHalfInt( const double x)
{
    // qDebug() << Q_FUNC_INFO;

    // K.-R. Koch, (243.5), (243.6).
    // constexpr double two = 2.0;
    constexpr double sqrt_pi = 1.772453850905516;  // = sqrt(3.14159);

    // assert( fabs(fmod(x, 0.5) ) < 1e-6 );
    assert(  int(2*x) % 1 == 0);

    if ( std::fabs( std::fmod(x, 1) ) < DBL_EPSILON ) {
        return  factorial( static_cast<unsigned int>(x)-1); // x \in N, x>0
    }
    if (std::fabs(std::fmod(2 * x, 1)) < DBL_EPSILON) {
        int const p = static_cast<int>(x - 0.5);
        int pr = 1; // product
        for ( int i=1; i<=2*p-1; i+=2 ) {
            pr = pr*i;
        }
        return  pr*sqrt_pi/pow(2.0,p);
    }

    return -1.0;
}


/*!
 * \brief cumulative density function
 * \param x real number
 * \return value of cumulative density function
 */
Prob ChiSquared::cdf( const double x) const
{
    if ( x<0.0 ) {
        return Prob(0);
    }

    // K.-R. Koch, (261.6).
    double sum  = 0.0;
    double prod = 1.0;
    constexpr int num_iter_max = 20;
    for (int i = 1; i < num_iter_max; i++) {
        prod *= static_cast<double>( m_nu + 2*i );
        double const summand = pow(x, i) / (prod);
        sum += summand;
        if ( summand < DBL_MIN ) {
            break;
        }
    }

    return Prob(
        pow( 0.5*x, 0.5*m_nu) * exp(-0.5*x) / GammaFctHalfInt( 0.5*(m_nu+2) )*(1.0 +sum )
        );

    //qDebug().noquote() << QString("chi2cdf(%1, %2) = %3").arg(x).arg(nu).arg(F);
}

/*!
 * \brief probability density function
 * \param x real number [0,inf)
 * \return value of the probability density function (pdf)
 */
double ChiSquared::pdf( const double x) const
{
    // qDebug() << Q_FUNC_INFO;
    if ( x < 0.0 ) {
        return  0.0;
    }

    // K.-R. Koch (261.1)
    return  1.0 / ( pow(2,0.5*m_nu) * GammaFctHalfInt(0.5*m_nu))
            * pow(x, 0.5*m_nu-1) * exp(-0.5*x);

    //qDebug().noquote() << QString("chi2pdf(%1,%2) = %3").arg(x).arg(nu).arg(f);
}


ChiSquared::ChiSquared(const int df ) : m_nu(df) {
    assert( df >= 0);
}

/*!
 * \brief Quantile of the chi-squared distribution.
 * \param P probability [0,1]
 * \return Quantile x so that cdf(x,nu) = P.
 */
double ChiSquared::icdf( const Prob P) const
{

    const double alpha = 1.0 -P();

    // approx., K.-R. Koch (261.11)
    const StandardNormal snd;
    const double xa = snd.icdf(P);
    const auto df = static_cast<double>(m_nu);
    const double t = 2.0/(9.0*df);
    double q = df* pow(  xa*sqrt( t )+1.0-t,  3 );

    // iterative refinement, K.-R. Koch (261.13)
    constexpr int num_iter_max = 10;
    for ( int i=0; i<num_iter_max; i++) {
        double const alpha_n = 1.0 - cdf(q)();
        double const summand = (alpha_n - alpha) / pdf(q);
        q += summand;
        if ( abs(summand) < DBL_EPSILON ) {
            break;
        }
    }

    //qDebug().noquote() << QString("chi2inv(%1,%2) = %3").arg(P).arg(nu).arg(q);
    return q;
}

double ChiSquared::rnd() const
{
    Stats::Gamma const d(m_nu / 2.0, 2.0);
    return d.rnd();
}




/*!
 * \brief factorial, compile-time enabled evaluation
 * \param n non-negative integer [0,17]
 * \return factorial n!
*/
constexpr unsigned int ChiSquared::factorial( const unsigned int n)
{
    // prevent silliness and overflow:
    assert( n < 18);     // 17! =  3.5569e+14

    /* if ( n==0 ) {
        return 1;
    }
    return n * factorial(n - 1); */

    unsigned int x = 1;   // 0! = 1
    for ( unsigned int i = 2; i <= n; ++i) {
        x *= i;
    }
    return x;
}


double Exponential::rnd() const
{
    // u ~ U[0,1]
    double const u = (std::rand() + 1) / static_cast<double>(RAND_MAX);
    // assert( u>0.0) ;
    return -log(u)/m_lambda;
}

Exponential::Exponential( const double lambda)
    : m_lambda(lambda)
{
    assert( lambda>0.0 );
}


Gamma::Gamma( const double alpha,
              const double beta)
    : m_alpha(alpha), m_beta(beta)
{
    assert( m_alpha > 0. );
    assert( m_beta  > 0. );
}


double Gamma::pdf( const double x ) const
{
    assert( x >= 0.0 );
    return pow( m_beta, m_alpha)
            / std::tgamma(m_alpha)
            * pow( x, m_alpha-1.0 )
            * exp( -m_beta*x);
}



Prob Gamma::cdf( const double x) const
{
    // K.R. Koch (243.11)
    assert( x>0.0 );
    double sum = 0.0;
    double den = 1.0;
    for ( int j=1; j<100; j++) {
        // (alpha+1)*(alpha+2)* ...*(alpha+j)
        den *= m_alpha+j;
        double const Delta = pow(m_beta * x, j) / den;
        sum += Delta;
        if ( Delta < 1e-6) {
            break;
        }
    }
    return Prob(
        (1.0+sum) * pow(m_beta,m_alpha)
            * pow(x,m_alpha) * exp(-m_beta*x)
            / std::tgamma(m_alpha+1.0)
        );
}

double Gamma::icdf( const Prob P ) const
{
    double y_old = mean();
    // double y_new = NAN;
    for ( int i=0; i<100; i++) {
        double const h = (cdf(y_old)() - P()) / pdf(y_old);
        const double y_new = y_old -h;
        if ( std::fabs( h ) < 1e-7 ) {
            return y_new;
        }
        y_old = y_new;
    }

    // TO DO case not converged.
    return y_old;
}


double Gamma::rnd() const
{
    // Marsaglia's simple transformation-rejection method
    Stats::StandardNormal const rng;
    const Stats::ContinuousUniform unif(0,1);

    const double d = m_alpha -1.0/3.0;
    double v = NAN;
    while ( true ) {
        double const x = rng.rnd();

        // rand(): [0,RANDMAX] (int)
        // double const u = static_cast<double>(rand() + 1)
        //                  / static_cast<double>(RAND_MAX + 1); // (0,1]
        const double u = unif.rnd();
        v = pow( 1.0 + x/sqrt(9.0*d), 3.0);
        if  ( v>0.0  &&  log(u) > x*x/2.0 +d -d*v +d*log(v) ) {
            break;
        }
    }
    return d*v*m_beta;
}


double Gamma::mode() const
{
    if ( m_alpha >= 1.0 ) {
        return (m_alpha -1.0)/m_beta;
    }
    return std::numeric_limits<double>::quiet_NaN();
}



ContinuousUniform::ContinuousUniform(const double a, const double b) : a(a), b(b)
{
    assert( b>a );
}


//! uniformly distributed random number in [a,b], b>a
double ContinuousUniform::rnd() const
{
    const double u = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    return a +u*(b -a);
}


double ContinuousUniform::pdf( const double x) const
{
    if ( x<a ) {
        return 0;
    }
    if ( x>b ) {
        return 0;
    }
    return 1./(b -a);
}


Prob ContinuousUniform::cdf( const double x) const
{
    if ( x<a ) {
        return Prob(0);
    }
    if ( x>b ) {
        return Prob(1);
    }
    return Prob( (x-a)/(b-a) );
}


double ContinuousUniform::icdf( const Prob P ) const
{
    return a +P()*(b-a);
}

} // namespace Stats
