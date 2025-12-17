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


double StandardNormal::rnd() const
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


/*!
 * \brief Gamma function for arguments being multiples of 0.5
 * \param x parameter
 * \return GammaFct(x)
 */
double ChiSquared::GammaFctHalfInt( const double x)
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

    return  pr*sqrt_pi/(double)pow(2,p);
}


/*!
 * \brief cumulative density function
 * \param x real number
 * \return value of cumulative density function
 */
Prob ChiSquared::cdf( const double x) const
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
        double const summand = pow(x, i) / (prod);
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
 * \brief probability density function
 * \param x real number [0,inf)
 * \return value of the probability density function (pdf)
 */
double ChiSquared::pdf( const double x) const
{
    if ( x<0 ) {
        return 0.;
    }

    // K.-R. Koch (261.1)
    return  1. / ( pow(2,0.5*nu) * GammaFctHalfInt(0.5*nu) )
            * pow(x, 0.5*nu-1) * exp(-0.5*x);

    //qDebug().noquote() << QString("chi2pdf(%1,%2) = %3").arg(x).arg(nu).arg(f);
}


ChiSquared::ChiSquared( const int nu ) : nu(nu) {
    assert( nu>=0 && "degrees of freedom negative");
}


/*!
 * \brief Quantile of the chi-squared distribution.
 * \param P probability [0,1]
 * \return Quantile x so that cdf(x,nu) = P.
 */
double ChiSquared::icdf( const Prob P) const
{
    const double alpha = 1.-P();

    // approx., K.-R. Koch (261.11)
    const StandardNormal normal;
    const double xa = normal.icdf(P);
    const auto df = static_cast<double>(nu);
    const double t = 2./(9.*df);
    double q = df* pow(  xa*sqrt( t )+1.-t,  3 );

    // iterative refinement of q, K.-R. Koch (261.13)
    constexpr int numIterMax = 10;
    constexpr double threshold = 1e-7;

    for ( int i=0; i<numIterMax; i++) {
        double const alpha_n = 1. - cdf(q)();
        double const summand = (alpha_n - alpha) / pdf(q);
        q += summand;
        if ( fabs(summand) < threshold ) {
            break;
        }
    }

    //qDebug().noquote() << QString("chi2inv(%1,%2) = %3").arg(P).arg(nu).arg(q);
    return q;
}


double ChiSquared::rnd() const
{
    Stats::Gamma const gamma( nu/2., 2.);
    return gamma.rnd();
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


double Exponential::rnd() const
{
    const ContinuousUniform uniform(0,1);
    const double u = uniform.rnd();  // ~U(0,1)

    // assert( u>0.0) ;
    return -log(u)/lambda;
}


Exponential::Exponential( const double lambda)
    : lambda(lambda)
{
    assert( lambda>0 );
}


Gamma::Gamma( const double alpha, const double beta)
    : alpha(alpha), beta(beta)
{
    assert( alpha > 0 && "parameter alpha not positive" );
    assert( beta  > 0 && "parameter beta not positive" );
}


double Gamma::pdf( const double x ) const
{
    assert( x>=0 );
    return pow( beta, alpha)/ std::tgamma(alpha) * pow( x, alpha-1.0 ) * exp( -beta*x);
}



Prob Gamma::cdf( const double x) const
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


double Gamma::icdf( const Prob P ) const
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


double Gamma::rnd() const
{
    // Marsaglia's simple transformation-rejection method
    const Stats::StandardNormal normal;
    const Stats::ContinuousUniform uniform(0,1);

    const double d = alpha -1./3.;
    double v = NAN;

    while ( true ) {
        const double x = normal.rnd();   // ~N(0,1)
        const double u = uniform.rnd();  // ~U(0,1)
        v = pow( 1.+x/sqrt(9.*d), 3.);
        if  ( v>0  &&  log(u) < 0.5*x*x +d -d*v +d*log(v) ) {
            break;
        }
    }
    return d*v*beta;
}


double Gamma::mode() const
{
    if ( alpha >= 1 ) {
        return (alpha -1.)/beta;
    }
    return std::numeric_limits<double>::quiet_NaN();
}



ContinuousUniform::ContinuousUniform( const double a, const double b) : a(a), b(b)
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
