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

#ifndef STATISTICS_H
#define STATISTICS_H

#include <cassert>
#include <cfloat>
#include <cmath>

//! Parametric probability distributions
namespace Stats {

//! Base class for parametric probability distributions
class Distribution
{
public:
    Distribution (const Distribution & other) = delete;   //!< Copy constructor
    Distribution & operator = (const Distribution & other) = delete;  //!< Assignment constructor

    virtual double pdf( double x) const = 0;   //!< Probability density function
    virtual double cdf( double x) const = 0;   //!< Cumulative distribution function
    virtual double icdf( double P) const = 0;  //!< Inverse cumulative distribution function
    virtual double mean()   const = 0;         //!< Mean
    virtual double var()    const = 0;         //!< Variance
    virtual double mode()   const = 0;         //!< Mode of distribution
    virtual double rnd()    const = 0;         //!< Random number

protected:
    virtual ~Distribution() = default;
    Distribution() = default;
};


//! Standard normal distribution
class StandardNormal : private Distribution
{
public:
    StandardNormal() = default;
    ~StandardNormal() override = default;

    double pdf(  double x ) const override;    //!< Probability density function
    double cdf(  double x ) const override;    //!< Cumulative distribution function
    double icdf( double P) const override;     //!< Inverse cumulative distribution function
    double mean()   const override { return 0.0; }  //!< Mean
    double var()    const override { return 1.0; }  //!< Variance
    double mode()   const override { return 0.0; }  //!< Mode of distribution
    double rnd()    const override;            //!< Random number

private:
    constexpr static const double s_normalizing_constant =  0.398942280401433; // = 1/sqrt(2*pi);
};


//! Gamma distribution
class Gamma : private Distribution
{
public:
    Gamma( double alpha, double beta);        //!< Value constructor
    ~Gamma() override = default;

    double pdf(  double x ) const override;   //!< Probability density function
    double cdf(  double x ) const override;   //!< Cumulative distribution function
    double icdf( double P ) const override;   //!< Inverse cumulative distribution function

    // Mean
    double mean()   const override { return m_alpha/m_beta;  }

    // Variance
    double var()    const override { return m_alpha/(m_beta*m_beta); }
    double mode()   const override;   //!< Mode of distribution
    double rnd()    const override;   //!< Random number

    double shape() const { return m_alpha;   }  //!< Get value of shape parameter
    double rate()  const { return m_beta;    }  //!< Get rate (inverse scale)
    double scale() const { return 1./m_beta; }  //!< Get value of scale parameter (inverse rate)

private:
    const double m_alpha;  //  > 0 shape
    const double m_beta;   //  > 0 rate
};


//! Chi-squared distribution
class ChiSquared : private Distribution
{
public:
    ChiSquared( int df );                    //!< Value Constructor
    ~ChiSquared() override = default;

    double pdf( double x) const override;    //!< Probability density function
    double cdf( double x) const override;    //!< Cumulative distribution function
    double icdf( double P) const override;   //!< Inverse cumulative distribution function
    double mean()   const override { return m_nu;   }    //!< Mean
    double var()    const override { return 2*m_nu; }    //!< Variance

    //! Mode of distribution
    double mode()   const override { return std::fmax( m_nu-2.0, 0.0); }
    double rnd()    const override;    //!< Random number

    int dof() const { return m_nu; }   //!< Get degrees of freedom

private:
    static double GammaFctHalfInt( double x);
    static unsigned int factorial( unsigned int n);

    const int m_nu; // degrees of freedom
};


//! Exponential distribution
class Exponential : private Distribution
{
public:
    Exponential( double lambda);         //!< Value constructor
    ~Exponential() override = default;

    //! Probability density function
    double pdf( double x) const override {
        return x>=0.0 ? m_lambda*exp(-m_lambda*x) : 0.0;
    }

    //! Cumulative distribution function
    double cdf( double x) const override {
        return x>=0.0 ? 1.0-exp(-m_lambda*x) : 0.0;
    }

    //! Inverse cumulative distribution function
    double icdf( double P) const override {
        assert( P>=0.0 );
        assert( P<1.0 );
        return -log(1.0 -P)/m_lambda;
    }

    //! Mean
    double mean()   const override { return 1.0/m_lambda; }

    //! Variance
    double var()    const override { return 1.0/(m_lambda*m_lambda); }

    //! Mode of distributiion
    double mode()   const override { return 0.0; }
    double rnd()    const override;  //!< Random number

    //! Get rate (inverse scale)
    double rate()  const {return m_lambda; }

    //! Get scale (inverse ratse)
    double scale() const {return 1./m_lambda;}

private:
    const double m_lambda;  // rate, inverse scale
};

} // namespace Stats

#endif // STATISTICS_H
