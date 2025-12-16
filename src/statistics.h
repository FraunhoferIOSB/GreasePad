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

#ifndef STATISTICS_H
#define STATISTICS_H

#include <cassert>
#include <cmath>


//! Parametric probability distributions
namespace Stats {

//! Probability [0,1]
class Prob
{
public:
    explicit Prob( const double P) : m_P(P) {
        assert( P>=0 );
        assert( P<=1 );
    }
    double operator()() const { return m_P; }  //!< get value
    [[nodiscard]] Prob complement() const { return Prob(1-m_P); }  //!< 1-P

private:
    double m_P;  // probability [0,1]
};


//! Base class for parametric probability distributions
class Distribution
{
public:
    Distribution (const Distribution & other) = delete;   //!< Copy constructor
    Distribution & operator = (const Distribution & other) = delete;  //!< Assignment constructor
    Distribution (const Distribution && other) = delete;  //!< Move constructor
    Distribution & operator = (const Distribution && other) = delete;  //!< Move assignment constructor

    [[nodiscard]] virtual double pdf( double x) const = 0;   //!< Probability density function
    [[nodiscard]] virtual Prob cdf( double x) const = 0;   //!< Cumulative distribution function
    [[nodiscard]] virtual double icdf( Prob P) const = 0;  //!< Inverse cumulative distribution function
    [[nodiscard]] virtual double mean()   const = 0;         //!< Mean
    [[nodiscard]] virtual double var()    const = 0;         //!< Variance
    [[nodiscard]] virtual double mode()   const = 0;         //!< Mode of distribution
    [[nodiscard]] virtual double rnd()    const = 0;         //!< Random number

protected:
    ~Distribution() = default;
    Distribution() = default;
};


//! Standard normal distribution
class StandardNormal : private Distribution
{
public:
    StandardNormal() = default;
    StandardNormal (const StandardNormal & other) = delete;   //!< Copy constructor
    StandardNormal (const StandardNormal && other) = delete;  //!< Move constructor
    StandardNormal & operator = (const StandardNormal & other) = delete;  //!< Assignment constructor
    StandardNormal & operator = (const StandardNormal && other) = delete;  //!< Move assignment constructor

    virtual ~StandardNormal() = default;

    [[nodiscard]] double pdf(  double x ) const override;    //!< Probability density function
    [[nodiscard]] Prob cdf(  double x ) const override;    //!< Cumulative distribution function
    [[nodiscard]] double icdf( Prob P) const override;     //!< Inverse cumulative distribution function
    [[nodiscard]] double mean()   const override { return 0.0; }  //!< Mean
    [[nodiscard]] double var()    const override { return 1.0; }  //!< Variance
    [[nodiscard]] double mode()   const override { return 0.0; }  //!< Mode of distribution
    [[nodiscard]] double rnd()    const override;            //!< Random number

private:
    constexpr static const double s_normalizing_constant =  0.398942280401433; // = 1/sqrt(2*pi);
};


//! Gamma distribution
class Gamma : private Distribution
{
public:
    Gamma( double alpha, double beta);        //!< Value constructor
    Gamma (const Gamma & other) = delete;   //!< Copy constructor
    Gamma (const Gamma && other) = delete;  //!< Move constructor
    Gamma & operator = (const Gamma & other) = delete;  //!< Assignment constructor
    Gamma & operator = (const Gamma && other) = delete;  //!< Move assignment constructor

    virtual ~Gamma() = default;

    [[nodiscard]] double pdf(  double x ) const override;   //!< Probability density function
    [[nodiscard]] Prob cdf(  double x ) const override;   //!< Cumulative distribution function
    [[nodiscard]] double icdf( Prob P ) const override;   //!< Inverse cumulative distribution function
    [[nodiscard]] double mean()   const override { return m_alpha/m_beta;  }
    [[nodiscard]] double var()    const override { return m_alpha/(m_beta*m_beta); }
    [[nodiscard]] double mode()   const override;   //!< Mode of distribution
    [[nodiscard]] double rnd()    const override;   //!< Random number
    [[nodiscard]] double shape() const { return m_alpha;   }  //!< Get value of shape parameter
    [[nodiscard]] double rate()  const { return m_beta;    }  //!< Get rate (inverse scale)
    [[nodiscard]] double scale() const { return 1./m_beta; }  //!< Get value of scale parameter (inverse rate)

private:
    const double m_alpha;  //  > 0 shape
    const double m_beta;   //  > 0 rate
};


//! Chi-squared distribution
class ChiSquared : private Distribution
{
public:
    explicit ChiSquared( int df );                    //!< Value Constructor
    ChiSquared( const ChiSquared &  other) = delete;   //!< Copy constructor
    ChiSquared (const ChiSquared && other) = delete;  //!< Move constructor
    ChiSquared & operator = (const ChiSquared & other) = delete;  //!< Assignment constructor
    ChiSquared & operator = (const ChiSquared && other) = delete;  //!< Move assignment constructor

    virtual ~ChiSquared() = default;

    [[nodiscard]] double pdf( double x) const override;    //!< Probability density function
    [[nodiscard]] Prob cdf( double x) const override;    //!< Cumulative distribution function
    [[nodiscard]] double icdf( Prob P) const override;   //!< Inverse cumulative distribution function
    [[nodiscard]] double mean()   const override { return m_nu;   }    //!< Mean
    [[nodiscard]] double var()    const override { return 2*m_nu; }    //!< Variance
    [[nodiscard]] double mode()   const override { return std::fmax( m_nu-2, 0.0); } //!< mode
    [[nodiscard]] double rnd()    const override;    //!< Random number
    [[nodiscard]] int dof() const { return m_nu; }   //!< Get degrees of freedom

private:
    static double GammaFctHalfInt( double x);
    constexpr static unsigned int factorial( unsigned int n);

    const int m_nu; // degrees of freedom
};


//! Exponential distribution
class Exponential : private Distribution
{
public:
    explicit Exponential( double lambda);         //!< Value constructor
    Exponential( const Exponential &  other) = delete;   //!< Copy constructor
    Exponential( const Exponential && other) = delete;  //!< Move constructor
    Exponential & operator = (const Exponential &  other) = delete;  //!< Assignment constructor
    Exponential & operator = (const Exponential && other) = delete;  //!< Move assignment constructor

    virtual ~Exponential() = default;

    //! Probability density function
    [[nodiscard]] double pdf( double x) const override {
        return x>=0.0 ? m_lambda*exp(-m_lambda*x) : 0.0;
    }

    //! Cumulative distribution function
    [[nodiscard]] Prob cdf( double x) const override {
        return x>=0.0 ? Prob( 1.0-exp(-m_lambda*x)) : Prob(0);
    }

    //! Inverse cumulative distribution function
    [[nodiscard]]  double icdf( const Prob P) const override {
        return -log(1.0 -P())/m_lambda;
    }

    [[nodiscard]]  double mean()   const override { return 1.0/m_lambda; }  //!< mean
    [[nodiscard]]  double var()    const override { return 1.0/(m_lambda*m_lambda); } //!< Variance
    [[nodiscard]] double mode()   const override { return 0.0; }  //!< mode
    [[nodiscard]] double rnd()    const override;  //!< Random number

    [[nodiscard]] double rate()  const {return m_lambda; }  //!< Get rate (inverse scale)
    [[nodiscard]] double scale() const {return 1./m_lambda;}   //!< Get scale (inverse ratse)

private:
    const double m_lambda;  // rate, inverse scale
};



//! Uniform distribution U(a,b)
class ContinuousUniform : private Distribution
{
public:
    explicit ContinuousUniform( double, double);
    ContinuousUniform (const ContinuousUniform & other) = delete;   //!< Copy constructor
    ContinuousUniform (const ContinuousUniform && other) = delete;  //!< Move constructor
    ContinuousUniform & operator = (const ContinuousUniform & other) = delete;  //!< Assignment operator
    ContinuousUniform & operator = (const ContinuousUniform && other) = delete;  //!< Move assignment operator

    virtual ~ContinuousUniform() = default;

    [[nodiscard]] double pdf(  double x ) const override;    //!< Probability density function
    [[nodiscard]] Prob cdf(  double x ) const override;    //!< Cumulative distribution function
    [[nodiscard]] double icdf( Prob P) const override;     //!< Inverse cumulative distribution function
    [[nodiscard]] double mean()   const override { return (b-a)/2; }  //!< Mean
    [[nodiscard]] double var()    const override { return (b-a)*(b-a)/12.; }  //!< Variance
    [[nodiscard]] double mode()   const override { return NAN; }  //!< Mode of distribution
    [[nodiscard]] double rnd()    const override;            //!< Random number

private:
    const double a;
    const double b;
};

} // namespace Stats

#endif // STATISTICS_H
