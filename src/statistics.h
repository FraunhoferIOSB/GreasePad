/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022 Jochen Meidow, Fraunhofer IOSB
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

#include <limits>
#include <cassert>

namespace Stats {

class Distribution
{
public:
    Distribution (const Distribution & other) = delete;
    Distribution & operator = (const Distribution & other) = delete;

    virtual double pdf( double x) const = 0;
    virtual double cdf( double x) const = 0;
    virtual double icdf( double P) const = 0;
    virtual double mean()   const = 0;
    virtual double var()    const = 0;
    virtual double mode()   const = 0;
    virtual double rnd()    const = 0;

protected:
    virtual ~Distribution() = default;
    Distribution() = default;
};

class StandardNormal : private Distribution
{
public:
    StandardNormal() = default;
    ~StandardNormal() override = default;

    double pdf(  double x ) const override;
    double cdf(  double x ) const override;
    double icdf( double P) const override;
    double mean()   const override { return 0.0; }
    double var()    const override { return 1.0; }
    double mode()   const override { return 0.0; }
    double rnd()    const override;

private:
    constexpr static const double s_normalizing_constant =  0.398942280401433; // = 1/sqrt(2*pi);
};


class Gamma : private Distribution
{
public:
    Gamma( double alpha, double beta);
    ~Gamma() override = default;

    double pdf(  double x ) const override;
    double cdf(  double x ) const override;
    double icdf( double P ) const override;
    double mean()   const override { return m_alpha/m_beta;  }
    double var()    const override { return m_alpha/(m_beta*m_beta); }
    double mode()   const override;
    double rnd()    const override;

    double shape() const { return m_alpha;   }
    double rate()  const { return m_beta;    }
    double scale() const { return 1./m_beta; }

private:
    const double m_alpha;  //  > 0 shape
    const double m_beta;   //  > 0 rate
};



class ChiSquared : private Distribution
{
public:
    ChiSquared( int df );
    ~ChiSquared() override = default;

    double pdf( double x) const override;
    double cdf( double x) const override;
    double icdf( double P) const override;
    double mean()   const override { return m_nu;   }
    double var()    const override { return 2*m_nu; }
    double mode()   const override { return std::fmax( m_nu-2.0, 0.0); }
    double rnd()    const override;

    int dof() const { return m_nu; }

private:
    static double GammaFctHalfInt( double x);
    static unsigned int factorial( unsigned int n);

    const int m_nu; // degrees of freedom
};


class Exponential : private Distribution
{
public:
    Exponential( double lambda);
    ~Exponential() override = default;

    double pdf( double x) const override {
        return x>=0.0 ? m_lambda*exp(-m_lambda*x) : 0.0;
    }
    double cdf( double x) const override {
        return x>=0.0 ? 1.0-exp(-m_lambda*x) : 0.0;
    }
    double icdf( double P) const override {
        assert( P>=0.0 );
        assert( P<1.0 );
        return -log(1.0 -P)/m_lambda;
    }
    double mean()   const override { return 1.0/m_lambda; }
    double var()    const override { return 1.0/(m_lambda*m_lambda); }
    double mode()   const override { return 0.0; }
    double rnd()    const override;

    double rate()  const {return m_lambda; }
    double scale() const {return 1./m_lambda;}

private:
    const double m_lambda;  // rate, inverse scale
};

} // namespace Stats

#endif // STATISTICS_H
