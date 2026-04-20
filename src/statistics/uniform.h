/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2026 Jochen Meidow, Fraunhofer IOSB
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

#ifndef UNIFORM_H
#define UNIFORM_H

#include <cassert>
#include <random>

#include "statistics/prob.h"

namespace Stats {

//! Uniform distribution U(a,b)
class ContinuousUniform
{
public:
    explicit ContinuousUniform( double a, double b);
    ContinuousUniform (const ContinuousUniform & other) = delete;   //!< Copy constructor
    ContinuousUniform (const ContinuousUniform && other) = delete;  //!< Move constructor
    ContinuousUniform & operator = (const ContinuousUniform & other) = delete;  //!< Assignment operator
    ContinuousUniform & operator = (const ContinuousUniform && other) = delete;  //!< Move assignment operator

    ~ContinuousUniform() = default;

    [[nodiscard]] inline double pdf(  double x ) const;    //!< Probability density function
    [[nodiscard]] inline Prob cdf(  double x ) const;    //!< Cumulative distribution function
    [[nodiscard]] inline double icdf( Prob P) const;     //!< Inverse cumulative distribution function
    [[nodiscard]] double mean() const { return (a+b)/2; }  //!< Mean
    [[nodiscard]] double var() const { return (b-a)*(b-a)/12.; }  //!< Variance
    [[nodiscard]] inline double rnd() const;            //!< Random number

private:
    const double a;
    const double b;
};


inline double ContinuousUniform::icdf( const Prob P ) const
{
    return a +P()*(b-a);
}


inline Prob ContinuousUniform::cdf( const double x) const
{
    if ( x<a ) {
        return Prob(0);
    }
    if ( x>b ) {
        return Prob(1);
    }
    return Prob( (x-a)/(b-a) );
}


inline double ContinuousUniform::pdf( const double x) const
{
    if ( x<a ) {
        return 0;
    }
    if ( x>b ) {
        return 0;
    }
    return 1./(b -a);
}

//! uniformly distributed random number in [a,b], b>a
inline double ContinuousUniform::rnd() const
{
    // const double u = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    static thread_local std::mt19937 engine{ std::random_device{} () };
    static thread_local std::uniform_real_distribution<double> dist(0.0, 1.0);
    const double u = dist(engine);

    return a +u*(b -a);
}



inline ContinuousUniform::ContinuousUniform( const double a, const double b) : a(a), b(b)
{
    assert( b>a );
}


} // namespace Stats

#endif // UNIFORM_H
