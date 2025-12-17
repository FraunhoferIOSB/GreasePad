#ifndef UNIFORM_H
#define UNIFORM_H

#include <cassert>
#include <cstdlib>

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

    [[nodiscard]] double pdf(  double x ) const;    //!< Probability density function
    [[nodiscard]] Prob cdf(  double x ) const;    //!< Cumulative distribution function
    [[nodiscard]] double icdf( Prob P) const;     //!< Inverse cumulative distribution function
    [[nodiscard]] double mean()   const { return (a+b)/2; }  //!< Mean
    [[nodiscard]] double var()    const { return (b-a)*(b-a)/12.; }  //!< Variance
    [[nodiscard]] double rnd()    const;            //!< Random number

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
    const double u = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    return a +u*(b -a);
}



inline ContinuousUniform::ContinuousUniform( const double a, const double b) : a(a), b(b)
{
    assert( b>a );
}



} // namespace Stats

#endif // UNIFORM_H
