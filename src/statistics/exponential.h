#ifndef EXPONENTIAL_H
#define EXPONENTIAL_H

#include <cassert>
#include <cmath>

#include "statistics/prob.h"
#include "statistics/uniform.h"


namespace Stats {


//! Exponential distribution
class Exponential
{
public:
    explicit Exponential( double lambda);         //!< Value constructor
    Exponential( const Exponential &  other) = delete;   //!< Copy constructor
    Exponential( const Exponential && other) = delete;  //!< Move constructor
    Exponential & operator = (const Exponential &  other) = delete;  //!< Assignment operator
    Exponential & operator = (const Exponential && other) = delete;  //!< Move assignment operator

    ~Exponential() = default;

    //! Probability density function
    [[nodiscard]] double pdf( const double x) const {
        return x>=0 ? lambda*exp(-lambda*x) : 0.;
    }

    //! Cumulative distribution function
    [[nodiscard]] Prob cdf( const double x) const {
        return x>=0 ? Prob( 1.-exp(-lambda*x)) : Prob(0);
    }

    //! Inverse cumulative distribution function
    [[nodiscard]]  double icdf( const Prob P) const {
        return -log( 1.-P() )/lambda;
    }

    [[nodiscard]] double mean()   const { return 1./lambda; }  //!< mean
    [[nodiscard]] double var()    const { return 1./(lambda*lambda); } //!< Variance
    [[nodiscard]] static double mode() { return 0.; }          //!< mode
    [[nodiscard]] double rnd()    const;  //!< Random number

    [[nodiscard]] double rate()  const {return lambda; }  //!< Get rate (inverse scale)
    [[nodiscard]] double scale() const {return 1./lambda;}   //!< Get scale (inverse rate)

private:
    const double lambda;  // rate, inverse scale
};


inline Exponential::Exponential( const double lambda)
    : lambda(lambda)
{
    assert( lambda>0 );
}


inline double Exponential::rnd() const
{
    const ContinuousUniform uniform(0,1);
    const double u = uniform.rnd();  // ~U(0,1)

    // assert( u>0.0) ;
    return -log(u)/lambda;
}

} // namespace Stats

#endif // EXPONENTIAL_H
