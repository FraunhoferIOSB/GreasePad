#ifndef PROB_H
#define PROB_H

#include <cassert>

namespace Stats {

//! Probability [0,1]
class Prob
{
public:
    explicit Prob( const double P) : P(P) {
        assert( P>=0 );
        assert( P<=1 );
    }
    double operator()() const { return P; }  //!< get value
    [[nodiscard]] Prob complement() const { return Prob(1-P); }  //!< 1-P

private:
    double P;  // probability [0,1]
};

} // namespace Stats

#endif // PROB_H
