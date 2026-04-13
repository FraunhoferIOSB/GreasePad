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

#ifndef PROB_H
#define PROB_H

#include <cassert>

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

} // namespace Stats

#endif // PROB_H
