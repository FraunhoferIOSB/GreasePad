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

#ifndef UDISTANCE_H
#define UDISTANCE_H

//#include <Eigen/Core>
//#include <Eigen/Dense>

#include <cassert>
#include <cmath>


namespace Uncertain {

//! Uncertain distance
class uDistance
{
public:
    //! Construct uncertain distance with distance and its variance
    uDistance( const double d, const double var_d) : m_d(d), m_var_d(var_d) {
        assert( var_d >= 0);
    }

    //! Get variance of distance
    [[nodiscard]] double var_d() const { return m_var_d; }

    //! Get distance
    [[nodiscard]] double d() const { return m_d; }

    //! Check if distance is greater than zero.
    [[nodiscard]] bool isGreaterThanZero( const double T) const {
        if (m_var_d <= 0.0) {
            return false;
        }
        return m_d / std::sqrt(m_var_d) > -T;
    }

    //! Check if distance is less than zero.
    [[nodiscard]] bool isLessThanZero( const double T) const {
        if (m_var_d <= 0.0) {
            return false;
        }
        return m_d / std::sqrt(m_var_d) < +T;
    }

private:
    double     m_d;
    double m_var_d;
};


} // namespace Uncertain

#endif // UDISTANCE_H
