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

#ifndef QUANTILES_H
#define QUANTILES_H

#include "statistics.h"


//! Quantiles for recognition and snapping
namespace Quantiles {

//! Quantiles for snapping of end-points
struct Snapping
{
public:
    //! Get quantile of standard normal distribution
    double quantile_stdNormDistr() const { return quantile_snd_; }

    //! Get quantile of chi-square distribution with two degeree of freedom
    double quantile_chi2_1dof() const { return quantile_chi2_1_; }
    void setAlpha( double alpha );  //!< Set significande level alpha

private:
    double quantile_snd_{};     // Quantile of standard normal distribution N(0,1)
    double quantile_chi2_1_{};  // Quantile of chi-squared distribution with 1 degree of freedom

    const Stats::ChiSquared     distr_chi2_1_{1};
    const Stats::StandardNormal distr_snd_{};
};

//! Quantiles for reconition of geometric relations
struct Recognition
{
public:
    //! Get quantile of chi-square distribution with one degeree of freedom
    double quantile_chi2_1dof() const { return quantile_chi2_1dof_; }

    //! Get quantile of chi-square distribution with two degeree of freedom
    double quantile_chi2_2dof() const { return quantile_chi2_2dof_; }
    void setAlpha( double alpha );  //!< Set significance level alpha

private:
    double quantile_chi2_1dof_{};
    double quantile_chi2_2dof_{};

    const Stats::ChiSquared  distr_chi2_1_{1};
    const Stats::Exponential distr_Exp_0p5_{0.5};
};

} // namespace Quantiles


#endif // QUANTILES_H
