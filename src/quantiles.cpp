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

#include "quantiles.h"
#include <cassert>

void Quantiles::Recognition::setAlpha( const double alpha )
{
    assert( alpha>=0. && alpha <= 1. && "invalid significance level");
    quantile_chi2_1dof_ =  distr_chi2_1_.icdf( 1.-alpha );
    quantile_chi2_2dof_ = distr_Exp_0p5_.icdf( 1.-alpha );
}

void Quantiles::Snapping::setAlpha( const double alpha )
{
    assert( alpha>=0. && alpha <= 1. && "invalid significance level");
    quantile_snd_    =    distr_snd_.icdf( 1.-alpha);
    quantile_chi2_1_ = distr_chi2_1_.icdf( 1.-alpha);
}
