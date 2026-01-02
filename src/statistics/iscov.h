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

#ifndef ISCOV_H
#define ISCOV_H


#include <Eigen/Core>
#include <Eigen/Eigenvalues>


namespace Stats {

using Eigen::MatrixXd;


//! Check if matrix MM is a proper covariance matrix
[[maybe_unused, nodiscard]] static bool isCovMat( const MatrixXd & MM )
{
    // qDebug() << Q_FUNC_INFO;
    const Eigen::SelfAdjointEigenSolver<MatrixXd> eig( MM, Eigen::ComputeEigenvectors);
    Eigen::VectorXcd ev = eig.eigenvalues();

    constexpr double threshold = 1e-6;
#ifdef QT_DEBUG
    if ( (ev.real().array() < -threshold ).any() ) {
        for ( Eigen::Index i=0; i< ev.size(); i++) {
            qDebug().noquote() << QStringLiteral( "(%1,%2)")
            .arg( ev(i).real() )
                .arg( ev(i).imag() );
        }
    }
#endif

    return ( ev.real().array() >= -threshold ).all();
}

} // namespace Stats

#endif // ISCOV_H
