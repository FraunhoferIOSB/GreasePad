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


#include "matfun.h"
#include "uncertain.h"

#include <cassert>
#include <cfloat>

#include <Eigen/Core>


using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Vector2d;

using Matfun::null;
using Matfun::sign;


namespace Uncertain {


//! Spherically normalize the entity
void BasicEntity2D::normalizeSpherical()
{
    // qDebug() << Q_FUNC_INFO;

    const Matrix3d I = Matrix3d::Identity();
    assert(m_val.norm() > 0.0);
    const Matrix3d Jac = (I - m_val*m_val.adjoint()/m_val.squaredNorm()) / m_val.norm();
    m_cov = Jac*m_cov*Jac.adjoint();
    m_val.normalize();  // x = x /norm(x)
}


//! Check if uncertainty entity is identical with uncertain entity 'us'
bool BasicEntity2D::isIdenticalTo( const BasicEntity2D & us,
                                   const double T) const
{
    BasicEntity2D a(*this);
    BasicEntity2D b(us);      // copies

    // fix ambiguities
    a.normalizeSpherical();
    b.normalizeSpherical();

    int idx = 0;                       // visitor
    a.v().cwiseAbs().maxCoeff( &idx ); // [~,idx] = max( abs(a) )
    a.m_val *= sign( a.v()(idx) );      // a = a*sign( a(idx) );
    b.m_val *= sign( b.v()(idx) );      // b = b*sign( b(idx) );

    const Matrix<double, 3, 2> Jac = null(a.v());         // (A.120)
    const Vector2d d = Jac.adjoint()*( a.v() -b.v() );     // (10.141)
    const Matrix2d Sigma_dd = Jac.adjoint() * (a.Cov() + b.Cov()) * Jac;

    return d.dot( Sigma_dd.ldlt().solve(d) ) < T;    // dof = 2
}

} // namespace Uncertain
