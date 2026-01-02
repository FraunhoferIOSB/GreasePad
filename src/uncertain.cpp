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
#include "usegment.h"

#include <QDebug>
#include <QStringLiteral>

#include <cassert>
#include <cfloat>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>


namespace Uncertain {

using Matfun::null;
using Matfun::sign;



//! Spherically normalize the entity
void BasicEntity2D::normalizeSpherical()
{
    // qDebug() << Q_FUNC_INFO;

    const Matrix3d I = Matrix3d::Identity();
    assert(m_val.norm() > 0.0);
    Matrix3d const Jac = (I - m_val*m_val.adjoint()/m_val.squaredNorm()) / m_val.norm();
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

    Matrix<double, 3, 2> const JJ = null(a.v());          // (A.120)
    Vector2d const d = JJ.adjoint()*( a.v() -b.v() );     // (10.141)
    Eigen::Matrix2d const Cov_dd = JJ.adjoint() * (a.Cov() + b.Cov()) * JJ;

    return d.dot(Cov_dd.ldlt().solve(d) ) < T;    // dof = 2
}

} // namespace Uncertain
