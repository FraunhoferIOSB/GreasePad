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

#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H

#include <QDataStream>
#include <QDebug>

#include <Eigen/Dense>

#include <memory>

#include "global.h"

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::ColMajor;


namespace Constraint {


class ConstraintBase
{
public:
    ConstraintBase & operator= (ConstraintBase &&) = delete;
    ConstraintBase( ConstraintBase &&) = delete;
    ConstraintBase (const ConstraintBase & other) = delete;
    ConstraintBase & operator= (const ConstraintBase & other) = delete;

protected:
    ConstraintBase();
    virtual ~ConstraintBase() = default;

    static MatrixXd null( const VectorXd &xs );
    static MatrixXd Rot_ab( const VectorXd &a,
                            const VectorXd &b);
public:
    enum Status { UNEVAL=0, REQUIRED, OBSOLETE };

    virtual void serialize( QDataStream &out) const;  //  = 0
    static std::shared_ptr<ConstraintBase> deserialize( QDataStream &in );

    virtual const char *type_name() const = 0;
    virtual int dof()   const = 0;
    virtual int arity() const = 0;

    Status status() const    { return m_status;   }   //  serialzation
    bool enforced() const    { return m_enforced; }
    bool required() const    { return m_status==REQUIRED; }
    bool obsolete() const    { return m_status==OBSOLETE; }
    bool unevaluated() const { return m_status==UNEVAL;   }

    void setStatus( const Status s) { m_status = s;   }
    void setEnforced( const bool b) { m_enforced = b; }

    virtual MatrixXd Jacobian(   const VectorXi &idxx, const VectorXd &l0,  const VectorXd &l) const = 0;
    virtual VectorXd contradict( const VectorXi &idxx, const VectorXd &l0 ) const = 0;

    template<typename T>
    bool is()  { return ( dynamic_cast<T*>(this) != nullptr);    }

    virtual std::shared_ptr<ConstraintBase> clone() const = 0;    // virtual constructor (copying)

private:
    Status m_status;    // { UNEVAL=0 | REQUIRED | OBSOLETE };
    bool   m_enforced;
};





class Copunctual : public ConstraintBase
{
public:
    MatrixXd Jacobian(   const VectorXi &idxx, const VectorXd &l0, const VectorXd &l) const override;
    VectorXd contradict( const VectorXi &idxx, const VectorXd &l0) const override;
    int dof() const override   {return s_dof;}
    int arity() const override {return s_arity;}   //!< number of involved entities

protected:
    const char* type_name() const override { return "copunctual"; }

private:
    std::shared_ptr<ConstraintBase> clone() const override;
    Matrix3d cof3( const Matrix3d &MM ) const;   //!< 3x3 cofactor matrix
    static const int s_dof   = 1;
    static const int s_arity = 3;
public:
    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Copunctual>();
    }
};


class Parallel : public ConstraintBase
{
public:
    MatrixXd Jacobian(   const VectorXi &idxx, const VectorXd &l0,  const VectorXd &l) const override;
    VectorXd contradict( const VectorXi &idxx, const VectorXd &l0) const override;
    int dof() const override   { return s_dof;   }
    int arity() const override { return s_arity; }

    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Parallel>();
    }

private:
    const char* type_name() const override { return "parallel"; }
    std::shared_ptr<ConstraintBase> clone() const override;

    static Eigen::Matrix3d S3();
    static const int s_dof   = 1;
    static const int s_arity = 2;
};

class Orthogonal : public ConstraintBase
{
public:
    MatrixXd Jacobian(   const VectorXi &idxx,  const VectorXd &l0,  const VectorXd &l) const override;
    VectorXd contradict( const VectorXi &idxx,  const VectorXd &l0) const override;
    int dof() const override   { return s_dof;   }
    int arity() const override { return s_arity; }

    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Orthogonal>();
    }
private:
    const char* type_name() const override { return "orthogonal"; }
    std::shared_ptr<ConstraintBase> clone() const override;

    static Matrix3d CC();
    static const int s_dof = 1;
    static const int s_arity = 2;
};

class Identical : public ConstraintBase
{
public:
    MatrixXd Jacobian( const VectorXi &idxx, const VectorXd &l0, const VectorXd &l) const override;
    VectorXd contradict( const VectorXi &idxx, const VectorXd &l0) const override;
    int dof() const override   {return s_dof;}
    int arity() const override {return s_arity;}

private:
    const char* type_name() const override {return "identical";}
    std::shared_ptr<ConstraintBase> clone() const override;

    static const int s_dof   = 2;
    static const int s_arity = 2;

    template <typename T>
    inline bool sameSign( T a, T b ) const {  return a*b >= 0.; }   // for debugging and assertion

    template <typename T>
    inline int sign(T val) const { return (T(0) < val) - (val < T(0));  }
};

} // namespace Constraint

#endif // CONSTRAINTS_H
