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


//! Geometric constraints (relations)
namespace Constraint {

//! Base class for constraints
class ConstraintBase
{
public:
    ConstraintBase & operator= ( ConstraintBase &&) = delete;
    ConstraintBase( ConstraintBase &&) = delete;
    ConstraintBase (const ConstraintBase & other) = delete;
    ConstraintBase & operator= ( const ConstraintBase & other) = delete;

protected:
    ConstraintBase();
    virtual ~ConstraintBase() = default;

    static MatrixXd null( const VectorXd &xs );  //!< Nullspace of row vector
    static MatrixXd Rot_ab( const VectorXd &a,
                            const VectorXd &b);  //!< Minimal rotation between two vectors as rotation matrix
public:
    //! Status: unevaluated, required, obsolete (redundant)
    enum Status { UNEVAL=0, REQUIRED, OBSOLETE };

    virtual void serialize( QDataStream &out) const;  //!< Serialize (Qt)
    static std::shared_ptr<ConstraintBase> deserialize( QDataStream &in ); //!< Deserialization (Qt)

    virtual const char *type_name() const = 0;  //!< Get type name of constraint
    virtual int dof()   const = 0;   //!< Get degrees of freedom for this constraint
    virtual int arity() const = 0;   //!< Get number of involved entities, i.e., straight lines

    Status status() const    { return m_status;   }         //!< Get status (required, obsolete, unevaluated)
    bool enforced() const    { return m_enforced; }         //!< Constraint is enforced?
    bool required() const    { return m_status==REQUIRED; } //!< Constraint is required?
    bool obsolete() const    { return m_status==OBSOLETE; } //!< Constraint is obsolete (redundant)?
    bool unevaluated() const { return m_status==UNEVAL;   } //!< Constraint is unevaluated?

    void setStatus( const Status s) { m_status = s;   }     //!< Set status (required, obsolete, unevaluated)
    void setEnforced( const bool b) { m_enforced = b; }     //!< Set status success of enforcement

    //! Compute Jacobian w.r.t. observations
    virtual MatrixXd Jacobian(   const VectorXi &idxx,
                                 const VectorXd &l0,  const VectorXd &l) const = 0;
    //! Compute contradictions with adjusted observations
    virtual VectorXd contradict( const VectorXi &idxx,
                                 const VectorXd &l0 ) const = 0;

    //! Check if constraint is of a certain, specified type
    template<typename T>
    bool is()  { return ( dynamic_cast<T*>(this) != nullptr);    }

    //! Clone constrints via nonvirtual interface pattern
    std::shared_ptr<ConstraintBase> clone() const;

private:
    virtual std::shared_ptr<ConstraintBase> doClone() const = 0;

    Status m_status;    // { UNEVAL=0 | REQUIRED | OBSOLETE };
    bool   m_enforced;
};


//! Concurrence constraint (three copunctual straight lines)
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
    std::shared_ptr<ConstraintBase> doClone() const override;
    Matrix3d cof3( const Matrix3d &MM ) const;   //!< 3x3 cofactor matrix
    static const int s_dof   = 1;
    static const int s_arity = 3;
public:
    //! Create concurrence constraint
    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Copunctual>();
    }
};

//! Parallelism constraint
class Parallel : public ConstraintBase
{
public:
    MatrixXd Jacobian(   const VectorXi &idxx, const VectorXd &l0,  const VectorXd &l) const override;
    VectorXd contradict( const VectorXi &idxx, const VectorXd &l0) const override;
    int dof() const override   { return s_dof;   }
    int arity() const override { return s_arity; }

    //! Create parallelism constraint
    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Parallel>();
    }

private:
    const char* type_name() const override { return "parallel"; }
    std::shared_ptr<ConstraintBase> doClone() const override;

    static Eigen::Matrix3d S3();
    static const int s_dof   = 1;
    static const int s_arity = 2;
};

//! Orthogonallity constraint
class Orthogonal : public ConstraintBase
{
public:
    MatrixXd Jacobian(   const VectorXi &idxx,  const VectorXd &l0,  const VectorXd &l) const override;
    VectorXd contradict( const VectorXi &idxx,  const VectorXd &l0) const override;
    int dof() const override   { return s_dof;   }
    int arity() const override { return s_arity; }

    //! Create orthogonallity constraint
    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Orthogonal>();
    }
private:
    const char* type_name() const override { return "orthogonal"; }
    std::shared_ptr<ConstraintBase> doClone() const override;

    static Matrix3d CC();
    static const int s_dof = 1;
    static const int s_arity = 2;
};

//! Identity constraint
class Identical : public ConstraintBase
{
public:
    MatrixXd Jacobian( const VectorXi &idxx, const VectorXd &l0, const VectorXd &l) const override;
    VectorXd contradict( const VectorXi &idxx, const VectorXd &l0) const override;
    int dof() const override   {return s_dof;}
    int arity() const override {return s_arity;}

private:
    const char* type_name() const override {return "identical";}
    std::shared_ptr<ConstraintBase> doClone() const override;

    static const int s_dof   = 2;
    static const int s_arity = 2;

    template <typename T>
    inline bool sameSign( T a, T b ) const {  return a*b >= 0.; }   // for debugging and assertion

    template <typename T>
    inline int sign(T val) const { return (T(0) < val) - (val < T(0));  }
};

} // namespace Constraint

#endif // CONSTRAINTS_H
