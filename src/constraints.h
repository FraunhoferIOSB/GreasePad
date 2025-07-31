/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2025 Jochen Meidow, Fraunhofer IOSB
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

#include <QDebug>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <memory>

#include "matrix.h"

//! Geometric constraints (relations)
namespace Constraint {

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::VectorXidx;

//! Base class for constraints
class ConstraintBase
{
public:
    ConstraintBase & operator= ( ConstraintBase &&) = delete;
    ConstraintBase( ConstraintBase &&) = delete;
    ConstraintBase (const ConstraintBase & other) = delete;
    ConstraintBase & operator= ( const ConstraintBase & other) = delete;

    ConstraintBase();
    virtual ~ConstraintBase() = default;

protected:
    static MatrixXd null( const VectorXd &xs );  //!< Nullspace of row vector
    static MatrixXd Rot_ab( const VectorXd &a,
                            const VectorXd &b);  //!< Minimal rotation between two vectors as rotation matrix
public:
    //! Status: unevaluated, required, obsolete (redundant)
    enum Status { UNEVAL=0, REQUIRED, OBSOLETE };

    // virtual void serialize( QDataStream &out) const;  //!< Serialize (Qt)
    // static std::shared_ptr<ConstraintBase> deserialize( QDataStream &in ); //!< Deserialization (Qt)

    [[nodiscard]] virtual const char *type_name() const = 0;  //!< Get type name of constraint
    [[nodiscard]] virtual int dof()   const = 0;   //!< Get degrees of freedom for this constraint
    [[nodiscard]] virtual int arity() const = 0;   //!< Get number of involved entities, i.e., straight lines

    [[nodiscard]] Status status() const    { return m_status;   }         //!< Get status (required, obsolete, unevaluated)
    [[nodiscard]] bool enforced() const    { return m_enforced; }         //!< Constraint is enforced?
    [[nodiscard]] bool required() const    { return m_status==REQUIRED; } //!< Constraint is required?
    [[nodiscard]] bool obsolete() const    { return m_status==OBSOLETE; } //!< Constraint is obsolete (redundant)?
    [[nodiscard]] bool unevaluated() const { return m_status==UNEVAL;   } //!< Constraint is unevaluated?

    void setStatus( const Status s) { m_status = s;   }     //!< Set status (required, obsolete, unevaluated)
    void setEnforced( const bool b) { m_enforced = b; }     //!< Set status success of enforcement

    //! Compute Jacobian w.r.t. observations
    [[nodiscard]] virtual MatrixXd Jacobian(   const Eigen::VectorXidx &idxx,
                                 const VectorXd &l0,  const VectorXd &l) const = 0;
    //! Compute contradictions with adjusted observations
    [[nodiscard]] virtual VectorXd contradict( const VectorXidx &idxx,
                                 const VectorXd &l0 ) const = 0;

    //! Check if constraint is of a certain, specified type
    template<typename T>
    bool is()  { return ( dynamic_cast<T*>(this) != nullptr);    }

    //! Clone constraints via nonvirtual interface pattern
    [[nodiscard]] std::shared_ptr<ConstraintBase> clone() const;

private:
    [[nodiscard]] virtual std::shared_ptr<ConstraintBase> doClone() const = 0;

    Status m_status;    // { UNEVAL=0 | REQUIRED | OBSOLETE };
    bool   m_enforced;

protected:
    template <typename T>
    int sign(T val) const { return (T(0) <= val) - (val < T(0));  }
};


//! Concurrence constraint (three copunctual straight lines)
class Copunctual : public ConstraintBase
{
public:
    [[nodiscard]] MatrixXd Jacobian(   const VectorXidx &idxx, const VectorXd &l0, const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict( const VectorXidx &idxx, const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   {return s_dof;}
    [[nodiscard]] int arity() const override {return s_arity;}   //!< number of involved entities

protected:
    [[nodiscard]] const char* type_name() const override { return "copunctual"; }

private:
    [[nodiscard]] std::shared_ptr<ConstraintBase> doClone() const override;
    static Matrix3d cof3(const Matrix3d &MM); //!< 3x3 cofactor matrix
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
    [[nodiscard]] MatrixXd Jacobian(   const VectorXidx &idxx, const VectorXd &l0,  const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict( const VectorXidx &idxx, const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

    //! Create parallelism constraint
    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Parallel>();
    }

private:
    [[nodiscard]] const char* type_name() const override { return "parallel"; }
    [[nodiscard]] std::shared_ptr<ConstraintBase> doClone() const override;

    static Eigen::Matrix3d S3();
    static const int s_dof   = 1;
    static const int s_arity = 2;
};

//! Vertical straight line
class Vertical : public ConstraintBase
{
public:
    [[nodiscard]] MatrixXd Jacobian(   const VectorXidx &idxx,  const VectorXd &l0,  const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict( const VectorXidx &idxx,  const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

    //! Create constraint
    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Vertical>();
    }
private:
    [[nodiscard]] const char* type_name() const override { return "vertical"; }
    [[nodiscard]] std::shared_ptr<ConstraintBase> doClone() const override;

    static const int s_dof = 1;
    static const int s_arity = 1;

    static Vector3d e2(); //!< [0,1,0]'
};


//! Horizontal straight line
class Horizontal : public ConstraintBase
{
public:
    [[nodiscard]] MatrixXd Jacobian(   const VectorXidx &idxx,  const VectorXd &l0,  const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict( const VectorXidx &idxx,  const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

    //! Create constraint
    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Horizontal>();
    }
private:
    [[nodiscard]] const char* type_name() const override { return "horizontal"; }
    [[nodiscard]] std::shared_ptr<ConstraintBase> doClone() const override;

    static const int s_dof = 1;
    static const int s_arity = 1;

    static Vector3d e1(); //!< [1,0,0]'
};


//! Diagonal straight line
class Diagonal : public ConstraintBase
{
public:
    [[nodiscard]] MatrixXd Jacobian(   const VectorXidx &idxx,  const VectorXd &l0,  const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict( const VectorXidx &idxx,  const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

    //! Create constraint
    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Diagonal>();
    }
private:
    [[nodiscard]] const char* type_name() const override { return "diagonal"; }
    [[nodiscard]] std::shared_ptr<ConstraintBase> doClone() const override;

    static const int s_dof = 1;
    static const int s_arity = 1;
};


//! Orthogonallity constraint
class Orthogonal : public ConstraintBase
{
public:
    [[nodiscard]] MatrixXd Jacobian(   const VectorXidx &idxx,  const VectorXd &l0,  const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict( const VectorXidx &idxx,  const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

    //! Create orthogonallity constraint
    static std::shared_ptr<ConstraintBase> create() {
        return std::make_shared<Orthogonal>();
    }
private:
    [[nodiscard]] const char* type_name() const override { return "orthogonal"; }
    [[nodiscard]] std::shared_ptr<ConstraintBase> doClone() const override;

    static Matrix3d CC();
    static const int s_dof = 1;
    static const int s_arity = 2;
};


//! Identity constraint
class Identical : public ConstraintBase
{
public:
    [[nodiscard]] MatrixXd Jacobian( const VectorXidx &idxx, const VectorXd &l0, const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict( const VectorXidx &idxx, const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   {return s_dof;}
    [[nodiscard]] int arity() const override {return s_arity;}

private:
    [[nodiscard]] const char* type_name() const override {return "identical";}
    [[nodiscard]] std::shared_ptr<ConstraintBase> doClone() const override;

    static const int s_dof   = 2;
    static const int s_arity = 2;

    template <typename T>
    bool sameSign( T a, T b ) const {  return a*b >= 0.; }   // for debugging and assertion
};

} // namespace Constraint

#endif // CONSTRAINTS_H
