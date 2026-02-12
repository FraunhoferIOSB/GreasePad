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

#ifndef CONSTRAINTS_H
#define CONSTRAINTS_H


#include <Eigen/Core>
#include <Eigen/Dense>

#include <functional>
#include <map>
#include <memory>


//! Geometric constraints (relations)
namespace Constraint {

using Eigen::MatrixXd;
using Eigen::Dynamic;
using Eigen::Index;
using Eigen::Vector;
using Eigen::VectorXd;


//! Base class for constraints
class ConstraintBase
{
public:
    ConstraintBase & operator= ( ConstraintBase &&) = delete;
    ConstraintBase( ConstraintBase &&) = delete;
    ConstraintBase (const ConstraintBase & other) = default;
    ConstraintBase & operator= ( const ConstraintBase & other) = delete;

    ConstraintBase();
    virtual ~ConstraintBase() = default;

    //! Status: unevaluated, required, obsolete (redundant)
    enum Status : int { UNEVAL=0, REQUIRED, OBSOLETE };

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
    [[nodiscard]] virtual MatrixXd Jacobian(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0,
        const VectorXd &l) const = 0;

    //! Compute contradictions with adjusted observations
    [[nodiscard]] virtual VectorXd contradict(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0 ) const = 0;

    //! Check if constraint is of a certain, specified type
    template<typename T>
    [[nodiscard]] bool isInstanceOf() const {
        return ( dynamic_cast<T const*>(this) != nullptr);
    }

    //! Clone constraints via nonvirtual interface pattern
    [[nodiscard]] virtual std::shared_ptr<ConstraintBase> clone() const = 0;

private:
    Status m_status;    // { UNEVAL=0 | REQUIRED | OBSOLETE };
    bool   m_enforced;
};


/*
 * The curiously recurring template pattern (CRTP)
 * is used to enable static polymorphism
 * and to avoid duplicate clone functions in the derived classes.
 */
template <typename Derived>
class ConstraintCRTP : public ConstraintBase
{
public:
    [[nodiscard]] std::shared_ptr<ConstraintBase> clone() const override {
        return std::make_shared<Derived>(static_cast<Derived const&> (*this));
    };

    ConstraintCRTP & operator= (const ConstraintCRTP &) = delete;
    ConstraintCRTP & operator= (ConstraintCRTP &&) = delete;
    ~ConstraintCRTP() override = default;

private:
    ConstraintCRTP( const ConstraintCRTP &) = default;
    ConstraintCRTP() = default;
    ConstraintCRTP(ConstraintCRTP&&) noexcept = default;
    friend Derived;
};

//! Concurrence constraint (three copunctual straight lines)
class Copunctual : public ConstraintCRTP<Copunctual>
{
public:
    [[nodiscard]] MatrixXd Jacobian(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0,
        const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   {return s_dof;}
    [[nodiscard]] int arity() const override {return s_arity;}   //!< number of involved entities

private:
    [[nodiscard]] const char* type_name() const override { return "copunctual"; }

    static const int s_dof   = 1;
    static const int s_arity = 3;
};


//! Parallelism constraint
class Parallel : public ConstraintCRTP<Parallel>
{
public:
    [[nodiscard]] MatrixXd Jacobian(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0,
        const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

private:
    [[nodiscard]] const char* type_name() const override { return "parallel"; }

    static const int s_dof   = 1;
    static const int s_arity = 2;
};


//! Vertical straight line
class Vertical : public ConstraintCRTP<Vertical>
{
public:
    [[nodiscard]] MatrixXd Jacobian(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0,
        const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

private:
    [[nodiscard]] const char* type_name() const override { return "vertical"; }

    static const int s_dof = 1;
    static const int s_arity = 1;
};


//! Horizontal straight line
class Horizontal : public ConstraintCRTP<Horizontal>
{
public:
    [[nodiscard]] MatrixXd Jacobian(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0,
        const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

private:
    [[nodiscard]] const char* type_name() const override { return "horizontal"; }

    static const int s_dof = 1;
    static const int s_arity = 1;
};


//! Diagonal straight line
class Diagonal : public ConstraintCRTP<Diagonal>
{
public:
    [[nodiscard]] MatrixXd Jacobian(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0,
        const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

private:
    [[nodiscard]] const char* type_name() const override { return "diagonal"; }

    static const int s_dof = 1;
    static const int s_arity = 1;
};


//! Orthogonality constraint
class Orthogonal : public ConstraintCRTP<Orthogonal>
{
public:
    [[nodiscard]] MatrixXd Jacobian(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0,
        const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   { return s_dof;   }
    [[nodiscard]] int arity() const override { return s_arity; }

private:
    [[nodiscard]] const char* type_name() const override { return "orthogonal"; }

    static const int s_dof = 1;
    static const int s_arity = 2;
};


/*
//! Identity constraint
class Identical : public ConstraintCRTP<Identical>
{
public:
    [[nodiscard]] MatrixXd Jacobian(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0,
        const VectorXd &l) const override;
    [[nodiscard]] VectorXd contradict(
        const Vector<Index,Dynamic> &idxx,
        const VectorXd &l0) const override;
    [[nodiscard]] int dof() const override   {return s_dof;}
    [[nodiscard]] int arity() const override {return s_arity;}

private:
    [[nodiscard]] const char* type_name() const override {return "identical";}

    static const int s_dof   = 2;
    static const int s_arity = 2;
};
*/


//! Factory for constraints, as singleton
class Factory {

private:
    std::map<char, std::function <std::shared_ptr<ConstraintBase>()> > m_map;

    ~Factory() { m_map.clear();  }

public:
    Factory(const Factory & other) = delete;  // not clonable
    Factory(Factory &&  other) = delete; // not movable
    Factory & operator= (Factory && other) = delete;
    Factory & operator= (const Factory & other) = delete;

    Factory() {
        // qDebug() << Q_FUNC_INFO;
        // qDebug() << typeid(Horizontal).name();
        // qDebug() << typeid(Horizontal).hash_code();
        m_map['h'] = [] {return std::make_shared<Horizontal>(); };
        m_map['v'] = [] {return std::make_shared<Vertical>();   };
        m_map['d'] = [] {return std::make_shared<Diagonal>();   };
        m_map['o'] = [] {return std::make_shared<Orthogonal>(); };
        m_map['c'] = [] {return std::make_shared<Copunctual>(); };
        m_map['p'] = [] {return std::make_shared<Parallel>();   };
    }

    static Factory *getInstance(){
        static Factory instance;
        return & instance;
    }

    std::shared_ptr<ConstraintBase> create(const char c) {
        // assert( m_map.find(c) != m_map.end() && "unknown key");  // C++20: contains(c)
        if ( m_map.find(c) == m_map.end() ) {
            return nullptr;
        }
        return m_map[c]();
    }
};

} // namespace Constraint

#endif // CONSTRAINTS_H
