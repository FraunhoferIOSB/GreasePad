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

#ifndef QCONSTRAINTS_H
#define QCONSTRAINTS_H

#include <QDebug>
#include <QGraphicsItem>
#include <QPen>
#include <QWidget>

#include "qcontainerfwd.h"
#include "qsharedpointer.h"
#include "qtypes.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <functional>
#include <map>
#include <memory>
#include <string>



namespace Uncertain {
class uStraightLineSegment;
} // namespace Uncertain


//! Graphics: Constraints
namespace QConstraint {

using Uncertain::uStraightLineSegment;
using VectorXidx = Eigen::Vector<Eigen::Index,Eigen::Dynamic>;


//! Graphics: Base class for markers depicting constraints
class QConstraintBase : public QGraphicsItem
{
public:
    QConstraintBase( const QConstraintBase && other)= delete;  //!< Move constructor
    QConstraintBase & operator= ( const QConstraintBase &) = delete ; //!< Copy assignment operator
    QConstraintBase & operator= ( QConstraintBase &&) = delete;       //!< Move assignment operator

    //! Clone this constraint via nonvirtual interface pattern
    [[nodiscard]] std::shared_ptr<QConstraintBase> clone() const;

    // T qgraphicsitem_cast(QGraphicsItem *item)
    // To make this function work correctly with custom items, reimplement the type() function for each custom QGraphicsItem subclass.
    enum {Type = UserType +364};
    [[nodiscard]] int type() const override { return Type; }   //!< Get graphics type (id)

    void serialize(   QDataStream &out );  //!< serialization
    bool deserialize( QDataStream &);      //!< deserialization

    void setColor( const QColor & col);    //!< Set color
    void setLineWidth( int w);       //!< Set line width
    void setLineStyle( int s);       //!< Set line style
    void setAltColor( const QColor &col ); //!< Set color for automatic colorization
    void setStatus( bool isrequired, bool isenforced);      //!< Set status

    virtual void setMarkerSize ( qreal s) = 0;  //!< Set marker size
    virtual void setGeometry( QVector<std::shared_ptr< const uStraightLineSegment>> &,
                              const VectorXidx &idx) = 0; //!< Set geometry

    static QPen defaultPenReq() { return s_defaultPenReq; }  //!< Get default pen for required constraints
    static QPen defaultPenRed() { return s_defaultPenRed; }  //!< Get default pen for redundant constraints

    static void setDefaultPenReq( const QPen &p) { s_defaultPenReq = p; }  //!< Set default pen for required constraints
    static void setDefaultPenRed( const QPen &p) { s_defaultPenRed = p; }  //!< Set default pen for redundant constraints
    static void setPenSelected(   const QPen &p) { s_penSelected = p;}     //!< Set pen for selection
    static void setDefaultMarkerSize( int s) { s_defaultMarkerSize = s; }  //!< Set default marker size

    static void toggleShow()      { s_show = !s_show; }  //!< Toggle visibility
    static void toggleShowColor() { s_showColor =! s_showColor; }  //!< Toggle colorization (on/off)
    static bool show() {return s_show;}  //!< Get status visibility

protected:
    QConstraintBase();  //!< standrad constructor
    QConstraintBase( const QConstraintBase & other);   //!< Copy constructor

    ~QConstraintBase() override = default;

    [[nodiscard]] virtual qreal markerSize() const = 0;  //!< Set marker size
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;  //!< Plot marker

    void mousePressEvent(QGraphicsSceneMouseEvent * /* event */) override; //!< Handle mouse press event

    static bool showColor() { return s_showColor; } //!< Get status automatic colorization (on/off)

    static  int s_defaultMarkerSize;  //!< Default marker size
    static QPen s_defaultPenReq;      //!< Default pen for required constraints
    static QPen s_defaultPenRed;      //!< Default pen for redundant constraints
    static QPen s_penSelected;        //!< Pen for selection

    QColor m_altColor;        //!< Color for automatic colorization (subtasks)
    QPen m_pen_req;           //!< Pen for required constraint
    QPen m_pen_red;           //!< Pen for redundant constraint

    [[nodiscard]] bool enforced() const { return m_is_enforced; }  //!< Get status enforcment (success/failure)
    [[nodiscard]] bool required() const {return m_is_required;}

    static constexpr double m_sc = 1000; //!< Scale factor coordinates screen // TODO(meijoc)

private:
    [[nodiscard]] virtual std::shared_ptr<QConstraintBase> doClone() const = 0;

    bool m_is_required = true;    //!< Constraint is required? (for painting)
    bool m_is_enforced = false;   //!< Constraint is enforced? (for painting)

    static bool s_showColor;
    static bool s_show;
};

//! Graphics: Three copunctual straight lines (circle)
class QCopunctual : public QConstraintBase,
        public QGraphicsEllipseItem
{
public:
    static std::shared_ptr<QConstraintBase> create();  //!< Create copunctual constraint

    void setGeometry( QVector<std::shared_ptr<const uStraightLineSegment>> &s,
                      const VectorXidx &idx) override;
    void setMarkerSize ( qreal s) override;
   [[nodiscard]] qreal markerSize() const override;

protected:
    QCopunctual();
    QCopunctual( const QCopunctual & other);    //!< Copy constructor

    [[nodiscard]] QRectF boundingRect() const override; //!< Get axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override; //!< Plot the circle

private:
    [[nodiscard]] std::shared_ptr<QConstraintBase> doClone() const override;
};


//! Vertical, horizontal or diagonal straight line
class QAligned : public QConstraintBase
{
public:
    static std::shared_ptr<QConstraintBase> create();

    void setGeometry( QVector<std::shared_ptr<const uStraightLineSegment>> &s,
                      const VectorXidx & idx) override;
    void setMarkerSize ( qreal s) override;
    [[nodiscard]] qreal markerSize() const override;

protected:
    QAligned();
    QAligned( const QAligned & other); //!< Copy constructor

    [[nodiscard]] QRectF boundingRect() const override;   //!< Get axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;  //!< Plot line
private:
    [[nodiscard]] std::shared_ptr<QConstraintBase> doClone() const override;
    QGraphicsLineItem a;
};



//! Graphics: Two orthogonal straight lines (square)
class QOrthogonal : public QConstraintBase,
        public QGraphicsRectItem
{
public:
    static std::shared_ptr<QConstraintBase> create();  //!< Create orthogonallity constraint

    void setGeometry( QVector<std::shared_ptr<const uStraightLineSegment>> &s,
                      const VectorXidx & idx) override;
    void setMarkerSize ( qreal s) override;
    [[nodiscard]] qreal markerSize() const override;

protected:
    QOrthogonal();
    QOrthogonal( const QOrthogonal & other); //!< Copy constructor

    [[nodiscard]] QRectF boundingRect() const override;   //!< Get axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;  //!< Plot square
private:
    [[nodiscard]] std::shared_ptr<QConstraintBase> doClone() const override;
};


//! Graphics: Marker for identity
/*
class QIdentical : public QConstraintBase,
        public QGraphicsPolygonItem
{
public:
    static std::shared_ptr<QConstraintBase> create();  //!< Create identity constraint

    void setMarkerSize ( qreal s) override;
    [[nodiscard]] qreal markerSize() const override;
    void setGeometry( QVector<std::shared_ptr<const uStraightLineSegment>> &s,
                      const VectorXidx &idx) override;

protected:
    QIdentical();
    QIdentical( const QIdentical & other);  //!< Copy constructor

    [[nodiscard]] QRectF boundingRect() const override; //!< Get axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override; //!< Plot marker

private:
    [[nodiscard]] std::shared_ptr<QConstraintBase> doClone() const override;
};
*/

//! Graphics: Marker for parallelism
class QParallel : public QConstraintBase
{
public:
    static std::shared_ptr<QConstraintBase> create();  //!< Create parallelism constraint

    void setGeometry( QVector<std::shared_ptr<const uStraightLineSegment >> &s,
                      const VectorXidx &idx) override;
    void setMarkerSize ( qreal s) override;
    [[nodiscard]] qreal markerSize() const override;

protected:
    QParallel();
    QParallel( const QParallel & other);  //!< Copy constructor

    [[nodiscard]] QRectF boundingRect() const override;   //!< Get axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;  //!< Plot marker
private:
    [[nodiscard]] std::shared_ptr<QConstraintBase> doClone() const override;

    QGraphicsLineItem a;
    QGraphicsLineItem b;
    static constexpr double s_sc    = 0.2;
    static constexpr double s_shear = 0.1;
};

class Factory {
private:
    std::map<std::string, std::function <std::shared_ptr<QConstraintBase >() > > m_map;

    ~Factory() {
        // qDebug() << Q_FUNC_INFO;
        m_map.clear();
    }
    Factory & operator= (const Factory & other) {
        if (this == &other) {
            return *this;
        }
        return *this;
    }

public:
    Factory(const Factory & ) = delete;  // not clonable
    Factory(const Factory && ) = delete; // not movable
    Factory & operator= ( Factory &&) = delete;

    Factory() {
        m_map["horizontal"] = [] {return QAligned::create();    };
        m_map["vertical"]   = [] {return QAligned::create();    };
        m_map["diagonal"]   = [] {return QAligned::create();    };
        m_map["orthogonal"] = [] {return QOrthogonal::create(); };
        m_map["copunctual"] = [] {return QCopunctual::create(); };
        m_map["parallel"]   = [] {return QParallel::create();   };
    }
    static Factory *getInstance(){
        static Factory instance;
        return & instance;
    }
    std::shared_ptr<QConstraintBase> create(const std::string &s) {
        // assert( m_map.find(c) != m_map.end() && "unknown key");  // C++20: contains(c)
        if ( m_map.find(s)== m_map.end() ) {
            return nullptr;
        }
        return m_map[s]();
    }
};


} // namespace QConstraint

#endif // QCONSTRAINTS_H
