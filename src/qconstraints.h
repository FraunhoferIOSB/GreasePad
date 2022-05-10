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

#ifndef QCONSTRAINTS_H
#define QCONSTRAINTS_H

#include <QDebug>
#include <QGraphicsItem>
#include <QPen>

#include <memory>



namespace Uncertain {
class uStraightLineSegment;
}


//! Graphics: Constraints
namespace QConstraint {

using Uncertain::uStraightLineSegment;

//! Graphics: Base class for markers depicting constraints
class QConstraintBase : public QGraphicsItem
{
protected:
    QConstraintBase();

    //! Copy constructor
    QConstraintBase( const QConstraintBase & other);

    //! Copy assignment constructor
    QConstraintBase & operator= (const QConstraintBase&) = delete ;

public:
    // T qgraphicsitem_cast(QGraphicsItem *item)
    // To make this function work correctly with custom items, reimplement the type() function for each custom QGraphicsItem subclass.
    enum {Type = UserType +364};

    //! Get graphics type (id)
    int type() const override { return Type; }

    void serialize(   QDataStream &out );  //!< serialization
    bool deserialize( QDataStream &);      //!< deserialization

    virtual std::shared_ptr<QConstraintBase> clone() const = 0; //!< Clone this constraint

    void setColor( const QColor & col);    //!< Set color
    void setLineWidth( const int w);       //!< Set line width
    void setLineStyle( const int s);       //!< Set line style
    void setAltColor( const QColor &col ); //!< Set color for automatic colorization
    void setStatus( bool isrequired,
                    bool isenforced);      //!< Set status

    //! Get status enforcment (success/failure)
    bool enforced() const { return m_is_enforced; }

    virtual void setMarkerSize ( qreal s) = 0;  //!< Set marker size
    virtual void setGeometry( const uStraightLineSegment &s,
                              const uStraightLineSegment &t) = 0; //!< Set geometry

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
    ~QConstraintBase() override = default;

    virtual qreal markerSize() const = 0;  //!< Set marker size
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;  //!< Plot marker

    void mousePressEvent(QGraphicsSceneMouseEvent *) override; //!< Handle mouse press event

    static bool showColor() { return s_showColor; } //!< Get status automatic colorization (on/off)

    static  int s_defaultMarkerSize;  //!< Default marker size
    static QPen s_defaultPenReq;      //!< Default pen for required constraints
    static QPen s_defaultPenRed;      //!< Default pen for redundant constraints
    static QPen s_penSelected;        //!< Pen for selection

    const double m_sc = 1000; //!< Scale factor coordinates screen // TODO(meijoc)
    QColor m_altColor;        //!< Color for automatic colorization (subtasks)
    QPen m_pen_req;           //!< Pen for required constraint
    QPen m_pen_red;           //!< Pen for redundant constraint

    bool m_is_required = true;    //!< Constraint is required? (for painting)
    bool m_is_enforced = false;   //!< Constraint is enforced? (for painting)

private:
    static bool s_showColor;
    static bool s_show;
};

//! Graphics: Three copunctual straight lines (circle)
class QCopunctual : public QConstraintBase,
        public QGraphicsEllipseItem
{
public:
    static std::shared_ptr<QConstraintBase> create();  //!< Create copunctual constraint

    void setGeometry( const uStraightLineSegment &s,
                      const uStraightLineSegment &t) override;
    void setMarkerSize ( qreal s) override;
    qreal markerSize() const override;

protected:
    QCopunctual();
    QCopunctual( const QCopunctual & other);    //!< Copy constructor

    QRectF boundingRect() const override; //!< Get axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override; //!< Plot the circle

private:
    std::shared_ptr<QConstraintBase> clone() const override;
};


//! Graphics: Two orthogonal straight lines (square)
class QOrthogonal : public QConstraintBase,
        public QGraphicsRectItem
{
public:
    static std::shared_ptr<QConstraintBase> create();  //!< Create orthogonallity constraint

    void setGeometry( const uStraightLineSegment &s,
                      const uStraightLineSegment &t) override;
    void setMarkerSize ( qreal s) override;
    qreal markerSize() const override;

protected:
    QOrthogonal();
    QOrthogonal( const QOrthogonal & other); //!< Copy constructor

    QRectF boundingRect() const override;   //!< Get axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;  //!< Plot square
private:
    std::shared_ptr<QConstraintBase> clone() const override;
};

//! Graphics: Marker for indentity
class QIdentical : public QConstraintBase,
        public QGraphicsPolygonItem
{
public:
    static std::shared_ptr<QConstraintBase> create();  //!< Create identity constraint

    void setMarkerSize ( qreal s) override;
    qreal markerSize() const override;
    void setGeometry( const uStraightLineSegment &s,
                      const uStraightLineSegment &t) override;

protected:
    QIdentical();
    QIdentical( const QIdentical & other);  //!< Copy constructor

    QRectF boundingRect() const override; //!< Get axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override; //!< Plot marker

private:
    std::shared_ptr<QConstraintBase> clone() const override;
};


//! Graphics: Marker for parallelism
class QParallel : public QConstraintBase
{
public:
    static std::shared_ptr<QConstraintBase> create();  //!< Create parallelism constraint

    void setGeometry( const uStraightLineSegment &s,
                      const uStraightLineSegment &t) override;
    void setMarkerSize ( qreal s) override;
    qreal markerSize() const override;

protected:
    QParallel();
    QParallel( const QParallel & other);  //!< Copy constructor

    QRectF boundingRect() const override;   //!< Get axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;  //!< Plot marker
private:
    std::shared_ptr<QConstraintBase> clone() const override;

    QGraphicsLineItem a;
    QGraphicsLineItem b;
    static constexpr double s_sc    = 0.2;
    static constexpr double s_shear = 0.1;
};

} // namespace QConstraint

#endif // QCONSTRAINTS_H
