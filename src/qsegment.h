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

#ifndef QSEGMENT_H
#define QSEGMENT_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <QColor>
#include <QDebug>
#include <QGraphicsItem>
#include <QLineF>
#include <QPainterPath>
#include <QPen>
#include <QWidget>

#include "qnamespace.h"
#include "qtypes.h"

#include <utility>

namespace Uncertain {
class uPoint;
} // namespace Uncertain


//! Graphics: Strokes and straight line segments
namespace QEntity {

using Uncertain::uPoint;

//! Graphics: Base class for straight line segments
class QSegment : public QGraphicsItem
{
public:
    QSegment( const uPoint & ux,
              const uPoint & uy);         //!< Value constructor (two uncorrelated uncertain points)

    QSegment( const QSegment & other) = delete;               //!< Copy constructor
    QSegment & operator= ( const QSegment & other ) = delete; //!< Copy assignment constructor
    QSegment( const QSegment && other) = delete;   //!< Move constructor
    QSegment & operator= ( const QSegment && other ) = delete; //!< Move assignment constructor

    enum {Type = UserType +164};                //!< Definition graphics type (id)

    [[nodiscard]] int type() const override { return Type; }  //!< Get graphics type (id)
    [[nodiscard]] QPen pen() const { return pen_;}            //!< Get pen

    void setPen( const QPen & p) { pen_ = p;}   //!< Set pen
    void setColor( const QColor & c) { pen_.setColor(c); }  //!< Set color
    void setLineWidth( const int w)  { pen_.setWidth(w); }  //!< Set line width
    void setLineStyle( const int s)  { pen_.setStyle( Qt::PenStyle(s)); } //!< Set line style

    virtual void serialize( QDataStream & out ) const; //!< Serialization
    bool deserialize( QDataStream &in );               //!< Deserialization

    //! Toggle visibility confidence regions
    static void toggleShowUncertainty()  { s_showUncertainty = !s_showUncertainty; }

    //! Get status visibility confidence regions
    static bool showUncertainty() { return s_showUncertainty; }

    static void setPenSelected( const QPen & p) { s_penSelected = p;}

    static Eigen::Matrix3d cof3(const Eigen::Matrix3d &MM);     //!< 3x3 cofactor matrix, i.e., transposed adjunct

protected:
    //! Standard constructor
    explicit QSegment( QGraphicsItem *parent = nullptr );
    ~QSegment() override = default;

    [[nodiscard]] QPainterPath shape() const override;    //!< Get bounding shape
    [[nodiscard]] QRectF boundingRect() const override;   //!< Get axis-aligned bounding box

    void mousePressEvent( QGraphicsSceneMouseEvent * /* event */) override; //!< Handle mouse press event

    //! Plot uncertain straight line segment (base class)
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;

private:
    void createSelectionPolygon( qreal halfWidth );
    void setShape( const uPoint &ux,
                   const uPoint &uy ); // with new or adjusted endpoints.

    static double getSelectionOffset(const uPoint &, const uPoint &);
    [[nodiscard]] QLineF line() const { return line_; }
    static QPolygonF toPoly(std::pair<Eigen::VectorXd, Eigen::VectorXd> p);

    QLineF line_;                               //!< straight line segment
    std::pair<QPolygonF, QPolygonF>  branch_;   //!< hyperbola branches
    std::pair<QPolygonF, QPolygonF>  ellipse_;  //!< ellipses of end-points
    QPen pen_;
    QPolygonF minBBox;   // used in shape() und paint()

    static bool s_showUncertainty;
    static QPen s_penSelected;
};


//! Graphics: Constrained straight line segment
class QConstrained : public QSegment
{
public:
    QConstrained();
    QConstrained( const uPoint & ux,
                  const uPoint & uy);  //!< Value constructor

    void setAltColor( const QColor &); //!< Set color for automatic colorization (subtasks)

    static bool show() { return s_show; }              //!< Get status of visibility
    static void toggleShow() { s_show = !s_show; }     //!< Toggle visibility
    static bool showColor() { return s_showColor; }    //!< Get status colorization
    static void toogleShowColor() { s_showColor = !s_showColor;}      //!< Toggle colorization / no colorization
    static void setPenDefault( const QPen & p) { s_defaultPen = p; }  //!< Set default pen
    static QPen defaultPen() { return s_defaultPen; }   //!< Get current default pen

protected:
    //! Plot uncertain constrained straight line segment
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;

private:
    QColor altColor;

    static bool s_show;
    static bool s_showColor;
    static QPen s_defaultPen;
};


//! Graphics: Unconstrained straight line segment
class QUnconstrained : public QSegment
{
public:
    explicit QUnconstrained( QGraphicsItem *parent=nullptr);  //!< Standard Constructor
    QUnconstrained( const uPoint & ux,
                    const uPoint & uy);  //!< Value constructor (two uncorrelated uncertain points)

    static void toggleShow() { s_show = !s_show; }  //!< Toggle visibility
    static bool show() { return s_show; }           //!< Get status visibility

    static void setPenDefault( const QPen &p) { s_defaultPen = p; }  //!< Set default pen
    static QPen defaultPen() { return s_defaultPen; }  //!< Get current default pen

protected:
    //! Plot unconstrained uncertain straight line segment
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;

private:
    static bool s_show;
    static QPen s_defaultPen;
};

} // namespace QEntity

#endif // QSEGMENT_H
