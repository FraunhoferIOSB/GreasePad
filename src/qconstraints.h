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


namespace QConstraint {

using Uncertain::uStraightLineSegment;

class QConstraintBase : public QGraphicsItem
{
protected:
    QConstraintBase();
public:
    QConstraintBase( const QConstraintBase & other) = delete;
    QConstraintBase & operator= (const QConstraintBase&) = delete ;

    // T qgraphicsitem_cast(QGraphicsItem *item)
    // To make this function work correctly with custom items, reimplement the type() function for each custom QGraphicsItem subclass.
    enum {Type = UserType +364};
    int type() const override { return Type; }

    void serialize(   QDataStream &out );
    bool deserialize( QDataStream &);

    virtual std::shared_ptr<QConstraintBase> create() const = 0;

    void setColor( const QColor & col) {
        m_pen_req.setColor(col);
        m_pen_red.setColor(col);
    }
    void setLineWidth( const int w) {
        m_pen_req.setWidth(w);
        m_pen_red.setWidth(w);
    }

    void setLineStyle( const int s) {
        m_pen_req.setStyle( Qt::PenStyle(s) );
        m_pen_red.setStyle( Qt::PenStyle(s) );
    }

    void setAltColor(const QColor &col );
    void setStatus( bool isrequired,
                    bool isenforced);

    bool enforced() const { return m_is_enforced; }


    virtual void setMarkerSize ( qreal s) = 0;
    virtual void setGeometry( const uStraightLineSegment &s,
                              const uStraightLineSegment &t) = 0;

    static QPen defaultPenReq() { return s_defaultPenReq; }
    static QPen defaultPenRed() { return s_defaultPenRed; }

    static void setDefaultPenReq( const QPen &p) { s_defaultPenReq = p; }
    static void setDefaultPenRed( const QPen &p) { s_defaultPenRed = p; }
    static void setPenSelected(   const QPen &p) { s_penSelected = p;}
    static void setDefaultMarkerSize( int s) { s_defaultMarkerSize = s; }

    static void toggleShow()      { s_show = !s_show; }
    static void toggleShowColor() { s_showColor =! s_showColor; }
    static bool show() {return s_show;}

protected:
    ~QConstraintBase() override = default;

    virtual qreal markerSize() const = 0;
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;

    void mousePressEvent(QGraphicsSceneMouseEvent *) override;

    bool m_is_required = true;    // for painting
    bool m_is_enforced = false;   // for painting

    static bool showColor() { return s_showColor; }

    static  int s_defaultMarkerSize;
    static QPen s_defaultPenReq;
    static QPen s_defaultPenRed;
    static QPen s_penSelected;

    const double m_sc = 1000;  // TODO(meijoc)  QConstraintBase
    QColor m_altColor;
    QPen m_pen_req;
    QPen m_pen_red;

private:
    static bool s_showColor;
    static bool s_show;
};

class QCopunctual : public QConstraintBase,
        public QGraphicsEllipseItem
{
public:
    QCopunctual();

    void setGeometry( const uStraightLineSegment &s,
                      const uStraightLineSegment &t) override;
    void setMarkerSize ( qreal s) override;
    qreal markerSize() const override;

protected:
    QRectF boundingRect() const override; //!< axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;

private:
    std::shared_ptr<QConstraintBase> create() const override;
};

class QOrthogonal : public QConstraintBase,
        public QGraphicsRectItem
{
public:
    QOrthogonal();

public:
    void setGeometry( const uStraightLineSegment &s,
                      const uStraightLineSegment &t) override;
    void setMarkerSize ( qreal s) override;
    qreal markerSize() const override;

protected:
    QRectF boundingRect() const override; //!< axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;
private:
    std::shared_ptr<QConstraintBase> create() const override;
};

class QIdentical : public QConstraintBase,
        public QGraphicsPolygonItem
{
public:
    QIdentical();

    void setMarkerSize ( qreal s) override;
    qreal markerSize() const override;
    void setGeometry( const uStraightLineSegment &s,
                      const uStraightLineSegment &t) override;

protected:
    QRectF boundingRect() const override; //!< axis-aligned bounding box
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;

private:
    std::shared_ptr<QConstraintBase> create() const override;
};


class QParallel : public QConstraintBase
{
public:
    QParallel();

    void setGeometry( const uStraightLineSegment &s,
                      const uStraightLineSegment &t) override;
    void setMarkerSize ( qreal s) override;
    qreal markerSize() const override;

protected:
    QRectF boundingRect() const override;
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override;
private:
    std::shared_ptr<QConstraintBase> create() const override;

    QGraphicsLineItem a;
    QGraphicsLineItem b;
    static constexpr double s_sc    = 0.2;
    static constexpr double s_shear = 0.1;

};

} // namespace QConstraint

#endif // QCONSTRAINTS_H
