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

#ifndef QSEGMENT_H
#define QSEGMENT_H

#include <Eigen/Dense>

#include <QDebug>
#include <QGraphicsItem>
#include <QPen>


namespace Uncertain {
class uPoint;
}



namespace QEntity {

using Uncertain::uPoint;

class
        QSegment : public QGraphicsItem
{
public:
    QSegment( const uPoint & ux,
              const uPoint & uy);
    QSegment( const QSegment & other) = delete;

    enum {Type = UserType +164};
    int type() const override { return Type; }

    void setColor( const QColor & c) { pen_.setColor(c); }
    void setLineWidth( const int w)  { pen_.setWidth(w); }
    void setLineStyle( const int s)  { pen_.setStyle( Qt::PenStyle(s)); }

protected:
    QSegment();
    ~QSegment() override = default;

    QPainterPath shape() const override;
    void mousePressEvent( QGraphicsSceneMouseEvent * /* event */) override;

    QPen pen() const { return pen_;}
    void setPen( const QPen & p) { pen_ = p;}

public:
   // QSegment & operator= ( const QSegment & other );

    virtual void serialize( QDataStream & out ) const;
    bool deserialize( QDataStream &in );

    static void toggleShowUncertainty()  { s_showUncertainty = !s_showUncertainty; }
    static bool showUncertainty() { return s_showUncertainty; }

protected:
    QRectF boundingRect() const override;
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override; //!< plot tracked positions

private:
    void createSelectionPolygon( qreal halfWidth );
    void setShape( const uPoint &ux,
                   const uPoint &uy ); // with new or adjusted endpoints.

    double getSelectionOffset( const uPoint &,
                               const uPoint &) const;
    QLineF line() const { return line_;}
    QPolygonF toPoly( std::pair<Eigen::VectorXd, Eigen::VectorXd> p);

    QLineF   line_;                             // straight line segment
    std::pair<QPolygonF, QPolygonF>  branch_;   // hyperbola branches
    std::pair<QPolygonF, QPolygonF>  ellipse_;  // ellipses of end-points
    QPen pen_;
    QPolygonF minBBox;   // used in shape() und paint()

    static bool s_showUncertainty;
};


class QConstrained : public QSegment
{
public:
    QConstrained();
    QConstrained( const uPoint & ux,
                  const uPoint & uy);

    // using QSegment::operator=;

    void setAltColor( const QColor &);

    static bool show() { return s_show; }
    static void toggleShow() { s_show = !s_show; }

    static bool showColor() { return s_showColor; }
    static void toogleShowColor() { s_showColor = !s_showColor;}

    static void setPenDefault( const QPen & p) { s_defaultPen = p; }
    static QPen defaultPen() { return s_defaultPen; }

protected:
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override; //!< plot tracked positions

private:
    QColor altColor;

    static bool s_show;
    static bool s_showColor;
    static QPen s_defaultPen;
};

class QUnconstrained : public QSegment
{
public:
    QUnconstrained();
    QUnconstrained( const uPoint & ux,
                    const uPoint & uy);

    // using QSegment::operator=;

    static void toggleShow() { s_show = !s_show; }
    static bool show() { return s_show; }

    static void setPenDefault( const QPen &p) { s_defaultPen = p; }
    static QPen defaultPen() { return s_defaultPen; }
protected:
   // QRectF boundingRect() const override { return QSegment::boundingRect(); }
    void paint( QPainter *painter,
                const QStyleOptionGraphicsItem *option,
                QWidget *widget) override; //!< plot tracked positions

private:
    static bool s_show;
    static QPen s_defaultPen;
};

} // namespace QEntity

#endif // QSEGMENT_H
