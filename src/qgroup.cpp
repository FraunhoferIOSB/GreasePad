#include "global.h"
#include "qgroup.h"

#include<QDebug>
#include<QGraphicsSceneMouseEvent>
#include<QPainter>

namespace QEntity {

bool QStroke::show_ = false;
bool QSegment::showUncertainty_ = false;

// QPen QGroup::penSelected  = QPen( Qt::black, 1, Qt::DashLine,  Qt::RoundCap,Qt::RoundJoin);


/*QGroup::QGroup()
{
    qDebug() << Q_FUNC_INFO;
    setFlag( QGraphicsItem::ItemIsSelectable,         true);
    setFlag( QGraphicsItem::ItemIsMovable,            false);
    setFlag( QGraphicsItem::ItemSendsGeometryChanges, false);

    addToGroup( &qStroke );
    addToGroup( &qUnconstrained);
    addToGroup( &qConstrained);
}*/


/*QGroup::QGroup( const QPolygonF &track)
    : qStroke(track)
{
    qDebug() << green << Q_FUNC_INFO << black;

    int N = track.length();
    Eigen::VectorXd xi(N);
    Eigen::VectorXd yi(N);
    for (int i=0; i<N; i++) {
        xi(i) = track.at(i).x();
        yi(i) = track.at(i).y();
    }
    xi /= 1000;
    yi /= 1000;
    std::pair<Uncertain::uPoint,Uncertain::uPoint> pp
            = Uncertain::uEndPoints( xi, yi);
    qUnconstrained.setShape( pp.first, pp.second );

    qConstrained = qUnconstrained;

    // !! order determines Z-values
    addToGroup( &qUnconstrained );
    addToGroup( &qConstrained );
    addToGroup( &qStroke );

    qStroke.setVisible( QStroke::show() );
    qConstrained.setVisible( QConstrained::show() );
    qUnconstrained.setVisible( QUnconstrained::show() );


    setFlag( QGraphicsItem::ItemIsSelectable,         true );
    setFlag( QGraphicsItem::ItemIsMovable,            false);
    setFlag( QGraphicsItem::ItemSendsGeometryChanges, false);


    createSelectionPolygon( pp.first, pp.second);   // now, after set shape

    setFlag( QGraphicsItem::ItemClipsChildrenToShape, true);
    setFlag( QGraphicsItem::ItemContainsChildrenInShape, true);
}*/


/*QGroup::QGroup( const QPolygonF &track,
                const uPoint &ux,
                const uPoint &uy)
    : qStroke(track)
{
    qDebug() << green << Q_FUNC_INFO << black;

    // (1) constrained
    qConstrained.setShape( ux, uy );

    // (2) unconstrained
    int N = track.length();
    Eigen::VectorXd xi(N);
    Eigen::VectorXd yi(N);
    for (int i=0; i<N; i++) {
        xi(i) = track.at(i).x();
        yi(i) = track.at(i).y();
    }
    xi /= 1000;
    yi /= 1000;
    std::pair<Uncertain::uPoint,Uncertain::uPoint> pp
            = Uncertain::uEndPoints( xi, yi);
    qUnconstrained.setShape( pp.first, pp.second );

    // !! order determines Z-values
    addToGroup( &qUnconstrained );
    addToGroup( &qConstrained );
    addToGroup( &qStroke );

    qStroke.setVisible( QStroke::show() );
    qConstrained.setVisible( QConstrained::show() );
    qUnconstrained.setVisible( QUnconstrained::show() );

    createSelectionPolygon( ux, uy);   // now, after set shape

    setFlag( QGraphicsItem::ItemIsSelectable,         true );
    setFlag( QGraphicsItem::ItemIsMovable,            false);
    setFlag( QGraphicsItem::ItemSendsGeometryChanges, false);
    setFlag( QGraphicsItem::ItemClipsChildrenToShape, true);
}*/


/*QGroup::QGroup()  {

    qDebug() << green << Q_FUNC_INFO << black;

    qStroke.setVisible( QStroke::show() );
    qConstrained.setVisible( QConstrained::show() );
    qUnconstrained.setVisible( QUnconstrained::show() );

    // !! order determines Z-values
    addToGroup( &qUnconstrained );
    addToGroup( &qConstrained );
    addToGroup( &qStroke );

    setFlag( QGraphicsItem::ItemIsSelectable,         true );
    setFlag( QGraphicsItem::ItemIsMovable,            false);
    setFlag( QGraphicsItem::ItemSendsGeometryChanges, false);
    setFlag( QGraphicsItem::ItemClipsChildrenToShape, true );
}*/

/*QGroup::QGroup( const QGroup & other)
{
    qDebug() << green << Q_FUNC_INFO << black;

    qStroke          = other.qStroke;
    qUnconstrained   = other.qUnconstrained;
    qConstrained     = other.qConstrained;
    selectionPolygon = other.selectionPolygon;

    qStroke.setVisible( QStroke::show() );
    qConstrained.setVisible( QConstrained::show() );
    qUnconstrained.setVisible( QUnconstrained::show() );

    // !! order determines Z-values
    addToGroup( &qUnconstrained );
    addToGroup( &qConstrained );
    addToGroup( &qStroke );

    setFlag( QGraphicsItem::ItemIsSelectable,         true );
    setFlag( QGraphicsItem::ItemIsMovable,            false);
    setFlag( QGraphicsItem::ItemSendsGeometryChanges, false);
    setFlag( QGraphicsItem::ItemClipsChildrenToShape, true );
}*/


/*void QGroup::paint( QPainter *painter,
                    const QStyleOptionGraphicsItem *option,
                    QWidget *widget)
{
    //qDebug() << red << Q_FUNC_INFO;
    Q_UNUSED(option)
    Q_UNUSED(widget)

    if ( !isSelected() ) {
        return;
    }

    painter->setPen( penSelected );
    painter->drawPolygon( selectionPolygon );
}*/

/*void QGroup::createSelectionPolygon( const uPoint & ux,
                                     const uPoint & uy)
{
    qDebug() << red << Q_FUNC_INFO;

    qreal selectionOffset = qConstrained.getSelectionOffset( ux, uy);

    selectionPolygon.clear();
    QRectF rect(
                -qConstrained.line().length()/2-selectionOffset,
                -selectionOffset,
                qConstrained.line().length()+2*selectionOffset,
                2*selectionOffset );
    QPolygonF poly(rect);

    QTransform t;
    t.translate( qConstrained.line().center().x(),
                 qConstrained.line().center().y() );
    t.rotate(   -qConstrained.line().angle());

    selectionPolygon = t.map( poly);
}*/



/*void QGroup::serialize( QDataStream &out ) const
{
    qDebug() << Q_FUNC_INFO;

    qStroke.serialize( out );
    qUnconstrained.serialize( out );
    qConstrained.serialize( out );
    out << selectionPolygon;
}*/

/*void QGroup::deserialize(QDataStream &in )
{
    // qDebug() << Q_FUNC_INFO;

    qStroke.deserialize( in );
    qUnconstrained.deserialize( in );
    qConstrained.deserialize( in );
    in >> selectionPolygon;

    // qDebug() << blue << selectionPolygon;
}*/


/*void QGroup::mousePressEvent( QGraphicsSceneMouseEvent *e)
{
    // qDebug() << Q_FUNC_INFO;
    if ( e->button() == Qt::RightButton ) {
        setSelected( !isSelected() );  // toggle selection
    }
}*/



/* QRectF QGroup::boundingRect() const
{
    // qDebug() << Q_FUNC_INFO << shape().boundingRect();
    // return shape().boundingRect();
    return qStroke.boundingRect();
} */









/*void QGroup::merge( const QGroup &other,
                    const uPoint &x,
                    const uPoint &y)
{
    qDebug() << Q_FUNC_INFO;

    qStroke.appendPointSet( other.qStroke.pointSet() );

    qConstrained.setShape( x,y);

    qUnconstrained = qConstrained;
}*/



/*void QGroup::setGeometry( const uPoint &ux, const uPoint &uy)
{
    // qDebug() << yellow << Q_FUNC_INFO << black;
    qConstrained.setShape( ux,uy);
    createSelectionPolygon( ux, uy);    // in setParamsConstrained
}*/

/*void QGroup::setColor(const QColor & col)
{
    qDebug() << Q_FUNC_INFO << col;
    qConstrained.setColor( col);
}*/

/*void QGroup::swapColor()
{
    qConstrained.swapColor();
}*/


/*QPainterPath QGroup::shape() const
{
    QPainterPath ret;
    ret.addPolygon(selectionPolygon);
    return ret;
}*/

/*QRectF QGroup::boundingRect() const
{
    // qDebug() << Q_FUNC_INFO;
    return shape().boundingRect();
}*/

} // namespace QEntity
