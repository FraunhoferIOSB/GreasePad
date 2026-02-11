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


#include "qassert.h"
#include "qcolor.h"
#include "qcontainerfwd.h"
#include "qhashfunctions.h"
#include "qlogging.h"
#include "qsharedpointer.h"
#include "qtypes.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include "adjustment.h"
#include "conncomp.h"
#include "constraints.h"
#include "geometry/acute.h"
#include "global.h"
#include "mainscene.h"
#include "matfun.h"
#include "matrix.h"
#include "qconstraints.h"
#include "qsegment.h"
#include "qstroke.h"
#include "quantiles.h"
#include "state.h"
#include "uncertain/quncertain.h"
#include "uncertain/udistance.h"
#include "uncertain/upoint.h"
#include "uncertain/usegment.h"
#include "uncertain/ustraightline.h"

#include <QApplication>
#include <QDebug>
#include <QMessageBox>
#include <QPolygonF>
#include <QtCompilerDetection>

#include <Eigen/Core>
#include <Eigen/SparseCore>

using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::Vector;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Index;
using Eigen::SparseMatrix;
using Eigen::ColMajor;
using Eigen::indexing::last;
using Eigen::Dynamic;

using Constraint::ConstraintBase;
using Constraint::Parallel;
using Constraint::Orthogonal;
using Constraint::Copunctual;
using Constraint::Vertical;
using Constraint::Horizontal;
using Constraint::Diagonal;

using Uncertain::uPoint;
using Uncertain::uStraightLine;
using Uncertain::uStraightLineSegment;
using Uncertain::uDistance;

using Graph::conncomp;
using Graph::IncidenceMatrix;

using Matfun::find;
using Matfun::sign;
using Matfun::unique;
using Matfun::spfind;

using TextColor::black;
using TextColor::blue;


bool State::considerOrthogonal_ = true;
bool State::considerParallel_   = true;
bool State::considerCopunctual_ = true;
bool State::considerVertical_   = false;
bool State::considerHorizontal_ = false;
bool State::considerDiagonal_   = false;

Quantiles::Recognition State::recogn_;
Quantiles::Snapping    State::snap_;



namespace {

//! Serialization of sparse incidence matrix
QDataStream & operator<< (QDataStream & out, const IncidenceMatrix & AA)
{
    // qDebug() << Q_FUNC_INFO;

    out << static_cast<uint>(AA.rows());
    out << static_cast<uint>(AA.cols());
    out << static_cast<uint>(AA.nonZeros());

    for( Index c=0; c<AA.outerSize(); ++c) {
        SparseMatrix<int,ColMajor>::InnerIterator it( AA, c);
        for( ; it; ++it) {
            out << static_cast<int>(it.row()) << static_cast<int>(it.col());
        }
    }
    return out;
}


//! Deserialization of sparse incidence matrix
QDataStream & operator>> (QDataStream & in,  IncidenceMatrix & AA)
{
    // qDebug() << Q_FUNC_INFO;

    uint nrows = 0;
    uint ncols = 0;
    uint nnz = 0;
    in >> nrows >> ncols >> nnz;

    // fill vector with triplets (i,j,value)
    std::vector< Eigen::Triplet<int> > tripletList;
    tripletList.reserve( nnz );
    int r = 0;
    int c = 0;
    for ( uint i=0; i<nnz; i++ ) {
        in >> r >> c;
        tripletList.emplace_back( r, c, 1 );
    }

    // fill sparse matrix
    AA.resize(static_cast<int>(nrows), static_cast<int>(ncols));
    AA.setFromTriplets( tripletList.begin(),
                       tripletList.end() );

    return in;
}


//! Serialization of geometric constraint
QDataStream & operator<< ( QDataStream & out, const ConstraintBase & c)
{
    // qDebug() <<  Q_FUNC_INFO << c.type_name() << c.type_name()[0];
    out << c.type_name()[0]; // first character, {'v','h','d','o','p','c'}
    out << c.status();    // { UNEVAL=0 | REQUIRED | OBSOLETE };
    out << c.enforced();

    return out;
}


//! Deserialization of geometric constraint
QDataStream & operator>> ( QDataStream &in, ConstraintBase &c )
{
    // qDebug() << Q_FUNC_INFO;
    int status = 0;  // underlying type of enum
    in >> status;
    c.setStatus( static_cast<ConstraintBase::Status>(status) );

    bool enforced = false;
    in >> enforced;
    c.setEnforced( enforced );

    return in;
}


} // namespace



//! Implementation details of class 'state' (pImpl idiom)
class impl {
private:
    using VectorXidx = Vector<Index,Dynamic>;

public:
    bool deserialize( QDataStream & in  );        //!< Serialization
    void serialize(   QDataStream & out ) const;  //!< Deserialization

    [[nodiscard]] QString StatusMsg() const;           //!< Create message for status bar

    void toggleVisibilityStrokes();        //!< Toggle the visibility of strokes (point sequences)
    void toggleVisibilityConstraints();    //!< Toggle the visibility of markers depicting geometric constraints
    void toggleVisibilityConstrained();    //!< Toggle the visibility of constrained segments
    void toggleVisibilityUnconstrained();  //!< Toggle the visibility of unconstrained segments

    void clearAll();        //!< Clear all, create blank state

    // augment
    void append( const QPolygonF & track);   //!< Augment state: append one track (segment)
    void reasoning_augment_and_adjust( const Quantiles::Snapping &snap);  //!< Augment state: reasoning & adjustment

    // reduce
    void remove_elements();              //!< Reduce state: delete selected elements
    void reasoning_reduce_and_adjust();  //!< Reduce state: reasoning & adjustment

    // augment & reduce
    void replaceGraphics();               //!< Graphics: Replace where necessary
    void graphicItemsAdd(QGraphicsScene *sc) const; //!< Graphics: add items

private:
    void identify_subtasks();

    // augment
    Index find_new_constraints();
    void find_adjacencies_of_latest_segment(const Quantiles::Snapping &snap);
    void merge_segment ( Index a );
    bool identities_removed();
    void snap_endpoints( Index numNewConstr );
    void update_segments( const VectorXidx & maps, const AdjustmentFramework & a);

    // reduce
    void remove_constraint( Index i );
    void remove_segment(    Index i );

    // augment & reduce
    void solve_subtask_greedy(int cc);

    //! Estimation of two points delimiting an uncertain straight line segment
    static std::pair<uPoint,uPoint> uEndPoints( const VectorXd & xi,
                                                const VectorXd & yi);
    static std::pair<VectorXd, VectorXd> trackCoords(const QPolygonF &poly);

    void setAltColors() const;

    bool is_vertical(   Index a);
    bool is_horizontal( Index a);
    bool is_diagonal(   Index a);

    bool are_parallel(   Index a, Index b);
    bool are_orthogonal( Index a, Index b);
    bool are_identical(  Index a, Index b);

    bool are_copunctual( Index a, Index b, Index c);

    void establish_vertical(   Index a);
    void establish_horizontal( Index a);
    void establish_diagonal(   Index a);

    void establish_parallel(   Index a, Index b );
    void establish_orthogonal( Index a, Index b );

    void establish_copunctual( Index a, Index b, Index c );

    [[nodiscard]] std::pair<Eigen::VectorXd, SparseMatrix<double> >
    a_Maker( const VectorXidx & maps_ ) const;

    [[nodiscard]] Index number_of_required_constraints() const; // for statistics only


    QVector< std::shared_ptr< const uStraightLineSegment> >   m_segm;
    QVector< std::shared_ptr< Constraint::ConstraintBase> >   m_constr;
    QVector< std::shared_ptr< QEntity::QStroke> >        m_qStroke;
    QVector< std::shared_ptr< QEntity::QUnconstrained> > m_qUnconstrained;
    QVector< std::shared_ptr< QEntity::QConstrained> >   m_qConstrained;
    QVector< std::shared_ptr< QConstraint::QConstraintBase> > m_qConstraint;

    Graph::IncidenceMatrix Adj;          // Adjacency of straight line segments
    Graph::IncidenceMatrix Rel;           // Present relation of segment (row) and constraint (column)
    Graph::IncidenceMatrix x_touches_l;  // "End-point x touches straight line l."  TODO transpose!?
    Graph::IncidenceMatrix y_touches_l;  // "End-point y touches straight line l."
    Graph::IncidenceMatrix PP;           // parallelism

    Eigen::ArrayXi arr_segm;
    Eigen::ArrayXi arr_constr;

    static constexpr double m_scale = 1000;
};


State::State( const State & other)
    : m_pImpl( std::make_unique<impl>(*other.m_pImpl) )
{
    // qDebug() << Q_FUNC_INFO;
}

State::~State()
{
    // qDebug() << Q_FUNC_INFO;
    clearAll();
}

State::State() : m_pImpl( std::make_unique<impl>())
{
    // qDebug() << Q_FUNC_INFO;
}



State & State::operator= ( const State & other )
{
    // qDebug() << Q_FUNC_INFO;
    if ( &other==this ) {
        return *this;
    }
    m_pImpl = std::make_unique<impl>( *other.m_pImpl);

    return *this;
}

bool State::augment( const QPolygonF &track)
{
    // qDebug() << Q_FUNC_INFO;
    pImpl()->append( track);
    pImpl()->reasoning_augment_and_adjust( snap_ );
    pImpl()->replaceGraphics();
    return true;
}

void State::clearAll()
{
    pImpl()->clearAll();
}


bool impl::deserialize( QDataStream & in )
{
    // qDebug() <<  Q_FUNC_INFO;
    // st_clear();  //  required? st: new()

    qDebug().noquote() << "(1) reading topology...";
    in >> Adj;
    in >> Rel;
    in >> x_touches_l;
    in >> y_touches_l;
    in >> PP;

    qDebug().noquote() << "(2) reading geometry...";
    qDebug().noquote() << "(2.1) reading uncertain straight line segments...";
    for ( Index s=0; s<Rel.rows(); s++ ) {
        auto seg = uStraightLineSegment::create();
        in >> *seg;
        if ( in.status()!=0) {
            return false;
        }
        m_segm.append( seg );
    }

    qDebug().noquote() << "(2.2) reading geometric constraints...";
    for ( Index i=0; i<Rel.cols(); i++ ) {
        // auto c = ConstraintBase::deserialize(in);  // calls 'create'

        char type_code = '\0';
        in >> type_code;
        // qDebug() << Q_FUNC_INFO << type_name;
        if ( in.status()!=0) {
            return false;
        }

        std::shared_ptr<ConstraintBase> c;
        c = Constraint::Factory::getInstance()->create(type_code);
        if ( c==nullptr ) {
            return false;
        }
        in >> *c;
        if ( in.status()!=0) {
            return false;
        }
        m_constr.append( c );
    }

    qDebug().noquote() << "(3) graphics...";

    qDebug().noquote() << "(3.1) constraints...";
    for ( const auto & con : std::as_const( m_constr)) {
        auto q = QConstraint::Factory::getInstance()->create( con->type_name());
        if ( !q->deserialize( in ) ) {
            return false;
        }
        m_qConstraint.append( q);
    }

    qDebug().noquote() << "(3.2) strokes ...";
    for ( Index i=0; i<Rel.rows(); i++) {
        auto q = std::make_shared<QEntity::QStroke>();
        if( !q->deserialize(in) ) {
            return false;
        }
        m_qStroke.append( q);
    }

    qDebug().noquote() << "(3.3) unconstrained segments...";
    for ( Index i=0; i<Rel.rows(); i++) {
        auto q = std::make_shared<QEntity::QUnconstrained>();
        if( !q->deserialize(in) ) {
            return false;
        }
        m_qUnconstrained.append( q);
    }

    qDebug().noquote() << "(3.4) constrained segments...";
    for ( Index i=0; i<Rel.rows(); i++) {
        auto q = std::make_shared<QEntity::QConstrained>();
        if( !q->deserialize(in) ) {
            return false;
        }
        m_qConstrained.append( q);
    }

    // determine subtasks, i.e., connected components
    identify_subtasks();

    return true;
}

void State::graphicItemsAdd( QGraphicsScene *sc) const
{
    pImpl()->graphicItemsAdd( sc);
}

void impl::graphicItemsAdd( QGraphicsScene *sc) const
{
    // qDebug() << Q_FUNC_INFO;

    setAltColors();
    for ( const auto & item : m_qStroke)
    {
        Q_ASSERT( !item->scene() );
        sc->addItem( item.get() );
    }
    for ( const auto & item : m_qUnconstrained)
    {
        Q_ASSERT( !item->scene() );
        sc->addItem( item.get() );
    }
    for ( const auto & item : m_qConstrained)
    {
        Q_ASSERT( !item->scene() );
        sc->addItem( item.get() );
    }
    for ( const auto & item : m_qConstraint)
    {
        Q_ASSERT( !item->scene() );
        sc->addItem( item.get() );
    }
    // qDebug() << Q_FUNC_INFO;
}


bool State::reduce()
{
    // qDebug() <<  Q_FUNC_INFO;
    pImpl()->remove_elements();   // ... but do not delete
    pImpl()->reasoning_reduce_and_adjust();
    pImpl()->replaceGraphics();

    return true;
}


void State::serialize( QDataStream  & out ) const
{
    pImpl()->serialize( out );
}

void impl::serialize( QDataStream &out ) const
{
    // qDebug() <<  Q_FUNC_INFO;

    // (1) topology
    out << Adj;
    out << Rel;
    out << x_touches_l;
    out << y_touches_l;
    out << PP;

    // (2) geometry
    for ( const auto & item : m_segm) {
        out << *item;
    }
    for ( const auto & item : m_constr) {
        out << *item;
    }

    // (3) graphics
    for ( const auto & item : m_qConstraint) {
        item->serialize( out );
    }
    for ( const auto & item : m_qStroke) {
        item->serialize( out );
    }
    for ( const auto & item : m_qUnconstrained) {
        item->serialize( out );
    }
    for ( const auto & item : m_qConstrained) {
        item->serialize( out );
    }

    qDebug().noquote() << "Export finished.";
}

void impl::find_adjacencies_of_latest_segment(const Quantiles::Snapping &snap)
{
    // qDebug() << Q_FUNC_INFO;

    const Index N = m_segm.length();


    for ( int i=0; i<N-1; i++)
    {
         bool are_adjacent = false;

        // pre-check (culling) via axis-aligned bounding boxes
        if ( !m_segm.at(i)->bounding_box().overlaps(
                 m_segm.last()->bounding_box()) ) {
            continue;
        }


        if ( m_segm.last()->touchedBy( m_segm.at(i)->ux(),
                                      snap.quantile_stdNormDistr(),
                                      snap.quantile_chi2_1dof()) ) {
            x_touches_l.set(i,last);
            are_adjacent = true;
        }

        if ( m_segm.last()->touchedBy( m_segm.at(i)->uy(),
                                      snap.quantile_stdNormDistr(),
                                      snap.quantile_chi2_1dof()) ) {
            y_touches_l.set(i,last);
            are_adjacent = true;
        }

        if ( m_segm.at(i)->touchedBy( m_segm.last()->ux() ,
                                     snap.quantile_stdNormDistr(),
                                     snap.quantile_chi2_1dof() ) ) {
            x_touches_l.set(last,i);
            are_adjacent = true;
        }

        if ( m_segm.at(i)->touchedBy( m_segm.last()->uy(),
                                     snap.quantile_stdNormDistr(),
                                     snap.quantile_chi2_1dof() ) ) {
            y_touches_l.set(last,i);
            are_adjacent = true;
        }

        if ( m_segm.at(i)->intersects( *m_segm.last()) )
        {
            are_adjacent = true;
        }

        if ( are_adjacent ) {
            Adj.set(i, last);
            Adj.set(last, i);
        }
    }
}


void impl::remove_segment(const Index i)
{
    // qDebug() << "removing segment #" << i+1;
    // qDebug() << Q_FUNC_INFO;

    m_segm.removeAt(i);
    Adj.reduce(i);     // A(i,:)= []; A(:,i)=[]
    PP.reduce(i);
    Rel.remove_row(i);   // B(i,:) = [];

    x_touches_l.reduce(i);
    y_touches_l.reduce(i);

    m_qStroke.removeAt( i );
    m_qUnconstrained.removeAt( i );
    m_qConstrained.removeAt( i );
}


void impl::reasoning_augment_and_adjust( const Quantiles::Snapping & snap)
{
    // qDebug() <<  Q_FUNC_INFO ;

    // (0) find adjacencies / connectivity
    find_adjacencies_of_latest_segment( snap);
    if ( identities_removed() ) {
        find_adjacencies_of_latest_segment( snap); // ! 2nd time
    }

    // (1) find constraints .....................................
    const Index num_new_constraints_ = find_new_constraints();
    assert( num_new_constraints_ >= 0 );

    qDebug().noquote() << QStringLiteral("%1 new constraint%2 found.")
                .arg(num_new_constraints_)
                .arg(num_new_constraints_==1 ? "" : "s");

    // (2) check for independence and consistency ...............
    identify_subtasks();  // connected components

    if (num_new_constraints_ > 0) {
        const VectorXi LabelsNewConstrIndividual = arr_constr.tail( num_new_constraints_);
        const VectorXi LabelsNewConstrUnique = unique( LabelsNewConstrIndividual);
        for ( const auto cc : LabelsNewConstrUnique) {
            qDebug().noquote() << blue << QStringLiteral("Reasoning for connected component #%1/%2...")
                        .arg(cc).arg(LabelsNewConstrUnique.size())
                     << black;
            solve_subtask_greedy(cc); // in [augment state]
        }
    } // if ( num_new_constraints_ > 0 )

    // (3) snap all segments adjacent to new segment .............
    snap_endpoints( num_new_constraints_ );
}


Index impl::find_new_constraints()
{
    // qDebug() <<  Q_FUNC_INFO;
    const Index previously = m_constr.length();

    const Index c = Adj.rows()-1;

    // orthogonality & identity .......................................
    /* if ( considerOrthogonality | considerIdentity ) {
        for ( SparseMatrix<int>::InnerIterator it(Adj,c) ; it; ++it) {
            Q_ASSERT( it.value()==1 );
            int a = it.index(); // a is direct neighbor of c.

            // a and c: potentially orthogonal or identical, not both
            if ( considerOrthogonality )
                if ( are_orthogonal( a, c) ) {
                    establish_orthogonality( a, c);
                    continue; // (a, c) are not identical, too.  !!!
                }
            if ( considerIdentity )
                if ( are_identical( a, c))
                    establish_identity( a, c);
        }
    }*/

    // unary relations ......................................
    if ( State::considerVertical() ) {
        if ( is_vertical(c)) {
            establish_vertical(c);
        }
    }
    if ( State::considerHorizontal() ) {
        if ( is_horizontal(c)) {
            establish_horizontal(c);
        }
    }
    if ( State::considerDiagonal() ) {
        if ( is_diagonal(c)) {
            establish_diagonal(c);
        }
    }


    if (  State::considerOrthogonal() ) {
        for ( SparseMatrix<int>::InnerIterator it(Adj,c) ; it; ++it) {
            Q_ASSERT( it.value()==1 );
            const int a = it.index(); // a is direct neighbor of c.
            // a and c: potentially orthogonal or identical, not both
            if ( are_orthogonal( a, c) ) {
                establish_orthogonal( a, c);
            }
        }
    }


    // if square: number of walks of length 2
    SparseMatrix<int> WW = Adj*Adj;


    // concurrence & parallelism (1) ..................................
    if ( State::considerCopunctual() || State::considerParallel() ) {
        // Find walks of length 2 between the vertices of the graph.
        const VectorXidx nbs = spfind<int>( WW.col(c) );
        for ( Index n=0; n<nbs.rows()-1; n++) {
            const Index a = nbs(n);  // [a] is a walk of length 2 away from [c].

            if ( !Adj.isSet(a,c) ) {
                // [a] and [c] are no neighbors.
                if ( State::considerParallel() ) {
                    // Check if [a] and [c] are parallel.
                    if ( !are_identical(a,c) && are_parallel( a, c) ) {
                        establish_parallel( a, c);
                    }
                }
                continue;
            }

            if ( State::considerCopunctual() ) {
                // [a] and [c] are adjacent and have at least one common neighbor.
                // Find all common neighbors of [a] and [c].
                const VectorXidx nbnb = spfind<int>( Adj.col(a) ); // neighbors of [a].
                for ( Index m=0; m<nbnb.rows()-1; m++) {
                    const Index b = nbnb(m);

                    if ( !Adj.isSet( b,c ) ) {
                        continue;   // [b] is not a common neighbor of [a] and [c].
                    }

                    // [b] is neighbor of [a] and [c].

                    if ( b > a ) {
                        if ( !are_identical(b,c) && !are_identical(a,c) && !are_identical(a,b) ) {
                            if ( are_copunctual( a, b, c) ) {
                                establish_copunctual( a, b, c );
                            }
                        }
                    }
                }
            }

        }
    }

    // parallelism (2) ......................................
    // case: straight line segment connects two segments
    const VectorXidx idx = spfind<int>( Adj.col(c) ); // neighbors of [c]

    // Check all pairs of neighbors.
    for ( Index i=0; i<idx.rows(); i++) {
        const Index a = idx(i);
        for ( Index j=i+1; j<idx.rows(); j++) {
            const Index b = idx(j);
            if ( WW.coeffRef(a,b) == 1 ) {
                // qDebug().noquote() << QString("There is just one walk of length 2 between [%1} and [%2].").arg(idx(i)).arg(idx(j));
                if ( are_identical(a,b) ) {
                    qDebug() << "bug\n";
                    assert( 0 && "bug!");
                    // establish_identical(a,b);
                }
                else {
                    if ( State::considerParallel() ) {
                        if ( are_parallel(a,b) ) {
                            establish_parallel(a,b);
                        }
                    }
                }
            }
        }
    }

    return m_constr.length() -previously;
}

void impl::solve_subtask_greedy( const int cc )
{
    // qDebug() << Q_FUNC_INFO;

    const VectorXidx maps_ = find(   arr_segm==cc );
    const VectorXidx mapc_ = find( arr_constr==cc );
    assert( mapc_.size()>0 );

    AdjustmentFramework a{ a_Maker( maps_)};

    //qDebug().noquote().nospace() << red << "greedy search..." << black;
    bool last_constraint_required = false;
    for ( Index c=0; c<mapc_.size(); c++) {

        if ( m_constr.at( mapc_(c) )->unevaluated() )
        {
            qDebug().noquote().nospace() << blue
                                         << QStringLiteral("Greedy search: adding constraint #%1 (%2) tentatively.")
                                            .arg(c+1).arg( m_constr.at(mapc_(c))->type_name()) << black;

            // add constraint tentative and  check dependency/consistency:
            m_constr.at( mapc_(c) )->setStatus( ConstraintBase::REQUIRED );

            // enforce constraints (adjustment)
            last_constraint_required =  a.enforce_constraints( m_constr, Rel,
                                                               maps_, mapc_ );

            if ( !last_constraint_required ) {
                m_constr.at( mapc_(c) )->setStatus( ConstraintBase::OBSOLETE );
            }
        }
    }

    // adjustment with consistent set of constraints ..................
    if ( !last_constraint_required ) {
        const Index C = number_of_required_constraints();
        qDebug().noquote() << blue << QStringLiteral("final adjustment with *%1* %2.")
                              .arg( C)
                              .arg( C==1 ? "constraint" : "consistent constraints" ) << black;
        // number of equations (not constraints!):
        // const int E = number_of_required_equations( mapc_ );  TODO
        last_constraint_required = a.enforce_constraints( m_constr, Rel,
                                                          maps_, mapc_);
    }
    // Q_ASSERT_X( last_constraint_required, "greedy search", "final adjustment with inconsistent set");
    if ( !last_constraint_required ) {
        qDebug().noquote() << "-> final adjustment with inconsistent set";
        QMessageBox msg(nullptr);
        msg.setIcon(            QMessageBox::Warning );
        msg.setWindowTitle(     QApplication::applicationName() );
        msg.setText(            QStringLiteral("Ooops, final adjustment with inconsistent set." ));
        msg.setStandardButtons( QMessageBox::Ok );
        msg.exec();
    }

    // update segments: project end points onto adjusted line ..............................

    qDebug() << "update segments...";
    update_segments( maps_ , a);
}


//! project end points of segments onto adjusted lines
void impl::update_segments( const VectorXidx & maps_,
                            const AdjustmentFramework & a)
{
    for ( Index s=0; s<maps_.size(); s++ ) {
        //  straight line:  s-th 3-vector
        const std::pair<Vector3d,Matrix3d> p = a.getEntity(s);
        const uStraightLine ul( p.first, p.second );

        const uPoint ux = m_segm.at( maps_(s) )->ux();
        const uPoint uy = m_segm.at( maps_(s) )->uy();

        const uPoint ua = ul.project(ux).sphericalNormalized();
        const uPoint ub = ul.project(uy).sphericalNormalized();

        // qDebug() << QString("subtask: replace segment %1 due to adjustment").arg(s);
        const auto us = std::make_shared<uStraightLineSegment>(ua,ub);
        m_segm.replace( maps_(s), us);

    }
}


void impl::snap_endpoints( const Index numNewConstr)
{

    // qDebug() << Q_FUNC_INFO;

    VectorXi LabelsNew = arr_constr.tail(numNewConstr);
    LabelsNew.conservativeResize( LabelsNew.rows()+1 );
    LabelsNew.coeffRef(LabelsNew.rows()-1) = arr_segm.tail(1)(0);
    const VectorXi LabelsNewUnique = unique(LabelsNew);

    for ( const int cc : LabelsNewUnique ) {
        qDebug().noquote() << blue << QString("  snap subtask %1/%2")
                              .arg( cc+1 ).arg( LabelsNewUnique.size() );

        const VectorXidx m = find( arr_segm==cc );
        for ( const auto s : m) {

            // (1) "is touching" ......................................
            uStraightLineSegment useg( *m_segm.at(s) );
            // uStraightLineSegment useg =  *m_segm.at(s);
            bool changed = false;

            for ( Index n=0; n<x_touches_l.cols(); n++) // but ColMajor
            {
                if ( x_touches_l.isSet( s, n) && !y_touches_l.isSet( s, n) ) {
                    if (useg.move_x_to(m_segm.at(n)->hl())) {
                        changed = true;
                        break;
                    }
                }
            }

            for ( Index n=0; n<y_touches_l.cols(); n++) {
                if ( y_touches_l.isSet( s, n)  && !x_touches_l.isSet( s, n) ) {
                    if (useg.move_y_to(m_segm.at(n)->hl())) {
                        changed = true;
                        break;
                    }
                }
            }

            if ( changed ) {
                auto us = std::make_shared<uStraightLineSegment>(useg);
                m_segm.replace( s, us);
            }

            // (2) "touched by" ................................
            for ( SparseMatrix<int,ColMajor>::InnerIterator it( x_touches_l, s) ; it; ++it) {
                const Index n = it.row(); // neighbor of segment s
                if ( ( arr_segm(n) != cc) && ( !y_touches_l.isSet(it.index(),s) ) ) {
                    auto us = std::make_shared<uStraightLineSegment>(  *m_segm.at(n) );
                    if (us->move_x_to(m_segm.at(s)->hl())) {
                        m_segm.replace( n, us);
                    }
                }
            }

            for (SparseMatrix<int, ColMajor>::InnerIterator it(y_touches_l, s); it; ++it) {
                const Index n = it.row() ; // neighbor of segment s
                if ( ( arr_segm(n) != cc) && ( !x_touches_l.isSet(it.index(),s) )) {
                    auto us = std::make_shared<uStraightLineSegment>( *m_segm.at(n) );
                    if (us->move_y_to(m_segm.at(s)->hl())) {
                        m_segm.replace( n, us);
                    }
                }
            }
        }
    }
}


bool impl::is_vertical( const Index a)
{
    return m_segm.at(a)->ul().isVertical( State::recogn_.quantile_chi2_1dof() );
}


bool impl::is_horizontal( const Index a)
{
    return m_segm.at(a)->ul().isHorizontal( State::recogn_.quantile_chi2_1dof() );
}


bool impl::is_diagonal( const Index a)
{
    return m_segm.at(a)->ul().isDiagonal( State::recogn_.quantile_chi2_1dof() );
}


bool impl::are_copunctual( const Index a, const Index b, const Index c)
{
    return  m_segm.at(c)->ul().isCopunctualWith( m_segm.at(a)->ul(),
                                                 m_segm.at(b)->ul(),
                                                 State::recogn_.quantile_chi2_1dof() );
}


bool impl::are_parallel( const Index a, const Index b)
{
    return  m_segm.at(a)->ul().isParallelTo( m_segm.at(b)->ul(),
                                             State::recogn_.quantile_chi2_1dof() );
}


bool impl::are_orthogonal( const Index a, const Index b)
{
    return  m_segm.at(a)->ul().isOrthogonalTo( m_segm.at(b)->ul(),
                                               State::recogn_.quantile_chi2_1dof() );
}


bool impl::are_identical( const Index a, const Index b)
{
    // (1) pre-check with acute angle
    const double alpha = Geometry::acute( m_segm.at(a)->ul().v(),
                                          m_segm.at(b)->ul().v() );
    constexpr double rho = 180./3.141592653589793; // = 180°/pi = 57.2958°
    constexpr double T_deg_pre_check = 20.0;
    if ( alpha*rho > T_deg_pre_check ) {
        return false;
    }

    // (2) statistical test
    return  m_segm.at(a)->ul().isIdenticalTo( m_segm.at(b)->ul(),
                                              State::recogn_.quantile_chi2_2dof() );
}


void impl::establish_parallel( const Index a,
                               const Index b)
{
    if ( PP.isSet(a,b) ) {
        // qDebug() << "already parallel!";
        // TODO(meijoc) set constraint to be unevaluated! but how??
        return;
    }
    PP.set( a,b);
    PP.set( b,a);

    m_qConstraint.append( QConstraint::QParallel::create() );
    m_constr.append( std::make_shared<Parallel>() );

    Rel.conservativeResize( Rel.rows(), Rel.cols()+1); //   append a column
    Rel.set( a, last );
    Rel.set( b, last );
}

void impl::establish_vertical( const Index a)
{
    m_qConstraint.append( QConstraint::QAligned::create());
    m_constr.append( std::make_shared<Vertical>() );

    Rel.conservativeResize( Rel.rows(), Rel.cols()+1); //  append a column
    Rel.set( a, last );
}

void impl::establish_horizontal( const Index a)
{
    m_qConstraint.append( QConstraint::QAligned::create());
    m_constr.append( std::make_shared<Horizontal>() );

    Rel.conservativeResize( Rel.rows(), Rel.cols()+1); //  append a column
    Rel.set( a, last );
}

void impl::establish_diagonal( const Index a)
{
    m_qConstraint.append( QConstraint::QAligned::create());
    m_constr.append( std::make_shared<Diagonal>() );

    Rel.conservativeResize( Rel.rows(), Rel.cols()+1); //  append a column
    Rel.set( a, last );
}

void impl::establish_orthogonal( const Index a,
                                 const Index b)
{
    m_qConstraint.append( QConstraint::QOrthogonal::create());
    m_constr.append( std::make_shared<Orthogonal>() );

    Rel.conservativeResize( Rel.rows(),
                           Rel.cols()+1); //  append a column
    Rel.set( a, last );
    Rel.set( b, last );
}


void impl::establish_copunctual( const Index a,
                                 const Index b,
                                 const Index c)
{
    m_qConstraint.append( QConstraint::QCopunctual::create() );
    m_constr.append( std::make_shared<Copunctual>() );

    Rel.conservativeResize( Rel.rows(), Rel.cols()+1) ; // append a column
    Rel.set( a, last );
    Rel.set( b, last );
    Rel.set( c, last );
}


/*
void impl::establish_identical( const Index a,  const Index b)
{
    m_qConstraint.append( QConstraint::QIdentical::create() );
    m_constr.append( std::make_shared<Identical>() );

    Bi.conservativeResize( Bi.rows(), Bi.cols()+1 ); // append column
    Bi.set( a, Bi.cols()-1 );   // B(a,end) = 1;
    Bi.set( b, Bi.cols()-1 );   // B(b,end) = 1;
}
*/


std::pair<Eigen::VectorXd, SparseMatrix<double> >
impl::a_Maker( const VectorXidx & maps_) const
{
    // qDebug() << Q_FUNC_INFO;

    const Index S = maps_.size();  // number of segments
    const Index N = 3*S;           // number of observations
    assert( S>0 );
    VectorXd l(N);                 // vector of observations
    SparseMatrix<double,ColMajor> Cov_ll(N,N);         // covariance matrix observations
    Cov_ll.setZero();
    Cov_ll.reserve(3*N);
    int idx = 0; // [~,idx] = max( abs(l(1:2)) )

    for (int s = 0; s < S; s++) {
        uStraightLine const ul = m_segm.at( maps_(s) )->ul().sphericalNormalized();

        Vector3d m = ul.v();

        // align signs consistently
        m.head(2).cwiseAbs().maxCoeff(&idx); // [~,idx] = max( abs(l(1:2)) )
        int const offset3 = 3*s;
        l.segment(offset3,3)  = sign( m(idx) ) * m;   // spherical normalized

        for ( int i=0; i<3; i++ ) {
            for ( int j=0; j<3; j++ ) {
                Cov_ll.insert( offset3+i, offset3+j ) = ul.Cov()(i,j);
            }
        }
    }

    return { l, Cov_ll};
}


void impl::remove_constraint( const Index i )
{

    Q_ASSERT( i>=0 );
    Q_ASSERT( i<m_constr.length() );

    if ( m_constr.at(i)->isInstanceOf<Parallel>() )
    {
        const VectorXidx idx = spfind<int>( Rel.col(i) );
        Q_ASSERT_X( idx.size() == 2, Q_FUNC_INFO,
                    QStringLiteral("parallel with %1 entities")
                    .arg( QString::number(idx.size())).toUtf8() );
        PP.unset( idx(0), idx(1));
        PP.unset( idx(1), idx(0));
    }

    m_constr.removeAt(i);
    Rel.remove_column(i);  // B(:,i)=[];

    m_qConstraint.removeAt(i);
}


Index impl::number_of_required_constraints() const
{
    auto predicate = []( auto & i){return (*i).required();};
    return std::count_if( m_constr.begin(), m_constr.end(), predicate);
}


bool impl::identities_removed()
{
    // qDebug() << Q_FUNC_INFO;

    // loop neighbors a of current segment c.

    bool found = false;

    for ( Index a=Adj.rows()-1; a>=0 ; a-- )   // ! decrement
    {
        // if a and c are neighbors, check for identity
        if ( Adj.isSet( a,last ) ) {
            if ( are_identical(a, Adj.cols()-1)) {
                found = true;

                // merge segment "a" with last segment in list
                // remove involved constraints
                merge_segment(a);

            } // identical
        } // if adjacent
    }

    return found;
}


// uncertaint segment via merged tracks
void impl::merge_segment( const Index a)
{
    // qDebug() << Q_FUNC_INFO << a;

    const QPolygonF merged_track
            = m_qStroke.at(a)->polygon()
            + m_qStroke.last()->polygon();

    m_qStroke.last() = std::make_shared<QEntity::QStroke>( merged_track );

    const std::pair<VectorXd, VectorXd> xiyi = trackCoords(merged_track); // {x_i, y_i}
    const std::pair<uPoint, uPoint> uxuy = uEndPoints(xiyi.first, xiyi.second); // ux, uy

    m_segm.last() = std::make_shared<uStraightLineSegment>(
        uxuy.first, uxuy.second);

    m_qUnconstrained.last() = std::make_shared<QEntity::QUnconstrained>(
        m_segm.last()->ux(),
        m_segm.last()->uy()   );

    m_qConstrained.last() = std::make_shared<QEntity::QConstrained>(
        m_segm.last()->ux(),
        m_segm.last()->uy()   );

    // inherit adjacencies of [a]
    for ( Index i=0; i<Adj.cols()-1; i++) {
        if ( Adj.isSet(i,a) ) {    // column "a" fix
            Adj.set( i, last );
            Adj.set( last, i );
        }
    }

    // delete constraints of segment [a] to be deleted.
    for (Index ic = Rel.cols()-1; ic >= 0; ic--) { // hint: Bi is sparse, but *ColMajor*, loop is inefficient...
        if ( Rel.isSet(a,ic) ) {
            remove_constraint(ic);
        }
    }

    for ( Index ii=0; ii<x_touches_l.cols(); ii++ )  { // but *ColMajor*...
        if ( x_touches_l.isSet( last,ii) ) {
            x_touches_l.unset( last,ii );  // explicit zero !
        }
        if ( y_touches_l.isSet( last,ii) ) {
            y_touches_l.unset( last,ii );
        }
        if ( x_touches_l.isSet(  ii,last) ) {
            x_touches_l.unset( ii,last );
        }
        if ( y_touches_l.isSet(  ii,last) ) {
            y_touches_l.unset( ii,last );
        }
    }

    // remove segment 'a'
    remove_segment(a);
}


QString State::StatusMsg() const
{
    return pImpl()->StatusMsg();
}

QString impl::StatusMsg() const
{
    const Index S = m_segm.length();
    const Index C = m_constr.length();
    const Index R = number_of_required_constraints();
    const int CC = arr_segm.size()>0 ? arr_segm.maxCoeff()+1 : 0;

    const QString s0 = QApplication::tr("%1 connected component%2, ").arg(CC).arg(CC == 1 ? "" : "s");
    const QString s1 = QApplication::tr("%1 segment%2, ").arg(S).arg(S == 1 ? "" : "s");
    const QString s2 = QApplication::tr("%1 of %2 constraint%3 required.").arg(R).arg(C).arg(C == 1 ? "" : "s");

    return s0 + s1 + s2;
}


void impl::identify_subtasks()
{
    const VectorXi bicoco = conncomp( Rel.biadjacency());
    arr_segm   = bicoco.head( m_segm.length()  ).array();
    arr_constr = bicoco.tail( m_constr.length()).array();
}


void impl::reasoning_reduce_and_adjust()
{
    // connected components / subtasks
    identify_subtasks();

    const int number_of_subtasks_ = arr_segm.size()>0 ? arr_segm.maxCoeff()+1 : 0;

    // greedy search
    for ( int cc=0; cc<number_of_subtasks_; cc++ )
    {
        bool greedySearchRequired = false;

        const VectorXidx mapc_ = find( arr_constr==cc );
        for ( auto c : mapc_ ) {
            if ( m_constr.at( c )->obsolete())  {
                greedySearchRequired = true;
                // replace constraint by unevaluated clone
                m_constr.replace( c, m_constr.at( c )->clone() );
                m_constr.at( c )->setStatus( ConstraintBase::UNEVAL );
            }
        }

        if ( greedySearchRequired ) {
            qDebug().nospace() <<  QString("Update of connected component #%1").arg(cc+1);
            solve_subtask_greedy(cc);  // reduce
        }
    }
    // qDebug() <<  Q_FUNC_INFO;
}


void impl::remove_elements()
{
    for ( Index i=m_qConstraint.size()-1; i>=0; i--) {
        if ( m_qConstraint.at(i)->isSelected() ) {
            remove_constraint( i );
        }
    }
    for ( Index i=m_qConstrained.size()-1; i>=0; i--) {
        if ( m_qConstrained.at(i)->isSelected()
             || m_qStroke.at(i)->isSelected()
             || m_qUnconstrained.at(i)->isSelected() ) {
            for (int c=0; c<Rel.cols(); c++) { // Bi is sparse, but ColMajor, loop is inefficient...
                if ( Rel.isSet(i,c) ) {
                    remove_constraint(c);
                    c--;
                }
            }
            remove_segment( i );
        }
    }
}

//! Replace graphics because of adjustment and/or snapping
void impl::replaceGraphics() {

    // qDebug() << Q_FUNC_INFO;
    Q_ASSERT( m_constr.size() == m_qConstraint.size() );
    Q_ASSERT( m_qConstraint.size() == Rel.cols()      );

    // *** Check segments. ***
    // If reference count is 1, the segment has been added or modified.
    // Then the corresponding graphics have to be replaced.
    for ( int s=0; s<m_qConstrained.length(); s++ ) {
        if ( m_segm.at(s).use_count()==1 ) {
            auto q =  std::make_shared<QEntity::QConstrained>(
                        m_segm.at(s)->ux(),
                        m_segm.at(s)->uy() );
            q->setPen( m_qConstrained.at(s)->pen() );
            m_qConstrained.replace( s, q);
        }
    }
    // *** Check constraints. ***
    // If any of the involved segements have been modified,
    // the constraint has to be modified/replaced, too.
    for( int c=0; c<m_constr.length(); c++)
    {
        bool modified = false;
        const VectorXidx idxx = spfind<int>( Rel.col(c) );

        if ( m_constr.at(c).use_count()==1 ) {
            modified = true; // actually not modified, but added
        }
        else {
            for ( int s=0; s<idxx.size(); s++) {
                if ( m_segm.at(s).use_count()==1 ) {
                    modified = true;
                    break;
                }
            }
        }

        if ( modified ) {
            auto q = m_qConstraint.at(c)->clone();
            Q_ASSERT( idxx.size()>0 && idxx.size()<4 ); // {1,2,3}-ary
            q->setStatus(   m_constr.at(c)->required(),
                            m_constr.at(c)->enforced() );
            q->setGeometry( m_segm, idxx  );
            m_qConstraint.replace( c, q);
        }
    }

}


void impl::append( const QPolygonF & track)
{
    m_qStroke.append( std::make_shared<QEntity::QStroke>( track) );

    // end-points of straight line segment approximating the stroke
    const std::pair<VectorXd, VectorXd> xiyi = trackCoords(track);
    const std::pair<uPoint, uPoint> uxuy = uEndPoints(xiyi.first, xiyi.second);

    // initially constrained==unconstrained

    m_qUnconstrained.append( std::make_shared<QEntity::QUnconstrained>(
                               uxuy.first, uxuy.second
                               ) );

    m_qConstrained.append( std::make_shared<QEntity::QConstrained>(
                             uxuy.first, uxuy.second
                             ) );

    m_segm.append(  std::make_shared<uStraightLineSegment>( uxuy.first, uxuy.second ) );

    Adj.augment();      // adjacency matrix
    x_touches_l.augment();
    y_touches_l.augment();
    Rel.conservativeResize( Rel.rows()+1, Rel.cols()  );     //append 1 row,  relations: +1 segment
    PP.augment();
}

void impl::setAltColors() const
{
    const int N = arr_segm.size()>0 ? arr_segm.maxCoeff()+1 : 0;

    for (int cc=0; cc<N; cc++) {

        // (0) color ...
        const int hue = 359 * cc / N;
        assert( hue>=0 && hue<= 359 );
        const QColor col =  QColor::fromHsv( hue,255,255, 255);  // hue, saturation, value, alpha

        // (1) segments ...
        const VectorXidx idx_s = find( arr_segm==cc );
        for ( const auto s : idx_s ) {
            m_qConstrained.at( s )->setAltColor(col);
        }

        // (2) constraints ...
        const VectorXidx idx_c = find( arr_constr==cc );
        for ( const auto c : idx_c ) {
            m_qConstraint.at( c )->setAltColor( col );
        }
    }
}

bool State::deserialize( QDataStream & in)
{
    return pImpl()->deserialize( in);
}

void State::toggleVisibilityStrokes()
{
    pImpl()->toggleVisibilityStrokes();
}

void impl::toggleVisibilityStrokes()
{
    for ( auto & item : m_qStroke) {
        item->setVisible( !item->isVisible() );
    }
}

void State::toggleVisibilityConstraints()
{
    pImpl()->toggleVisibilityConstraints();
}

void impl::toggleVisibilityConstraints()
{
    for ( auto & item : m_qConstraint) {
        item->setVisible( !item->isVisible() );
    }
}


void State::toggleVisibilityConstrained()
{
    pImpl()->toggleVisibilityConstrained();
}

void impl::toggleVisibilityConstrained()
{
    for ( auto & item : m_qConstrained) {
        item->setVisible( !item->isVisible() );
    }
}

void State::toggleVisibilityUnconstrained()
{
    pImpl()->toggleVisibilityUnconstrained();
}

void impl::toggleVisibilityUnconstrained()
{
    for ( auto & item : m_qUnconstrained) {
        item->setVisible( !item->isVisible());
    }
}

void impl::clearAll()
{
    m_segm.clear();
    m_constr.clear();

    m_qStroke.clear();
    m_qConstrained.clear();
    m_qUnconstrained.clear();
    m_qConstraint.clear();

    x_touches_l.resize(0,0);
    y_touches_l.resize(0,0);
    Adj.resize(0,0);
    PP.resize(0,0);
    Rel.resize(0,0);
}

std::pair<VectorXd, VectorXd> impl::trackCoords(const QPolygonF &poly)
{
    const Index N = poly.length();
    VectorXd xi(N);
    VectorXd yi(N);
    for ( int i=0; i<N; i++ ) {
        xi(i) = poly.at(i).x();
        yi(i) = poly.at(i).y();
    }

    return { xi/m_scale, yi/m_scale };
}



std::pair<uPoint,uPoint> impl::uEndPoints(const VectorXd &xi,
                                          const VectorXd &yi)
{
    Q_ASSERT( xi.size()>0 );
    Q_ASSERT( yi.size()==xi.size() );

    const uStraightLine l = uStraightLine::estim(xi,yi);
    const double phi  = l.angle_rad();
    const VectorXd zi = sin(phi)*xi -cos(phi)*yi;

    int argmin = 0;
    zi.minCoeff( &argmin);
    const Vector3d x1 (xi(argmin), yi(argmin), 1);

    int argmax = 0;
    zi.maxCoeff( &argmax);
    const Vector3d x2 (xi(argmax), yi(argmax), 1);

    const Matrix3d Zeros = Matrix3d::Zero(3,3);

    const uDistance ud1 = uPoint(x1, Zeros).distanceEuclideanTo(l);
    const Matrix3d Cov1_xx = Vector3d( ud1.var_d(), ud1.var_d(), 0).asDiagonal(); // isotropic

    const uDistance ud2 = uPoint(x2, Zeros).distanceEuclideanTo(l);
    const Matrix3d Cov2_xx = Vector3d( ud2.var_d(), ud2.var_d(), 0.).asDiagonal();

    const uPoint first_  = l.project(uPoint(x1, Cov1_xx));
    const uPoint second_ = l.project(uPoint(x2, Cov2_xx));

    return { first_, second_};
}
