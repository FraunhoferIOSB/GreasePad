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

#include "global.h"

#include "qassert.h"
#include "qcolor.h"
#include "qcontainerfwd.h"
#include "qlogging.h"
#include "qstringliteral.h"
#include "qtypes.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "adjustment.h"
#include "conncomp.h"
#include "constraints.h"
#include "mainscene.h"
#include "matrix.h"
#include "qconstraints.h"
#include "qsegment.h"
#include "qstroke.h"
#include "quantiles.h"
#include "state.h"
#include "uncertain.h"
#include "upoint.h"
#include "usegment.h"
#include "ustraightline.h"

#include <QApplication>
#include <QDebug>
#include <QMessageBox>
#include <QPolygonF>
#include <QtCompilerDetection>

#include <Eigen/Core>
#include <Eigen/SparseCore>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::VectorXi;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Index;
using Eigen::SparseMatrix;
using Eigen::RowVectorXi;
using Eigen::ColMajor;


using Constraint::ConstraintBase;
using Constraint::Parallel;
using Constraint::Orthogonal;
using Constraint::Copunctual;
using Constraint::Identical;
using Constraint::Vertical;
using Constraint::Horizontal;
using Constraint::Diagonal;

using Uncertain::uPoint;
using Uncertain::uStraightLine;
using Uncertain::uStraightLineSegment;
using Uncertain::uDistance;

using Graph::IncidenceMatrix;


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
static QDataStream & operator<< ( QDataStream & out, const IncidenceMatrix & AA);
//! Deserialization of sparse incidence matrix
static QDataStream & operator>> ( QDataStream & in,  IncidenceMatrix & AA);

//! Serialization of geometric constraint
static QDataStream & operator<< ( QDataStream & out, const ConstraintBase & c);
//! Deserialization of geometric constraint
static QDataStream & operator>> ( QDataStream & in,  ConstraintBase & c);
} // namespace



//! Implementation details of class 'state' (pImpl idiom)
class impl {

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
    // augment
    Index find_new_constraints();
    static VectorXi find_in_sparse_column( const SparseMatrix<int> &AA, Index k);
    void find_adjacencies_of_latest_segment(const Quantiles::Snapping &snap);
    void merge_segment ( Index a );
    bool identities_removed();
    void snap_endpoints( Index nnc);

    // reduce
    void remove_constraint( Index i );
    void remove_segment(    Index i );

    // augment & reduce
    void search_subtask( const Eigen::RowVectorXi & mapc_,
                         const Eigen::RowVectorXi & maps_);

    //! Estimation of two points delimiting an uncertain straight line segment
    static std::pair<uPoint,uPoint> uEndPoints( const Eigen::VectorXd & xi,
                                                const Eigen::VectorXd & yi);
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
    void establish_identical(  Index a, Index b );

    void establish_copunctual( Index a, Index b, Index c );

    [[nodiscard]] std::pair<Eigen::VectorXd, SparseMatrix<double> >
    a_Maker( const Eigen::RowVectorXi & maps_ ) const;

    [[nodiscard]] Index number_of_required_constraints() const; // for statistics only


    QVector< std::shared_ptr< const uStraightLineSegment> >   m_segm;
    QVector< std::shared_ptr< Constraint::ConstraintBase> >   m_constr;
    QVector< std::shared_ptr< QEntity::QStroke> >        m_qStroke;
    QVector< std::shared_ptr< QEntity::QUnconstrained> > m_qUnconstrained;
    QVector< std::shared_ptr< QEntity::QConstrained> >   m_qConstrained;
    QVector< std::shared_ptr< QConstraint::QConstraintBase> > m_qConstraint;

    Graph::IncidenceMatrix Adj;          // Adjacency of straight line segments
    Graph::IncidenceMatrix Bi;           // Present relation of segment (row) and constraint (column)
    Graph::IncidenceMatrix x_touches_l;  // "End-point x touches straight line l."  TODO transpose!?
    Graph::IncidenceMatrix y_touches_l;  // "End-point y touches straight line l."
    Graph::IncidenceMatrix PP;           // parallelism

    static VectorXi unique( const VectorXi &x) ;

    template <typename T>
    [[nodiscard]] int sign( T val) const { return (T(0) <= val) - (val < T(0)); }   // sign(0) := 1
};



VectorXi impl::find_in_sparse_column( const SparseMatrix<int> &AA, const Index k)
{
    int nnz=0;
    for ( SparseMatrix<int>::InnerIterator it(AA,k); it; ++it ) {
        nnz++;
    }

    VectorXi idx(nnz);
    int i=0;
    for (SparseMatrix<int>::InnerIterator it(AA, k); it; ++it) {
        idx(i++) = it.index();
    }

    return idx;
}


VectorXi impl::unique(const VectorXi &x) 
{
    // qDebug() << Q_FUNC_INFO;

    Q_ASSERT( (x.array()>=0).all() );

    if (x.size()==0) {
        return VectorXi(0);
    }

    int const m = x.maxCoeff();
    VectorXi t = VectorXi::Zero(m + 1);
    for ( Index i=0; i<x.size(); i++ ) {
        t( x(i) ) = 1;
    }
    int const s = t.sum();

    VectorXi u = VectorXi::Constant(s,-1);
    for (int k=0, i=0; i<m+1; i++) {
        if ( t(i)>0 ) {
            u( k++) = i;
        }
    }

    Q_ASSERT( (u.array()>=0).all() );

    return u;
}


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
    in >> Bi;
    in >> x_touches_l;
    in >> y_touches_l;
    in >> PP;

    qDebug().noquote() << "(2) reading geometry...";
    qDebug().noquote() << "(2.1) reading uncertain straight line segments...";
    for ( Index s=0; s<Bi.rows(); s++ ) {
        // auto seg = std::make_shared<uStraightLineSegment>();
        auto seg = uStraightLineSegment::create();
        if ( !seg->deserialize( in ) ) {
            return false;
        }
        m_segm.append( seg );
    }

    qDebug().noquote() << "(2.2) reading geometric constraints...";
    for ( Index i=0; i<Bi.cols(); i++ ) {
        // auto c = ConstraintBase::deserialize(in);  // calls 'create'

        char *type_name = nullptr;
        in >> type_name;
        // qDebug() << Q_FUNC_INFO << type_name;
        if ( in.status()!=0) {
            return false;
        }

        std::shared_ptr<ConstraintBase> c;
        // (1) unary
        if ( std::strcmp( type_name, "vertical")==0 ){
            c = Vertical::create();
        }
        if ( std::strcmp( type_name, "horizontal")==0 ){
            c = Horizontal::create();
        }
        if ( std::strcmp( type_name, "diagonal")==0 ){
            c = Diagonal::create();
        }

        // (2) binary
        if ( std::strcmp( type_name, "orthogonal")==0 ){
            c = Orthogonal::create();
        }
        if ( std::strcmp( type_name, "parallel")==0 ){
            c = Parallel::create();
        }

        // (3) ternary
        if ( std::strcmp( type_name, "copunctual")==0 ){
            c = Copunctual::create();
        }

        if ( c==nullptr ) {
            return false;
        }

        in >> *c;  // .get();

        m_constr.append( c );
    }

    qDebug().noquote() << "(3) graphics...";

    qDebug().noquote() << "    constraints...";
    for ( const auto & con : std::as_const( m_constr)) {

        if ( con->is<Vertical>() ) {
            auto q = QConstraint::QAligned::create();
            if ( !q->deserialize( in ) ) {
                return false;
            }
            m_qConstraint.append( q);
        }

        if ( con->is<Horizontal>() ) {
            auto q = QConstraint::QAligned::create();
            if ( !q->deserialize( in ) ) {
                return false;
            }
            m_qConstraint.append( q);
        }

        if ( con->is<Diagonal>() ) {
            auto q = QConstraint::QAligned::create();
            if ( !q->deserialize( in ) ) {
                return false;
            }
            m_qConstraint.append( q);
        }

        if ( con->is<Orthogonal>() ) {
            auto q = QConstraint::QOrthogonal::create();
            if ( !q->deserialize( in ) ) {
                return false;
            }
            m_qConstraint.append( q);
        }
        if ( con->is<Parallel>() ) {
            auto q = QConstraint::QParallel::create();
            if ( !q->deserialize( in ) ) {
                return false;
            }
            m_qConstraint.append( q);
        }
        if ( con->is<Copunctual>() ) {
            auto q = QConstraint::QCopunctual::create();
            if ( !q->deserialize( in ) ) {
                return false;
            }
            m_qConstraint.append( q);
        }
    }

    qDebug().noquote() << "    strokes ...";
    for ( Index i=0; i<Bi.rows(); i++) {
        auto q = std::make_shared<QEntity::QStroke>();
        if( !q->deserialize(in) ) {
            return false;
        }
        m_qStroke.append( q);
    }

    qDebug().noquote() << "    unconstrained segments...";
    for ( Index i=0; i<Bi.rows(); i++) {
        auto q = std::make_shared<QEntity::QUnconstrained>();
        if( !q->deserialize(in) ) {
            return false;
        }
        m_qUnconstrained.append( q);
    }

    qDebug().noquote() << "    constrained segments...";
    for ( Index i=0; i<Bi.rows(); i++) {
        auto q = std::make_shared<QEntity::QConstrained>();
        if( !q->deserialize(in) ) {
            return false;
        }
        m_qConstrained.append( q);
    }

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
    out << Bi;
    out << x_touches_l;
    out << y_touches_l;
    out << PP;

    // (2) geometry
    for ( const auto & item : m_segm) {
        item->serialize( out );
    }
    for ( const auto & item : m_constr) {
        // item->serialize( out );
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
        // pre-check (culling) via axis-aligned bounding boxes
        if ( !m_segm.at(i)->bounding_box().overlaps(
                 m_segm.last()->bounding_box()) ) {
            continue;
        }


        if ( m_segm.last()->touchedBy( m_segm.at(i)->ux(),
                                      snap.quantile_stdNormDistr(),
                                      snap.quantile_chi2_1dof()) ) {
            x_touches_l.set( i, N-1);
            Adj.set(i, N-1);
            Adj.set(N-1, i);
        }

        if ( m_segm.last()->touchedBy( m_segm.at(i)->uy(),
                                      snap.quantile_stdNormDistr(),
                                      snap.quantile_chi2_1dof()) ) {
            y_touches_l.set(i,N-1);
            Adj.set(i, N-1);
            Adj.set(N-1, i);
        }

        if ( m_segm.at(i)->touchedBy( m_segm.last()->ux() ,
                                     snap.quantile_stdNormDistr(),
                                     snap.quantile_chi2_1dof() ) ) {
            x_touches_l.set(N-1,i);
            Adj.set(i, N-1);
            Adj.set(N-1, i);
        }

        if ( m_segm.at(i)->touchedBy( m_segm.last()->uy(),
                                     snap.quantile_stdNormDistr(),
                                     snap.quantile_chi2_1dof() ) ) {
            y_touches_l.set(N-1,i);
            Adj.set(i, N-1);
            Adj.set(N-1, i);
        }

        if ( m_segm.at(i)->intersects( *m_segm.last()) )
        {
            Adj.set(i, N-1);
            Adj.set(N-1, i);
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
    Bi.remove_row(i);   // B(i,:) = [];

    x_touches_l.reduce(i);
    y_touches_l.reduce(i);
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
    Graph::ConnComp const CoCoBi(Bi.biadjacency());

    if (num_new_constraints_ > 0) {
        VectorXi const LabelsNewConstrIndividual = CoCoBi.tail( num_new_constraints_);
        VectorXi LabelsNewConstrUnique = unique( LabelsNewConstrIndividual);

        for (Index k = 0; k < LabelsNewConstrUnique.size(); k++) {
            int const cc = LabelsNewConstrUnique(k);

            Eigen::RowVectorXi const maps_ = CoCoBi.mapHead(cc, m_segm.length());
            Eigen::RowVectorXi const mapc_ = CoCoBi.mapTail( cc, m_constr.length());

            assert( mapc_.size()> 0);
            qDebug().noquote() << blue << QStringLiteral("Reasoning for connected component #%1/%2...")
                        .arg(cc).arg(LabelsNewConstrUnique.size())
                     << black;

            search_subtask( mapc_, maps_ ); // in [augment state]
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

        VectorXi nbs = find_in_sparse_column( WW, c);
        for ( Index n=0; n<nbs.rows()-1; n++) {
            const int a = nbs(n);  // [a] is a walk of length 2 away from [c].

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
                VectorXi nbnb = Adj.findInColumn(a); // neighbors of [a].
                for ( Index m=0; m<nbnb.rows()-1; m++) {
                    const int b = nbnb(m);

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
    VectorXi idx = Adj.findInColumn(c); // neighbors of [c]

    // Check all pairs of neighbors.
    for ( Index i=0; i<idx.rows(); i++) {
        for ( Index j=i+1; j<idx.rows(); j++) {
            if ( WW.coeffRef( idx(i), idx(j) )==1 ) {
                // qDebug().noquote() << QString("There is just one walk of length 2 between [%1} and [%2].").arg(idx(i)).arg(idx(j));
                if ( are_identical( idx(i), idx(j) ) ) {
                    establish_identical(idx(i), idx(j));
                }
                else {
                    if ( State::considerParallel() ) {
                        if ( are_parallel( idx(i), idx(j) ) ) {
                            establish_parallel( idx(i), idx(j));
                        }
                    }
                }
            }
        }
    }

    return m_constr.length() -previously;
}

void impl::search_subtask( const Eigen::RowVectorXi & mapc_,
                           const Eigen::RowVectorXi & maps_ )
{
    // qDebug() << Q_FUNC_INFO;
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

            // number of equations (not constraints!):
            //const int E = number_of_required_equations( mapc_ ); TODO
            last_constraint_required =  a.enforce_constraints( &m_constr,
                                                               &Bi,
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
        last_constraint_required = a.enforce_constraints( &m_constr, &Bi,
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

    for ( Index s=0; s<maps_.size(); s++ ) {
        //  straight line:  s-th 3-vector
        // const StraightLine ul( a.getEntity(s, 3) );
        std::pair<VectorXd,MatrixXd> const p = a.getEntity( s, 3);

        const uStraightLine ul(  static_cast<Vector3d>(p.first),
                                 static_cast<Matrix3d>(p.second) );
        const uPoint ux = m_segm.at( maps_(s) )->ux();
        const uPoint uy = m_segm.at(maps_(s))->uy();

        uPoint const ua = ul.project(ux).sphericalNormalized();
        uPoint const ub = ul.project(uy).sphericalNormalized();

        // qDebug() << QString("subtask: replace segment %1 due to adjustment").arg(s);
        auto us = std::make_shared<uStraightLineSegment>(ua,ub);
        m_segm.replace( maps_(s), us);
    }
}

void impl::snap_endpoints( const Index nnc)
{
    // qDebug() << Q_FUNC_INFO;

    Graph::ConnComp const CoCoBi( Bi.biadjacency() );
    VectorXi LabelsNew = CoCoBi.tail( nnc);  // possibly empty

    LabelsNew.conservativeResize(LabelsNew.rows()+1, LabelsNew.cols());
    LabelsNew.coeffRef(LabelsNew.rows()-1) = CoCoBi.label( m_segm.size()-1 );

    VectorXi LabelsNewUnique = unique(LabelsNew);

    for (Index ll = 0; ll < LabelsNewUnique.size(); ll++) {
        int const cc = LabelsNewUnique(ll);
        qDebug().noquote() << blue << QString("  snap subtask %1/%2")
                              .arg( cc+1 ).arg( LabelsNewUnique.size() );

        VectorXi m = CoCoBi.mapHead(cc, m_segm.size());
        for ( Index i=0; i< m.size(); i++) {
            const int s = m(i); // triggering segment...

            // (1) "is touching" ......................................
            uStraightLineSegment useg( *m_segm.at(s) );
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
                Index const n = it.row(); // neighbor of segment s
                if ( ( CoCoBi.label(n) != cc) && ( !y_touches_l.isSet(it.index(),s) ) ) {
                    auto us = std::make_shared<uStraightLineSegment>(  *m_segm.at(n));
                    if (us->move_x_to(m_segm.at(s)->hl())) {
                        m_segm.replace( n, us);
                    }
                }
            }

            for (SparseMatrix<int, ColMajor>::InnerIterator it(y_touches_l, s); it; ++it) {
                Index const n = it.row() ; // neighbor of segment s
                if ( (CoCoBi.label(n) != cc) && ( !x_touches_l.isSet(it.index(),s) )) {
                    auto us = std::make_shared<uStraightLineSegment>(
                        *m_segm.at(n)); // static_cast<int>(n)));
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
    return m_segm.at(a)->isVertical(
                State::recogn_.quantile_chi2_1dof()
                );
}

bool impl::is_horizontal( const Index a)
{
    return m_segm.at(a)->isHorizontal(
                State::recogn_.quantile_chi2_1dof()
                );
}

bool impl::is_diagonal( const Index a)
{
    return m_segm.at(a)->isDiagonal(
                State::recogn_.quantile_chi2_1dof() )
            ;
}

bool impl::are_copunctual( const Index a,
                           const Index b,
                           const Index c)
{
    return  m_segm.at(c)->isCopunctualWith( m_segm.at(a)->ul(),
                                            m_segm.at(b)->ul(),
                                            State::recogn_.quantile_chi2_1dof() );
}

bool impl::are_parallel( const Index a,
                         const Index b)
{
    return  m_segm.at(a)->isParallelTo( m_segm.at(b)->ul(),
                                        State::recogn_.quantile_chi2_1dof() );
}

bool impl::are_orthogonal( const Index a,
                           const Index b)
{
    return  m_segm.at(a)->isOrthogonalTo( m_segm.at(b)->ul(),
                                          State::recogn_.quantile_chi2_1dof() );
}


bool impl::are_identical( const Index a,
                          const Index b)
{
    // pre-check with acute angle
    double const alpha = m_segm.at(a)->ul().acute( m_segm.at(b)->ul() );
    if ( alpha*57.2958 > 20.0 ) {  // 180°/pi = 57.2958°
        return false;
    }

    // statistical test
    return  m_segm.at(a)->straightLineIsIdenticalTo( m_segm.at(b)->ul(),
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

    Bi.conservativeResize( Bi.rows(),
                           Bi.cols()+1); //   append a column
    Bi.set( a, Bi.cols()-1 ); // B(a,end) = 1;
    Bi.set( b, Bi.cols()-1 ); // B(b,end) = 1;
}

void impl::establish_vertical( const Index a)
{
    m_qConstraint.append( QConstraint::QAligned::create());
    m_constr.append( std::make_shared<Vertical>() );

    Bi.conservativeResize( Bi.rows(), Bi.cols()+1); //  append a column
    Bi.set( a, Bi.cols()-1 );   // B(a,end)  = 1;
}

void impl::establish_horizontal( const Index a)
{
    m_qConstraint.append( QConstraint::QAligned::create());
    m_constr.append( std::make_shared<Horizontal>() );

    Bi.conservativeResize( Bi.rows(), Bi.cols()+1); //  append a column
    Bi.set( a, Bi.cols()-1 );   // B(a,end)  = 1;
}

void impl::establish_diagonal( const Index a)
{
    m_qConstraint.append( QConstraint::QAligned::create());
    m_constr.append( std::make_shared<Diagonal>() );

    Bi.conservativeResize( Bi.rows(), Bi.cols()+1); //  append a column
    Bi.set( a, Bi.cols()-1 );   // B(a,end)  = 1;
}

void impl::establish_orthogonal( const Index a,
                                 const Index b)
{
    m_qConstraint.append( QConstraint::QOrthogonal::create());
    m_constr.append( std::make_shared<Orthogonal>() );

    Bi.conservativeResize( Bi.rows(),
                           Bi.cols()+1); //  append a column
    Bi.set( a, Bi.cols()-1 );   // B(a,end)  = 1;
    Bi.set( b, Bi.cols()-1 );   // B(b,end)  = 1;
}


void impl::establish_copunctual( const Index a,
                                 const Index b,
                                 const Index c)
{
    m_qConstraint.append( QConstraint::QCopunctual::create() );
    m_constr.append( std::make_shared<Copunctual>() );

    Bi.conservativeResize( Bi.rows(), Bi.cols()+1) ; // append a column
    Bi.set( a, Bi.cols()-1 ); // B(a,end) = 1;
    Bi.set( b, Bi.cols()-1 ); // B(b,end) = 1;
    Bi.set( c, Bi.cols()-1 ); // B(c,end) = 1;
}


void impl::establish_identical( const Index a,  const Index b)
{
    m_qConstraint.append( QConstraint::QIdentical::create() );
    m_constr.append( std::make_shared<Identical>() );

    Bi.conservativeResize( Bi.rows(), Bi.cols()+1 ); // append column
    Bi.set( a, Bi.cols()-1 );   // B(a,end) = 1;
    Bi.set( b, Bi.cols()-1 );   // B(b,end) = 1;
}

std::pair<Eigen::VectorXd, SparseMatrix<double> >
impl::a_Maker( const Eigen::RowVectorXi & maps_) const
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

    if ( m_constr.at(i)->is<Parallel>() )
    {
        VectorXi idx = Bi.findInColumn( i);
        Q_ASSERT_X( idx.size() == 2, Q_FUNC_INFO,
                    QStringLiteral("parallel with %1 entities")
                    .arg( QString::number(idx.size())).toUtf8() );
        PP.unset( idx(0), idx(1));
        PP.unset( idx(1), idx(0));
    }

    m_constr.removeAt(i);
    Bi.remove_column(i);  // B(:,i)=[];
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
        if ( Adj.isSet( a,Adj.cols()-1 ) ) {
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

void impl::merge_segment( const Index a)
{
    // qDebug() << Q_FUNC_INFO << a;

    const Index idx = m_segm.size()-1;  // zero-based

    const QPolygonF merged_track
            = m_qStroke.at(a)->polygon()
            + m_qStroke.last()->polygon();


    m_qStroke.replace( idx, std::make_shared<QEntity::QStroke>( merged_track ) );

    // uncertaint segment via merged tracks
    /* QPolygonF poly = qStroke.at(idx)->polygon();
    const int N = poly.length();
    Eigen::VectorXd xi(N);
    Eigen::VectorXd yi(N);
    for (int i=0; i<N; i++) {
        xi(i) = poly.at(i).x();
        yi(i) = poly.at(i).y();
    }
    xi /= 1000;
    yi /= 1000; */
    std::pair<VectorXd, VectorXd> const xiyi = trackCoords(merged_track); // {x_i, y_i}
    std::pair<uPoint, uPoint> const uxuy = uEndPoints(xiyi.first, xiyi.second); // ux, uy

    auto um = std::make_shared<uStraightLineSegment>( uxuy.first, uxuy.second);
    m_segm.replace( idx, um );

    m_qUnconstrained.replace( idx,
                            std::make_shared<QEntity::QUnconstrained>(
                                m_segm.last()->ux(),
                                m_segm.last()->uy()
                                ) );

    m_qConstrained.replace( idx,
                          std::make_shared<QEntity::QConstrained>(
                              m_segm.last()->ux(),
                              m_segm.last()->uy()
                              ));

    // inherit adjacencies of [a]
    for ( Index i=0; i<Adj.cols()-1; i++) {
        if ( Adj.isSet(i,a) ) {    // column "a" fix
            Adj.set( i, Adj.cols()-1 );
            Adj.set( Adj.rows()-1, i );
        }
    }

    // delete constraints of segment [a] to be deleted.
    for (Index ic = Bi.cols()-1; ic >= 0; ic--) { // hint: Bi is sparse, but *ColMajor*, loop is inefficient...
        if ( Bi.isSet(a,ic) ) {
            remove_constraint(ic);
            m_qConstraint.removeAt(ic);
        }
    }

    for ( Index ii=0; ii<x_touches_l.cols(); ii++ )  { // but *ColMajor*...
        if ( x_touches_l.isSet(  x_touches_l.rows()-1,ii) ) {
            x_touches_l.unset( x_touches_l.rows()-1,ii );  // explicit zero !
        }
        if ( y_touches_l.isSet(  y_touches_l.rows()-1,ii) ) {
            y_touches_l.unset( y_touches_l.rows()-1,ii );
        }
        if ( x_touches_l.isSet(  ii,x_touches_l.cols()-1) ) {
            x_touches_l.unset( ii,x_touches_l.cols()-1 );
        }
        if ( y_touches_l.isSet(  ii,y_touches_l.cols()-1) ) {
            y_touches_l.unset( ii,y_touches_l.cols()-1 );
        }
    }

    // remove segment 'a'
    remove_segment(a);
    m_qStroke.removeAt(a);
    m_qUnconstrained.removeAt(a);
    m_qConstrained.removeAt(a);
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

    Graph::ConnComp const CoCoBi(Bi.biadjacency()); // TODO(meijoc)
    int const CC = CoCoBi.number();

    QString const s0 = QApplication::tr("%1 connected component%2, ").arg(CC).arg(CC == 1 ? "" : "s");
    QString const s1 = QApplication::tr("%1 segment%2, ").arg(S).arg(S == 1 ? "" : "s");
    QString const s2
        = QApplication::tr("%1 of %2 constraint%3 required.").arg(R).arg(C).arg(C == 1 ? "" : "s");

    return s0 + s1 + s2;
}


void impl::reasoning_reduce_and_adjust() {

    // connected components / subtasks
    Graph::ConnComp const CoCoBi(Bi.biadjacency());
    int const number_of_subtasks_ = CoCoBi.number();

    // greedy search
    for ( int cc=0; cc<number_of_subtasks_; cc++ )
    {
        RowVectorXi const maps_ = CoCoBi.mapHead(cc, m_segm.length());
        RowVectorXi mapc_ = CoCoBi.mapTail( cc,  m_constr.length() );

        bool greedySearchRequired = false;

        for ( Index c=0; c<mapc_.size(); c++ ) {
            if ( m_constr.at( mapc_(c) )->obsolete())  {
                greedySearchRequired = true;
                // replace constraint
                m_constr.replace( mapc_(c),
                                 m_constr.at( mapc_(c) )->clone() );
                m_constr.at( mapc_(c) )->setStatus( ConstraintBase::UNEVAL );
            }
        }

        if ( greedySearchRequired ) {
            qDebug().nospace() <<  QString("Update of connected component #%1").arg(cc+1);
            search_subtask( mapc_, maps_ );  // reduce
        }
    }
}

void impl::remove_elements()
{
    for ( Index i=m_qConstraint.size()-1; i>=0; i--) {
        if ( m_qConstraint.at(i)->isSelected() ) {
            remove_constraint( i );
            m_qConstraint.removeAt( i );
        }
    }
    for ( Index i=m_qConstrained.size()-1; i>=0; i--) {
        if ( m_qConstrained.at(i)->isSelected()
             || m_qStroke.at(i)->isSelected()
             || m_qUnconstrained.at(i)->isSelected() ) {
            for (int c = 0; c < /*static_cast<int>(*/Bi.cols();
                 c++) { // Bi is sparse, but ColMajor, loop is inefficient...
                if ( Bi.isSet(i,c) ) {
                    remove_constraint(c);
                    m_qConstraint.removeAt(c);
                    c--;
                }
            }
            remove_segment( i );
            m_qStroke.removeAt( i );
            m_qUnconstrained.removeAt( i );
            m_qConstrained.removeAt( i );
        }
    }
}

//! Replace graphics because of adjustment and/or snapping
void impl::replaceGraphics() {

    // qDebug() << Q_FUNC_INFO;
    Q_ASSERT( m_constr.size() == m_qConstraint.size() );
    Q_ASSERT( m_qConstraint.size() == Bi.cols()      );

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
        VectorXi const idxx = Bi.findInColumn(c);

        if ( m_constr.at(c).use_count()==1 ) {
            modified = true; // actually not modified, but added
        }
        else {
            for ( int s=0; s<static_cast<int>(idxx.size()); s++) {
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
            //q->setGeometry( *m_segm.at(idxx(0)),
            //                *m_segm.at(idxx(1))  );
            q->setGeometry( m_segm, idxx  );
            m_qConstraint.replace( c, q);
        }
    }

}


void impl::append( const QPolygonF & track)
{
    /* int N = track.length();
    VectorXd xi(N);
    VectorXd yi(N);
    for (int i=0; i<N; i++) {
        xi(i) = track.at(i).x();
        yi(i) = track.at(i).y();
    }
    xi /= 1000;
    yi /= 1000; */

    // end-points of straight line segment approximating the stroke
    std::pair<VectorXd, VectorXd> const xiyi = trackCoords(track);
    std::pair<uPoint, uPoint> const uxuy = uEndPoints(xiyi.first, xiyi.second);

    // initially constrained==constrained
    m_qStroke.append( std::make_shared<QEntity::QStroke>( track) );

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
    Bi.conservativeResize( Bi.rows()+1, Bi.cols()  );     //append 1 row,  relations: +1 segment
    PP.augment();
}

void impl::setAltColors() const
{
    // qDebug() << Q_FUNC_INFO;

    Graph::ConnComp const CoCoBi(Bi.biadjacency());
    int const N = CoCoBi.number();
    for (int cc=0; cc<N; cc++) {

        // (0) color ...
        int const hue = 359 * cc / N;
        assert( hue>=0 && hue<= 359 );
        const QColor col =  QColor::fromHsv( hue,255,255,   255);

        // (1) segments ...
        VectorXi idx_s = CoCoBi.mapHead(cc, m_qConstrained.length());
        for ( Index s=0; s<idx_s.size(); s++) {
            m_qConstrained.at( idx_s(s) )->setAltColor(col);
        }

        // (2) constraints ...
        VectorXi idx_c = CoCoBi.mapTail(cc, m_qConstraint.length());
        for ( Index c=0; c<idx_c.size(); c++) {
            m_qConstraint.at( idx_c(c) )->setAltColor( col );
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
    Bi.resize(0,0);
}

std::pair<VectorXd, VectorXd> impl::trackCoords(const QPolygonF &poly)
{
    const Index N = poly.length();
    Eigen::VectorXd xi(N);
    Eigen::VectorXd yi(N);
    for ( int i=0; i<N; i++ ) {
        xi(i) = poly.at(i).x();
        yi(i) = poly.at(i).y();
    }
    xi /= 1000;
    yi /= 1000;
    return {xi, yi};
}



std::pair<uPoint,uPoint> impl::uEndPoints( const Eigen::VectorXd & xi,
                                           const Eigen::VectorXd & yi)
{
    Q_ASSERT( xi.size()>0 );
    Q_ASSERT( yi.size()==xi.size() );

    const uStraightLine l(xi,yi);
    const double phi  = l.angle_rad();
    const VectorXd zi = sin(phi)*xi -cos(phi)*yi;

    int idx = 0;

    Vector3d x1;
    zi.minCoeff( &idx);
    x1 << xi(idx), yi(idx), 1;

    Vector3d x2;
    zi.maxCoeff( &idx);
    x2 << xi(idx), yi(idx), 1;

    const Matrix3d Zeros = Matrix3d::Zero(3,3);
    Matrix3d Cov_xx = Matrix3d::Zero();

    uDistance const ud1 = uPoint(x1, Zeros).distanceEuclideanTo(l);
    Cov_xx.diagonal() << ud1.var_d(), ud1.var_d(), 0.;      // isotropic
    uPoint const first_ = l.project(uPoint(x1, Cov_xx));

    uDistance const ud2 = uPoint(x2, Zeros).distanceEuclideanTo(l);
    Cov_xx.diagonal() << ud2.var_d(), ud2.var_d(), 0.;
    uPoint const second_ = l.project(uPoint(x2, Cov_xx));

    return { first_, second_};
}



namespace {

QDataStream & operator<< (QDataStream & out, const IncidenceMatrix & AA)
{
    qDebug() << Q_FUNC_INFO;

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


QDataStream & operator>> (QDataStream & in,  IncidenceMatrix & AA)
{
    qDebug() << Q_FUNC_INFO;

    uint nrows = 0;
    uint ncols = 0;
    uint nnz = 0;
    in >> nrows >> ncols >> nnz;

    std::vector< Eigen::Triplet<int> > tripletList;
    tripletList.reserve( nnz );
    int r = 0;
    int c = 0;
    for ( uint i=0; i<nnz; i++ ) {
        in >> r >> c;
        // tripletList.emplace_back( Eigen::Triplet<int>(r, c, 1) );
        tripletList.emplace_back( r, c, 1 );
    }

    AA.resize(static_cast<int>(nrows), static_cast<int>(ncols));
    AA.setFromTriplets( tripletList.begin(),
                        tripletList.end() );

    return in;
}


QDataStream & operator<< ( QDataStream & out, const ConstraintBase & c)
{
    // qDebug() <<  Q_FUNC_INFO << c.type_name();
    out << c.type_name();
    out << c.status();    // { UNEVAL=0 | REQUIRED | OBSOLETE };
    out << c.enforced();
    return out;
}

QDataStream & operator>> ( QDataStream &in, ConstraintBase &c )
{
    // qDebug() << Q_FUNC_INFO;
    int status = 0;
    in >> status;
    c.setStatus( static_cast<ConstraintBase::Status>(status) );

    bool enforced = false;
    in >> enforced;
    c.setEnforced( enforced );

    return in;
}
} // namespace

