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

#include "adjustment.h"
#include "conncomp.h"
#include "constraints.h"
#include "mainscene.h"

#include "qconstraints.h"
#include "qsegment.h"
#include "qstroke.h"

#include "state.h"
#include "upoint.h"
#include "usegment.h"
#include "ustraightline.h"

#include <QApplication>
#include <QDebug>
#include <QMessageBox>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Index;
using Eigen::SparseMatrix;
using Eigen::RowVectorXi;

using Constraint::ConstraintBase;
using Constraint::Parallel;
using Constraint::Orthogonal;
using Constraint::Copunctual;
using Constraint::Identical;

using Uncertain::uPoint;
using Uncertain::uStraightLine;
using Uncertain::uStraightLineSegment;

using Graph::IncidenceMatrix;


bool State::considerOrthogonality_ = true;
bool State::considerParallelism_   = true;
bool State::considerConcurrence_   = true;

Quantiles::Recognition State::recogn_;
Quantiles::Snapping    State::snap_;


QDataStream & operator<< (QDataStream & out, const IncidenceMatrix & AA);
QDataStream & operator>> (QDataStream & in,  IncidenceMatrix & AA);

/*QDebug operator << (QDebug d, const Eigen::VectorXi &v);
QDebug operator << (QDebug d, const Eigen::VectorXi &v)
{
    for ( Eigen::Index r=0; r<v.rows(); r++) {
        d << v(r) << " ";
    }
    d << endl;

    return d;
}*/

/*QDebug operator << (QDebug d, const SparseMatrix<int,ColMajor> &MM);
QDebug operator << (QDebug d, const SparseMatrix<int,ColMajor> &MM)
{
    // qDebug() << "sparse:";
    for ( Index r=0; r<MM.rows(); r++) {
        for ( Index c=0; c<MM.cols(); c++) {
            d << MM.coeff(r,c);
        }
        d << endl;
    }

    //    for ( Index k=0; k<MM.outerSize(); ++k)
    //        for (SparseMatrix<int>::InnerIterator it(MM,k); it; ++it)
    //            d << QStringLiteral("(%1,%2) %3\n" ).arg(it.row()).arg(it.col()).arg(it.value());

    return d;
}*/


class impl {

public:
    bool deserialize( QDataStream & in  );
    void serialize(   QDataStream & out ) const;

    QString StatusMsg() const;

    void toggleVisibilityStrokes();
    void toggleVisibilityConstraints();
    void toggleVisibilityConstrained();
    void toggleVisibilityUnconstrained();

    void clearAll();

    //augment
    void append( const QPolygonF & track);
    void reasoning_augment_and_adjust( const Quantiles::Snapping &snap);

    // reduce
    void remove_elements();
    void reasoning_reduce_and_adjust();

    // augment & reduce
    void replaceGraphics();
    void graphicItemsAdd( QGraphicsScene *sc) const;

private:
    // augment
    int find_new_constraints();
    VectorXi find_in_sparse_column( const SparseMatrix<int> &AA, int k);
    void find_adjacencies_of_latest_segment(const Quantiles::Snapping &snap);
    void merge_segment ( int a);
    bool identities_removed();
    void snap_endpoints( int nnc);

    // reduce
    void remove_constraint( int i );
    void remove_segment( int i );

    // augment & reduce
    void search_subtask( const Eigen::RowVectorXi & mapc_,
                         const Eigen::RowVectorXi & maps_);

    std::pair<VectorXd,VectorXd> trackCoords( const QPolygonF & poly) const;

    void setAltColors() const;

    bool are_converging( int a, int b, int c);
    bool are_parallel(   int a, int b);
    bool are_orthogonal( int a, int b);
    bool are_identical(  int a, int b);

    void establish_convergence(   int a, int b, int c );
    void establish_parallelism(   int a, int b );
    void establish_orthogonality( int a, int b );
    void establish_identity(      int a, int b );

    std::pair<Eigen::VectorXd, SparseMatrix<double> >
    a_Maker( const Eigen::RowVectorXi & maps_ ) const;

    int number_of_required_constraints() const; // for statistics only


    QList< std::shared_ptr< const uStraightLineSegment> > segm_;
    QList< std::shared_ptr< Constraint::ConstraintBase> >             constr_;
    QList< std::shared_ptr< QEntity::QStroke> >         qStroke;
    QList< std::shared_ptr< QEntity::QUnconstrained> >  qUnconstrained;
    QList< std::shared_ptr< QEntity::QConstrained> >    qConstrained;
    QList< std::shared_ptr< QConstraint::QConstraintBase> > qConstraint;

    Graph::IncidenceMatrix Adj;          // Adjacency of straight line segments
    Graph::IncidenceMatrix Bi;           // Present relation of segment (row) and constraint (column)
    Graph::IncidenceMatrix x_touches_l;  // "End-point x touches straight line l."  TODO transpose!?
    Graph::IncidenceMatrix y_touches_l;  // "End-point y touches straight line l."
    Graph::IncidenceMatrix PP;           // parallelism

    VectorXi unique( const VectorXi &x) const;

    template <typename T>
    inline int sign( T val) const { return (T(0) < val) - (val < T(0)); }
};



VectorXi impl::find_in_sparse_column( const SparseMatrix<int> &AA, const int k)
{
    int nnz=0;
    for ( SparseMatrix<int>::InnerIterator it(AA,k); it; ++it ) {
        nnz++;
    }

    VectorXi idx(nnz);
    int i=0;
    for ( SparseMatrix<int>::InnerIterator it(AA,k); it; ++it ) {
        idx(i++) = it.index();
    }

    return idx;
}

/*static MatrixXd Rot_ab( const VectorXd &a, const VectorXd &b)
{
    Q_ASSERT( a.size()==b.size());
    Q_ASSERT( fabs( a.norm()-1.) < T_ZERO );
    Q_ASSERT( fabs( b.norm()-1.) < T_ZERO );

    return MatrixXd::Identity( a.size(),a.size())
            +2*b*a.adjoint()
            -(a+b)*(a+b).adjoint()/(1.+a.dot(b));
}*/

/*static MatrixXd null( const VectorXd &xs )
{
    // cf. PCV, eq. (A.120)

    //if ( fabs(xs.norm()-1.) > T_ZERO )
    //    qDebug() << xs;

    QString what = QStringLiteral("norm(x) = %1").arg( QString::number(xs.norm()) );
    Q_ASSERT_X( fabs(xs.norm()-1.) <= T_ZERO,
                "util::null(x)",
                what.toStdString().data() ) ;

    Index  N = xs.size();

    VectorXd x0 = xs.head(N-1);
    double   xN = xs(N-1);
    if ( xN < 0.0 ) {
        x0 = -x0;
        xN = -xN;
    }

    MatrixXd JJ(N,N-1);
    JJ.topRows(N-1)  = MatrixXd::Identity(N-1,N-1) -x0*x0.adjoint()/(1.+xN);
    JJ.bottomRows(1) = -x0.adjoint();

    VectorXd check = JJ.adjoint()*xs;
    Q_ASSERT_X( check.norm() <= T_ZERO, "nullspace", "not a zero vector");

    return JJ;
}*/


VectorXi impl::unique(const VectorXi &x) const
{
    // qDebug() << Q_FUNC_INFO;

    Q_ASSERT( (x.array()>=0).all() );

    if (x.size()==0) {
        return VectorXi(0);
    }

    int m = x.maxCoeff();
    VectorXi t = VectorXi::Zero(m+1);
    for ( Index i=0; i<x.size(); i++ ) {
        t( x(i) ) = 1;
    }
    int s = t.sum();

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
    : m_pImpl( std::make_unique<impl>(*other.m_pImpl) ) //  new impl( *other.m_pImpl))
{
    // qDebug() << Q_FUNC_INFO;
}

State::~State()
{
    // qDebug() << Q_FUNC_INFO;
    clearAll();
}

State::State() : m_pImpl( std::make_unique<impl>()) // new impl() ){
{
    // qDebug() << Q_FUNC_INFO;
}



State & State::operator= ( const State & other )
{
    // qDebug() << Q_FUNC_INFO;

    if ( &other==this ) {

        return *this;
    }

    // m_pImpl.reset( new impl(* other.m_pImpl));
    m_pImpl = std::make_unique<impl>( *other.m_pImpl);
    /*qStroke = other.qStroke;
    qUnconstrained  = other.qUnconstrained;
    qConstrained    = other.qConstrained;
    qConstraint     = other.qConstraint;

    segm_    = other.segm_;
    constr_ = other.constr_;

    Adj  = other.Adj;
    Bi   = other.Bi;
    PP   = other.PP;
    x_touches_l  = other.x_touches_l;
    y_touches_l  = other.y_touches_l;*/

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

/*QDataStream & operator>> ( QDataStream & in, IncidenceMatrix & AA );
QDataStream & operator>> ( QDataStream & in, IncidenceMatrix & AA )
{
    // qDebug() << Q_FUNC_INFO;

    uint nrows;
    uint ncols;
    uint nnz;
    int r;
    int c;
    in >> nrows >> ncols >> nnz;

    qDebug()<< nrows << ncols << nnz;

    assert( nnz < UINT_MAX);

    std::vector< Eigen::Triplet<int> > tripletList;
    tripletList.reserve( nnz );
    for ( uint i=0; i<nnz; i++ ) {
        in >> r >> c;
        tripletList.emplace_back( Eigen::Triplet<int>(r, c, 1) );
    }
    assert( in.status() == 0 );

    AA.resize( int(nrows),int(ncols) );
    AA.setFromTriplets( tripletList.begin(), tripletList.end() );

    qDebug() << AA;
    //    for ( int ir=0; ir<AA.rows(); ir++) {
    //        for ( int ic=0; ic< AA.cols(); ic++ ) {
    //            qDebug().noquote().nospace() << AA.coeff(ir,ic);
    //        }
    //        qDebug() << "\n";
    //    }
    return in;
}*/

bool impl::deserialize( QDataStream &in )
{
    // qDebug() <<  Q_FUNC_INFO;

    // st_clear();  //  required? st: new()


    qDebug() << "(1) reading topology...";
    in >> Adj;
    in >> Bi;
    in >> x_touches_l;
    in >> y_touches_l;
    in >> PP;
    //    if ( !Adj.deserialize( in ) ) {
    //        return false;
    //    }
    //    if ( !Bi.deserialize( in ) ) {
    //        return false;
    //    }
    //    if ( !x_touches_l.deserialize( in ) ) {
    //        return false;
    //    }
    //    if ( !y_touches_l.deserialize( in ) ) {
    //        return false;
    //    }
    //    if ( !PP.deserialize( in ) ) {
    //        return false;
    //    }


    qDebug() << "(2) reading geometry...";
    for ( Index s=0; s<Bi.rows(); s++ ) {
        // auto seg = std::make_shared<uStraightLineSegment>();
        auto seg = uStraightLineSegment::create();
        if ( !seg->deserialize( in ) ) {
            return false;
        }
        segm_.append( seg );
    }

    for ( Index i=0; i<Bi.cols(); i++ ) {
        auto c = ConstraintBase::deserialize(in);  // calls 'create'
        if ( c==nullptr ) {
            return false;
        }
        constr_.append( c );
    }

    qDebug() << "(3) graphics...";

    qDebug() << "    constraints...";
    for ( const auto & con : qAsConst( constr_)) {

        if ( con->is<Orthogonal>() ) {
            auto q = QConstraint::QOrthogonal::create();
            if ( !q->deserialize( in ) ) {
                return false;
            }
            qConstraint.append( q);
        }
        if ( con->is<Parallel>() ) {
            auto q = QConstraint::QParallel::create();
            if ( !q->deserialize( in ) ) {
                return false;
            }
            qConstraint.append( q);
        }
        if ( con->is<Copunctual>() ) {
            auto q = QConstraint::QCopunctual::create();
            if ( !q->deserialize( in ) ) {
                return false;
            }
            qConstraint.append( q);
        }
    }

    qDebug() << "    strokes ...";
    for ( Index i=0; i<Bi.rows(); i++) {
        auto q = std::make_shared<QEntity::QStroke>();
        if( !q->deserialize(in) ) {
            return false;
        }
        qStroke.append( q);
    }

    qDebug() << "   unconstrained segments...";
    for ( Index i=0; i<Bi.rows(); i++) {
        auto q = std::make_shared<QEntity::QUnconstrained>();
        if( !q->deserialize(in) ) {
            return false;
        }
        qUnconstrained.append( q);
    }

    qDebug() << "   constrained segments...";
    for ( Index i=0; i<Bi.rows(); i++) {
        auto q = std::make_shared<QEntity::QConstrained>();
        if( !q->deserialize(in) ) {
            return false;
        }
        qConstrained.append( q);
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
    for ( const auto & item : qStroke)
    {
        Q_ASSERT( !item->scene() );
        sc->addItem( item.get() );
    }
    for ( const auto & item : qUnconstrained)
    {
        Q_ASSERT( !item->scene() );
        sc->addItem( item.get() );
    }
    for ( const auto & item : qConstrained)
    {
        Q_ASSERT( !item->scene() );
        sc->addItem( item.get() );
    }
    for ( const auto & item : qConstraint)
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

/*QDataStream & operator<< (QDataStream & out, const IncidenceMatrix & AA);
QDataStream & operator<< (QDataStream & out, const IncidenceMatrix & AA)
{
    // qDebug() << Q_FUNC_INFO;

    out <<  uint( AA.rows() );
    out <<  uint( AA.cols() );
    out <<  uint( AA.nonZeros() );

    for( Index c=0; c<AA.outerSize(); ++c) {
        SparseMatrix<int,ColMajor>::InnerIterator it( AA, c);
        for( ; it; ++it) {
            out << int(it.row()) << int(it.col());
        }
    }

    return out;
}*/

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
    //    Adj.serialize( out );
    //    Bi.serialize(  out );
    //    x_touches_l.serialize( out );
    //    y_touches_l.serialize( out );
    //    PP.serialize( out );

    // (2) geometry
    for ( const auto & item : segm_) {
        item->serialize( out );
    }
    for ( const auto & item : constr_) {
        item->serialize( out );
    }

    // (3) graphics
    for ( auto & item : qConstraint) {
        item->serialize( out );
    }
    for ( const auto & item : qStroke) {
        item->serialize( out );
    }
    for ( const auto & item : qUnconstrained) {
        item->serialize( out );
    }
    for ( const auto & item : qConstrained) {
        item->serialize( out );
    }

    qDebug() << "Export finished.";
}

void impl::find_adjacencies_of_latest_segment( const Quantiles::Snapping & snap )
{
    // qDebug() << Q_FUNC_INFO;

    int N = segm_.length();

    for ( int i=0; i<N-1; i++)
    {
        // pre-check (culling) via axis-aligned bounding boxes
        if ( !segm_.at(i)->bounding_box().overlaps(
                 segm_.last()->bounding_box()) ) {
            continue;
        }


        if ( segm_.last()->touchedBy( segm_.at(i)->ux(),
                                      snap.quantile_stdNormDistr(),
                                      snap.quantile_chi2_1dof()) ) {
            x_touches_l.set( i, N-1);
            Adj.set(i, N-1);
            Adj.set(N-1, i);
        }

        if ( segm_.last()->touchedBy( segm_.at(i)->uy(),
                                      snap.quantile_stdNormDistr(),
                                      snap.quantile_chi2_1dof()) ) {
            y_touches_l.set(i,N-1);
            Adj.set(i, N-1);
            Adj.set(N-1, i);
        }

        if ( segm_.at(i)->touchedBy( segm_.last()->ux() ,
                                     snap.quantile_stdNormDistr(),
                                     snap.quantile_chi2_1dof() ) ) {
            x_touches_l.set(N-1,i);
            Adj.set(i, N-1);
            Adj.set(N-1, i);
        }

        if ( segm_.at(i)->touchedBy( segm_.last()->uy(),
                                     snap.quantile_stdNormDistr(),
                                     snap.quantile_chi2_1dof() ) ) {
            y_touches_l.set(N-1,i);
            Adj.set(i, N-1);
            Adj.set(N-1, i);
        }

        if ( segm_.at(i)->intersects( *segm_.last()) )
        {
            Adj.set(i, N-1);
            Adj.set(N-1, i);
        }
    }
}


void impl::remove_segment(const int i)
{
    // qDebug() << "removing segment #" << i+1;
    // qDebug() << Q_FUNC_INFO;

    segm_.removeAt(i);
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
    int num_new_constraints_ = find_new_constraints();
    assert( num_new_constraints_ >= 0 );

    qDebug() << QString("%1 new constraint%2 found.")
                .arg(num_new_constraints_)
                .arg(num_new_constraints_==1 ? "" : "s");

    // (2) check for independence and consistency ...............
    Graph::ConnComp  CoCoBi( Bi.biadjacency() );


    if ( num_new_constraints_ > 0 ) {

        VectorXi LabelsNewConstrIndividual = CoCoBi.tail( num_new_constraints_);
        VectorXi LabelsNewConstrUnique = unique( LabelsNewConstrIndividual);

        for ( Index k=0; k<LabelsNewConstrUnique.size(); k++) {
            int cc = LabelsNewConstrUnique(k);

            Eigen::RowVectorXi maps_ = CoCoBi.mapHead( cc,  segm_.length());
            Eigen::RowVectorXi mapc_ = CoCoBi.mapTail( cc, constr_.length());

            assert( mapc_.size()> 0);
            qDebug() << blue << QString("Reasoning for connected component #%1/%2...")
                        .arg(cc).arg(LabelsNewConstrUnique.size())
                     << black;

            search_subtask( mapc_, maps_ ); // in [augment state]
        }

    }  // if ( num_new_constraints_ > 0 )

    // (3) snap all segments adjacent to new segment .............
    snap_endpoints( num_new_constraints_ );
}

int impl::find_new_constraints()
{
    // qDebug() <<  Q_FUNC_INFO;
    const int previously = constr_.length();

    const int c = int(Adj.rows()-1);

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

    if (  State::considerOrthogonality() ) {
        for ( SparseMatrix<int>::InnerIterator it(Adj,c) ; it; ++it) {
            Q_ASSERT( it.value()==1 );
            const int a = it.index(); // a is direct neighbor of c.
            // a and c: potentially orthogonal or identical, not both
            if ( are_orthogonal( a, c) ) {
                establish_orthogonality( a, c);
            }
        }
    }


    // if square: number of walks of length 2
    SparseMatrix<int> WW = Adj*Adj;

    // concurrence & parallelism (1) ..................................
    if ( State::considerConcurrence() || State::considerParallelism() ) {
        // Find walks of length 2 between the vertices of the graph.

        VectorXi nbs = find_in_sparse_column( WW, c);
        for ( Index n=0; n<nbs.rows()-1; n++) {
            const int a = nbs(n);  // [a] is a walk of length 2 away from [c].

            if ( !Adj.isSet(a,c) ) {
                // [a] and [c] are no neighbors.
                if ( State::considerParallelism() ) {
                    // Check if [a] and [c] are parallel.
                    if ( !are_identical(a,c) && are_parallel( a, c) ) {
                        establish_parallelism( a, c);
                    }
                }
                continue;
            }

            if ( State::considerConcurrence() ) {
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
                            if ( are_converging( a, b, c) ) {
                                establish_convergence( a, b, c );
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
                    establish_identity(idx(i), idx(j));
                }
                else {
                    if ( State::considerParallelism() ) {
                        if ( are_parallel( idx(i), idx(j) ) ) {
                            establish_parallelism( idx(i), idx(j));
                        }
                    }
                }
            }
        }
    }

    return constr_.length() -previously;
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

        if ( constr_.at( mapc_(c) )->unevaluated() )
        {
            qDebug().noquote().nospace() << blue
                                         << QString("Greedy search: adding constraint #%1 (%2) tentatively.")
                                            .arg(c+1).arg( constr_.at(mapc_(c))->type_name()) << black;

            // add constraint tentative and  check dependency/consistency:
            constr_.at( mapc_(c) )->setStatus( ConstraintBase::REQUIRED );

            // number of equations (not constraints!):
            //const int E = number_of_required_equations( mapc_ ); TODO
            last_constraint_required =  a.enforce_constraints( &constr_,
                                                               &Bi,
                                                               maps_, mapc_ );


            if ( !last_constraint_required ) {
                constr_.at( mapc_(c) )->setStatus( ConstraintBase::OBSOLETE );
            }
        }
    }

    // adjustment with consistent set of constraints ..................
    if ( !last_constraint_required ) {
        const int C = number_of_required_constraints();
        qDebug().noquote() << blue << QString("final adjustment with %1 %2.")
                              .arg( C)
                              .arg( C==1 ? "constraint" : "consistent constraints" ) << black;
        // number of equations (not constraints!):
        // const int E = number_of_required_equations( mapc_ );  TODO
        last_constraint_required = a.enforce_constraints( &constr_, &Bi,
                                                          maps_, mapc_);
    }
    // Q_ASSERT_X( last_constraint_required, "greedy search", "final adjustment with inconsistent set");
    if ( !last_constraint_required ) {
        qDebug().noquote() << "-> final adjustment with inconsistent set";
        QMessageBox msg;
        msg.setIcon(            QMessageBox::Warning );
        msg.setWindowTitle(     QApplication::applicationName() );
        msg.setText(            "Ooops, final adjustment with inconsistent set." );
        msg.setStandardButtons( QMessageBox::Ok );
        msg.exec();
    }

    // update segments: project end points onto adjusted line ..............................

    qDebug() << "update segments...";

    for ( Index s=0; s<maps_.size(); s++ ) {
        //  straight line:  s-th 3-vector
        // const StraightLine ul( a.getEntity(s, 3) );
        std::pair<VectorXd,MatrixXd> p = a.getEntity( s, 3);

        const uStraightLine ul(  static_cast<Vector3d>(p.first),
                                 static_cast<Matrix3d>(p.second) );
        const uPoint ux = segm_.at( maps_(s) )->ux();
        const uPoint uy = segm_.at( maps_(s) )->uy();

        uPoint ua = ul.project(ux).sphericalNormalized();
        uPoint ub = ul.project(uy).sphericalNormalized();

        // qDebug() << QString("subtask: replace segment %1 due to adjustment").arg(s);
        auto us = std::make_shared<uStraightLineSegment>(ua,ub);
        segm_.replace( maps_(s), us);
    }
}

void impl::snap_endpoints( const int nnc)
{
    // qDebug() << Q_FUNC_INFO;

    Graph::ConnComp CoCoBi( Bi.biadjacency() );
    VectorXi LabelsNew = CoCoBi.tail( nnc);  // possibly empty

    LabelsNew.conservativeResize(LabelsNew.rows()+1, LabelsNew.cols());
    LabelsNew.coeffRef(LabelsNew.rows()-1) = CoCoBi.label( segm_.size()-1 );

    VectorXi LabelsNewUnique = unique(LabelsNew);


    for ( Index ll=0; ll<LabelsNewUnique.size(); ll++) {
        int cc = LabelsNewUnique(ll);
        qDebug().noquote() << blue << QString("  snap subtask %1/%2")
                              .arg( cc+1 ).arg( LabelsNewUnique.size() );

        VectorXi m = CoCoBi.mapHead(cc, segm_.size());
        for ( Index i=0; i< m.size(); i++) {
            const int s = m(i); // triggering segment...

            // (1) "is touching" ......................................
            uStraightLineSegment useg( *segm_.at(s) );
            bool changed = false;

            for ( Index n=0; n<x_touches_l.cols(); n++) // but ColMajor
            {
                if ( x_touches_l.isSet( s, n) && !y_touches_l.isSet( s, n) ) {
                    if ( useg.move_x_to( segm_.at( int(n) )->hl() ) ) {
                        changed = true;
                        break;
                    }
                }
            }

            for ( Index n=0; n<y_touches_l.cols(); n++) {
                if ( y_touches_l.isSet( s, n)  && !x_touches_l.isSet( s, n) ) {
                    if ( useg.move_y_to( segm_.at( int(n) )->hl() ) ) {
                        changed = true;
                        break;
                    }
                }
            }

            if ( changed ) {
                auto us = std::make_shared<uStraightLineSegment>(useg);
                segm_.replace( s, us);
            }

            // (2) "touched by" ................................
            for ( SparseMatrix<int,ColMajor>::InnerIterator it( x_touches_l, s) ; it; ++it) {
                Index n = it.row() ; // neighbor of segment s
                if ( ( CoCoBi.label(n) != cc) && ( !y_touches_l.isSet(it.index(),s) ) ) {
                    auto us = std::make_shared<uStraightLineSegment>(*segm_.at( int(n) ));
                    if ( us->move_x_to(  segm_.at(s)->hl()  ) ) {
                        segm_.replace( int(n), us);
                    }
                }
            }

            for ( SparseMatrix<int,ColMajor>::InnerIterator it( y_touches_l, s) ; it; ++it) {
                Index n = it.row() ; // neighbor of segment s
                if ( (CoCoBi.label(n) != cc) && ( !x_touches_l.isSet(it.index(),s) )) {
                    auto us = std::make_shared<uStraightLineSegment>( *segm_.at( int(n)));
                    if ( us->move_y_to(  segm_.at(s)->hl()  ) ) {
                        segm_.replace( int(n), us);
                    }
                }
            }
        }
    }
}

bool impl::are_converging( const int a,
                           const int b,
                           const int c)
{
    return  segm_.at(c)->isCopunctualWith( *segm_.at(a),
                                             *segm_.at(b),
                                             State::recogn_.quantile_chi2_1dof() );
}

bool impl::are_parallel( const int a,
                         const int b)
{
    return  segm_.at(a)->isParallelTo( *segm_.at(b),
                                         State::recogn_.quantile_chi2_1dof() );
}

bool impl::are_orthogonal( const int a,
                           const int b)
{
    return  segm_.at(a)->isOrthogonalTo( *segm_.at(b),
                                           State::recogn_.quantile_chi2_1dof() );
}

bool impl::are_identical( const int a,
                          const int b)
{
    return  segm_.at(a)->straightLineIsIdenticalTo( *segm_.at(b),
                                                           State::recogn_.quantile_chi2_2dof() );
}

void impl::establish_parallelism( const int a,
                                  const int b)
{
    if ( PP.isSet(a,b) ) {
        // qDebug() << "already parallel!";
        // TODO(meijoc) set constraint to be unevaluated! but how??
        return;
    }
    PP.set( a,b);
    PP.set( b,a);

    qConstraint.append( QConstraint::QParallel::create() );
    constr_.append( std::make_shared<Parallel>() );

    Bi.conservativeResize( Bi.rows(),
                           Bi.cols()+1); //   append a column
    Bi.set( a, Bi.cols()-1 ); // B(a,end) = 1;
    Bi.set( b, Bi.cols()-1 ); // B(b,end) = 1;
}


void impl::establish_orthogonality( const int a,
                                    const int b)
{
    qConstraint.append( QConstraint::QOrthogonal::create());
    constr_.append( std::make_shared<Orthogonal>() );

    Bi.conservativeResize( Bi.rows(),
                           Bi.cols()+1); //  append a column
    Bi.set( a, Bi.cols()-1 );   // B(a,end)  = 1;
    Bi.set( b, Bi.cols()-1 );   // B(b,end)  = 1;
}


void impl::establish_convergence( const int a,
                                  const int b,
                                  const int c)
{
    qConstraint.append( QConstraint::QCopunctual::create() );
    constr_.append( std::make_shared<Copunctual>() );

    Bi.conservativeResize( Bi.rows(), Bi.cols()+1) ; // append a column
    Bi.set( a, Bi.cols()-1 ); // B(a,end) = 1;
    Bi.set( b, Bi.cols()-1 ); // B(b,end) = 1;
    Bi.set( c, Bi.cols()-1 ); // B(c,end) = 1;
}


void impl::establish_identity( const int a,  const int b)
{
    qConstraint.append( QConstraint::QIdentical::create() );
    constr_.append( std::make_shared<Identical>() );

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
    int idx;                       // [~,idx] = max( abs(l(1:2)) )

    for ( int s=0; s<S; s++ ) {

        uStraightLine ul = segm_.at( maps_(s) )->ul().sphericalNormalized();

        Vector3d m = ul.v();

        // align signs consistently
        m.head(2).cwiseAbs().maxCoeff( &idx );  // [~,idx] = max( abs(l(1:2)) )
        int offset3 = 3*s;
        l.segment(offset3,3)  = sign( m(idx) ) * m;   // spherical normalized

        for ( int i=0; i<3; i++ ) {
            for ( int j=0; j<3; j++ ) {
                Cov_ll.insert( offset3+i, offset3+j ) = ul.Cov()(i,j);
            }
        }
    }

    return { l, Cov_ll};
}


void impl::remove_constraint( const int i )
{

    Q_ASSERT( i>=0 );
    Q_ASSERT( i<constr_.length() );

    if ( constr_.at(i)->is<Parallel>() )
    {
        VectorXi idx = Bi.findInColumn( i);
        Q_ASSERT_X( idx.size() == 2, Q_FUNC_INFO,
                    QStringLiteral("parallel with %1 entities")
                    .arg( QString::number(idx.size())).toUtf8() );
        PP.unset( idx(0), idx(1));
        PP.unset( idx(1), idx(0));
    }

    constr_.removeAt(i);
    Bi.remove_column(i);  // B(:,i)=[];
}


int impl::number_of_required_constraints() const
{
    auto predicate = []( auto & i){return (*i).required();};
    return std::count_if( constr_.begin(), constr_.end(), predicate);
}

/* int impl::number_of_required_equations( const Eigen::RowVectorXi & mapc_) const
{
    int E = 0;
    for ( Index c=0; c<mapc_.size(); c++ ) {
        if ( constr_.at( mapc_(c) )->required() ) {
            E += constr_.at( mapc_(c) )->dof();
        }
    }
    return E;
}*/

bool impl::identities_removed()
{
    // qDebug() << Q_FUNC_INFO;

    // loop neighbors a of current segment c.

    bool found = false;

    for ( int a=int(Adj.rows()-1); a>=0 ; a-- )   // ! decrement
    {
        // if a and c are neighbors, check for identity
        if ( Adj.isSet( a,Adj.cols()-1 ) ) {
            if (  are_identical( a, int(Adj.cols()-1) ))
            {
                found = true;

                // merge segment "a" with last segment in list
                // remove involved constraints
                merge_segment(a);

            } // identical
        } // if adjacent
    }

    return found;
}

void impl::merge_segment( const int a)
{
    qDebug() << Q_FUNC_INFO << a;

    const int idx = segm_.size()-1;  // zero-based

    QPolygonF merged_track
            = qStroke.at(a)->polygon()
            + qStroke.last()->polygon();


    qStroke.replace( idx, std::make_shared<QEntity::QStroke>( merged_track ) );

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
    std::pair<VectorXd,VectorXd> xiyi = trackCoords( merged_track ); // {x_i, y_i}
    std::pair<uPoint,uPoint> uxuy = Uncertain::uEndPoints( xiyi.first, xiyi.second);   // ux, uy
    // std::pair<uPoint,uPoint> uxuy = Uncertain::uEndPoints( xi, yi);
    auto um = std::make_shared<uStraightLineSegment>( uxuy.first, uxuy.second);
    segm_.replace( idx, um );

    qUnconstrained.replace( idx,
                            std::make_shared<QEntity::QUnconstrained>(
                                segm_.last()->ux(),
                                segm_.last()->uy()
                                ) );

    qConstrained.replace( idx,
                          std::make_shared<QEntity::QConstrained>(
                              segm_.last()->ux(),
                              segm_.last()->uy()
                              ));

    // inherit adjacencies of [a]
    for ( Index i=0; i<Adj.cols()-1; i++) {
        if ( Adj.isSet(i,a) ) {    // column "a" fix
            Adj.set( i, Adj.cols()-1 );
            Adj.set( Adj.rows()-1, i );
        }
    }

    // delete constraints of segment [a] to be deleted.
    for ( int ic=int(Bi.cols()-1); ic>=0; ic-- ) {   // hint: Bi is sparse, but *ColMajor*, loop is inefficient...
        if ( Bi.isSet(a,ic) ) {
            remove_constraint(ic);
            qConstraint.removeAt(ic);
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
    qStroke.removeAt(a);
    qUnconstrained.removeAt(a);
    qConstrained.removeAt(a);
}

QString State::StatusMsg() const
{
    return pImpl()->StatusMsg();
}

QString impl::StatusMsg() const
{
    int S = segm_.length();
    int C = constr_.length();
    int R = number_of_required_constraints();

    Graph::ConnComp CoCoBi( Bi.biadjacency() ); // TODO(meijoc)
    int CC = CoCoBi.number();

    QString s0 = QApplication::tr( "%1 connected component%2, " )
            .arg(CC)
            .arg( CC==1 ? "" : "s");
    QString s1 = QApplication::tr( "%1 segment%2, " )
            .arg(S)
            .arg( S==1 ? "" : "s");
    QString s2 = QApplication::tr( "%1 of %2 constraint%3 required." )
            .arg(R)
            .arg(C)
            .arg(C==1 ? "" : "s");

    return s0 + s1 + s2;
}


void impl::reasoning_reduce_and_adjust() {

    // connected components / subtasks
    Graph::ConnComp CoCoBi( Bi.biadjacency() );
    int number_of_subtasks_ = CoCoBi.number();

    // greedy search
    for ( int cc=0; cc<number_of_subtasks_; cc++ )
    {
        RowVectorXi maps_ = CoCoBi.mapHead( cc,  segm_.length()   );
        RowVectorXi mapc_ = CoCoBi.mapTail( cc,  constr_.length() );

        bool greedySearchRequired = false;

        for ( Index c=0; c<mapc_.size(); c++ ) {
            if ( constr_.at( mapc_(c) )->obsolete())  {
                greedySearchRequired = true;
                // replace constraint
                constr_.replace( mapc_(c),
                                 constr_.at( mapc_(c) )->clone() );
                constr_.at( mapc_(c) )->setStatus( ConstraintBase::UNEVAL );
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
    for ( int i=qConstraint.size()-1; i>=0; i--) {
        if ( qConstraint.at(i)->isSelected() ) {
            remove_constraint( i );
            qConstraint.removeAt( i );
        }
    }
    for ( int i=qConstrained.size()-1; i>=0; i--) {
        if ( qConstrained.at(i)->isSelected()
             || qStroke.at(i)->isSelected()
             || qUnconstrained.at(i)->isSelected() ) {

            for ( int c=0; c<int(Bi.cols()); c++ ) { // Bi is sparse, but ColMajor, loop is inefficient...
                if ( Bi.isSet(i,c) ) {
                    remove_constraint(c);
                    qConstraint.removeAt(c);
                    c--;
                }
            }
            remove_segment( i );
            qStroke.removeAt( i );
            qUnconstrained.removeAt( i );
            qConstrained.removeAt( i );
        }
    }
}

//! Replace graphics because of adjustment and/or snapping
void impl::replaceGraphics() {

    // qDebug() << Q_FUNC_INFO;
    Q_ASSERT( constr_.size() == qConstraint.size() );
    Q_ASSERT( qConstraint.size() == Bi.cols()      );

    // *** Check segments. ***
    // If reference count is 1, the segment has been added or modified.
    // Then the corresponding graphics have to be replaced.
    for ( int s=0; s<qConstrained.length(); s++ ) {
        if ( segm_.at(s).use_count()==1 ) {
            auto q =  std::make_shared<QEntity::QConstrained>(
                        segm_.at(s)->ux(),
                        segm_.at(s)->uy() );
            q->setPen( qConstrained.at(s)->pen() );
            qConstrained.replace( s, q);
        }
    }
    // *** Check constraints. ***
    // If any of the involved segements have been modified,
    // the constraint has to be modified/replaced, too.
    for( int c=0; c<constr_.length(); c++)
    {
        bool modified = false;
        VectorXi idxx = Bi.findInColumn(c);

        if ( constr_.at(c).use_count()==1 ) {
            modified = true; // actually not modified, but added
        }
        else {
            for ( int s=0; s<static_cast<int>(idxx.size()); s++) {
                if ( segm_.at(s).use_count()==1 ) {
                    modified = true;
                    break;
                }
            }
        }

        if ( modified ) {
            auto q = qConstraint.at(c)->clone();
            Q_ASSERT( idxx.size()>1 && idxx.size()<4 ); // {2,3}-ary
            q->setStatus(   constr_.at(c)->required(),
                            constr_.at(c)->enforced() );
            q->setGeometry( *segm_.at(idxx(0)),
                            *segm_.at(idxx(1))  );
            qConstraint.replace( c, q);
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
    std::pair<VectorXd,VectorXd> xiyi = trackCoords( track );
    std::pair<uPoint,uPoint> uxuy = Uncertain::uEndPoints( xiyi.first, xiyi.second);

    // initially constrained==constrained
    qStroke.append( std::make_shared<QEntity::QStroke>( track) );

    qUnconstrained.append( std::make_shared<QEntity::QUnconstrained>(
                               uxuy.first, uxuy.second
                               ) );

    qConstrained.append( std::make_shared<QEntity::QConstrained>(
                             uxuy.first, uxuy.second
                             ) );

    segm_.append(  std::make_shared<uStraightLineSegment>( uxuy.first, uxuy.second ) );

    Adj.augment();      // adjacency matrix
    x_touches_l.augment();
    y_touches_l.augment();
    Bi.conservativeResize( Bi.rows()+1, Bi.cols()  );     //append 1 row,  relations: +1 segment
    PP.augment();
}

void impl::setAltColors() const
{
    // qDebug() << Q_FUNC_INFO;

    Graph::ConnComp CoCoBi( Bi.biadjacency() );
    int N  = CoCoBi.number();
    for (int cc=0; cc<N; cc++) {

        // (0) color ...
        int hue = 359*cc/N;
        assert( hue>=0 && hue<= 359 );
        QColor col =  QColor::fromHsv( hue,255,255,   255);

        // (1) segments ...
        VectorXi idx_s = CoCoBi.mapHead(cc, qConstrained.length());
        for ( Index s=0; s<idx_s.size(); s++) {
            qConstrained.at( idx_s(s) )->setAltColor(col);
        }

        // (2) constraints ...
        VectorXi idx_c = CoCoBi.mapTail(cc, qConstraint.length());
        for ( Index c=0; c<idx_c.size(); c++) {
            qConstraint.at( idx_c(c) )->setAltColor( col );
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
    for ( auto & item : qStroke) {
        item->setVisible( !item->isVisible() );
    }
}

void State::toggleVisibilityConstraints()
{
    pImpl()->toggleVisibilityConstraints();
}

void impl::toggleVisibilityConstraints()
{
    for ( auto & item : qConstraint) {
        item->setVisible( !item->isVisible() );
    }
}


void State::toggleVisibilityConstrained()
{
    pImpl()->toggleVisibilityConstrained();
}

void impl::toggleVisibilityConstrained()
{
    for ( auto & item : qConstrained) {
        item->setVisible( !item->isVisible() );
    }
}

void State::toggleVisibilityUnconstrained()
{
    pImpl()->toggleVisibilityUnconstrained();
}

void impl::toggleVisibilityUnconstrained()
{
    for ( auto & item : qUnconstrained) {
        item->setVisible( !item->isVisible());
    }
}

void impl::clearAll()
{
    segm_.clear();
    constr_.clear();

    qStroke.clear();
    qConstrained.clear();
    qUnconstrained.clear();
    qConstraint.clear();

    x_touches_l.resize(0,0);
    y_touches_l.resize(0,0);
    Adj.resize(0,0);
    PP.resize(0,0);
    Bi.resize(0,0);
}



std::pair<VectorXd,VectorXd> impl::trackCoords( const QPolygonF & poly ) const
{
    const int N = poly.length();
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


QDataStream & operator<< (QDataStream & out, const IncidenceMatrix & AA)
{
    qDebug() << Q_FUNC_INFO;

    out <<  uint( AA.rows() );
    out <<  uint( AA.cols() );
    out <<  uint( AA.nonZeros() );

    for( Index c=0; c<AA.outerSize(); ++c) {
        SparseMatrix<int,ColMajor>::InnerIterator it( AA, c);
        for( ; it; ++it) {
            out << int(it.row()) << int(it.col());
        }
    }
    return out;
}


QDataStream & operator>> (QDataStream & in,  IncidenceMatrix & AA)
{
    qDebug() << Q_FUNC_INFO;

    uint nrows;
    uint ncols;
    uint nnz;
    in >> nrows >> ncols >> nnz;

    std::vector< Eigen::Triplet<int> > tripletList;
    tripletList.reserve( nnz );
    int r;
    int c;
    for ( uint i=0; i<nnz; i++ ) {
        in >> r >> c;
        tripletList.emplace_back( Eigen::Triplet<int>(r, c, 1) );
    }

    AA.resize( int(nrows), int(ncols) );
    AA.setFromTriplets( tripletList.begin(),
                        tripletList.end() );

    return in;
}
