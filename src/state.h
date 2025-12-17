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

#ifndef STATE_H
#define STATE_H

#include <memory>

#include "quantiles.h"

#include <QDataStream>
#include <QPolygonF>


class QString;
class QGraphicsScene;

class impl;

//! Class for a set of uncertain straight line segments and geometric relations
class State
{
public:
    State();                                      //!< Standard constructor
    ~State();
    State & operator= ( const State & other);     //!< Copy assignment operator
    State( const State & other);                  //!< Copy constructor
    State & operator= ( const State && other) = delete;     //!< Move assignment operator
    State( const State && other) = delete;                  //!< Move constructor

    void serialize(   QDataStream & out ) const;  //!< Serialization
    bool deserialize( QDataStream & in);          //!< Deserialization
    [[nodiscard]] QString StatusMsg() const;                    //!< Create message for status bar

    bool augment( const QPolygonF & track);  //!< Augment the state by adding a stroke (segment)
    bool reduce();                           //!< Reduce the state by deleteting selected segments and/or constraints
    void clearAll();                         //!< Clear the state, blank state
    void graphicItemsAdd( QGraphicsScene *sc ) const;  //!< Add graphics to the scene

    void toggleVisibilityStrokes();        //!< Toggle the visibility of strokes (point sequences)
    void toggleVisibilityConstraints();    //!< Toggle the visibility of markers deptcting geometric constraints
    void toggleVisibilityConstrained();    //!< Toggle the visibility of constrained segments
    void toggleVisibilityUnconstrained();  //!< Toggle the visibility of unconstrained segments

    //! Toggle consideration of orthogonality (recognition and enforcement)
    static void toggleConsiderOrthogonal() { considerOrthogonal_ = !considerOrthogonal_; }

    //! Toggle consideration of parallelism (recognition and enforcement)
    static void toggleConsiderParallel()   { considerParallel_ = !considerParallel_;   }

    //! Toggle consideration of concurrence (recognition and enforcement)
    static void toggleConsiderCopunctual() { considerCopunctual_ = !considerCopunctual_;   }

    //! Toggle consideration of vertical lines (recognition and enforcement)
    static void toggleConsiderVertical() { considerVertical_ = !considerVertical_;   }

    //! Toggle consideration of horizontal lines (recognition and enforcement)
    static void toggleConsiderHorizontal() { considerHorizontal_ = !considerHorizontal_;   }

    //! Toggle consideration of diagonal lines (recognition and enforcement)
    static void toggleConsiderDiagonal() { considerDiagonal_ = !considerDiagonal_;   }


    //! Get status consideration orthogonality
    static bool considerOrthogonal() { return considerOrthogonal_; }

    //! Get status consideration parallelism
    static bool considerParallel()   { return considerParallel_;   }

    //! Get status consideration concurrence (copunctuality)
    static bool considerCopunctual()   { return considerCopunctual_;  }

    //! Get status consideration vertical/horizontal/diagonal
    static bool considerVertical()   { return considerVertical_;   }
    static bool considerHorizontal() { return considerHorizontal_; }
    static bool considerDiagonal()   { return considerDiagonal_;   }

    //! Set significane level for recognition tasks
    static void setAlphaRecognition( const double alpha) {
        recogn_.setAlpha( Stats::Prob(alpha) );  }

    //! Set significane level for snapping of end points
    static void setAlphaSnapping(    const double alpha) {
        snap_.setAlpha( Stats::Prob(alpha) );  }

    static Quantiles::Recognition recogn_;  //!< Quantiles for recognition
    static Quantiles::Snapping    snap_;    //!< Quantiles for snapping

private:
    static bool considerOrthogonal_;
    static bool considerParallel_;
    static bool considerCopunctual_;
    static bool considerVertical_;
    static bool considerHorizontal_;
    static bool considerDiagonal_;

    impl* pImpl() { return m_pImpl.get(); }
    [[nodiscard]] const impl* pImpl() const { return m_pImpl.get(); }

    std::unique_ptr<impl> m_pImpl;
};

#endif // STATE_H
