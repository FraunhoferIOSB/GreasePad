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

#ifndef STATE_H
#define STATE_H

#include <memory>

#include "quantiles.h"

#include <QDataStream>

class QString;
class QGraphicsScene;

class impl;

class State
{
public:
    State();
    ~State();
    State & operator= ( const State & other);
    State( const State & other);

private:
    impl* pImpl() { return m_pImpl.get(); }
    const impl* pImpl() const { return m_pImpl.get(); }

    std::unique_ptr<impl> m_pImpl;

public:
    void serialize(   QDataStream & out ) const;
    bool deserialize( QDataStream & in);
    QString StatusMsg() const;

    bool augment( const QPolygonF &track);
    bool reduce();
    void clearAll();
    void graphicItemsAdd( QGraphicsScene *sc ) const;

    void toggleVisibilityStrokes();
    void toggleVisibilityConstraints();
    void toggleVisibilityConstrained();
    void toggleVisibilityUnconstrained();

    static void toggleConsiderOrthogonal() { considerOrthogonality_ = !considerOrthogonality_; }
    static void toggleConsiderParallel()   { considerParallelism_   = !considerParallelism_;   }
    static void toggleConsiderConcurrent() { considerConcurrence_   = !considerConcurrence_;   }

    static bool considerOrthogonality() { return considerOrthogonality_; }
    static bool considerParallelism()   { return considerParallelism_;   }
    static bool considerConcurrence()   { return considerConcurrence_;  }

    static void setAlphaRecognition( const double alpha) { recogn_.setAlpha(alpha);  }
    static void setAlphaSnapping(    const double alpha) {   snap_.setAlpha(alpha);  }

private:
    static bool considerOrthogonality_;
    static bool considerParallelism_;
    static bool considerConcurrence_;

public:
    static Quantiles::Recognition recogn_;  // Quantiles for recognition
    static Quantiles::Snapping    snap_;    //      ... and for snapping.
};

#endif // STATE_H
