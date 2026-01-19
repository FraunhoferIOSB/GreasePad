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

#include "commands.h"
#include "mainscene.h"
#include "state.h"

#include <QDebug>
#include <QGraphicsItem>
#include <QStringLiteral>
#include <QUndoCommand>

#include <memory>
#include <utility>

namespace Cmd {

GUI::MainScene * Cmd::Undo::s_scene = nullptr;

Undo::Undo( QUndoCommand *parent, State *st)
    : QUndoCommand(parent), current_state_(st)
{
    // qDebug() << Q_FUNC_INFO;
}


void Undo::undo()
{
    //qDebug() << Q_FUNC_INFO;

    *current_state_ = *prev_state_;  // set
    scene()->removeAllItems();
    scene()->addGraphicItems( prev_state_.get() );
}


void Undo::redo()
{
    //qDebug() << Q_FUNC_INFO;

    *current_state_ = *next_state_;    // copy content
    scene()->removeAllItems();
    scene()->addGraphicItems( next_state_.get() );
}


AddStroke::AddStroke( State *curr,
                      std::unique_ptr<State> &p,
                      std::unique_ptr<State> &n,
                      QUndoCommand *parent) : Undo(parent, curr)
{
    // qDebug() << Q_FUNC_INFO;
    setText( QStringLiteral("add stroke") );

    prev_state_ = std::move(p);
    next_state_ = std::move(n);
}


DeleteSelection::DeleteSelection( State *st,
                                  std::unique_ptr<State> & p,
                                  std::unique_ptr<State> & n,
                                  QUndoCommand *parent) : Undo(parent, st)
{
    // qDebug() << Q_FUNC_INFO ;
    setText( QStringLiteral("delete selected item%1")
             .arg( scene()->selectedItems().size()==1 ? "" : "s") );

    prev_state_ = std::move(p);
    next_state_ = std::move(n);
}


TabulaRasa::TabulaRasa( State *st, QUndoCommand *parent ) : Undo(parent, st)
{
    // qDebug() << Q_FUNC_INFO ;
    setText( QStringLiteral("clear all") );

    prev_state_ = std::make_unique<State>(*st);   // copy
    next_state_ = std::make_unique<State>();
}


ReplaceStateWithFileContent::ReplaceStateWithFileContent( const QString & fileName,
                                                          State *curr,
                                                          std::unique_ptr<State> & p,
                                                          std::unique_ptr<State> & n,
                                                          QUndoCommand *parent)
    : Undo(parent, curr)
{
    // qDebug() << Q_FUNC_INFO;
    setText( fileName );

    prev_state_ = std::move(p);  // copy
    next_state_ = std::move(n);
}

} // namespace Cmd
