/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2023 Jochen Meidow, Fraunhofer IOSB
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
#include "global.h"
#include "mainscene.h"
#include "mainwindow.h"
// #include "uncertain.h"

#include <QDebug>
#include <QGraphicsItem>

namespace Cmd {

GUI::MainScene * Cmd::Undo::s_scene = nullptr;

Undo::Undo( QUndoCommand *parent) : QUndoCommand(parent)
{
    // qDebug() << Q_FUNC_INFO;
}


AddStroke::AddStroke( State *curr,
                      std::unique_ptr<State> &p,
                      std::unique_ptr<State> &n,
                      QUndoCommand *parent) : Undo(parent)
{
    // qDebug() << Q_FUNC_INFO;
    setText( "add stroke" );
    current_state_ = curr;      // set pointer
    prev_state_ = std::move(p);
    next_state_ = std::move(n);
}

void AddStroke::redo()
{
    // qDebug() << Q_FUNC_INFO;
    *current_state_ = *next_state_;    // copy content
    scene()->removeAllItems();
    scene()->addGraphicItems( next_state_.get() );
}

void AddStroke::undo()
{
    // qDebug() << Q_FUNC_INFO;
    *current_state_ = *prev_state_;  // set
    scene()->removeAllItems();
    scene()->addGraphicItems( prev_state_.get() );
}


DeleteSelection::DeleteSelection( State *st,
                                  std::unique_ptr<State> & p,
                                  std::unique_ptr<State> & n,
                                  QUndoCommand *parent) : Undo(parent)
{
    setText( QString("delete selected item%1")
             .arg( scene()->selectedItems().size()==1 ? "" : "s") );

    current_state_ = st;   // pointer
    prev_state_ = std::move(p);
    next_state_ = std::move(n);
}

void DeleteSelection::redo()
{
    *current_state_ = *next_state_;
    scene()->removeAllItems();
    scene()->addGraphicItems( next_state_.get() );
}

void DeleteSelection::undo()
{
    *current_state_ = *prev_state_;
    scene()->removeAllItems();
    scene()->addGraphicItems( prev_state_.get() );
}


TabulaRasa::TabulaRasa( State *st, QUndoCommand *parent ) : Undo(parent)
{
    // qDebug() << Q_FUNC_INFO ;
    setText( "clear all" );
    current_state_ = st;   // pointer
    prev_state_ = std::make_unique<State>(*st);   // copy
    next_state_ = std::make_unique<State>();
    //next_state_ = std::make_unique<State>(*st);
    //next_state_->clearAll();
}


void TabulaRasa::redo()
{
    // qDebug() << Q_FUNC_INFO;
    *current_state_ = *next_state_;
    scene()->removeAllItems();
}

void TabulaRasa::undo()
{
    // qDebug() << Q_FUNC_INFO;
    *current_state_ = *prev_state_;
    scene()->addGraphicItems( prev_state_.get() );
}

ReplaceStateWithFileContent::ReplaceStateWithFileContent( const QString & fileName,
                                                          State *curr,
                                                          std::unique_ptr<State> & p,
                                                          std::unique_ptr<State> & n,
                                                          QUndoCommand *parent)
    : Undo(parent)
{
    // qDebug() << Q_FUNC_INFO;
    setText( fileName );

    current_state_ = curr;   // pointer
    //prev_state_ = std::make_unique<State>(*Old);  // copy
    prev_state_ = std::move(p);  // copy
    next_state_ = std::move(n);
}

void ReplaceStateWithFileContent::redo()
{
    // qDebug() << Q_FUNC_INFO;
    *current_state_ = *next_state_;
    scene()->removeAllItems();
    scene()->addGraphicItems( next_state_.get() );
}

void ReplaceStateWithFileContent::undo()
{
    // qDebug() << Q_FUNC_INFO;
    *current_state_ = *prev_state_;
    scene()->removeAllItems();
    scene()->addGraphicItems( prev_state_.get() );
}

} // namespace Cmd
