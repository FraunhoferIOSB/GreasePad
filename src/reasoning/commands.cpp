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
#include "gui/logger.h"
#include "gui/mainscene.h"
#include "reasoning/state.h"

#include <QDebug>
#include <QGraphicsItem>
#include <QStringLiteral>
#include <QTextCharFormat>
#include <QUndoCommand>

#include <memory>
#include <utility>

namespace Cmd {


GUI::MainScene * Undo::s_scene = nullptr;

Undo::Undo( QUndoStack *stack,
            QUndoCommand *parent,
            std::unique_ptr<State> &p,
            std::unique_ptr<State> &n, State *st)
    :  QUndoCommand(parent),
    next_state_(std::move(n)),
    prev_state_(std::move(p)),
    current_state_(st),
    m_stack(stack)
{
    // qDebug() << Q_FUNC_INFO;
}


void Undo::undo()
{
    // qDebug() << Q_FUNC_INFO;
    Logger::log( Logger::Category::Interaction, "undo: " + text());

    *current_state_ = *prev_state_;  // set
    auto *scn = scene();
    if (scn==nullptr) {
        return;
    }
    scn->removeAllItems();
    scn->addGraphicItems( prev_state_.get() );
}


void Undo::redo()
{
    // qDebug() << Q_FUNC_INFO;
    if ( m_stack->index() != m_stack->count() ) {
        Logger::log( Logger::Category::Interaction, "redo: " + text());
    }

    *current_state_ = *next_state_;    // copy content
    auto *scn = scene();
    if (scn==nullptr) {
        return;
    }
    scn->removeAllItems();
    scn->addGraphicItems( next_state_.get() );
}


AddStroke::AddStroke( QUndoStack *stack,
                      State *curr,
                      std::unique_ptr<State> &p,
                      std::unique_ptr<State> &n,
                      QUndoCommand *parent) : Undo(stack, parent, p, n, curr)
{
    // qDebug() << Q_FUNC_INFO;
    setText( "add a segment" );
    // Logger::log( Logger::Interaction, text());
}


DeleteSelection::DeleteSelection( QUndoStack* stack,
                                  State *st,
                                  std::unique_ptr<State> & p,
                                  std::unique_ptr<State> & n,
                                  QUndoCommand *parent) : Undo(stack, parent, p,n, st)
{
    // qDebug() << Q_FUNC_INFO ;
    auto *scn = scene();
    if (scn==nullptr) {
        return;
    }

    setText( QString( scn->selectedItems().size()==1 ?
                        "delete selected item" : "delete selected items") );
}


TabulaRasa::TabulaRasa( QUndoStack*stack,
                        State *st,
                        std::unique_ptr<State> &p,
                        std::unique_ptr<State> &n,
                        QUndoCommand *parent )
    : Undo(stack, parent, p,n,  st)
{
    // qDebug() << Q_FUNC_INFO ;
    setText( "tabula rasa" );
    Logger::log( Logger::Category::Interaction, text());
}


ReplaceStateWithFileContent::ReplaceStateWithFileContent( QUndoStack *stack, const QString & fileName,
                                                          State *curr,
                                                          std::unique_ptr<State> & p,
                                                          std::unique_ptr<State> & n,
                                                          QUndoCommand *parent)
    : Undo(stack, parent, p,n, curr)
{
    // qDebug() << Q_FUNC_INFO;
    setText( fileName );
    Logger::log( Logger::Category::Interaction, text());
}

} // namespace Cmd
