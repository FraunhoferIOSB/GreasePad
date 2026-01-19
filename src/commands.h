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

#ifndef COMMANDS_H
#define COMMANDS_H

#include <memory>

#include <QGraphicsScene>
#include <QString>
#include <QUndoCommand>

class State;
namespace  GUI {
class MainScene;
} // namespace GUI


//! Namespace for commands (und/redo)
namespace Cmd {

//! Base class for undo/redo commands
class Undo : public QUndoCommand
{
public:
    Undo( const Undo &) = delete;      //!< Copy constructor
    Undo (Undo &&) = delete;           //!< Move constructor
    Undo & operator= (const Undo & other) = delete; //!< Copy assignment operator
    Undo & operator= (Undo && other) = delete;      //!< Move assignment operator

    //! Set pointer to scene
    static void setScene ( GUI::MainScene * sc) {  s_scene = sc; }
    //! Get pointer to scene
    static GUI::MainScene * scene() { return s_scene;  }

protected:
    explicit Undo( QUndoCommand *parent); //!< Standard constructor
    ~Undo() override = default;

    State *current_state_{};             //!< Pointer to current state
    std::unique_ptr<State> next_state_;  //!< Pointer to next state (redo)
    std::unique_ptr<State> prev_state_;  //!< Pointer to previous state (undo)

private:
    static GUI::MainScene *s_scene;

    void redo() override;
    void undo() override;
};


//! Command 'add a stroke'
class AddStroke : public Undo
{
public:
    //! Value constructor
    AddStroke( State *curr,
               std::unique_ptr<State> & p,
               std::unique_ptr<State> & n,
               QUndoCommand *parent);
    AddStroke( const AddStroke &) = delete;
    AddStroke( AddStroke &&) = delete;
    AddStroke operator= (const AddStroke &) = delete;
    AddStroke operator= ( AddStroke &&) = delete;
    ~AddStroke() override = default;

private:
};


//! Command 'delete selection'
class DeleteSelection : public Undo
{
public:
    //! Value constructor
    DeleteSelection( State *st,
                     std::unique_ptr<State> &p,
                     std::unique_ptr<State> &n,
                     QUndoCommand *parent);
};


//! Command 'clear all'
class TabulaRasa : public Undo
{
public:
    //! Value constructor
    TabulaRasa( State *st, QUndoCommand *parent);
};


//! Command 'replace view with file content'
class ReplaceStateWithFileContent : public Undo
{
public:
    //! Value constructor (filename)
    ReplaceStateWithFileContent( const QString &fileName,
                                 State *curr,
                                 std::unique_ptr<State> &p,
                                 std::unique_ptr<State> &n,
                                 QUndoCommand *parent);
};

} // namespace Cmd

#endif // COMMANDS_H
