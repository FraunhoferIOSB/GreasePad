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

#ifndef COMMANDS_H
#define COMMANDS_H

#include <memory>

#include <QGraphicsScene>
#include <QUndoCommand>

class State;
namespace  GUI {
class MainScene;
}



namespace Cmd {

class Undo : public QUndoCommand
{
public:
    Undo( const Undo &) = delete;
    Undo (Undo &&) = delete;
    Undo & operator= (const Undo & other) = delete;
    Undo & operator= (Undo && other) = delete;

protected:
    Undo();
    ~Undo() override = default;

    State *current_state_{};
    std::unique_ptr<State> next_state_;
    std::unique_ptr<State> prev_state_;

public:
    static void setScene ( GUI::MainScene * sc) {  s_scene = sc; }
    static GUI::MainScene * scene() { return s_scene;  }

private:
    static GUI::MainScene *s_scene;
};


class AddStroke : public Undo
{
public:
    AddStroke( State *curr,
               std::unique_ptr<State> & p,
               std::unique_ptr<State> & n);
    AddStroke( const AddStroke &) = delete;
    AddStroke( AddStroke &&) = delete;
    AddStroke operator= (const AddStroke &) = delete;
    AddStroke operator= ( AddStroke &&) = delete;
    ~AddStroke() override = default;

private:
    void redo() override;
    void undo() override;
};

class DeleteSelection : public Undo
{
public:
    DeleteSelection( State *st,
                     std::unique_ptr<State> &p,
                     std::unique_ptr<State> &n);

private:
    void redo() override;
    void undo() override;
};


class TabulaRasa : public Undo
{
public:
    TabulaRasa( State *st);
private:
    void redo() override;
    void undo() override;
};

class ReplaceStateWithFileContent : public Undo
{
public:
    ReplaceStateWithFileContent( const QString &fileName,
                                 State *curr,
                                 std::unique_ptr<State> &p,
                                 std::unique_ptr<State> &n);
private:
    void redo() override;
    void undo() override;
};

} // namespace Cmd

#endif // COMMANDS_H
