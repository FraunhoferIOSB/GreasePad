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

#ifndef LOGGER_H
#define LOGGER_H

#include <QObject>
#include <QStringLiteral>
#include <QTextCharFormat>

#include "qtmetamacros.h"


class Logger : public QObject {
    Q_OBJECT

public:
    enum class Category {
        Application,
        Interaction,
        Testing,
        Reasoning
    };
    Q_ENUM(Category)

    static Logger* instance(); // singleton
    static void log(Category cat, const QString &msg);
    static QString name(Category cat);

Q_SIGNALS:
    void messageLogged(Category cat, const QString &msg, const QTextCharFormat &fmt);

private:
    Logger() = default;

    static QTextCharFormat formatFor(Category cat);
};

#endif // LOGGER_H
