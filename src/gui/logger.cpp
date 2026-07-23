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

#include "logger.h"

#include <QStringLiteral>
#include <QTextCharFormat>

#include "qnamespace.h"  // Qt::black, ...
#include "qtmetamacros.h"  // Q_EMIT

class QString;



Logger* Logger::instance()
{
    static Logger s_logger;
    return &s_logger;
}


void Logger::log(Category cat, const QString & msg)
{
    Q_EMIT instance()->messageLogged(cat, msg, formatFor(cat));
}


QString Logger::name(Category cat)
{
    switch (cat) {
    case Category::Application: return QStringLiteral("App");
    case Category::Interaction: return QStringLiteral("Interaction");
    case Category::Testing:     return QStringLiteral("Testing");
    case Category::Reasoning:   return QStringLiteral("Reasoning");
    default:                    return QStringLiteral("unknown");
    }
}


QTextCharFormat Logger::formatFor(Category cat)
{
    QTextCharFormat fmt;
    switch (cat) {
    case Category::Application:
        fmt.setForeground(Qt::black);
        break;
    case Category::Interaction:
        fmt.setForeground(Qt::darkGreen);
        break;
    case Category::Testing:
        fmt.setForeground(Qt::darkRed);
        break;
    case Category::Reasoning:
        fmt.setForeground(Qt::black);
        // fmt.setFontWeight(QFont::Bold);
        break;
    default:
        fmt.setForeground(Qt::black);
    }
    return fmt;
}
