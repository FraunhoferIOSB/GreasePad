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

#ifndef GLOBAL_H
#define GLOBAL_H

// #include <vld.h>

// *.pro
//  INCLUDEPATH += "C:/Program Files (x86)/Visual Leak Detector/include/"
//  LIBS      += -L"C:/Program Files (x86)/Visual Leak Detector/lib/Win64"

#include <QTextStream>

namespace TextColor {

[[nodiscard]] inline QTextStream & green(  QTextStream & out)  { out << "\033[0;32m"; return out; }
[[nodiscard]] inline QTextStream & red(    QTextStream & out)  { out << "\033[0;31m"; return out; }
[[nodiscard]] inline QTextStream & blue(   QTextStream & out)  { out << "\033[0;34m"; return out; }
[[nodiscard]] inline QTextStream & black ( QTextStream & out)  { out << "\033[0;30m"; return out; }
[[nodiscard]] inline QTextStream & yellow( QTextStream & out)  { out << "\033[0;33m"; return out; }
[[nodiscard]] inline QTextStream & purple( QTextStream & out)  { out << "\033[0;35m"; return out; }
[[nodiscard]] inline QTextStream & cyan (  QTextStream & out)  { out << "\033[4;36m"; return out; }

} // namespace TextColor

#endif // GLOBAL_H
