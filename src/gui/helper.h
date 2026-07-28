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


#ifndef HELPER_H
#define HELPER_H

#include <memory>

#include <QAction>
#include <QIcon>
#include <QKeySequence>
#include <QString>

namespace GUI {

static std::unique_ptr<QAction> makeAction( const QString & name,
    const QString & toolTip,
    const QKeySequence & shortCut,
    const QIcon & icon,
    bool isEnabled, bool isCheckable, bool isChecked, bool iconIsVisibleInMenu)
{
    std::unique_ptr<QAction> a = std::make_unique<QAction>(name);

    a->setToolTip(toolTip);
    a->setShortcut(shortCut);
    a->setIcon(icon);
    a->setEnabled(isEnabled);
    a->setCheckable(isCheckable);
    a->setChecked(isChecked);
    a->setIconVisibleInMenu(iconIsVisibleInMenu);

    return a;
}

} // namespace GUI

#endif // HELPER_H
