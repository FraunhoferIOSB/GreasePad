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

#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication const a(argc, argv);

    QApplication::setApplicationName(    "GreasePad" );
    QApplication::setApplicationVersion( "1.2.0" );
    QApplication::setOrganizationName(   "Fraunhofer IOSB" );
    QApplication::setOrganizationDomain( "iosb.fraunhofer.de" );

    GUI::MainWindow win;
    win.show();

    // binary file as optional program argument
    QStringList const args = QApplication::arguments();
    if ( args.length()>1 ) {
        win.readBinaryFile( args.at(1));
    }

    return QApplication::exec();
}



