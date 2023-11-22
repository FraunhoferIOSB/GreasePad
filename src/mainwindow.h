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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QStatusBar>

#include "state.h"


class MainScene;
class QUndoStack;
class QGraphicsPixmapItem;
class QDoubleSpinBox;
class QLabel;

//! Graphical user interface
namespace GUI {

class FormatTool;
class MainView;


//! Main window
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow( QWidget *parent = nullptr); //!< standard constructor
    ~MainWindow() override;

    // Where required, read binary file as program argument, see main.cpp
    void readBinaryFile( const QString & fileName);  //!< Read binary file

protected:
    void closeEvent( QCloseEvent *) override; //!< Handle close event

private:
    State curr_state;

    void createSceneAndView();
    void createActions();
    void createMenus();
    void createToolBars();
    void createBoxes();
    void createStatusBar();
    void establishConnections();

    std::unique_ptr<QGraphicsPixmapItem> m_pixmap;  // optional background
    std::unique_ptr<MainView>   m_view;
    std::unique_ptr<MainScene>  m_scene;
    std::unique_ptr<QUndoStack> m_undoStack;

    enum {UndoLimit = 5};

    struct SpinBox  {
        double min;
        double max;
        double step;
           int decimals;
        double default_val;
    };

    const SpinBox alphaBox   = { 0.01, 1.00, 0.01,  2, 0.05 };
    const SpinBox opacityBox = { 0.00, 1.00, 0.05,  2, 0.50 };

    std::unique_ptr<QAction>   actionAbout;
    std::unique_ptr<QAction>   actionAboutQt;
    std::unique_ptr<QAction>   actionBackgroundImageLoad;
    std::unique_ptr<QAction>   actionBackgroundImageRemove;
    std::unique_ptr<QAction>   actionBackgroundImageToggleShow;
    std::unique_ptr<QAction>   actionBasicDocumentation;
    std::unique_ptr<QAction>   actionBinaryRead;
    std::unique_ptr<QAction>   actionBinarySave;
    std::unique_ptr<QAction>   actionChangeFormat;
    std::unique_ptr<QAction>   actionDeleteSelection;
    std::unique_ptr<QAction>   actionDeselectAll;
    std::unique_ptr<QAction>   actionExit;
    std::unique_ptr<QAction>   actionExportSaveAs;
    std::unique_ptr<QAction>   actionFitInView;
    std::unique_ptr<QAction>   actionItemMoveToBottom;
    std::unique_ptr<QAction>   actionItemMoveToTop;
    std::unique_ptr<QAction>   actionRedo;
    std::unique_ptr<QAction>   actionTabulaRasa;

    std::unique_ptr<QAction>   actionToggleConsiderOrthogonal{};
    std::unique_ptr<QAction>   actionToggleConsiderParallel{};
    std::unique_ptr<QAction>   actionToggleConsiderCopunctual{};
    std::unique_ptr<QAction>   actionToggleConsiderVertical{};
    std::unique_ptr<QAction>   actionToggleConsiderHorizontal{};
    std::unique_ptr<QAction>   actionToggleConsiderDiagonal{};

    std::unique_ptr<QAction>   actionToggleSelection;
    std::unique_ptr<QAction>   actionToggleShowConstrained{};
    std::unique_ptr<QAction>   actionToggleShowConstraints{};
    std::unique_ptr<QAction>   actionToggleShowColoration{};
    std::unique_ptr<QAction>   actionToggleShowStrokes{};
    std::unique_ptr<QAction>   actionToggleShowUnconstrained;
    std::unique_ptr<QAction>   actionToggleShowUncertainty;
    std::unique_ptr<QAction>   actionUndo;

    std::unique_ptr<QMenu>     menuConstr{};
    std::unique_ptr<QMenu>     menuEdit{};
    std::unique_ptr<QMenu>     menuFile{};
    std::unique_ptr<QMenu>     menuHelp{};
    std::unique_ptr<QMenu>     menuShow{};

    std::unique_ptr<QToolBar>  barBackground{};
    std::unique_ptr<QToolBar>  barConstr{};
    std::unique_ptr<QToolBar>  barEdit{};
    std::unique_ptr<QToolBar>  barNavigation{};
    std::unique_ptr<QToolBar>  barShow{};
    std::unique_ptr<QToolBar>  barTesting{};

    std::unique_ptr<QDoubleSpinBox> spinBoxAlphaRecognition{};
    std::unique_ptr<QDoubleSpinBox> spinBoxAlphaSnap{};
    std::unique_ptr<QDoubleSpinBox> spinBoxOpacity{};

    std::unique_ptr<FormatTool> penTool;

    std::unique_ptr<QLabel>   labelBoxAlphaRecogn;
    std::unique_ptr<QLabel>   labelBoxAlphaSnap;
    std::unique_ptr<QLabel>   labelBoxOpacity;

    bool openImageFile( const QString &fileName );
    bool maybeSave();

    void setCurrentFileName( const QString &fileName);
    bool fileSave();
    bool fileSaveAs();
    QString fileName_;

    void slotAbout();
    void slotAboutQt();
    void slotBackgroundImageLoad();
    void slotBackgroundImageRemove();
    void slotBasicDocumentation();
    void slotCmdAddStroke( QPainterPath *);
    void slotCmdDeleteSelection();
    void slotCmdTabulaRasa();
    void slotDeselectAll();
    bool slotExportSaveAs();
    void slotFileOpen();
    void slotFitInView();
    void slotItemMoveToBottom();
    void slotItemMoveToTop();
    void slotSelectionChanged();
    void slotShowStatus( const QString & s) { statusBar()->showMessage( s, 0); }
    void slotStackIndexChanged() {  statusBar()->showMessage( curr_state.StatusMsg() ); }

    void slotToggleConsiderOrthogonal();
    void slotToggleConsiderParallel();
    void slotToggleConsiderCopunctual();
    void slotToggleConsiderVertical();
    void slotToggleConsiderHorizontal();
    void slotToggleConsiderDiagonal();

    void slotToggleSelection();
    void slotToggleShowBackgroundImage();
    void slotToggleShowConstrained();
    void slotToggleShowConstraints();
    void slotToggleShowColored();
    void slotToggleShowStrokes();
    void slotToggleShowUnconstrained();
    void slotToggleShowUncertainty();
    void slotUndoLastAction();
    void slotUpdateLineStyle(  int s);
    void slotUpdateColor(      const QColor & col);
    void slotUpdateLineWidth(  int w);
    void slotUpdateMarkerSize( int sz);
    void slotValueChangedAlphaSnap( double);
    void slotValueChangedOpacity(   double );
    void slotValueChangedAlphaRecognition( double );
};

} // namespace GUI

#endif // MAINWINDOW_H
