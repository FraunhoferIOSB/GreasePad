/*
 * This file is part of the GreasePad distribution (https://github.com/FraunhoferIOSB/GreasePad).
 * Copyright (c) 2022-2025 Jochen Meidow, Fraunhofer IOSB
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
#include "mainscene.h"
#include "mainview.h"
#include "mainwindow.h"
#include "qconstraints.h"
#include "qformattool.h"
#include "qsegment.h"
#include "qstroke.h"

#include "qgraphicsitem.h"
#include "qlogging.h"
#include "qnamespace.h"
#include "qtconfiginclude.h"
#include "qtdeprecationdefinitions.h"
#include "qtpreprocessorsupport.h"
#include "qtypes.h"

#include <QApplication>
#include <QBuffer>
#include <QClipboard>
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontDialog>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsView>
#include <QImageReader>
#include <QKeySequence>
#include <QLabel>
#include <QList>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMimeData>
#include <QObject>
#include <QOverload>
#include <QPdfWriter>
#include <QSizePolicy>
#include <QStringLiteral>
#include <QStringView>
#include <QStyleOptionGraphicsItem>
#include <QSvgGenerator>
#include <QToolBar>
#include <QWheelEvent>

#include <Eigen/src/Core/util/Macros.h>

#include <memory>



namespace GUI {

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_pixmap(nullptr)
{
    qDebug() <<  Q_FUNC_INFO;

    setWindowIcon( QIcon( QPixmap( ":/icons/Tango/preferences-desktop-peripherals.svg" )));


    // createPenTool()
    penTool = std::make_unique<FormatTool>("Properties", this);

    setAttribute( Qt::WA_StaticContents);
    setAttribute( Qt::WA_AcceptTouchEvents);

    setSizePolicy( QSizePolicy::Expanding,
                   QSizePolicy::Expanding);

    // create undo stack .............................................
    m_undoStack = std::make_unique<QUndoStack>(this);
    m_undoStack->setUndoLimit( UndoLimit );

    // curr_state = std::make_unique<State>();
    // curr_state = new State();

    double const alpha = 0.1;
    State::setAlphaRecognition( alpha );
    State::setAlphaSnapping(    alpha );

    createSceneAndView();
    // createUndoView();
    createActions();
    createMenus();
    createBoxes();
    establishConnections();
    createToolBars();
    createStatusBar();

    setCurrentFileName( QString());

    QConstraint::QConstraintBase::setDefaultPenReq(
                QPen( Qt::blue, 2, Qt::SolidLine, Qt::RoundCap, Qt::MiterJoin)
                );

    QConstraint::QConstraintBase::setDefaultPenRed(
                QPen( Qt::blue, 2, Qt::DotLine,  Qt::RoundCap, Qt::RoundJoin)
                );

    QConstraint::QConstraintBase::setPenSelected(
                QPen(  Qt::darkGreen,2, Qt::DashLine,  Qt::RoundCap, Qt::RoundJoin)
                );

    QEntity::QConstrained::setPenDefault(
                QPen( Qt::black,  2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin )
                );

    QEntity::QUnconstrained::setPenDefault(
                QPen( Qt::lightGray, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin )
                );

    QEntity::QStroke::setPenDefault(
                QPen( Qt::darkRed, 2, Qt::SolidLine, Qt::RoundCap)
                );

    QEntity::QSegment::setPenSelected(
                QPen( Qt::darkGreen, 2, Qt::DashLine, Qt::RoundCap)
                );

    QEntity::QStroke::setPenSelected(
                QPen( Qt::darkGreen, 2, Qt::DashLine, Qt::RoundCap)
                );
}

MainWindow::~MainWindow()
{
    // qDebug() << Q_FUNC_INFO;

    m_undoStack->clear();
    curr_state.clearAll();

    if ( m_pixmap!=nullptr ) {
        m_scene->removeItem( m_pixmap.get() );
    }

    qDebug() << "Bye bye.";
}

void MainWindow::createSceneAndView()
{
    m_scene = std::make_unique<MainScene>( this );
    Cmd::Undo::setScene( m_scene.get());

    m_view  = std::make_unique<MainView>( m_scene.get(), this );
    setCentralWidget( m_view.get() );
    showMaximized();     // after 'setCentralWidget'
}

void MainWindow::slotToggleShowUnconstrained()
{
    // qDebug() << Q_FUNC_INFO;
    QEntity::QUnconstrained::toggleShow();
    curr_state.toggleVisibilityUnconstrained();
    m_scene->update();
}

void MainWindow::slotToggleShowConstrained()
{
    // qDebug() << Q_FUNC_INFO;
    QEntity::QConstrained::toggleShow();
    curr_state.toggleVisibilityConstrained();
    m_scene->update();
}



void MainWindow::closeEvent( QCloseEvent * event)
{
    Q_UNUSED( event )
    // qDebug() << Q_FUNC_INFO;
    // TODO(meijoc)  event->setAccepted( maybeSave() ); //? true : false);
    /* if ( maybeSave() ) {
        // writeSettings();
        event->accept();
    } else {
        event->ignore();
    }*/
}

void MainWindow::createActions()
{
    actionUndo = std::unique_ptr<QAction>( m_undoStack->createUndoAction( this, QStringLiteral("Undo")) );
    actionUndo->setShortcuts( QKeySequence::Undo );
    actionUndo->setIcon( style()->standardIcon( QStyle::SP_ArrowBack) );

    actionRedo = std::unique_ptr<QAction>( m_undoStack->createRedoAction( this, QStringLiteral("Redo")) );
    actionRedo->setShortcuts( QKeySequence::Redo );
    actionRedo->setIcon( style()->standardIcon( QStyle::SP_ArrowForward) );

    actionExit = std::make_unique<QAction>( "Exit" );
    actionExit->setShortcut( QKeySequence( QStringLiteral("Ctrl+Q") ) );
    actionExit->setIcon( style()->standardIcon( QStyle::SP_BrowserStop));

    actionTabulaRasa = std::make_unique<QAction>( "Delete all" );
    actionTabulaRasa->setToolTip(  QStringLiteral("Make a clean sweep (blank state)") );
    actionTabulaRasa->setIcon( QPixmap(":/icons/Tango/Edit-clear.svg" ) );
    actionTabulaRasa->setIcon( style()->standardIcon( QStyle::SP_DialogResetButton ) );

    actionDeleteSelection = std::make_unique<QAction>( "Delete selected items" );
    actionDeleteSelection->setShortcut( QKeySequence::Delete );
    actionDeleteSelection->setEnabled( false );
    actionDeleteSelection->setIcon( style()->standardIcon( QStyle::SP_DialogCancelButton) );

    actionToggleSelection = std::make_unique<QAction>( "Select all" );
    actionToggleSelection->setToolTip(  QStringLiteral("Select all visible items") );
    actionToggleSelection->setShortcut( QKeySequence::SelectAll);

    actionDeselectAll = std::make_unique<QAction>( "Deselect all" );
    actionDeselectAll->setToolTip(  QStringLiteral("Deselect all items") );
    actionDeselectAll->setShortcut( QKeySequence( QStringLiteral("Ctrl+Shift+A")) );

    actionBackgroundImageLoad = std::make_unique<QAction>( "Load background image..." );
    actionBackgroundImageLoad->setToolTip( QStringLiteral("Load background image from file") );
    actionBackgroundImageLoad->setIcon( QIcon( QPixmap( ":/icons/Tango/Image-x-generic.svg" )));

    actionBackgroundImageRemove = std::make_unique<QAction>( "Remove background image");
    actionBackgroundImageRemove->setToolTip(  QStringLiteral("Remove loaded background image") );
    actionBackgroundImageRemove->setDisabled( true );
    actionBackgroundImageRemove->setIcon( style()->standardIcon( QStyle::SP_DialogCancelButton ) );

    actionBackgroundImageToggleShow = std::make_unique<QAction>( "Show background image");
    actionBackgroundImageToggleShow->setToolTip(   QStringLiteral("Show/hide loaded background image") );
    actionBackgroundImageToggleShow->setIcon(      QPixmap( ":/icons/Tango/Image-x-generic.svg" ));
    actionBackgroundImageToggleShow->setCheckable( true );
    actionBackgroundImageToggleShow->setDisabled(  true );
    actionBackgroundImageToggleShow->setChecked(   false );
    actionBackgroundImageToggleShow->setIconVisibleInMenu( false );

    actionToggleShowStrokes = std::make_unique<QAction>( "Show strokes" );
    actionToggleShowStrokes->setToolTip(   QStringLiteral("Show pen strokes (mouse tracks)") );
    actionToggleShowStrokes->setCheckable( true );
    actionToggleShowStrokes->setIcon(      QPixmap( ":/icons/show_strokes.svg") );
    actionToggleShowStrokes->setChecked(   QEntity::QStroke::show() );
    actionToggleShowStrokes->setIconVisibleInMenu( false );

    actionToggleShowUnconstrained = std::make_unique<QAction>( "Show unconstrained segments" );
    actionToggleShowUnconstrained->setToolTip(   QStringLiteral("Show unconstrained segments") );
    actionToggleShowUnconstrained->setCheckable( true );
    actionToggleShowUnconstrained->setChecked(   QEntity::QUnconstrained::show() );
    actionToggleShowUnconstrained->setIcon(      QPixmap(":/icons/show_unconstrained.svg") );
    actionToggleShowUnconstrained->setIconVisibleInMenu( false );

    actionToggleShowConstrained = std::make_unique<QAction>( "Show constrained segments" );
    actionToggleShowConstrained->setToolTip(   QStringLiteral("Show constrained segments") );
    actionToggleShowConstrained->setCheckable( true );
    actionToggleShowConstrained->setChecked(   QEntity::QConstrained::show() );
    actionToggleShowConstrained->setIcon(      QPixmap(":/icons/show_constrained.svg"));
    actionToggleShowConstrained->setIconVisibleInMenu( false );

    actionToggleShowUncertainty = std::make_unique<QAction>( "Show uncertainty" );
    actionToggleShowUncertainty->setShortcut(  QKeySequence( "Ctrl+U" ) );
    actionToggleShowUncertainty->setToolTip(   QStringLiteral("Show confidence regions") );
    actionToggleShowUncertainty->setCheckable( true );
    actionToggleShowUncertainty->setChecked(   QEntity::QSegment::showUncertainty() );
    actionToggleShowUncertainty->setIcon(      QPixmap( ":/icons/show_uncertain.svg" ));
    actionToggleShowUncertainty->setIconVisibleInMenu(false);

    actionToggleShowConstraints = std::make_unique<QAction>( "Show constraints" );
    actionToggleShowConstraints->setToolTip(   QStringLiteral("Show constraints") );
    actionToggleShowConstraints->setCheckable( true );
    actionToggleShowConstraints->setChecked(   QConstraint::QConstraintBase::show() );
    actionToggleShowConstraints->setIcon(      QPixmap( ":/icons/show_constraints.svg" ));
    actionToggleShowConstraints->setIconVisibleInMenu( false );

    actionToggleShowColoration = std::make_unique<QAction>( "Colorize connected components" );
    actionToggleShowColoration->setToolTip(   QStringLiteral("Colorize connected components, i.e., subtasks") );
    actionToggleShowColoration->setCheckable( true );
    actionToggleShowColoration->setChecked(   QEntity::QConstrained::showColor() );
    actionToggleShowColoration->setIcon(      QPixmap( ":/icons/show_cc.svg" ));
    actionToggleShowColoration->setIconVisibleInMenu( false );

    actionExportSaveAs = std::make_unique<QAction>( "Export as..." );
    actionExportSaveAs->setDisabled( false );
    actionExportSaveAs->setToolTip(  QStringLiteral("Export entire scene as SVG or in PDF.") );
    actionExportSaveAs->setIcon(     style()->standardIcon( QStyle::SP_FileIcon) );

    actionBinaryRead = std::make_unique<QAction>( "Open..." );
    actionBinaryRead->setShortcut( QKeySequence::Open );
    actionBinaryRead->setToolTip(  QStringLiteral("load file content"));
    actionBinaryRead->setIcon(     style()->standardIcon( QStyle::SP_DialogOpenButton) );
    actionBinaryRead->setDisabled( false);

    actionBinarySave = std::make_unique<QAction>( "Save" );
    actionBinarySave->setDisabled( true );
    actionBinarySave->setIcon( style()->standardIcon( QStyle::SP_DialogSaveButton) );
    actionBinarySave->setShortcut( QKeySequence(QStringLiteral("Ctrl+S")) );

    actionFitInView = std::make_unique<QAction>( "Fit in view" );
    // actionFitInView->setIcon( QIcon(QPixmap( ":/icons/fit_in_view.svg" )));
    actionFitInView->setIcon( QIcon(QPixmap( ":/icons/Tango/view-fullscreen.svg")));
    actionFitInView->setIconVisibleInMenu(   true );
    actionFitInView->setToolTip(             QStringLiteral("Scale scene to fit in view") );

    actionBasicDocumentation = std::make_unique<QAction>( "Getting Started..." );
    actionBasicDocumentation->setToolTip( QStringLiteral("Basic Documentation") );
    actionBasicDocumentation->setIcon( style()->standardIcon( QStyle::SP_MessageBoxQuestion ));

    actionAbout = std::make_unique<QAction>( "About GreasePad" );
    actionAbout->setIcon( style()->standardIcon( QStyle::SP_MessageBoxInformation )); // _FileDialogInfoView) );

    actionAboutQt = std::make_unique<QAction>( "About Qt" );
    actionAboutQt->setIcon( style()->standardIcon( QStyle::SP_TitleBarMenuButton) );

    actionToggleConsiderOrthogonal = std::make_unique<QAction>( "Consider orthogonal" );
    actionToggleConsiderOrthogonal->setToolTip(   QStringLiteral("Consider orthogonal") );
    actionToggleConsiderOrthogonal->setCheckable( true );
    actionToggleConsiderOrthogonal->setChecked(   State::considerOrthogonal() );
    actionToggleConsiderOrthogonal->setIconVisibleInMenu( false );
    actionToggleConsiderOrthogonal->setIcon( QPixmap( ":/icons/consider_orthogonality.svg" ));
    actionToggleConsiderOrthogonal->setShortcut( QKeySequence(QStringLiteral("o")) );

    actionToggleConsiderParallel = std::make_unique<QAction>( "Consider parallel" );
    actionToggleConsiderParallel->setToolTip(   QStringLiteral("Consider parallel") );
    actionToggleConsiderParallel->setCheckable( true );
    actionToggleConsiderParallel->setChecked(   State::considerParallel() );
    actionToggleConsiderParallel->setIconVisibleInMenu( false );
    actionToggleConsiderParallel->setIcon( QPixmap( ":/icons/consider_parallelism.svg" ));
    actionToggleConsiderParallel->setShortcut( QKeySequence(QStringLiteral("p")) );

    actionToggleConsiderCopunctual = std::make_unique<QAction>( "Consider copunctual" );
    actionToggleConsiderCopunctual->setToolTip( QStringLiteral("Consider copunctual") );
    actionToggleConsiderCopunctual->setCheckable( true );
    actionToggleConsiderCopunctual->setChecked( State::considerCopunctual() );
    actionToggleConsiderCopunctual->setIconVisibleInMenu( false );
    actionToggleConsiderCopunctual->setIcon( QPixmap( ":/icons/consider_concurrence.svg" ));
    actionToggleConsiderCopunctual->setShortcut( QKeySequence("c") );

    actionToggleConsiderVertical = std::make_unique<QAction>( "Consider vertical" );
    actionToggleConsiderVertical->setToolTip( QStringLiteral("Consider vertical") );
    actionToggleConsiderVertical->setCheckable( true );
    actionToggleConsiderVertical->setChecked( State::considerVertical() );
    actionToggleConsiderVertical->setIconVisibleInMenu( false );
    actionToggleConsiderVertical->setIcon( QPixmap( ":/icons/consider_vert.svg" ));
    actionToggleConsiderVertical->setShortcut( QKeySequence(QStringLiteral("v")) );

    actionToggleConsiderHorizontal = std::make_unique<QAction>( "Consider horizontal" );
    actionToggleConsiderHorizontal->setToolTip( QStringLiteral("Consider horizontal") );
    actionToggleConsiderHorizontal->setCheckable( true );
    actionToggleConsiderHorizontal->setChecked( State::considerHorizontal() );
    actionToggleConsiderHorizontal->setIconVisibleInMenu( false );
    actionToggleConsiderHorizontal->setIcon( QPixmap( ":/icons/consider_horiz.svg" ));
    actionToggleConsiderHorizontal->setShortcut( QKeySequence(QStringLiteral("h")) );

    actionToggleConsiderDiagonal = std::make_unique<QAction>( "Consider diagonal" );
    actionToggleConsiderDiagonal->setToolTip( QStringLiteral("Consider diagonal") );
    actionToggleConsiderDiagonal->setCheckable( true );
    actionToggleConsiderDiagonal->setChecked( State::considerHorizontal() );
    actionToggleConsiderDiagonal->setIconVisibleInMenu( false );
    actionToggleConsiderDiagonal->setIcon( QPixmap( ":/icons/consider_diag.svg" ));
    actionToggleConsiderDiagonal->setShortcut( QKeySequence(QStringLiteral("d")) );

    actionItemMoveToBottom = std::make_unique<QAction>( QStringLiteral("Move selected items to bottom") );
    actionItemMoveToBottom->setToolTip( QStringLiteral("Send items to back (visual stacking)") );
    actionItemMoveToBottom->setShortcut( QKeySequence(QStringLiteral("Ctrl+-")) );

    actionItemMoveToTop = std::make_unique<QAction>( QStringLiteral("Move selected items to top") );
    actionItemMoveToTop->setToolTip( QStringLiteral("Bring items to front (visual stacking)") );
    actionItemMoveToTop->setShortcut( QKeySequence(QStringLiteral("Ctrl++")) );

    actionChangeFormat = std::make_unique<QAction>( "Format selected entities", this);
    actionChangeFormat->setToolTip( QStringLiteral("Format selected entities") );
    actionChangeFormat->setShortcut( QKeySequence(QStringLiteral("Ctrl+F")));

    actionSetFont = std::make_unique<QAction>( "Set GUI Font...", this);
}

void MainWindow::createBoxes()
{
    qDebug() << Q_FUNC_INFO;

    spinBoxAlphaRecognition = std::make_unique<QDoubleSpinBox>( this );
    spinBoxAlphaRecognition->setRange(      alphaBox.min,alphaBox.max);
    spinBoxAlphaRecognition->setSingleStep( alphaBox.step);
    spinBoxAlphaRecognition->setDecimals(   alphaBox.decimals);
    spinBoxAlphaRecognition->setValue(      alphaBox.default_val);
    spinBoxAlphaRecognition->setPrefix(  QString::fromUtf8("α="));
    spinBoxAlphaRecognition->setSuffix(  QStringLiteral(" "));
    spinBoxAlphaRecognition->setToolTip( QStringLiteral("significance level") );

    QFont font = spinBoxAlphaRecognition->font();
    font.setPointSize( font.pointSize()+1 );
    spinBoxAlphaRecognition->setFont(font);

    spinBoxAlphaSnap = std::make_unique<QDoubleSpinBox>( this );
    spinBoxAlphaSnap->setRange(      alphaBox.min,alphaBox.max);
    spinBoxAlphaSnap->setSingleStep( alphaBox.step);
    spinBoxAlphaSnap->setDecimals(   alphaBox.decimals);
    spinBoxAlphaSnap->setValue(      alphaBox.default_val);
    spinBoxAlphaSnap->setPrefix(     QString::fromUtf8("α=") );
    spinBoxAlphaSnap->setSuffix(     QStringLiteral(" ") );
    spinBoxAlphaSnap->setToolTip(    QStringLiteral("significance level") );
    spinBoxAlphaSnap->setFont( font );

    spinBoxOpacity = std::make_unique<QDoubleSpinBox>( this );
    spinBoxOpacity->setRange(       opacityBox.min, opacityBox.max);
    spinBoxOpacity->setSingleStep(  opacityBox.step);
    spinBoxOpacity->setDecimals(    opacityBox.decimals);
    spinBoxOpacity->setValue(       opacityBox.default_val);
    spinBoxOpacity->setToolTip(     QStringLiteral("Opacity of background image"));
    spinBoxOpacity->setDisabled(    true );
    spinBoxOpacity->setFont( font);
}



void MainWindow::createMenus()
{
    // qDebug() << Q_FUNC_INFO;

    // set font size ................................................
    //QFont f = this->font();
    // f.setPointSize(FONT_SIZE_IN_PT);
    //this->setFont(f);

    menuFile = std::make_unique<QMenu>( tr("File") );
    menuFile->addAction( actionBinaryRead.get()   );
    menuFile->addAction( actionBinarySave.get() );
    menuFile->addAction( tr("Save As..."), QKeySequence(QStringLiteral("Ctrl+Shift+S")),
                        this, &MainWindow::fileSaveAs );
    menuFile->addAction( actionExportSaveAs.get() );

    menuFile->addSeparator();
    menuFile->addAction( actionBackgroundImageLoad.get() );
    menuFile->addAction( actionSetFont.get());
    menuFile->addSeparator();  // .............................................
    menuFile->addAction( actionExit.get() );


    menuBar()->addMenu( menuFile.get() );

    // edit ....................................................................
    menuEdit = std::make_unique<QMenu>( tr("&Edit") );
    menuEdit->addAction( actionUndo.get());
    menuEdit->addAction( actionRedo.get());
    menuEdit->addSeparator();
    menuEdit->addAction( actionTabulaRasa.get() );
    menuEdit->addAction( actionDeleteSelection.get() );
    menuEdit->addAction( m_view->actionCopySvgToClipboard.get() );
    menuEdit->addAction( m_view->actionCopyPdfToClipboard.get() );
    menuEdit->addAction( m_view->actionCopyScreenshotToClipboard.get() );

    menuEdit->addAction( actionBackgroundImageRemove.get() );
    menuEdit->addAction( actionFitInView.get() );
    menuEdit->addSeparator();

    menuEdit->addAction( actionItemMoveToTop.get() );
    menuEdit->addAction( actionItemMoveToBottom.get() );
    menuEdit->addAction( actionToggleSelection.get() );
    menuEdit->addAction( actionDeselectAll.get() );

    menuEdit->addAction( actionChangeFormat.get() );


    menuBar()->addMenu( menuEdit.get() );




    menuConstr = std::make_unique<QMenu>( tr("&Constraints") );
    menuConstr->addAction( actionToggleConsiderOrthogonal.get() );
    menuConstr->addAction( actionToggleConsiderParallel.get()   );
    menuConstr->addAction( actionToggleConsiderCopunctual.get() );
    menuConstr->addSeparator(); // -----------------------------------
    menuConstr->addAction( actionToggleConsiderVertical.get()   );
    menuConstr->addAction( actionToggleConsiderHorizontal.get() );
    menuConstr->addAction( actionToggleConsiderDiagonal.get() );
    menuBar()->addMenu ( menuConstr.get());

    menuShow = std::make_unique<QMenu>( tr("Display") );
    menuShow->addAction( actionToggleShowConstrained.get()   );
    menuShow->addAction( actionToggleShowUnconstrained.get() );
    menuShow->addAction( actionToggleShowStrokes.get()       );
    menuShow->addAction( actionToggleShowConstraints.get()   );
    menuShow->addAction( actionToggleShowUncertainty.get()   );
    menuShow->addAction( actionToggleShowColoration.get()    );
    menuShow->addAction( m_view->actionToggleShowBackgroundTiles.get() );
    menuShow->addAction( actionBackgroundImageToggleShow.get() );
    menuBar()->addMenu( menuShow.get());

    // help .........................................................
    menuHelp = std::make_unique<QMenu>( tr("Help") );
    menuHelp->addAction( actionBasicDocumentation.get() );
    menuHelp->addAction( actionAbout.get()  );
    menuHelp->addAction( actionAboutQt.get());
    menuBar()->addMenu( menuHelp.get() );
}

void MainWindow::createStatusBar()
{
    // status bar: initial message
    // statusBar()->setStyleSheet( "Color: black; font-weight: bold; font-size: 30px" );
    statusBar()->showMessage(
                QStringLiteral("Ready. Max. %1 commands on stack. Add a single pen stroke...")
                .arg( m_undoStack->undoLimit()), 0);
}


void MainWindow::createToolBars()
{
    qDebug() << Q_FUNC_INFO;

    barEdit = std::make_unique<QToolBar>( "Edit", this);
    barEdit->addAction( actionTabulaRasa.get() );
    barEdit->addAction( actionDeleteSelection.get() );
    barEdit->addAction( actionUndo.get() );
    barEdit->addAction( actionRedo.get() );
    addToolBar( barEdit.get());

    barNavigation = std::make_unique<QToolBar>( "Navigation", this);
    barNavigation->addAction( m_view->actionZoomIn.get()  );
    barNavigation->addAction( m_view->actionZoomOut.get() );
    barNavigation->addAction( actionFitInView.get() );
    // barNavigation->setToolButtonStyle(Qt::ToolButtonFollowStyle);
    addToolBar( barNavigation.get());

    barConstr = std::make_unique<QToolBar>( "Constraints to consider" );
    barConstr->addAction( actionToggleConsiderOrthogonal.get() );
    barConstr->addAction( actionToggleConsiderParallel.get()   );
    barConstr->addAction( actionToggleConsiderCopunctual.get() );
    barConstr->addAction( actionToggleConsiderVertical.get()   );
    barConstr->addAction( actionToggleConsiderHorizontal.get() );
    barConstr->addAction( actionToggleConsiderDiagonal.get()   );
    // barConstr->setHidden( true );
    addToolBar( barConstr.get() );
    //barConstraints->setToolButtonStyle(Qt::ToolButtonFollowStyle);

    barShow = std::make_unique<QToolBar>( "Show" );
    barShow->addAction( actionToggleShowConstraints.get()   );
    barShow->addAction( actionToggleShowStrokes.get()       );
    barShow->addAction( actionToggleShowConstrained.get()   );
    barShow->addAction( actionToggleShowUnconstrained.get() );
    barShow->addAction( actionToggleShowUncertainty.get()   );
    barShow->addAction( m_view->actionToggleShowBackgroundTiles.get() );
    barShow->addAction( actionToggleShowColoration.get()      );
    barShow->addAction( actionBackgroundImageToggleShow.get() );
    addToolBar( barShow.get() );

    addToolBarBreak( Qt::TopToolBarArea );


    labelBoxAlphaRecogn = std::make_unique<QLabel>( " Recognition ");
    //QFont font = labelBoxAlphaRecogn->font();
    //font.setPointSize( font.pointSize()+1);
    //labelBoxAlphaRecogn->setFont(font);

    labelBoxAlphaSnap   = std::make_unique<QLabel>( " Snap " );
    //labelBoxAlphaSnap->setFont(font);

    labelBoxOpacity  = std::make_unique<QLabel>( " Opacity " );
    //labelBoxOpacity->setFont(font);

    barTesting = std::make_unique<QToolBar>( "Testing" );
    barTesting->addWidget( labelBoxAlphaRecogn.get());
    barTesting->addWidget( spinBoxAlphaRecognition.get() );
    barTesting->addWidget( labelBoxAlphaSnap.get() );
    barTesting->addWidget( spinBoxAlphaSnap.get() );
    addToolBar( barTesting.get() );

    barBackground = std::make_unique<QToolBar>( "Background" );
    barBackground->addWidget( labelBoxOpacity.get() );
    barBackground->addWidget( spinBoxOpacity.get() );
    barBackground->setToolTip( QStringLiteral("Background") );
    barBackground->setEnabled( true );
    addToolBar( barBackground.get() );
}


void MainWindow::establishConnections()
{
    connect( m_undoStack.get(), &QUndoStack::indexChanged,
             this,              &MainWindow::slotStackIndexChanged);

    // edit ............................................................
    connect( m_scene.get(),   &MainScene::signalCmdAddStroke,
             this,            &MainWindow::slotCmdAddStroke);
    connect( m_scene.get(),   &MainScene::signalCmdDeleteSelection,    // pressed key [del] via emit signal
             this,            &MainWindow::slotCmdDeleteSelection);
    connect( actionDeleteSelection.get(), &QAction::triggered,          // select menu item
             this,                        &MainWindow::slotCmdDeleteSelection);
    connect( actionTabulaRasa.get(),  &QAction::triggered,
             this,                    &MainWindow::slotCmdTabulaRasa);
    connect( actionToggleSelection.get(),  &QAction::triggered,
             this,                         &MainWindow::slotToggleSelection);
    connect( actionDeselectAll.get(),  &QAction::triggered,
             this,                     &MainWindow::slotDeselectAll);


    connect( penTool.get(), &GUI::FormatTool::signalMarkerSizeChanged,
             this,          &MainWindow::slotUpdateMarkerSize);
    connect( penTool.get(), &GUI::FormatTool::signalLineWidthChanged,
             this,          &MainWindow::slotUpdateLineWidth);
    connect( penTool.get(), &GUI::FormatTool::signalColorChanged,
             this,          &MainWindow::slotUpdateColor);
    connect( penTool.get(), &GUI::FormatTool::signalLineStyleChanged,
             this,          &MainWindow::slotUpdateLineStyle);


    connect( actionChangeFormat.get(), &QAction::triggered,
             penTool.get(),            &FormatTool::show);

    connect( actionSetFont.get(), &QAction::triggered,
             this,                &MainWindow::slotSetFont);



    // If necessary, enable/disable the menu entry "delete selection"
    connect( m_scene.get(), &MainScene::selectionChanged,
             this,          &MainWindow::slotSelectionChanged);

    connect( m_scene.get(), &MainScene::signalUndoLastAction,
             this,          &MainWindow::slotUndoLastAction);

    // background
    connect( actionBackgroundImageLoad.get(),   &QAction::triggered,
             this,                              &MainWindow::slotBackgroundImageLoad);
    connect( actionBackgroundImageRemove.get(), &QAction::triggered,
             this,                              &MainWindow::slotBackgroundImageRemove);

    // search, consider
    connect( actionToggleConsiderOrthogonal.get(), &QAction::triggered,
             this,        &MainWindow::slotToggleConsiderOrthogonal);
    connect( actionToggleConsiderParallel.get(),  &QAction::triggered,
             this,        &MainWindow::slotToggleConsiderParallel);
    connect( actionToggleConsiderCopunctual.get(),  &QAction::triggered,
             this,        &MainWindow::slotToggleConsiderCopunctual);
    connect( actionToggleConsiderVertical.get(),  &QAction::triggered,
             this,        &MainWindow::slotToggleConsiderVertical);
    connect( actionToggleConsiderHorizontal.get(),  &QAction::triggered,
             this,        &MainWindow::slotToggleConsiderHorizontal);
    connect( actionToggleConsiderDiagonal.get(),  &QAction::triggered,
             this,        &MainWindow::slotToggleConsiderDiagonal);

    // show/display
    connect( actionBackgroundImageToggleShow.get(), &QAction::triggered,
             this,                                  &MainWindow::slotToggleShowBackgroundImage);
    connect( actionToggleShowStrokes.get(),    &QAction::triggered,
             this,                             &MainWindow::slotToggleShowStrokes);
    connect( actionToggleShowUncertainty.get(),    &QAction::triggered,
             this,                                 &MainWindow::slotToggleShowUncertainty);
    connect( actionToggleShowConstrained.get(),    &QAction::triggered,
             this,                                 &MainWindow::slotToggleShowConstrained);
    connect( actionToggleShowUnconstrained.get(),   &QAction::triggered,
             this,                                  &MainWindow::slotToggleShowUnconstrained);
    connect( actionToggleShowConstraints.get(),  &QAction::triggered,
             this,                               &MainWindow::slotToggleShowConstraints);
    connect( actionToggleShowColoration.get(),   &QAction::triggered,
             this,                               &MainWindow::slotToggleShowColored);

    connect( actionBinaryRead.get(),   &QAction::triggered,
             this,                     &MainWindow::slotFileOpen);
    connect( actionBinarySave.get(),   &QAction::triggered,
             this,                     &MainWindow::fileSave);

    connect( actionExportSaveAs.get(), &QAction::triggered,
             this,                     &MainWindow::slotExportSaveAs);


    connect( actionExit.get(),       &QAction::triggered,
             this,                   &MainWindow::close);


    // help
    connect( actionAboutQt.get(),  &QAction::triggered,
             this,                 &MainWindow::slotAboutQt);
    connect( actionAbout.get(),  &QAction::triggered,
             this,               &MainWindow::slotAbout);
    connect( actionBasicDocumentation.get(), &QAction::triggered,
             this,                           &MainWindow::slotBasicDocumentation);

    // view
    connect( actionFitInView.get(),  &QAction::triggered,
             this,                   &MainWindow::slotFitInView);

    //edit
    connect( actionItemMoveToTop.get(), &QAction::triggered,
             this,                      &MainWindow::slotItemMoveToTop);
    connect( actionItemMoveToBottom.get(), &QAction::triggered,
             this,                         &MainWindow::slotItemMoveToBottom);

    connect( spinBoxAlphaRecognition.get(), QOverload<double>::of(&QDoubleSpinBox::valueChanged),
             this,                          &MainWindow::slotValueChangedAlphaRecognition);
    connect( spinBoxAlphaSnap.get(), QOverload<double>::of(&QDoubleSpinBox::valueChanged),
             this,                   &MainWindow::slotValueChangedAlphaSnap);



    connect( spinBoxOpacity.get(), QOverload<double>::of(&QDoubleSpinBox::valueChanged),
             this,                 &MainWindow::slotValueChangedOpacity);

    connect ( m_view.get(), &MainView::signalShowStatus,
              this,         &MainWindow::slotShowStatus );
}

bool MainWindow::maybeSave()
{
    // qDebug() <<  Q_FUNC_INFO;

    if ( !isWindowModified() ) {
        return true;
    }

    const QMessageBox::StandardButton ret
        = QMessageBox::warning(this,
                               QCoreApplication::applicationName(),
                               tr("The sketch has been modified.\n"
                                  "Do you want to save your changes?"),
                               QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);

    switch ( static_cast<int>(ret)) {
    case QMessageBox::Save:
        return fileSaveAs();
    case QMessageBox::Cancel:
        qDebug() << "termination canceled.";
        return false;
    default:
        break;
    }
    return true;

}

bool MainWindow::openImageFile(const QString &fileName)
{
    // qDebug() << Q_FUNC_INFO;

    QImage loadedImage;
    if ( !loadedImage.load(fileName) ) {
        return false;
    }

    if ( actionBackgroundImageRemove->isEnabled() ) {
        slotBackgroundImageRemove();
    }

    // set ....................................................................
    m_pixmap = std::make_unique<QGraphicsPixmapItem>( QPixmap::fromImage(loadedImage) );
    m_pixmap->setOpacity( spinBoxOpacity->value() );
    m_pixmap->setZValue(0);
    m_scene->addItem( m_pixmap.get() );
    // spinBoxOpacity->setEnabled(true);

    m_view->setSceneRect( m_scene->itemsBoundingRect());

    actionBackgroundImageRemove->setEnabled(true);
    actionBackgroundImageToggleShow->setEnabled(true);
    actionBackgroundImageToggleShow->setChecked(true);

    statusBar()->showMessage( QStringLiteral("<%1> loaded.").arg(fileName) );

    return true;
}



void MainWindow::readBinaryFile( const QString & fileName)
{
    // qDebug() << Q_FUNC_INFO;
    QFile file( fileName );
    if ( !file.open(QFile::ReadOnly) )
    {
        QMessageBox::warning(
                    this, tr("QDataStream"),
                    tr("Cannot open file %1:\n%2.").arg( QDir::toNativeSeparators(fileName),
                                                         file.errorString()) );
        return;
    }

    QDataStream in( &file );    // read the data serialized from the file
    State newState;
    if ( !newState.deserialize( in ) )
    {
        file.close();
        statusBar()->showMessage( QStringLiteral("Data import failed.") );
        QMessageBox::warning(  this, QStringLiteral("Data import"), QStringLiteral("Data import failed.") );
        return;
    }

    file.close();
    QFileInfo const fileInfo( file.fileName() );


    std::unique_ptr<State> next_state
            =  std::make_unique<State>(newState);

    std::unique_ptr<State> prev_state
            = std::make_unique<State>(curr_state);  // copy

    m_undoStack->push(
                new Cmd::ReplaceStateWithFileContent (
                    fileInfo.fileName(),
                    &curr_state,
                    prev_state,
                    next_state,
                    nullptr)
                );

    // setWindowModified( false );
    setCurrentFileName( fileName);
    actionBinarySave->setEnabled( true);
}

void MainWindow::slotCmdTabulaRasa()
{
    m_undoStack->push(
                new Cmd::TabulaRasa( &curr_state, nullptr )
                );

    setWindowModified( true );
}

// invoked by MainScene:signalCmdAddStroke:
void MainWindow::slotCmdAddStroke( QPainterPath *path)
{
    // qDebug() << Q_FUNC_INFO;

    // convert path to polyline
    QPolygonF poly( path->elementCount()) ;
    for ( int i=0; i<path->elementCount(); i++ ) {
        poly[i] = path->elementAt(i);
    }
    /* qDebug() << cyan <<  QString("tracking (%1,%2) ...(%3,%4)")
                .arg( poly.first().x())
                .arg( poly.first().y())
                .arg( poly.last().x() )
                .arg( poly.last().y() ); */


    // try...
    std::unique_ptr<State> next_state_
            = std::make_unique<State>( curr_state);
    if ( next_state_->augment( poly ) )
    {
        std::unique_ptr<State> prev_state_
                = std::make_unique<State>(curr_state);
        m_undoStack->push(
                    new Cmd::AddStroke( &curr_state,
                                        prev_state_,
                                        next_state_,
                               nullptr)
                    );
        setWindowModified( true );
    }
}

void MainWindow::slotCmdDeleteSelection()
{
    // qDebug() << Q_FUNC_INFO;

    if ( m_scene->selectedItems().isEmpty() ) {
        return;
    }

    std::unique_ptr<State> next_state_
            = std::make_unique<State>(curr_state);
    if ( next_state_->reduce() )
    {
        std::unique_ptr<State> prev_state_
                = std::make_unique<State>(curr_state); // copy

        m_undoStack->push(
                    new Cmd::DeleteSelection( &curr_state,
                                              prev_state_,
                                              next_state_, nullptr)
                    );
        setWindowModified( true );
    }
}


bool MainWindow::slotExportSaveAs()
{
    qDebug() << Q_FUNC_INFO;

    QString const filter = "all formats (*.pdf *.svg);;"
            "scalable vector graphics (*.svg);;"
            "portable document format (*.pdf)";

    QString fileName = QFileDialog::getSaveFileName(
                this,
                QStringLiteral("Export drawing"),
                QDir::currentPath() + "/" + "untitled",
                filter );

    if ( fileName.isEmpty() ) {
        return false;
    }

    // check if already opened
    QFile openFile( fileName );
    if( !openFile.open( QFile::ReadWrite) ){
        QMessageBox::critical( this, QStringLiteral("Can't Open file"),
                               QStringLiteral("Can't access to the file."));
        return false;
    }
    openFile.close();

    if ( fileName.endsWith( QStringLiteral(".pdf"), Qt::CaseInsensitive)) {
        m_scene->export_view_as_pdf( fileName );
        return true;
    }

    if ( fileName.endsWith( QStringLiteral(".svg"), Qt::CaseInsensitive)) {
        m_scene->export_view_as_svg( fileName );
        return true;
    }

    return false;
}


void MainWindow::slotToggleSelection()
{
    // qDebug() << Q_FUNC_INFO;

    for ( auto & item : m_scene->items() ) {
        if ( item->isVisible() ) {
            // item->setSelected( true ); // select all
            item->setSelected( !item->isSelected() ); // toggle selection
        }
    }
    m_scene->update();
}

void MainWindow::slotDeselectAll()
{
    // qDebug() << Q_FUNC_INFO;

    for ( auto & item : m_scene->items() ) {
        item->setSelected( false );
    }
    m_scene->update();
}


void MainWindow::slotSetFont()
{
    bool ok = false;
    QFont const font  = QFontDialog::getFont( &ok, QApplication::font(), this);
    // QApplication::setFont(QFontDialog::getFont(0, QApplication::font(), this));
    if (ok) {
        spinBoxAlphaRecognition->setFont( font );
        spinBoxAlphaSnap->setFont( font );
        spinBoxOpacity->setFont( font );

        menuBar()->setFont( font );
        menuEdit->setFont( font );
        menuFile->setFont( font );
        menuShow->setFont( font );
        menuConstr->setFont( font );
        menuHelp->setFont( font );

        labelBoxOpacity->setFont( font );
        labelBoxAlphaSnap->setFont( font );
        labelBoxAlphaRecogn->setFont( font );

        statusBar()->setFont( font );
    }
}

void MainWindow::slotAboutQt()
{
    QMessageBox::aboutQt( this );
}


void MainWindow::slotBackgroundImageLoad()
{
    // qDebug() << Q_FUNC_INFO;

    QString filter = "Image files (";
    const QList<QByteArray> barray ( QImageReader::supportedImageFormats() );
    for ( const auto & item : barray) {
        filter += QStringLiteral( "*.%1 ").arg( item.data() );
    }
    filter += QStringLiteral(")");
    for ( const auto & item : barray) {
        filter += QStringLiteral(";; *.%1").arg(item.data());
    }


    QString const fileName = QFileDialog::getOpenFileName( this,   QStringLiteral("Open Image File"),
                                                     QDir::currentPath(), filter);
    if ( !fileName.isEmpty() ) {
        if ( openImageFile( fileName ) ) {
            m_pixmap->setZValue( -1 );
            spinBoxOpacity->setEnabled( true);
        }
        else {
            QMessageBox::warning( this,   QStringLiteral("image import"),
                                  QStringLiteral("%1: Image import failed").arg(fileName) );
        }

    }
}

void MainWindow::slotBackgroundImageRemove()
{
    // qDebug() << Q_FUNC_INFO;
    m_scene->removeItem( m_pixmap.get() );
    m_pixmap = nullptr;

    actionBackgroundImageRemove->setDisabled( true );
    actionBackgroundImageToggleShow->setDisabled( true );
    spinBoxOpacity->setDisabled( true );
}

void MainWindow::slotFitInView()
{
    m_view->fitInView( m_scene->itemsBoundingRect(),
                        Qt::KeepAspectRatio);
}

void MainWindow::slotFileOpen()
{
    // qDebug() << Q_FUNC_INFO;
    if (maybeSave()) {
        QString const fileName = QFileDialog::getOpenFileName(
                    this,                 QStringLiteral("GReasePad file"),
                    QDir::currentPath(),  QStringLiteral("GReasePad files (*.grp)") );

        if ( fileName.isEmpty() ) {
            return;
        }
        readBinaryFile( fileName);
    }
}

bool MainWindow::fileSaveAs()
{
    // qDebug() << Q_FUNC_INFO;
    QString const fileName = QFileDialog::getSaveFileName(
                this,                 QStringLiteral("Save file"),
                QDir::currentPath(),  QStringLiteral("GReasePad files (*.grp)"));

    if ( fileName.isEmpty() ) {
        return false;
    }

    setCurrentFileName( fileName );
    actionBinarySave->setEnabled( true );

    return fileSave();
}


bool MainWindow::fileSave()
{
    // qDebug() << Q_FUNC_INFO;
    if ( fileName_.isEmpty() ) {
        return fileSaveAs();
    }

    QFile file( fileName_ );
    if ( !file.open(QFile::WriteOnly) ) {
        QMessageBox::warning( this, tr("QDataStream"),
                              tr("Cannot write file %1:\n%2.").arg( QDir::toNativeSeparators(fileName_),
                                                                    file.errorString()));
        return false;
    }

    QDataStream out( &file );
    curr_state.serialize( out );
    file.close();

    statusBar()->showMessage( "File <"+ fileName_ + "> saved.");
    setWindowModified( false);

    return true;
}


void MainWindow::slotSelectionChanged()
{
    // enable 'delete' if one or more items are selected.
   actionDeleteSelection->setEnabled( m_scene->selectedItems().count()>0 );
}

void MainWindow::slotToggleShowBackgroundImage()
{
    m_pixmap->setVisible( !m_pixmap->isVisible() );
    barBackground->setEnabled( !barBackground->isEnabled() );
}

void MainWindow::slotToggleShowConstraints()
{
    // qDebug() << Q_FUNC_INFO;
    QConstraint::QConstraintBase::toggleShow();
    curr_state.toggleVisibilityConstraints( );
    m_scene->update();
}

void MainWindow::slotToggleShowStrokes()
{
    // qDebug() << Q_FUNC_INFO;
    QEntity::QStroke::toggleShow();
    curr_state.toggleVisibilityStrokes( );
    m_scene->update();
}

void MainWindow::slotToggleShowUncertainty()
{
    // qDebug() << Q_FUNC_INFO;
    QEntity::QSegment::toggleShowUncertainty();
    m_scene->update();
}




void MainWindow::slotToggleShowColored()
{
    qDebug() << Q_FUNC_INFO;
    QEntity::QConstrained::toogleShowColor();
    QConstraint::QConstraintBase::toggleShowColor();
    m_scene->update();
}

void MainWindow::slotUndoLastAction()
{
    qDebug() << Q_FUNC_INFO;
    if ( m_undoStack->index() == m_undoStack->count() ) {
        m_undoStack->undo();
    }
    else {
        m_undoStack->redo();
    }
}

void MainWindow::slotValueChangedOpacity( const double v)
{
    qDebug() << Q_FUNC_INFO;
    if ( m_pixmap!=nullptr ) {
        m_pixmap->setOpacity( v );
    }
}

void MainWindow::slotValueChangedAlphaRecognition( const double alpha_val)
{
    qDebug() << Q_FUNC_INFO;
    State::setAlphaRecognition(alpha_val);
}

void MainWindow::slotValueChangedAlphaSnap( const double alpha_val)
{
    qDebug() << Q_FUNC_INFO;
    State::setAlphaSnapping(alpha_val);
}

void MainWindow::slotBasicDocumentation()
{
    QMessageBox::about( this, QApplication::applicationName() + " " + QApplication::applicationVersion(),
                        "<h2> Basic documentation</h2>"
                        "<table style=\"white-space: nowrap;\">"
                        "<tr><td>[left mouse]    </td><td>draw straight strokes (no polylines)</td></tr>"
                        "<tr><td>[right mouse]   </td><td>select/unselect graphical elements</td></tr>"
                        "<tr><td>[mouse wheel]   </td><td>zoom in or out</td></tr>"
                        "</table>"
                        "<h2>Keyboard shortcuts</h2>"
                        "<table>"
                        "<tr><td>Navigation    </td><td>[+]/[-]</td>            <td>zoom in/out</td></tr>"
                        "<tr><td></td>              <td>[&#8592;]/[&#8594;]</td><td>move left/right</td></tr>"
                        "<tr><td></td>              <td>[&#8593;]/[&#8595;]</td><td>move up/down</td></tr>"
                        "<tr><td>Selection</td><td>[Ctrl]+[A]</td>        <td>select all</td></tr>"
                        "<tr><td></td>         <td>[Ctrl]+[Shift]+[A]</td><td>deselect all</td></tr>"
                        "<tr><td>Editing</td><td>[Ctrl]+[Z]</td><td>undo</td></tr>"
                        "<tr><td> </td>       <td>[Ctrl]+[Y]</td><td>redo</td></tr>"
                        "<tr><td></td>       <td>[Del]</td><td>delete selected items</td></tr>"
                        "<tr><td></td>       <td>[Ctrl]+[F]</td><td>format selected entities</td></tr>"
                        "<tr><td>Show</td><td>[Ctrl]+[U]</td><td>confidence regions (uncertainty)</td></tr>"
                        "<tr><td></td>    <td>[Ctrl]+[T]</td><td>background tiles</td></tr>"
                        "</table>"
                        );

}
void MainWindow::slotAbout()
{
    QMessageBox msgBox;
    msgBox.setTextFormat( Qt::RichText );
    msgBox.setText( QStringLiteral(
                        "<b>%1 %2: Freehand Drawing guided by Geometric Reasoning</b><br><br>"
                        "<b>Copyright.</b> \xa9 2022-2025 <a href=\"mailto:jochen.meidow@iosb.fraunhofer.de\">Jochen Meidow</a>, <a href=\"https://www.iosb.fraunhofer.de/en.html\">%3</a>, Germany.<br><br>"
                        "<b>Licensing.</b> Distributed under the <a href=\"https://www.gnu.org/licenses/gpl-3.0.en.html\">GNU General Public Licence</a>, Version 3.<br /><br />"
                        "<b>Dependencies.</b> This software uses the C++ template library <a href=\"https://eigen.tuxfamily.org\">Eigen</a>, version %4.%5.%6, "
                        "and the <a href=\"https://www.qt.io\">Qt</a> widget toolkit, version %7.<br /><br />"
                        "<b>Credits.</b> The author thanks Wolfgang Förstner and Horst Hammer for their inspiring collaboration.<br /><br />"
                        "<b>Reference</b>. Details of the implemented methods can be found in the following paper:"
                        "<blockquote>"
                        "J. Meidow (2023) Geometric Reasoning for the freehand-drawn Delineation of 2D Geometries, ISPRS Journal of Photogrammetry and Remote Sensing, vol. 201, pp. 67-77.</li>"
                        // "<li>J. Meidow and L. Lucks (2019) Draw and Order - Modeless Interactive Acquisition of Outlines, ISPRS - Annals of Photogrammetry, Remote Sensing and Spatial Information Sciences, vol. IV-2/W7, pp. 103-110.</li>"
                        // "<li>J. Meidow, H. Hammer and L. Lucks (2020) Delineation and Construction of 2D Geometries by Freehand Drawing and Geometric Reasoning, ISPRS - Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences, vol. V-5-2020, pp. 77-84.</li>"
                        "</blockquote>"
                        "Please cite this papers when using %1 or parts of it in an academic publication.")
                    .arg( QApplication::applicationName() )
                    .arg( QApplication::applicationVersion())
                    .arg( QApplication::organizationName())
                    .arg( EIGEN_WORLD_VERSION )
                    .arg( EIGEN_MAJOR_VERSION )
                    .arg( EIGEN_MINOR_VERSION )
                    .arg( QT_VERSION_STR )
                    );
    msgBox.setStandardButtons( QMessageBox::Ok );
    // msgBox.setIcon(); TODO(meijoc)
    // ?? msgBox.setIcon( windowIcon() );
    msgBox.exec();
}



void MainWindow::setCurrentFileName( const QString &fileName )
{
    fileName_ = fileName;

    QString shownName;
    if (fileName.isEmpty()) {
        shownName = QStringLiteral("untitled.grp");
    }
    else {
        shownName = QFileInfo(fileName).fileName();
    }
    setWindowTitle( QStringLiteral("[*]%1 - %2 %3")
                    .arg( shownName, QCoreApplication::applicationName(), QCoreApplication::applicationVersion())
                    );
    setWindowModified( false );
}


void MainWindow::slotUpdateLineStyle( const int s)
{
    // qDebug() << Q_FUNC_INFO;
    for ( auto & item : m_scene->selectedItems() )
    {
        if ( item->type()==QConstraint::QConstraintBase::Type ) {
            auto *a = qgraphicsitem_cast<QConstraint::QConstraintBase*>(item);
            a->setLineStyle( s );
        }
        if ( item->type()==QEntity::QSegment::Type ) {
            auto *c = qgraphicsitem_cast<QEntity::QSegment*>(item);
            c->setLineStyle( s );
        }
        m_scene->update();
    }
}

void MainWindow::slotUpdateColor( const QColor & col)
{
    // qDebug() << Q_FUNC_INFO << col;
    for ( auto & item : m_scene->selectedItems() )
    {
        if ( item->type()==QConstraint::QConstraintBase::Type ) {
            auto *i = qgraphicsitem_cast<QConstraint::QConstraintBase*>(item);
            i->setColor( col );
            continue;
        }
        if ( item->type()==QGraphicsPolygonItem::Type ) {
            auto *i = qgraphicsitem_cast<QEntity::QStroke*>(item);
            i->setColor( col );
            continue;
        }
        if ( item->type()==QEntity::QSegment::Type ) {
            auto *i = qgraphicsitem_cast<QEntity::QSegment*>(item);
            i->setColor( col );
        }
    }
    m_scene->update();
}


void MainWindow::slotUpdateLineWidth( const int w)
{
    // qDebug() << Q_FUNC_INFO;
    for ( auto & item : m_scene->selectedItems() )
    {
        if ( item->type()==QConstraint::QConstraintBase::Type ) {
            auto *a = qgraphicsitem_cast<QConstraint::QConstraintBase*>(item);
            a->setLineWidth( w );
        }
        if ( item->type()==QGraphicsPolygonItem::Type ) {
            auto *b = qgraphicsitem_cast<QEntity::QStroke*>(item);
            b->setLineWidth( w );
        }
        if ( item->type()==QEntity::QSegment::Type ) {
            auto *c = qgraphicsitem_cast<QEntity::QSegment*>(item);
            c->setLineWidth( w );
        }
    }
    m_scene->update();
}

void MainWindow::slotUpdateMarkerSize( const int sz)
{
   // qDebug() << Q_FUNC_INFO;
   for ( auto & item : m_scene->selectedItems() ) {
       //       auto *g = qgraphicsitem_cast<QConstraint::QConstraintBase*>(item);
       //       if ( g != nullptr ) {
       //           g->setMarkerSize( sz );
       //       }
       if ( item->type()==QConstraint::QConstraintBase::Type ) {
           auto *g = qgraphicsitem_cast<QConstraint::QConstraintBase*>(item);
           g->setMarkerSize( sz );
       }
   }
   m_scene->update();
}


void MainWindow::slotItemMoveToBottom()
{
    qDebug() << Q_FUNC_INFO;
    if ( m_scene->selectedItems().isEmpty() ) {
        return;
    }

    constexpr double shift = -0.1;
    const auto constSelectedItems = m_scene->selectedItems();
    for ( const auto & selectedItem : constSelectedItems ) {
        qreal zValue =  selectedItem->zValue();
        const auto constCollidingItems = selectedItem->collidingItems();
        for ( auto *const item : constCollidingItems ) {
            if ( item->zValue() <= zValue ) {
                zValue = item->zValue() +shift;
            }
        }
        selectedItem->setZValue( zValue );
        // selectedItem->setSelected( false ); // ??
    }
}

void MainWindow::slotItemMoveToTop()
{
    qDebug() << Q_FUNC_INFO;

    if ( m_scene->selectedItems().isEmpty() ) {
        return;
    }

    const auto constSelectedItems = m_scene->selectedItems();
    constexpr double shift = +0.1;
    for ( const auto & selectedItem : constSelectedItems ) {
        qreal zValue =  selectedItem->zValue();

        const auto collidingItems = selectedItem->collidingItems();
        for ( auto *const item : collidingItems ) {
            if ( item->zValue() >= zValue ) {
                zValue = item->zValue() +shift;
            }
        }
        selectedItem->setZValue( zValue );
        // selectedItem->setSelected( false );
    }
}

void MainWindow::slotToggleConsiderOrthogonal() {
    State::toggleConsiderOrthogonal();
    if ( State::considerOrthogonal() ) {
        statusBar()->showMessage(QStringLiteral("constraint 'orthogonal' enabled."));
    }
    else {
        statusBar()->showMessage(QStringLiteral("constraint 'orthogonal' disabled."));
    }
}

void MainWindow::slotToggleConsiderParallel()   {
    State::toggleConsiderParallel();
    if ( State::considerParallel() ) {
        statusBar()->showMessage(QStringLiteral("constraint 'parallel' enabled."));
    }
    else {
        statusBar()->showMessage(QStringLiteral("constraint 'parallel' disabled."));
    }
}

void MainWindow::slotToggleConsiderCopunctual() {
    State::toggleConsiderCopunctual();
    if ( State::considerCopunctual() ) {
        statusBar()->showMessage(QStringLiteral("constraint 'copunctual' enabled."));
    }
    else {
        statusBar()->showMessage(QStringLiteral("constraint 'copunctual' disabled."));
    }
}

void MainWindow::slotToggleConsiderVertical()   {
    State::toggleConsiderVertical();
    if ( State::considerVertical() ) {
        statusBar()->showMessage(QStringLiteral("constraint 'vertical' enabled."));
    }
    else {
        statusBar()->showMessage(QStringLiteral("constraint 'vertical' disabled."));
    }
}

void MainWindow::slotToggleConsiderHorizontal() {
    State::toggleConsiderHorizontal();
    if ( State::considerHorizontal() ) {
        statusBar()->showMessage(QStringLiteral("constraint 'horizontal' enabled."));
    }
    else {
        statusBar()->showMessage(QStringLiteral("constraint 'horizontal' disabled."));
    }
}

void MainWindow::slotToggleConsiderDiagonal() {
    State::toggleConsiderDiagonal();
    if ( State::considerDiagonal() ) {
        statusBar()->showMessage( QStringLiteral("constraint 'diagonal' enabled."));
    }
    else {
        statusBar()->showMessage( QStringLiteral("constraint 'diagonal' disabled."));
    }
}

} // namespace GUI
