#include "MainWindow.h"

#include <QAction>
#include <QCloseEvent>
#include <QDockWidget>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSettings>
#include <QStackedWidget>
#include <QStatusBar>
#include <QToolBar>

#include "ConnectPane.h"
#include "CpMemPane.h"
#include "CpPane.h"
#include "DAQPane.h"
#include "GraphPane.h"
#include "ImagePane.h"
#include "OutputPane.h"
#include "ParmsPane.h"
#include "ProjectPane.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , m_projectPane(nullptr)
    , m_outputPane(nullptr)
    , m_connectPane(nullptr)
    , m_cpPane(nullptr)
    , m_daqPane(nullptr)
    , m_cpMemPane(nullptr)
    , m_graphPane(nullptr)
    , m_parmsPane(nullptr)
    , m_imagePane(nullptr)
    , m_centralStack(nullptr)
    , m_sbStatus(nullptr)
    , m_sbCount(nullptr)
    , m_sbTimer(nullptr)
    , m_sbCom(nullptr)
    , m_sbConnect(nullptr)
{
    setWindowTitle(tr("jack (Qt)"));

    // cm_init() unconditionally resets every cm.obj[] slot (cmapi.cpp), so
    // it must run before anything calls cm_register() - CpPane does, from
    // its constructor. This matches the MFC app's real ordering too:
    // CMainFrame::OnCreate() calls cm_init() long before any view (which
    // only register with cm_register() in their own OnInitialUpdate()) is
    // ever constructed.
    const uint32_t result = m_deviceBridge.init();

    createCentralPages();
    createDockPanes();
    createMenuAndToolbar();
    createStatusBar();

    connect(&m_deviceBridge, &DeviceBridge::cmTraffic, m_outputPane, &OutputPane::onCmTraffic);
    connect(m_connectPane, &ConnectPane::logMessage, m_outputPane, &OutputPane::logMessage);
    connect(m_connectPane, &ConnectPane::connectionStateChanged, this,
            [this](bool /*connected*/, const QString &label) { m_sbConnect->setText(label); });
    connect(m_cpPane, &CpPane::logMessage, m_outputPane, &OutputPane::logMessage);
    connect(m_cpPane, &CpPane::deviceVersionReceived, m_connectPane, &ConnectPane::onDeviceVersion);
    connect(m_daqPane, &DAQPane::logMessage, m_outputPane, &OutputPane::logMessage);
    connect(m_daqPane, &DAQPane::packetCountChanged, this,
            [this](quint32 count) { m_sbCount->setText(tr("Count: %1").arg(count)); });
    connect(m_cpMemPane, &CpMemPane::logMessage, m_outputPane, &OutputPane::logMessage);
    connect(m_cpMemPane, &CpMemPane::testPassChanged, this,
            [this](quint32 pass) { m_sbCount->setText(tr("Count: %1").arg(pass)); });
    connect(m_daqPane, &DAQPane::plotReady, m_graphPane, &GraphPane::plot);
    connect(m_cpMemPane, &CpMemPane::plotReady, m_graphPane, &GraphPane::plot);

    m_outputPane->logMessage(tr("cm_init() returned %1").arg(result));

    restoreSettings();
}

void MainWindow::createCentralPages()
{
    m_centralStack = new QStackedWidget(this);

    m_connectPane = new ConnectPane(m_centralStack);
    m_pageIndex.insert(QStringLiteral("connect"), m_centralStack->addWidget(m_connectPane));

    m_cpPane = new CpPane(m_centralStack);
    m_pageIndex.insert(QStringLiteral("cp"), m_centralStack->addWidget(m_cpPane));

    m_daqPane = new DAQPane(m_centralStack);
    m_pageIndex.insert(QStringLiteral("daq"), m_centralStack->addWidget(m_daqPane));

    m_cpMemPane = new CpMemPane(m_centralStack);
    m_pageIndex.insert(QStringLiteral("cp_mem"), m_centralStack->addWidget(m_cpMemPane));

    setCentralWidget(m_centralStack);
}

void MainWindow::createDockPanes()
{
    // Project View + Parameters, tabbed together on the left - the Qt
    // equivalent of GetParmsWnd().AttachToTabWnd(&GetProjectWnd(), ...) in
    // CMainFrame::CreateDockingWindows().
    m_projectPane = new ProjectPane(this);
    auto *projectDock = new QDockWidget(tr("Project View"), this);
    projectDock->setObjectName(QStringLiteral("ProjectDock"));
    projectDock->setWidget(m_projectPane);
    addDockWidget(Qt::LeftDockWidgetArea, projectDock);

    m_parmsPane = new ParmsPane(this);
    auto *parmsDock = new QDockWidget(tr("Parameters"), this);
    parmsDock->setObjectName(QStringLiteral("ParmsDock"));
    parmsDock->setWidget(m_parmsPane);
    addDockWidget(Qt::LeftDockWidgetArea, parmsDock);
    tabifyDockWidget(projectDock, parmsDock);
    projectDock->raise();

    connect(m_projectPane, &ProjectPane::pageSelected, this, [this](const QString &pageId) {
        if (m_pageIndex.contains(pageId))
            m_centralStack->setCurrentIndex(m_pageIndex.value(pageId));
    });

    // Output ("Log") pane, docked bottom - CMainFrame::DockPane(&GetOutputWnd()).
    m_outputPane = new OutputPane(this);
    auto *outputDock = new QDockWidget(tr("Log"), this);
    outputDock->setObjectName(QStringLiteral("OutputDock"));
    outputDock->setWidget(m_outputPane);
    addDockWidget(Qt::BottomDockWidgetArea, outputDock);

    // Graph + Image, tabbed together and hidden by default - the Qt
    // equivalent of CMainFrame::CreateDockingWindows() floating GraphWnd
    // and ImageWnd into the same tabbed bar and calling
    // m_pTabbedBar.ShowPane(FALSE, ...).
    m_graphPane = new GraphPane(this);
    auto *graphDock = new QDockWidget(tr("Graph"), this);
    graphDock->setObjectName(QStringLiteral("GraphDock"));
    graphDock->setWidget(m_graphPane);
    addDockWidget(Qt::RightDockWidgetArea, graphDock);

    m_imagePane = new ImagePane(this);
    auto *imageDock = new QDockWidget(tr("2D Image"), this);
    imageDock->setObjectName(QStringLiteral("ImageDock"));
    imageDock->setWidget(m_imagePane);
    addDockWidget(Qt::RightDockWidgetArea, imageDock);
    tabifyDockWidget(graphDock, imageDock);

    graphDock->hide();
    imageDock->hide();
}

void MainWindow::createMenuAndToolbar()
{
    QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
    QAction *exitAction = fileMenu->addAction(tr("E&xit"), this, &QWidget::close);
    exitAction->setShortcut(QKeySequence::Quit);

    QMenu *viewMenu = menuBar()->addMenu(tr("&View"));
    for (QDockWidget *dock : findChildren<QDockWidget *>())
        viewMenu->addAction(dock->toggleViewAction());

    QMenu *helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(tr("&About..."), this, [this]() {
        QMessageBox::about(this, tr("About jack"), tr("jack (Qt port) - Phase 1"));
    });

    QToolBar *toolBar = addToolBar(tr("Main"));
    toolBar->setObjectName(QStringLiteral("MainToolBar"));
    //toolBar->addAction(tr("Connect"));
    //toolBar->addAction(tr("Disconnect"));
}

void MainWindow::createStatusBar()
{
    // Mirrors CMainFrame's MF_SBAR_STATUS/COUNT/TIMER/COM/CONNECT panes.
    m_sbStatus = new QLabel(tr("Ready"), this);
    m_sbCount = new QLabel(tr("Count: 0"), this);
    m_sbTimer = new QLabel(tr("Timer: --"), this);
    m_sbCom = new QLabel(tr("Com: --"), this);
    m_sbConnect = new QLabel(tr("Disconnected"), this);

    for (QLabel *label : {m_sbCount, m_sbTimer, m_sbCom, m_sbConnect}) {
        label->setMinimumWidth(90);
        label->setAlignment(Qt::AlignCenter);
    }

    statusBar()->addWidget(m_sbStatus, 1);
    statusBar()->addPermanentWidget(m_sbCount);
    statusBar()->addPermanentWidget(m_sbTimer);
    statusBar()->addPermanentWidget(m_sbCom);
    statusBar()->addPermanentWidget(m_sbConnect);
}

void MainWindow::restoreSettings()
{
    QSettings settings;
    restoreGeometry(settings.value(QStringLiteral("MainWindow/geometry")).toByteArray());
    restoreState(settings.value(QStringLiteral("MainWindow/state")).toByteArray());
}

void MainWindow::saveSettings()
{
    QSettings settings;
    settings.setValue(QStringLiteral("MainWindow/geometry"), saveGeometry());
    settings.setValue(QStringLiteral("MainWindow/state"), saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    // ConView::OnClosing(): close any open device connection while the
    // window (and everything connectionStateChanged reaches, like
    // m_sbConnect below) is still fully alive - see ConnectPane::
    // closeConnection()'s comment for why this can't wait for destructors.
    m_connectPane->closeConnection();
    saveSettings();
    QMainWindow::closeEvent(event);
}
