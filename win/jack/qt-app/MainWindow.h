#pragma once

#include <QHash>
#include <QMainWindow>

#include "DeviceBridge.h"

class QStackedWidget;
class QLabel;
class OutputPane;
class ProjectPane;
class ConnectPane;
class CpPane;
class DAQPane;
class CpMemPane;
class GraphPane;
class ParmsPane;
class ImagePane;

// Qt shell replacing CMainFrame (MainFrm.h/.cpp). Phase 1 scope: dock
// layout, menu/toolbar/status bar skeleton, settings persistence, and the
// DeviceBridge wiring. Phase 2 ported ConnectPane (replacing ConView).
// Phase 3 ported CpPane, DAQPane, and CpMemPane (replacing CpView, DAQView,
// CpMemView). Phase 4 ported GraphPane (replacing GraphWnd/GraphSet with
// QCustomPlot), ParmsPane (replacing ParmsWnd - its real content is the
// per-lane plot scale settings GraphPane reads), and ImagePane (replacing
// ImageWnd/ImageSet/2DImage - mostly vestigial in the original; see
// ImagePane.h). Every view is now ported.
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    void createDockPanes();
    void createCentralPages();
    void createMenuAndToolbar();
    void createStatusBar();

    void restoreSettings();
    void saveSettings();

    DeviceBridge m_deviceBridge;

    ProjectPane *m_projectPane;
    OutputPane *m_outputPane;
    ConnectPane *m_connectPane;
    CpPane *m_cpPane;
    DAQPane *m_daqPane;
    CpMemPane *m_cpMemPane;
    GraphPane *m_graphPane;
    ParmsPane *m_parmsPane;
    ImagePane *m_imagePane;

    QStackedWidget *m_centralStack;
    QHash<QString, int> m_pageIndex;

    // Mirrors CMainFrame's MF_SBAR_* status bar panes (MainFrm.h).
    QLabel *m_sbStatus;
    QLabel *m_sbCount;
    QLabel *m_sbTimer;
    QLabel *m_sbCom;
    QLabel *m_sbConnect;
};
