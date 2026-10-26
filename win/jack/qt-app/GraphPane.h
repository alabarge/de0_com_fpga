#pragma once

#include <QWidget>

#include "PlotData.h"

class QCustomPlot;
class QCPAxisRect;
class QMouseEvent;
class QPushButton;
class QLabel;

// Replacement for CGraphWnd/CGraphSet (GraphWnd.h/.cpp, GraphSet.h/.cpp):
// renders DAQPane's live ADC plot and CpMemPane's memory-buffer plot using
// QCustomPlot instead of GigaSoft ProEssentials. See
// third_party/qcustomplot/README.md for the GPL/commercial licensing note
// that comes with that swap.
//
// Like the original, this pane only accepts one graph type at a time - the
// Settings dialog's Type combo (None/CPU Memory/DAQ) gates which producer's
// plot() calls actually draw anything, same as CGraphWnd checking
// `m_cpGphType == m_plot.plot` before calling PlotMemory()/PlotDAQ().
//
// The real per-plot Y-axis scale and legend visibility come from
// ParmsPane's settings (replacing ParmsWnd's cplot.*/qplot.* property
// grid), read fresh in applyAxisScale() on every plot() call - not from
// this pane's own Settings dialog. This pane's own Auto/Y Min/Y Max only
// set the axis range once, immediately when Settings closes, before any
// real data has arrived - the same narrow role CGraphWnd::ResetGraph()
// gives m_cpGphXmin/.../m_cpGphAuto.
//
// Dual-axis DAQ plots (lanes[1] > 0) get Axis B its own full-width stacked
// lane (m_laneBRect) below Axis A's, rather than a second Y-scale sharing
// Axis A's chart area - see rebuildGraphs().
class GraphPane : public QWidget
{
    Q_OBJECT

public:
    explicit GraphPane(QWidget *parent = nullptr);

public slots:
    // Replacement for CGraphWnd::Plot(PPLOT) -> PlotGraph().
    void plot(const PlotData &data);

private slots:
    void onSettingsClicked();
    void onExportClicked();

private:
    void rebuildGraphs(int laneCount, bool dualAxis);
    void plotMemory(const PlotData &data);
    void plotDaq(const PlotData &data);
    void applyAxisScale(const PlotData &data);
    void updateWindowTitle();
    void loadSettings();
    void saveSettings();

    QCustomPlot *m_plot;
    // Axis B's own stacked lane (full-width, below Axis A's) - see
    // rebuildGraphs(): inserted into m_plot's layout only while dualAxis is
    // true, so a single-lane CpMemPane plot still gets the whole chart.
    QCPAxisRect *m_laneBRect;
    QPushButton *m_settingsButton;
    QPushButton *m_exportButton;
    QLabel *m_titleLabel;
    QLabel *m_cursorLabel;

    // CGraphSet's editable fields (GraphWnd.h's m_cpGph* members).
    PlotData::GraphType m_graphType;
    double m_xMin, m_xMax, m_yMin, m_yMax;
    bool m_autoScale;
    bool m_cursorEnabled;
    bool m_smallPoints;

    int m_lanes[2];
    int m_lastType; // GraphWnd.cpp's m_type: detects a lane/window-shape change to rebuild graphs
};
