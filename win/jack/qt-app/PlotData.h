#pragma once

#include <QVector>

// Replacement for appType.h's PLOT/PPLOT struct, which DAQView.cpp/
// CpMemView.cpp heap-allocate (`new PLOT`) and hand off via
// PostMessage(WM_PLOT_MSG, (WPARAM)pPlot, 0) for GraphWnd to delete once
// plotted. DAQPane/CpMemPane/GraphPane are all GUI-thread Qt objects
// talking to each other directly through a signal/slot connection (not a
// cross-thread PostMessage), so a plain value type passed by const
// reference replaces the heap-allocate/PostMessage/delete dance entirely -
// QVector manages its own storage, no manual new/delete anywhere.
//
// PLOT also carries legend/autoScale[2]/yMin[2]/yMax[2] fields, sourced
// from CappDoc ini entries (qplotLegend/qplotAutoA/cplotYminA/...). These
// are genuinely user-editable - via ParmsPane (replacing ParmsWnd), the
// one real content of CappDoc's "DOC_TAG_PARM" property grid - but that
// wasn't built yet when GraphPane first went in, so they were dropped
// from here on the (wrong) assumption that nothing edited them. GraphPane
// now reads them straight out of ParmsPane's QSettings keys
// ("cplot.yMinA", "qplot.autoB", etc.) in applyAxisScale(), keyed by
// PlotData::plot and lane, so they don't need to round-trip through this
// struct at all.
struct PlotData {
    enum GraphType { None = 0, CpuMem = 1, Daq = 2 };

    GraphType plot = None;
    int type = 0;     // CpuMem: the CFG_* data type. Daq: always 1 (all channels) in this port.
    int lanes[2] = {0, 0};
    double freq = 0.0; // Daq only: sample rate, for the X axis
    int length = 0;    // samples per lane
    int chan = 0;
    double xMin = 0.0;
    double xMax = 0.0;
    int record = 0;

    // Flat, lane-major: lanes[0]+lanes[1] lanes of `length` points each,
    // lane 0 first - same layout DAQView.cpp/CpMemView.cpp built for
    // ProEssentials's PEvsetW() "wide" setter.
    QVector<float> dataX;
    QVector<float> dataY;
};
