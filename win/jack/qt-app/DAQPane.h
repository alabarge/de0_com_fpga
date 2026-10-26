#pragma once

#include <QWidget>
#include <QColor>
#include <QFile>
#include <QVector>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "cm_const.h"
#include "cm_msg.h"
#include "daq_msg.h"
#include "cmapi.h"

#include "PlotData.h"

class QCheckBox;
class QLineEdit;
class QPushButton;
class QPlainTextEdit;

// Replacement for CDAQView (DAQView.h/.cpp): ADC acquisition control, file
// capture, and the live hex/decimal sample dump. Registers as the CM DAQ
// client (CM_ID_DAQ_CLI) the same way CpPane registers as the CP client -
// see CpPane.h for why cm_register()'s callback is handled via a queued
// Qt signal instead of DAQView.cpp's own ring buffer/thread.
//
// DAQView::OnPlotRun() sends live samples to GraphWnd (GigaSoft
// ProEssentials); plotReady() is the replacement, connected to GraphPane
// (QCustomPlot) by MainWindow.
//
// Only FIFO is a real transport in this Qt port (see ConnectPane), so of
// DAQView's five per-transport fifo_head()/lan_head()/udp_head()/
// opto_head()/com_head() calls on Start, only the FIFO one applies.
class DAQPane : public QWidget
{
    Q_OBJECT

public:
    explicit DAQPane(QWidget *parent = nullptr);
    ~DAQPane() override;

signals:
    void logMessage(const QString &text, const QColor &color = QColor(0, 0, 0));
    void packetCountChanged(quint32 count);
    void plotReady(const PlotData &data);

private slots:
    void onBrowseClicked();
    void onStartClicked();
    void onDoneIndDelayed();

    // Queued delivery of cm_register(CM_ID_DAQ_CLI, ...)'s callback.
    void onCmMessage(quintptr msgPtr);

private:
    void handlePipeData(pcm_pipe_daq_t pipe);
    void plotRun();
    void updateTextDump();
    void log(const QString &text, const QColor &color = QColor(0, 0, 0));
    void loadSettings();
    void saveSettings();

    static uint32_t daqMsgThunk(pcm_msg_t msg, void *member);

    QLineEdit *m_fileEdit;
    QPushButton *m_browseButton;
    QCheckBox *m_toFileCheck;
    QCheckBox *m_hexCheck;
    QCheckBox *m_rampCheck;
    QCheckBox *m_showCheck;
    QCheckBox *m_floatCheck;
    QCheckBox *m_plotCheck;
    QCheckBox *m_rawCheck;
    QLineEdit *m_packetsEdit;
    QPushButton *m_startButton;
    QPlainTextEdit *m_dumpEdit;

    QFile m_captureFile;
    bool m_running;

    quint32 m_pktCount;
    quint32 m_blkCount;
    qint64 m_xferTotal;
    double m_xferLen[10];
    double m_deltaT[10];
    quint32 m_lastStampUs;
    quint32 m_adcRate;

    QVector<qint32> m_adcBuf; // DAQ_MAX_PIPE_RUN * DAQ_MAX_SAM * DAQ_MAX_CH
};
