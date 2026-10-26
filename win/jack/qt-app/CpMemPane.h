#pragma once

#include <QWidget>
#include <QColor>
#include <QVector>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "cm_const.h"
#include "cm_msg.h"
#include "cp_msg.h"
#include "cmapi.h"

#include "PlotData.h"

class QCheckBox;
class QComboBox;
class QLineEdit;
class QPushButton;
class QPlainTextEdit;
class QTimer;

// Replacement for CCpMemView (CpMemView.h/.cpp): CPU address-space block
// read/write (chunked CP_BLOCK_REQ/RESP transfers), file load/save of the
// local memory buffer, and a write-read-verify memory test (AWGN or
// uniform-random pattern, optionally cycling passes).
//
// Registers as the CM "CP Memory" client (CM_ID_CP_MEM_CLI) the same way
// CpPane/DAQPane register as their own clients - see CpPane.h for why
// cm_register()'s callback is a queued Qt signal here instead of
// CpMemView.cpp's ring buffer/dedicated worker thread.
//
// CpMemView.cpp owns its working buffer on the document (CappDoc::m_pCpBuf,
// 64KB, APP_MAX_CP_MEM_SIZE) because that's where every view keeps shared
// state in the MFC app. Nothing else references it, so here it's just a
// private member (m_buf) - same locality choice as DAQPane's m_adcBuf.
//
// OnPlot() sends the buffer to GraphWnd (GigaSoft ProEssentials);
// plotReady() is the replacement, connected to GraphPane by MainWindow.
class CpMemPane : public QWidget
{
    Q_OBJECT

public:
    explicit CpMemPane(QWidget *parent = nullptr);
    ~CpMemPane() override;

signals:
    void logMessage(const QString &text, const QColor &color = QColor(0, 0, 0));
    void testPassChanged(quint32 pass);
    void plotReady(const PlotData &data);

private slots:
    void onBrowseClicked();
    void onReadClicked();  // file -> buffer
    void onWriteClicked(); // buffer -> file
    void onGetClicked();   // device -> buffer (Get/Abort toggle)
    void onSetClicked();   // buffer -> device (Set/Abort toggle)
    void onClearClicked();
    void onTestClicked();
    void onTypeOrShowChanged();
    void onTimeout();

    // Queued delivery of cm_register(CM_ID_CP_MEM_CLI, ...)'s callback.
    void onCmMessage(quintptr msgPtr);

private:
    struct ChunkInfo {
        uint32_t cmType; // CFG_INT8U/CFG_INT16U/CFG_INT32U - the wire type
        uint32_t count;  // elements per 256-byte chunk: 256/128/64
    };

    ChunkInfo chunkInfo() const;
    void computeRequestCounts(uint32_t memLen, uint32_t &numCollect, uint32_t &numRequest) const;
    void sendBlockRequest(uint32_t index, bool write);
    void handleBlockReadResp(pcp_block_msg_t resp);
    void handleBlockWriteResp(pcp_block_msg_t resp);
    void readMemData();
    void writeMemData();
    void generateTestPattern();
    void validateTestPattern();
    void plotRun();
    void updateTextDump();
    void stopAll(const QString &idleButtonText, QPushButton *button);
    void log(const QString &text, const QColor &color = QColor(0, 0, 0));
    void loadSettings();
    void saveSettings();

    static uint32_t cpmMsgThunk(pcm_msg_t msg, void *member);

    QLineEdit *m_fileEdit;
    QPushButton *m_browseButton;
    QPushButton *m_readButton;
    QPushButton *m_writeButton;
    QCheckBox *m_binCheck;
    QCheckBox *m_verifyCheck;
    QCheckBox *m_allCheck;
    QCheckBox *m_awgnCheck;
    QCheckBox *m_cycleCheck;
    QCheckBox *m_hexCheck;
    QCheckBox *m_showCheck;
    QCheckBox *m_toFileCheck; // present for fidelity; CpMemView.cpp never actually reads this flag either
    QPushButton *m_getButton;
    QPushButton *m_setButton;
    QPushButton *m_clearButton;
    QPushButton *m_testButton;
    QLineEdit *m_startEdit;
    QLineEdit *m_lengthEdit;
    QLineEdit *m_xferEdit;
    QComboBox *m_typeCombo;
    QPlainTextEdit *m_dumpEdit;

    QVector<quint8> m_buf; // CP_MAX_BUFFER (APP_MAX_CP_MEM_SIZE)
    QTimer *m_timeoutTimer;

    bool m_getting;
    bool m_setting;
    bool m_testing;
    uint32_t m_memIndex;
    uint32_t m_numRequest;
    uint32_t m_passNumber;
    uint32_t m_lenDel;
};
