#pragma once

#include <QWidget>
#include <QColor>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "cm_const.h"
#include "cm_msg.h"
#include "cp_msg.h"
#include "cmapi.h"

class QComboBox;
class QCheckBox;
class QLineEdit;
class QPushButton;
class QPlainTextEdit;
class QTimer;

// Replacement for CCpView (CpView.h/.cpp): the "CP" (Control Panel) client -
// CPU address-space read/write, trace-flag control, and the periodic
// ping/version handshake with the firmware's CP server.
//
// cm_register(CM_ID_CP_CLI, ...)'s callback can fire from whatever thread
// the CM layer's I/O lives on. CpView.cpp marshals this onto the GUI thread
// with its own ring buffer + CRITICAL_SECTION + condition variable +
// dedicated worker thread (cp_qmsg()/cp_thread()) purely to do that safely.
// Qt's queued signal/slot connections do the same job for free - see
// DeviceBridge - so none of that machinery is reproduced here.
class CpPane : public QWidget
{
    Q_OBJECT

public:
    explicit CpPane(QWidget *parent = nullptr);
    ~CpPane() override;

signals:
    void logMessage(const QString &text, const QColor &color = QColor(0, 0, 0));

    // CpView.cpp's cp_msg() forwards CP_VER_RESP to ConView via
    // MainFrm::OnMessageView(IDD_CONNECT, ...); MainWindow connects this
    // signal straight to ConnectPane instead - no MainWindow-mediated
    // view lookup needed with Qt signals.
    void deviceVersionReceived(quint32 fwVer, quint32 sysid, quint32 stampEpoch, quint8 fifoRev,
                                quint8 adcRev);

private slots:
    void onGetClicked();
    void onSetClicked();
    void onTraceGetClicked();
    void onTraceSetClicked();
    void onTraceOnClicked();
    void onTraceOffClicked();
    void onTraceCheckToggled();
    void onPingTimer();
    void onPingTimeout();

    // Queued delivery of cm_register(CM_ID_CP_CLI, ...)'s callback.
    void onCmMessage(quintptr msgPtr);

private:
    void buildAddressMapReport();
    void sendMemRequest(bool write);
    void sendTraceRequest(bool set);
    void checkToControl(); // trace checkboxes -> trace hex field
    void controlToCheck(); // trace hex field -> trace checkboxes
    void handleVersionResponse(pcp_ver_msg_t ver);
    void handleMemResponse(pcp_mem_msg_t mem);
    void handleTraceResponse(pcp_trace_msg_t trace);
    void setValueField(uint32_t type, double value);
    void log(const QString &text, const QColor &color = QColor(0, 0, 0));

    static uint32_t cpMsgThunk(pcm_msg_t msg, void *member);

    QLineEdit *m_addrEdit;
    QLineEdit *m_valueEdit;
    QComboBox *m_typeCombo;
    QCheckBox *m_wrCheck;
    QPushButton *m_getButton;
    QPushButton *m_setButton;

    QLineEdit *m_traceEdit;
    QPushButton *m_traceGetButton;
    QPushButton *m_traceSetButton;
    QPushButton *m_traceOnButton;
    QPushButton *m_traceOffButton;
    QCheckBox *m_ciCheck;
    QCheckBox *m_cmCheck;
    QCheckBox *m_irqCheck;
    QCheckBox *m_dvrCheck;
    QCheckBox *m_runCheck;
    QCheckBox *m_errCheck;
    QCheckBox *m_idCheck;
    QCheckBox *m_routeCheck;
    QCheckBox *m_uartCheck;

    QPlainTextEdit *m_reportEdit;

    QTimer *m_pingTimer;
    QTimer *m_pingTimeoutTimer;

    uint8_t m_cmHandle;
    uint32_t m_cpAddr;
    uint32_t m_cpTrace;
    uint32_t m_cpType;
};
