#pragma once

#include <QWidget>
#include <QColor>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "cm_const.h"
#include "cm_msg.h"
#include "cmapi.h"
#include "comapi.h"
#include "fifoapi.h"

class QComboBox;
class QPushButton;
class QCheckBox;
class QLineEdit;
class QGroupBox;

// Replacement for CConView (ConView.h/.cpp). ConView talks to 7 transports
// (OPTO/FIFO/COM/LAN/UDP/TCP/UART) with near-identical enumerate/connect/
// disconnect code repeated per type; FIFO and COM are wired up for real
// here (COM added after FIFO - see refreshComPorts()/connectCom()). COM is
// the startup default (ConnectPane/type in settings, see the constructor) -
// simpler to test device I/O against than FIFO's USB FTDI dependency. The
// remaining entries in the Type combo are present for UI fidelity but log
// a "not implemented" message, ready to fill in the same way.
//
// The Device Versions/Services/Clients group only populates once
// onDeviceVersion() is called - wired by MainWindow to CpPane's
// deviceVersionReceived signal, the direct Qt replacement for the MFC
// app's MainFrm::OnMessageView(IDD_CONNECT, ...) relay from CpView.
class ConnectPane : public QWidget
{
    Q_OBJECT

public:
    explicit ConnectPane(QWidget *parent = nullptr);
    ~ConnectPane() override;

    // ConView::OnClosing(): close the open connection before the app exits.
    // Called from MainWindow::closeEvent(), while the window is still fully
    // alive - see ~ConnectPane()'s comment for why this can't just live in
    // the destructor.
    void closeConnection();

signals:
    void logMessage(const QString &text, const QColor &color = QColor(0, 0, 0));
    void connectionStateChanged(bool connected, const QString &label);

public slots:
    // ConView.cpp's OnCmMsg(): CP_VER_RESP display + the services/clients
    // cm_id_exists() checks.
    void onDeviceVersion(quint32 fwVer, quint32 sysid, quint32 stampEpoch, quint8 fifoRev,
                          quint8 adcRev);

private slots:
    void onPortTypeChanged(int index);
    void onRefreshClicked();
    void onConnectClicked();

private:
    enum ConType {
        ConCom = 0,
        ConUart = 1,
        ConOpto = 2,
        ConFifo = 3,
        ConLan = 4,
        ConUdp = 5,
        ConTcp = 6,
        ConNone = -1,
    };

    void refreshFifoPorts();
    void connectFifo();
    void disconnectFifo();
    void refreshComPorts();
    void connectCom();
    void disconnectCom();
    void updateControlStates();
    void log(const QString &text, const QColor &color = QColor(0, 0, 0));

    QComboBox *m_typeCombo;
    QComboBox *m_portCombo;
    QComboBox *m_speedCombo;
    QLineEdit *m_ipAddrEdit;
    QLineEdit *m_macAddrEdit;
    QLineEdit *m_ipPortEdit;
    QCheckBox *m_startupCheck;
    QPushButton *m_refreshButton;
    QPushButton *m_connectButton;

    QGroupBox *m_devGroup;
    QLineEdit *m_fwRevEdit;
    QLineEdit *m_sysIdEdit;
    QLineEdit *m_stampEdit;
    QLineEdit *m_fifoRevEdit;
    QLineEdit *m_adcRevEdit;

    QCheckBox *m_srvCmCheck;
    QCheckBox *m_srvCpCheck;
    QCheckBox *m_srvDaqCheck;
    QCheckBox *m_cliCmCheck;
    QCheckBox *m_cliCpCheck;
    QCheckBox *m_cliDaqCheck;

    fifo_dev_info_t *m_fifoInfo;
    com_dev_info_t *m_comInfo;
    int m_comPort; // selected index into m_portCombo, APP_PORT_NONE (-1) if none
    int m_nCom;    // currently-connected transport, ConNone if disconnected
};
