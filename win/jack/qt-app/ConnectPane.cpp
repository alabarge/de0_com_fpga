#include "ConnectPane.h"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QTimer>
#include <QVBoxLayout>

#include <iterator>

#include "appCfg.h"
#include "fpga_build.h"

namespace {
constexpr int PortRole = Qt::UserRole;

// ConView.cpp: static DWORD fifo_speed[]/com_speed[].
constexpr quint32 kFifoSpeed = 480000000;
constexpr quint32 kComSpeed[] = {115200, 57600, 19200, 9600};

QString formatSpeed(quint32 bps)
{
    return (bps < 1000000) ? QObject::tr("%1 Kbps").arg(bps / 1000.0, 0, 'f', 1)
                            : QObject::tr("%1 Mbps").arg(bps / 1000000.0, 0, 'f', 1);
}
}

ConnectPane::ConnectPane(QWidget *parent)
    : QWidget(parent)
    , m_fifoInfo(nullptr)
    , m_comInfo(nullptr)
    , m_comPort(APP_PORT_NONE)
    , m_nCom(ConNone)
{
    m_typeCombo = new QComboBox(this);
    // app.rc IDD_CONNECT DLGINIT order for IDC_CON_PORT_TYPE: COM, UART,
    // OPTO, FIFO, LAN, UDP, TCP - matches APP_CON_* ordinals in appCfg.h.
    m_typeCombo->addItems({tr("COM"), tr("UART"), tr("OPTO"), tr("FIFO"), tr("LAN"), tr("UDP"), tr("TCP")});

    m_portCombo = new QComboBox(this);
    m_speedCombo = new QComboBox(this);
    m_ipAddrEdit = new QLineEdit(this);
    m_macAddrEdit = new QLineEdit(this);
    m_ipPortEdit = new QLineEdit(this);
    m_startupCheck = new QCheckBox(tr("Startup"), this);
    m_refreshButton = new QPushButton(tr("Refresh"), this);
    m_connectButton = new QPushButton(tr("Connect"), this);

    auto *connGroup = new QGroupBox(tr("Message Connection"), this);
    auto *connForm = new QFormLayout(connGroup);
    connForm->addRow(tr("Type"), m_typeCombo);
    connForm->addRow(tr("Port"), m_portCombo);
    connForm->addRow(tr("Speed"), m_speedCombo);
    connForm->addRow(tr("IP Addr"), m_ipAddrEdit);
    connForm->addRow(tr("MAC Addr"), m_macAddrEdit);
    connForm->addRow(tr("IP Port"), m_ipPortEdit);

    m_fwRevEdit = new QLineEdit(this);
    m_sysIdEdit = new QLineEdit(this);
    m_stampEdit = new QLineEdit(this);
    m_fifoRevEdit = new QLineEdit(this);
    m_adcRevEdit = new QLineEdit(this);
    for (QLineEdit *edit : {m_fwRevEdit, m_sysIdEdit, m_stampEdit, m_fifoRevEdit, m_adcRevEdit}) {
        edit->setReadOnly(true);
        edit->setEnabled(false);
    }

    // Populated once onDeviceVersion() is called - see CpPane's
    // deviceVersionReceived signal.
    m_devGroup = new QGroupBox(tr("Device Versions (awaiting connection)"), this);
    auto *devForm = new QFormLayout(m_devGroup);
    devForm->addRow(tr("Stamp"), m_stampEdit);
    devForm->addRow(tr("F/W"), m_fwRevEdit);
    devForm->addRow(tr("FIFO"), m_fifoRevEdit);
    devForm->addRow(tr("ADC"), m_adcRevEdit);
    devForm->addRow(tr("System ID"), m_sysIdEdit);

    m_srvCmCheck = new QCheckBox(tr("CM"), this);
    m_srvCpCheck = new QCheckBox(tr("CP"), this);
    m_srvDaqCheck = new QCheckBox(tr("DAQ"), this);
    m_cliCmCheck = new QCheckBox(tr("CM"), this);
    m_cliCpCheck = new QCheckBox(tr("CP"), this);
    m_cliDaqCheck = new QCheckBox(tr("DAQ"), this);
    for (QCheckBox *c : {m_srvCmCheck, m_srvCpCheck, m_srvDaqCheck, m_cliCmCheck, m_cliCpCheck, m_cliDaqCheck}) {
        c->setEnabled(false);
    }

    auto *srvGroup = new QGroupBox(tr("Services"), this);
    auto *srvLayout = new QVBoxLayout(srvGroup);
    for (QCheckBox *c : {m_srvCmCheck, m_srvCpCheck, m_srvDaqCheck})
        srvLayout->addWidget(c);

    auto *cliGroup = new QGroupBox(tr("Clients"), this);
    auto *cliLayout = new QVBoxLayout(cliGroup);
    for (QCheckBox *c : {m_cliCmCheck, m_cliCpCheck, m_cliDaqCheck})
        cliLayout->addWidget(c);

    auto *topRow = new QHBoxLayout();
    topRow->addWidget(m_refreshButton);
    topRow->addWidget(m_connectButton);
    topRow->addWidget(m_startupCheck);
    topRow->addStretch();

    auto *groups = new QHBoxLayout();
    groups->addWidget(connGroup);
    groups->addWidget(m_devGroup);
    groups->addWidget(srvGroup);
    groups->addWidget(cliGroup);

    auto *layout = new QVBoxLayout(this);
    layout->addLayout(topRow);
    layout->addLayout(groups);
    layout->addStretch();

    QSettings settings;
    m_startupCheck->setChecked(settings.value(QStringLiteral("ConnectPane/autoConnect"), false).toBool());

    connect(m_typeCombo, &QComboBox::currentIndexChanged, this, &ConnectPane::onPortTypeChanged);
    connect(m_typeCombo, &QComboBox::currentIndexChanged, this, [](int index) {
        QSettings().setValue(QStringLiteral("ConnectPane/type"), index);
    });
    connect(m_refreshButton, &QPushButton::clicked, this, &ConnectPane::onRefreshClicked);
    connect(m_connectButton, &QPushButton::clicked, this, &ConnectPane::onConnectClicked);
    connect(m_startupCheck, &QCheckBox::toggled, this, [](bool checked) {
        QSettings settings;
        settings.setValue(QStringLiteral("ConnectPane/autoConnect"), checked);
    });

    // ConView.cpp's own default is a hardcoded APP_CON_FIFO (appCfg.h), but
    // it persists the user's actual selection across sessions via the ini's
    // con.comCon field (OnCbnSelchangeConPortType()) - this port didn't
    // have that persistence yet, so it always reset to FIFO on launch.
    // Added now, defaulting to COM: it's the transport actually worth
    // testing against first on this board.
    const int startupType = settings.value(QStringLiteral("ConnectPane/type"), ConCom).toInt();
    m_typeCombo->setCurrentIndex(startupType);
    onPortTypeChanged(startupType);
    updateControlStates();

    // ConView::OnInitialUpdate(): cascading auto-connect timers if Startup
    // is checked, otherwise a single delayed refresh so the port list is
    // live even before the user touches anything.
    if (m_startupCheck->isChecked()) {
        QTimer::singleShot(250, this, &ConnectPane::onRefreshClicked);
        QTimer::singleShot(750, this, &ConnectPane::onConnectClicked);
    } else {
        QTimer::singleShot(1500, this, &ConnectPane::onRefreshClicked);
    }
}

ConnectPane::~ConnectPane()
{
    // ConView::OnClosing()'s "close the open connection before the app
    // exits" now runs from MainWindow::closeEvent() (see closeConnection()
    // below) while the whole window is still alive - not here. By the time
    // a child widget's destructor runs, sibling teardown (e.g. the status
    // bar's m_sbConnect label) may already have happened, and disconnectFifo
    // ()/disconnectCom() emit connectionStateChanged(), which MainWindow
    // connects to a lambda that touches m_sbConnect: emitting a signal from
    // a destructor with no guarantee the receiver side is still alive is
    // exactly the crash Qt's docs warn against. closeEvent() already leaves
    // m_nCom at ConNone in the normal path, so this is just a safety net for
    // any close path that skips closeEvent (e.g. a forced process exit).
    closeConnection();
}

void ConnectPane::closeConnection()
{
    if (m_nCom == ConFifo)
        disconnectFifo();
    else if (m_nCom == ConCom)
        disconnectCom();
}

void ConnectPane::onPortTypeChanged(int index)
{
    m_portCombo->clear();
    m_speedCombo->clear();
    m_comPort = APP_PORT_NONE;

    if (index == ConFifo) {
        m_speedCombo->addItem(formatSpeed(kFifoSpeed));
    } else if (index == ConCom) {
        // ConView::OnCbnSelchangeConPortType(): populate with com_speed[],
        // defaulting to whichever entry matches CFG_BAUD_RATE.
        int defaultIndex = 0;
        for (size_t i = 0; i < std::size(kComSpeed); ++i) {
            m_speedCombo->addItem(formatSpeed(kComSpeed[i]));
            if (kComSpeed[i] == CFG_BAUD_RATE)
                defaultIndex = static_cast<int>(i);
        }
        m_speedCombo->setCurrentIndex(defaultIndex);
    }

    // updateControlStates() below applies ConView::OnUpdatePortSpeed()'s
    // enable rule (COM-only, and only while disconnected).
    updateControlStates();
    onRefreshClicked();
}

void ConnectPane::onRefreshClicked()
{
    if (m_nCom != ConNone)
        return;

    m_portCombo->clear();
    m_comPort = APP_PORT_NONE;

    if (m_typeCombo->currentIndex() == ConFifo)
        refreshFifoPorts();
    else if (m_typeCombo->currentIndex() == ConCom)
        refreshComPorts();
    else
        log(tr("%1 is not yet implemented in the Qt port").arg(m_typeCombo->currentText()),
            QColor(111, 52, 196));

    updateControlStates();
}

void ConnectPane::refreshFifoPorts()
{
    uint32_t devCnt = fifo_query(&m_fifoInfo);
    if (devCnt == 0) {
        log(tr("Warning : No FIFO Devices Available"), QColor(111, 52, 196));
        return;
    }
    if (devCnt > APP_MAX_DEVICES) {
        log(tr("Error : Maximum Attached Devices Exceeded, %1").arg(devCnt), QColor(255, 0, 0));
        return;
    }

    bool devFound = false;
    for (uint32_t i = 0; i < devCnt; ++i) {
        const QString serial = QString::fromLatin1(m_fifoInfo[i].serial);
        const QString desc = QString::fromLatin1(m_fifoInfo[i].desc);

        // "O2" marks an Omniware FIFO device serial, same check ConView
        // uses to tell real hardware apart from unrelated FTDI devices.
        const bool isOmniware = serial.size() >= 2 && serial[0] == QLatin1Char('O') && serial[1] == QLatin1Char('2');
        const bool idOk = (m_fifoInfo[i].id == FIFO_VID_PID || m_fifoInfo[i].id == FIFO_VID_PID_ALT)
                           && m_fifoInfo[i].flags != 1;

        if (isOmniware && idOk)
            log(tr("FIFO.%1 :").arg(i), QColor(0, 0, 255));
        else
            log(tr("FIFO.%1 : Unavailable").arg(i), QColor(255, 0, 0));

        log(tr(" flags  : %1").arg(m_fifoInfo[i].flags, 8, 16, QLatin1Char('0')));
        log(tr(" type   : %1").arg(m_fifoInfo[i].type, 8, 16, QLatin1Char('0')));
        log(tr(" id     : %1").arg(m_fifoInfo[i].id, 8, 16, QLatin1Char('0')));
        log(tr(" locid  : %1").arg(m_fifoInfo[i].locid, 8, 16, QLatin1Char('0')));
        log(tr(" serial : %1").arg(serial));
        log(tr(" desc   : %1").arg(desc));

        if (isOmniware && m_fifoInfo[i].handle == nullptr
            && (m_fifoInfo[i].id == FIFO_VID_PID || m_fifoInfo[i].id == FIFO_VID_PID_ALT)) {
            m_portCombo->addItem(tr("FIFO.%1").arg(i), static_cast<uint>(i));
            devFound = true;
        }
    }

    m_comPort = devFound ? 0 : APP_PORT_NONE;
    if (devFound)
        m_portCombo->setCurrentIndex(0);
}

void ConnectPane::refreshComPorts()
{
    // ConView.cpp's COM branch: com_query() fills a fixed COM_MAX_DEVICES
    // slot array (slot index == COM port/locid), unlike FIFO's compact
    // devCnt-length array - so the item data stored per combo entry is the
    // locid itself, not the loop index.
    uint32_t devCnt = com_query(&m_comInfo);
    if (devCnt == 0) {
        log(tr("Warning : No COM Devices Available"), QColor(111, 52, 196));
        return;
    }
    if (devCnt > COM_MAX_DEVICES) {
        log(tr("Error : Maximum Attached Devices Exceeded, %1").arg(devCnt), QColor(255, 0, 0));
        return;
    }

    bool devFound = false;
    for (uint32_t i = 0; i < COM_MAX_DEVICES; ++i) {
        if (m_comInfo[i].locid == 0)
            continue;

        const QString serial = QString::fromLatin1(m_comInfo[i].serial);
        const QString desc = QString::fromLatin1(m_comInfo[i].desc);

        log(tr("COM.%1 :").arg(m_comInfo[i].locid), QColor(0, 0, 255));
        log(tr(" flags  : %1").arg(m_comInfo[i].flags, 8, 16, QLatin1Char('0')));
        log(tr(" type   : %1").arg(m_comInfo[i].type, 8, 16, QLatin1Char('0')));
        log(tr(" id     : %1").arg(m_comInfo[i].id, 8, 16, QLatin1Char('0')));
        log(tr(" locID  : %1").arg(m_comInfo[i].locid, 8, 16, QLatin1Char('0')));
        log(tr(" serial : %1").arg(serial));
        log(tr(" desc   : %1").arg(desc));

        m_portCombo->addItem(tr("COM.%1").arg(m_comInfo[i].locid), m_comInfo[i].locid);
        devFound = true;
    }

    m_comPort = devFound ? 0 : APP_PORT_NONE;
    if (devFound)
        m_portCombo->setCurrentIndex(0);
}

void ConnectPane::onConnectClicked()
{
    // ConView::OnBnClickedConConnect(): clear the previous connection's
    // version/services/clients display unconditionally before (dis)connecting.
    m_fwRevEdit->clear();
    m_sysIdEdit->clear();
    m_stampEdit->clear();
    m_fifoRevEdit->clear();
    m_adcRevEdit->clear();
    for (QCheckBox *c : {m_srvCmCheck, m_srvCpCheck, m_srvDaqCheck, m_cliCmCheck, m_cliCpCheck, m_cliDaqCheck})
        c->setChecked(false);
    m_devGroup->setTitle(tr("Device Versions (awaiting connection)"));

    if (m_nCom == ConNone) {
        // Connecting: dispatch on the selected Type.
        if (m_typeCombo->currentIndex() == ConFifo)
            connectFifo();
        else if (m_typeCombo->currentIndex() == ConCom)
            connectCom();
        else
            log(tr("%1 is not yet implemented in the Qt port").arg(m_typeCombo->currentText()),
                QColor(111, 52, 196));
    } else {
        // Disconnecting: dispatch on whichever transport is actually
        // connected (m_typeCombo stays locked to it, just disabled).
        if (m_nCom == ConFifo)
            disconnectFifo();
        else if (m_nCom == ConCom)
            disconnectCom();
    }
}

void ConnectPane::connectFifo()
{
    if (m_comPort == APP_PORT_NONE) {
        log(tr("Error : No Devices Attached or Selected"), QColor(255, 0, 0));
        return;
    }

    const uint port = m_portCombo->currentData().toUInt();
    const int32_t result = fifo_init(0, CM_PORT_COM0, port);
    if (result != FIFO_OK) {
        log(tr("Error : Failed to Open FIFO.%1 Device").arg(port), QColor(255, 0, 0));
        return;
    }

    m_nCom = ConFifo;

    const QString serial = QString::fromLatin1(m_fifoInfo[port].serial);
    log(tr("Opened FIFO.%1 (%2) for Messaging").arg(port).arg(serial));

    uint32_t libRev = 0, sysRev = 0, apiRev = 0;
    fifo_rev(&libRev, &sysRev, &apiRev);
    log(tr("FIFO.%1 : ftd2xx.lib %2, ftd2xx.sys %3, fifoapi.dll %4")
            .arg(port)
            .arg(libRev, 8, 16, QLatin1Char('0'))
            .arg(sysRev, 8, 16, QLatin1Char('0'))
            .arg(apiRev, 8, 16, QLatin1Char('0')));

    uint32_t sysid = 0, stamp = 0, cmDat = 0;
    fifo_sysid(&sysid, &stamp, &cmDat);
    log(tr("FIFO.%1 : sysID %2, stamp %3, cm %4")
            .arg(port)
            .arg(sysid)
            .arg(stamp)
            .arg(cmDat, 8, 16, QLatin1Char('0')));

    const QByteArray devName = QStringLiteral("%1 build 0").arg(QCoreApplication::applicationName()).toLocal8Bit();
    cm_send_reg_req(APP_DEVID, CM_PORT_COM0, CM_REG_OPEN,
                    reinterpret_cast<uint8_t *>(const_cast<char *>(devName.constData())));

    updateControlStates();
    emit connectionStateChanged(true, tr("Connected"));
}

void ConnectPane::disconnectFifo()
{
    const uint port = m_portCombo->currentData().toUInt();

    const QByteArray devName = QStringLiteral("%1 build 0").arg(QCoreApplication::applicationName()).toLocal8Bit();
    cm_send_reg_req(APP_DEVID, CM_PORT_COM0, CM_REG_CLOSE,
                    reinterpret_cast<uint8_t *>(const_cast<char *>(devName.constData())));

    const QString serial = m_fifoInfo ? QString::fromLatin1(m_fifoInfo[port].serial) : QString();
    log(tr("Closed FIFO.%1 (%2)").arg(port).arg(serial));

    fifo_final();

    m_nCom = ConNone;
    updateControlStates();
    emit connectionStateChanged(false, tr("Disconnected"));
}

void ConnectPane::connectCom()
{
    if (m_comPort == APP_PORT_NONE) {
        log(tr("Error : No Devices Attached or Selected"), QColor(255, 0, 0));
        return;
    }

    const uint port = m_portCombo->currentData().toUInt();
    const quint32 speed = kComSpeed[m_speedCombo->currentIndex()];
    const int32_t result = com_init(speed, CM_PORT_COM0, port);
    if (result != COM_OK) {
        log(tr("Error : Failed to Open COM.%1 Device").arg(port), QColor(255, 0, 0));
        return;
    }

    m_nCom = ConCom;

    // ConView.cpp logs `desc` here but `serial` in disconnectCom() below -
    // a small inconsistency in the original, kept as-is rather than
    // "fixed" into behavior it never had.
    const QString desc = QString::fromLatin1(m_comInfo[port].desc);
    log(tr("Opened COM.%1 (%2) for Messaging").arg(port).arg(desc));

    uint32_t libRev = 0, sysRev = 0, apiRev = 0;
    com_rev(&libRev, &sysRev, &apiRev);
    // ConView.cpp's format string only prints libRev/apiRev - sysRev is
    // fetched but never actually shown, same as the original.
    log(tr("COM.%1 : wsc64.lib %2, comapi.dll %3")
            .arg(port)
            .arg(libRev, 8, 16, QLatin1Char('0'))
            .arg(apiRev, 8, 16, QLatin1Char('0')));

    uint32_t sysid = 0, stamp = 0, cmDat = 0;
    com_sysid(&sysid, &stamp, &cmDat);
    log(tr("COM.%1 : sysID %2, stamp %3, cm %4")
            .arg(port)
            .arg(sysid)
            .arg(stamp)
            .arg(cmDat, 8, 16, QLatin1Char('0')));

    const QByteArray devName = QStringLiteral("%1 build 0").arg(QCoreApplication::applicationName()).toLocal8Bit();
    cm_send_reg_req(APP_DEVID, CM_PORT_COM0, CM_REG_OPEN,
                    reinterpret_cast<uint8_t *>(const_cast<char *>(devName.constData())));

    updateControlStates();
    emit connectionStateChanged(true, tr("Connected"));
}

void ConnectPane::disconnectCom()
{
    const uint port = m_portCombo->currentData().toUInt();

    const QByteArray devName = QStringLiteral("%1 build 0").arg(QCoreApplication::applicationName()).toLocal8Bit();
    cm_send_reg_req(APP_DEVID, CM_PORT_COM0, CM_REG_CLOSE,
                    reinterpret_cast<uint8_t *>(const_cast<char *>(devName.constData())));

    const QString serial = m_comInfo ? QString::fromLatin1(m_comInfo[port].serial) : QString();
    log(tr("Closed COM.%1 (%2)").arg(port).arg(serial));

    com_final();

    m_nCom = ConNone;
    updateControlStates();
    emit connectionStateChanged(false, tr("Disconnected"));
}

void ConnectPane::updateControlStates()
{
    const bool connected = (m_nCom != ConNone);

    m_connectButton->setText(connected ? tr("Disconnect") : tr("Connect"));
    m_connectButton->setEnabled(m_comPort != APP_PORT_NONE || connected);
    m_refreshButton->setEnabled(!connected);

    m_typeCombo->setEnabled(!connected);
    m_portCombo->setEnabled(!connected);
    m_ipAddrEdit->setEnabled(!connected);
    m_ipPortEdit->setEnabled(!connected);
    m_macAddrEdit->setEnabled(!connected && m_typeCombo->currentIndex() != ConUdp);

    // ConView::OnUpdatePortSpeed(): speed is only user-editable for COM,
    // and only while disconnected.
    m_speedCombo->setEnabled(!connected && m_typeCombo->currentIndex() == ConCom);
}

void ConnectPane::log(const QString &text, const QColor &color)
{
    emit logMessage(text, color);
}

void ConnectPane::onDeviceVersion(quint32 fwVer, quint32 sysid, quint32 stampEpoch, quint8 fifoRev,
                                   quint8 adcRev)
{
    // ConView::OnCmMsg(): show all connected service providers/clients.
    m_srvCmCheck->setChecked(cm_id_exists(CM_ID_INSTANCE));
    m_srvCpCheck->setChecked(cm_id_exists(CM_ID_CP_SRV));
    m_srvDaqCheck->setChecked(cm_id_exists(CM_ID_DAQ_SRV));
    m_cliCmCheck->setChecked(cm_id_exists(CM_ID_INSTANCE));
    m_cliCpCheck->setChecked(cm_id_exists(CM_ID_CP_CLI));
    m_cliDaqCheck->setChecked(cm_id_exists(CM_ID_DAQ_CLI));

    m_devGroup->setTitle(tr("Device Versions"));
    m_fwRevEdit->setText(QStringLiteral("%1.%2.%3.%4")
                              .arg((fwVer >> 24) & 0xFF)
                              .arg((fwVer >> 16) & 0xFF)
                              .arg((fwVer >> 8) & 0xFF)
                              .arg(fwVer & 0xFF));
    m_sysIdEdit->setText(QString::number(sysid));
    m_stampEdit->setText(QString::number(stampEpoch));
    m_fifoRevEdit->setText(QString::number(fifoRev));
    m_adcRevEdit->setText(QString::number(adcRev));

    log(tr("Quartus Build : %1").arg(QDateTime::fromSecsSinceEpoch(stampEpoch).toString()));

    if (sysid != FPGA_PID)
        QMessageBox::warning(this, tr("jack"), tr("Hardware System ID Doesn't Match Application!"));
}
