#include "CpPane.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>

#include "fpga_build.h"
#include "fw_cfg.h"
#include "system.h"

namespace {
CpPane *g_instance = nullptr; // for the static cm_register thunk, see cpMsgThunk()

QString hex32(quint32 v)
{
    return QStringLiteral("0x%1").arg(v, 8, 16, QLatin1Char('0')).toUpper();
}
}

CpPane::CpPane(QWidget *parent)
    : QWidget(parent)
    , m_cmHandle(0)
    , m_cpAddr(0)
    , m_cpTrace(0)
    , m_cpType(CFG_INT8U)
{
    g_instance = this;

    QSettings settings;
    m_cpAddr = settings.value(QStringLiteral("CpPane/addr"), 0).toUInt();
    m_cpTrace = settings.value(QStringLiteral("CpPane/trace"), 0).toUInt();
    m_cpType = settings.value(QStringLiteral("CpPane/type"), CFG_INT8U).toUInt();

    m_addrEdit = new QLineEdit(this);
    m_valueEdit = new QLineEdit(this);
    m_typeCombo = new QComboBox(this);
    // app.rc IDD_CP DLGINIT order for IDC_CP_DATA_TYPE, matching the
    // CFG_INT8U..CFG_FP64 ordinals in fw_cfg.h.
    m_typeCombo->addItems({tr("INT8U"), tr("INT8S"), tr("INT16U"), tr("INT16S"), tr("INT32U"),
                           tr("INT32S"), tr("FP32"), tr("INT64U"), tr("INT64S"), tr("FP64")});
    m_wrCheck = new QCheckBox(tr("W/R"), this);
    m_getButton = new QPushButton(tr("Get"), this);
    m_setButton = new QPushButton(tr("Set"), this);

    auto *memGroup = new QGroupBox(tr("CPU Address Space Read/Write"), this);
    auto *memForm = new QFormLayout(memGroup);
    memForm->addRow(tr("Address"), m_addrEdit);
    memForm->addRow(tr("Value"), m_valueEdit);
    memForm->addRow(tr("Type"), m_typeCombo);
    auto *memButtons = new QHBoxLayout();
    memButtons->addWidget(m_getButton);
    memButtons->addWidget(m_setButton);
    memButtons->addWidget(m_wrCheck);
    memButtons->addStretch();
    memForm->addRow(memButtons);

    m_traceEdit = new QLineEdit(this);
    m_traceGetButton = new QPushButton(tr("Get"), this);
    m_traceSetButton = new QPushButton(tr("Set"), this);
    m_traceOnButton = new QPushButton(tr("On"), this);
    m_traceOffButton = new QPushButton(tr("Off"), this);
    m_ciCheck = new QCheckBox(tr("CI"), this);
    m_cmCheck = new QCheckBox(tr("CM"), this);
    m_irqCheck = new QCheckBox(tr("IRQ"), this);
    m_dvrCheck = new QCheckBox(tr("DVR"), this);
    m_runCheck = new QCheckBox(tr("RUN"), this);
    m_errCheck = new QCheckBox(tr("ERR"), this);
    m_idCheck = new QCheckBox(tr("ID"), this);
    m_routeCheck = new QCheckBox(tr("ROUTE"), this);
    m_uartCheck = new QCheckBox(tr("UART"), this);

    auto *traceGroup = new QGroupBox(tr("Trace Flags"), this);
    auto *traceLayout = new QVBoxLayout(traceGroup);
    auto *traceRow = new QHBoxLayout();
    traceRow->addWidget(m_traceEdit);
    traceRow->addWidget(m_traceGetButton);
    traceRow->addWidget(m_traceSetButton);
    traceRow->addWidget(m_traceOnButton);
    traceRow->addWidget(m_traceOffButton);
    traceLayout->addLayout(traceRow);
    auto *checkGrid = new QHBoxLayout();
    auto *checkCol1 = new QVBoxLayout();
    for (QCheckBox *c : {m_ciCheck, m_cmCheck, m_idCheck})
        checkCol1->addWidget(c);
    auto *checkCol2 = new QVBoxLayout();
    for (QCheckBox *c : {m_irqCheck, m_dvrCheck, m_routeCheck})
        checkCol2->addWidget(c);
    auto *checkCol3 = new QVBoxLayout();
    for (QCheckBox *c : {m_runCheck, m_errCheck, m_uartCheck})
        checkCol3->addWidget(c);
    checkGrid->addLayout(checkCol1);
    checkGrid->addLayout(checkCol2);
    checkGrid->addLayout(checkCol3);
    checkGrid->addStretch();
    traceLayout->addLayout(checkGrid);
    traceLayout->addStretch();

    auto *leftColumn = new QVBoxLayout();
    leftColumn->addWidget(memGroup);
    leftColumn->addWidget(traceGroup);
    leftColumn->addStretch();

    m_reportEdit = new QPlainTextEdit(this);
    m_reportEdit->setReadOnly(true);

    auto *layout = new QHBoxLayout(this);
    layout->addLayout(leftColumn);
    layout->addWidget(m_reportEdit);

    m_pingTimer = new QTimer(this);
    m_pingTimeoutTimer = new QTimer(this);
    m_pingTimeoutTimer->setSingleShot(true);

    connect(m_getButton, &QPushButton::clicked, this, &CpPane::onGetClicked);
    connect(m_setButton, &QPushButton::clicked, this, &CpPane::onSetClicked);
    connect(m_traceGetButton, &QPushButton::clicked, this, &CpPane::onTraceGetClicked);
    connect(m_traceSetButton, &QPushButton::clicked, this, &CpPane::onTraceSetClicked);
    connect(m_traceOnButton, &QPushButton::clicked, this, &CpPane::onTraceOnClicked);
    connect(m_traceOffButton, &QPushButton::clicked, this, &CpPane::onTraceOffClicked);
    for (QCheckBox *c : {m_ciCheck, m_cmCheck, m_irqCheck, m_dvrCheck, m_runCheck, m_errCheck,
                         m_idCheck, m_routeCheck, m_uartCheck})
        connect(c, &QCheckBox::toggled, this, &CpPane::onTraceCheckToggled);
    connect(m_pingTimer, &QTimer::timeout, this, &CpPane::onPingTimer);
    connect(m_pingTimeoutTimer, &QTimer::timeout, this, &CpPane::onPingTimeout);

    m_addrEdit->setText(hex32(m_cpAddr));
    m_traceEdit->setText(hex32(m_cpTrace));
    m_typeCombo->setCurrentIndex(static_cast<int>(m_cpType));
    setValueField(m_cpType, 0.0);
    controlToCheck();

    buildAddressMapReport();

    // CpView::OnInitialUpdate(): cp_init() - register as the CM Control
    // Panel client.
    m_cmHandle = cm_register(CM_ID_CP_CLI, &CpPane::cpMsgThunk, this, nullptr);
}

CpPane::~CpPane()
{
    if (g_instance == this)
        g_instance = nullptr;
}

void CpPane::buildAddressMapReport()
{
    QString text;
    text += tr("NIOS Address Map, from system.h :\n");
    text += tr("Module            Address\n");
    text += tr("============      ==========\n");

#ifdef SDRAM_BASE
    text += tr("  sdram           0x%1\n").arg(static_cast<quint32>(SDRAM_BASE), 8, 16, QLatin1Char('0'));
#ifdef SDRAM_SPAN
    text += tr("                  0x%1\n")
                .arg(static_cast<quint32>(SDRAM_BASE) + static_cast<quint32>(SDRAM_SPAN) - 1, 8, 16,
                     QLatin1Char('0'));
#endif
#endif
#ifdef STAMP_BASE
    text += tr("  stamp           0x%1\n").arg(static_cast<quint32>(STAMP_BASE), 8, 16, QLatin1Char('0'));
#endif
#ifdef GPX_BASE
    text += tr("  gpx             0x%1\n").arg(static_cast<quint32>(GPX_BASE), 8, 16, QLatin1Char('0'));
#endif
#ifdef GPI_BASE
    text += tr("  gpi             0x%1\n").arg(static_cast<quint32>(GPI_BASE), 8, 16, QLatin1Char('0'));
#endif
#ifdef COM_BASE
    text += tr("  com             0x%1\n").arg(static_cast<quint32>(COM_BASE), 8, 16, QLatin1Char('0'));
#ifdef COM_SPAN
    text += tr("                  0x%1\n")
                .arg(static_cast<quint32>(COM_BASE) + static_cast<quint32>(COM_SPAN) - 1, 8, 16,
                     QLatin1Char('0'));
#endif
#endif
#ifdef ADC_BASE
    text += tr("  adc             0x%1\n").arg(static_cast<quint32>(ADC_BASE), 8, 16, QLatin1Char('0'));
#endif
#ifdef UPDATE_BASE
    text += tr("  update          0x%1\n").arg(static_cast<quint32>(UPDATE_BASE), 8, 16, QLatin1Char('0'));
#endif

    text += tr("  pid             %1\n").arg(FPGA_PID);
    text += tr("  stamp           %1\n").arg(FPGA_EPOCH);
    text += QStringLiteral("\n");

    text += tr("CM Constants (cm_const.h):\n");
    text += tr("  CM_ID_CP_SRV       0x%1\n").arg(CM_ID_CP_SRV, 2, 16, QLatin1Char('0'));
    text += tr("  CM_DEV_WIN         0x%1\n").arg(CM_DEV_WIN, 2, 16, QLatin1Char('0'));
    text += tr("  CM_ENDIAN          0x%1\n").arg(CM_ENDIAN, 2, 16, QLatin1Char('0'));
    text += tr("  CM_MAX_MSG_INT8U   0x%1\n").arg(CM_MAX_MSG_INT8U, 4, 16, QLatin1Char('0'));
    text += tr("  CM_MAX_OBJS        0x%1\n").arg(CM_MAX_OBJS, 2, 16, QLatin1Char('0'));

    m_reportEdit->setPlainText(text.toUpper());
}

void CpPane::onGetClicked()
{
    sendMemRequest(false);
}

void CpPane::onSetClicked()
{
    sendMemRequest(true);
}

void CpPane::sendMemRequest(bool write)
{
    m_cpAddr = m_addrEdit->text().toUInt(nullptr, 16);

    pcmq_t slot = cm_alloc();
    if (slot == nullptr)
        return;

    auto *msg = reinterpret_cast<pcp_mem_msg_t>(slot->buf);
    msg->p.srvid = CM_ID_CP_SRV;
    msg->p.msgid = CP_MEM_REQ;
    msg->p.flags = write ? CP_MEM_WR : CP_MEM_RD;
    msg->p.status = CP_OK;
    msg->b.address = m_cpAddr;
    msg->b.value = write ? static_cast<uint32_t>(m_valueEdit->text().toUInt(nullptr, 16)) : 0;
    msg->b.type = m_cpType;

    cm_send_t ps = {};
    ps.msg = reinterpret_cast<pcm_msg_t>(msg);
    ps.dst_cmid = CM_ID_CP_SRV;
    ps.src_cmid = CM_ID_CP_CLI;
    ps.msglen = sizeof(cp_mem_msg_t);
    cm_send(CM_MSG_REQ, &ps);
}

void CpPane::onTraceGetClicked()
{
    sendTraceRequest(false);
}

void CpPane::onTraceSetClicked()
{
    sendTraceRequest(true);
}

void CpPane::sendTraceRequest(bool set)
{
    m_cpTrace = m_traceEdit->text().toUInt(nullptr, 16);

    pcmq_t slot = cm_alloc();
    if (slot == nullptr)
        return;

    auto *msg = reinterpret_cast<pcp_trace_msg_t>(slot->buf);
    msg->p.srvid = CM_ID_CP_SRV;
    msg->p.msgid = CP_TRACE_REQ;
    msg->p.flags = set ? CP_TRACE_SET : CP_TRACE_GET;
    msg->p.status = CP_OK;
    msg->b.trace = m_cpTrace;
    msg->b.debug = 0;

    cm_send_t ps = {};
    ps.msg = reinterpret_cast<pcm_msg_t>(msg);
    ps.dst_cmid = CM_ID_CP_SRV;
    ps.src_cmid = CM_ID_CP_CLI;
    ps.msglen = sizeof(cp_trace_msg_t);
    cm_send(CM_MSG_REQ, &ps);
}

void CpPane::onTraceOnClicked()
{
    m_cpTrace = 0xFFFFFFFF;
    m_traceEdit->setText(hex32(m_cpTrace));
    controlToCheck();
}

void CpPane::onTraceOffClicked()
{
    m_cpTrace = 0;
    m_traceEdit->setText(hex32(m_cpTrace));
    controlToCheck();
}

void CpPane::onTraceCheckToggled()
{
    checkToControl();
}

void CpPane::checkToControl()
{
    uint32_t trace = m_traceEdit->text().toUInt(nullptr, 16);

    auto apply = [&trace](QCheckBox *check, uint32_t bit) {
        if (check->isChecked())
            trace |= bit;
        else
            trace &= ~bit;
    };
    apply(m_ciCheck, CFG_TRACE_CI);
    apply(m_cmCheck, CFG_TRACE_CM);
    apply(m_dvrCheck, CFG_TRACE_DRIVER);
    apply(m_errCheck, CFG_TRACE_ERROR);
    apply(m_idCheck, CFG_TRACE_ID);
    apply(m_runCheck, CFG_TRACE_RUN);
    apply(m_irqCheck, CFG_TRACE_IRQ);
    apply(m_routeCheck, CFG_TRACE_ROUTE);
    apply(m_uartCheck, CFG_TRACE_UART);

    m_cpTrace = trace;
    m_traceEdit->setText(hex32(trace));
}

void CpPane::controlToCheck()
{
    const uint32_t trace = m_cpTrace;

    const QSignalBlocker b1(m_ciCheck), b2(m_cmCheck), b3(m_dvrCheck), b4(m_errCheck), b5(m_idCheck),
        b6(m_runCheck), b7(m_irqCheck), b8(m_routeCheck), b9(m_uartCheck);

    m_ciCheck->setChecked(trace & CFG_TRACE_CI);
    m_cmCheck->setChecked(trace & CFG_TRACE_CM);
    m_dvrCheck->setChecked(trace & CFG_TRACE_DRIVER);
    m_errCheck->setChecked(trace & CFG_TRACE_ERROR);
    m_idCheck->setChecked(trace & CFG_TRACE_ID);
    m_runCheck->setChecked(trace & CFG_TRACE_RUN);
    m_irqCheck->setChecked(trace & CFG_TRACE_IRQ);
    m_routeCheck->setChecked(trace & CFG_TRACE_ROUTE);
    m_uartCheck->setChecked(trace & CFG_TRACE_UART);
}

void CpPane::setValueField(uint32_t type, double value)
{
    switch (type) {
    case CFG_INT8U:
        m_valueEdit->setText(QStringLiteral("0x%1").arg(static_cast<quint8>(value), 2, 16, QLatin1Char('0')).toUpper());
        break;
    case CFG_INT8S:
        m_valueEdit->setText(QString::number(static_cast<qint8>(value)));
        break;
    case CFG_INT16U:
        m_valueEdit->setText(QStringLiteral("0x%1").arg(static_cast<quint16>(value), 4, 16, QLatin1Char('0')).toUpper());
        break;
    case CFG_INT16S:
        m_valueEdit->setText(QString::number(static_cast<qint16>(value)));
        break;
    case CFG_INT32U:
        m_valueEdit->setText(hex32(static_cast<quint32>(value)));
        break;
    case CFG_INT32S:
        m_valueEdit->setText(QString::number(static_cast<qint32>(value)));
        break;
    case CFG_INT64U:
        m_valueEdit->setText(QStringLiteral("0x%1").arg(static_cast<quint64>(value), 16, 16, QLatin1Char('0')).toUpper());
        break;
    case CFG_INT64S:
        m_valueEdit->setText(QString::number(static_cast<qint64>(value)));
        break;
    case CFG_FP32:
    case CFG_FP64:
        m_valueEdit->setText(QString::number(value, 'e', 6));
        break;
    default:
        m_valueEdit->setText(hex32(static_cast<quint32>(value)));
        break;
    }
}

// cm_register() callback - fires on whatever thread the CM layer's I/O
// lives on. Pass only the pointer through; Qt queues the emit onto this
// object's (GUI) thread automatically since the calling thread differs
// from the receiver's. cm_free() happens in onCmMessage(), once the
// message is actually processed - same lifetime the MFC ring buffer gave
// it (owned until the handler frees it, not time-boxed).
uint32_t CpPane::cpMsgThunk(pcm_msg_t msg, void * /*member*/)
{
    if (g_instance) {
        QMetaObject::invokeMethod(g_instance, &CpPane::onCmMessage, Qt::QueuedConnection,
                                   reinterpret_cast<quintptr>(msg));
    }
    return CP_OK;
}

void CpPane::onCmMessage(quintptr msgPtr)
{
    auto *msg = reinterpret_cast<pcm_msg_t>(msgPtr);
    const uint16_t cmMsg = MSG(msg->p.srvid, msg->p.msgid);

    if (msg->p.status != CM_OK) {
        log(tr("Error : CM Message status %1, srvid %2, msgid %3")
                .arg(msg->p.status, 2, 16, QLatin1Char('0'))
                .arg(msg->p.srvid, 2, 16, QLatin1Char('0'))
                .arg(msg->p.msgid, 2, 16, QLatin1Char('0')),
            QColor(255, 0, 0));
    } else if (cmMsg == MSG(CM_ID_CP_SRV, CP_TRACE_RESP)) {
        handleTraceResponse(reinterpret_cast<pcp_trace_msg_t>(msg));
    } else if (cmMsg == MSG(CM_ID_CP_SRV, CP_MEM_RESP)) {
        handleMemResponse(reinterpret_cast<pcp_mem_msg_t>(msg));
    } else if (cmMsg == MSG(CM_ID_CP_SRV, CP_PING_RESP)) {
        m_pingTimeoutTimer->start(30000);
    } else if (cmMsg == MSG(CM_ID_CP_SRV, CP_VER_RESP)) {
        handleVersionResponse(reinterpret_cast<pcp_ver_msg_t>(msg));
    }

    cm_free(msg);
}

void CpPane::handleMemResponse(pcp_mem_msg_t mem)
{
    m_cpAddr = mem->b.address;
    m_addrEdit->setText(hex32(m_cpAddr));
    setValueField(m_cpType, static_cast<double>(mem->b.value));

    QSettings settings;
    settings.setValue(QStringLiteral("CpPane/addr"), m_cpAddr);
}

void CpPane::handleTraceResponse(pcp_trace_msg_t trace)
{
    m_cpTrace = trace->b.trace;
    m_traceEdit->setText(hex32(m_cpTrace));
    controlToCheck();

    QSettings settings;
    settings.setValue(QStringLiteral("CpPane/trace"), m_cpTrace);
}

void CpPane::handleVersionResponse(pcp_ver_msg_t ver)
{
    // CpView.cpp's cp_msg(): start the keep-alive ping if the firmware
    // advertises CFG_FEATURE_PING.
    if (ver->b.feature & CFG_FEATURE_PING) {
        m_pingTimeoutTimer->start(30000);
        m_pingTimer->start(10000);
    }

    emit deviceVersionReceived(ver->b.fw_ver, ver->b.sysid, ver->b.stamp_epoch, ver->b.vhdl[0],
                                ver->b.vhdl[1]);
}

void CpPane::onPingTimer()
{
    pcmq_t slot = cm_alloc();
    if (slot == nullptr)
        return;

    auto *msg = reinterpret_cast<pcp_ping_msg_t>(slot->buf);
    msg->p.srvid = CM_ID_CP_SRV;
    msg->p.msgid = CP_PING_REQ;
    msg->p.flags = CP_NO_FLAGS;
    msg->p.status = CP_OK;

    cm_send_t ps = {};
    ps.msg = reinterpret_cast<pcm_msg_t>(msg);
    ps.dst_cmid = CM_ID_CP_SRV;
    ps.src_cmid = CM_ID_CP_CLI;
    ps.msglen = sizeof(cp_ping_msg_t);
    cm_send(CM_MSG_REQ, &ps);
}

void CpPane::onPingTimeout()
{
    // CpView.cpp's OnTimer(ID_TIMER_PING_TIMEOUT) also tells ConView to
    // disconnect via a WM_DISCONNECT message - but ConView's message map
    // never handles WM_DISCONNECT, so that call was already a no-op in the
    // MFC app. Only the warning dialog has a real effect; replicated here.
    QMessageBox::warning(this, tr("jack"), tr("Communications Timeout, Port Disconnected"));
}

void CpPane::log(const QString &text, const QColor &color)
{
    emit logMessage(text, color);
}
