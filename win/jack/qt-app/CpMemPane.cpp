#include "CpMemPane.h"

#include <cmath>
#include <cstdlib>

#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>

#include "appCfg.h"
#include "fw_cfg.h"

namespace {
CpMemPane *g_instance = nullptr; // for the static cm_register thunk, see cpmMsgThunk()

constexpr int kBufferSize = APP_MAX_CP_MEM_SIZE; // CpMemView.h's CP_MAX_BUFFER
constexpr int kMaxInt8u = kBufferSize;
constexpr int kMaxInt16u = kBufferSize / 2;
constexpr int kMaxInt32u = kBufferSize / 4;

// CpMemView.cpp: static DOUBLE Mean[]/STDEV[], indexed by CFG_INT8U..CFG_FP32.
constexpr double kMean[] = {128.0, 0.0, 32768.0, 0.0, 2147483648.0, 0.0, 0.0};
constexpr double kStdev[] = {12.8, 12.8, 3276.8, 3276.8, 214748364.8, 214748364.8, 12.8};

// Ported verbatim from app/boxmuller.cpp (public domain, see that file's
// header) so Test patterns are bit-for-bit reproducible with the MFC app
// for the same pass number.
double g_n2 = 0.0;
bool g_n2Cached = false;

double randNormal(double mean, double stddev)
{
    if (!g_n2Cached) {
        double x, y, r;
        do {
            x = 2.0 * rand() / RAND_MAX - 1;
            y = 2.0 * rand() / RAND_MAX - 1;
            r = x * x + y * y;
        } while (r == 0.0 || r > 1.0);
        const double d = std::sqrt(-2.0 * std::log(r) / r);
        const double n1 = x * d;
        g_n2 = y * d;
        g_n2Cached = true;
        return n1 * stddev + mean;
    }
    g_n2Cached = false;
    return g_n2 * stddev + mean;
}

void randSeed(unsigned int seed)
{
    g_n2 = 0.0;
    g_n2Cached = false;
    srand(seed);
}
}

CpMemPane::CpMemPane(QWidget *parent)
    : QWidget(parent)
    , m_buf(kBufferSize, 0)
    , m_getting(false)
    , m_setting(false)
    , m_testing(false)
    , m_memIndex(0)
    , m_numRequest(0)
    , m_passNumber(0)
    , m_lenDel(0xFFFFFFFFu)
{
    g_instance = this;

    m_fileEdit = new QLineEdit(this);
    m_browseButton = new QPushButton(tr("..."), this);
    m_browseButton->setMaximumWidth(28);
    m_readButton = new QPushButton(tr("Read"), this);
    m_writeButton = new QPushButton(tr("Write"), this);
    m_binCheck = new QCheckBox(tr("Binary"), this);
    m_verifyCheck = new QCheckBox(tr("Verify"), this);
    m_allCheck = new QCheckBox(tr("All"), this);
    m_awgnCheck = new QCheckBox(tr("AWGN"), this);
    m_cycleCheck = new QCheckBox(tr("Cycle"), this);
    m_hexCheck = new QCheckBox(tr("Hex"), this);
    m_showCheck = new QCheckBox(tr("Show"), this);
    m_toFileCheck = new QCheckBox(tr("File"), this);
    m_getButton = new QPushButton(tr("Get"), this);
    m_setButton = new QPushButton(tr("Set"), this);
    m_clearButton = new QPushButton(tr("Clear"), this);
    m_testButton = new QPushButton(tr("Test"), this);
    m_startEdit = new QLineEdit(this);
    m_lengthEdit = new QLineEdit(this);
    m_xferEdit = new QLineEdit(this);
    m_xferEdit->setReadOnly(true);
    m_typeCombo = new QComboBox(this);
    // app.rc IDD_CP_MEM DLGINIT order for IDC_CP_MEM_TYPE, matching
    // CFG_INT8U..CFG_FP32 in fw_cfg.h (no 64-bit entries here, unlike
    // CpPane's Data Type combo).
    m_typeCombo->addItems({tr("INT8U"), tr("INT8S"), tr("INT16U"), tr("INT16S"), tr("INT32U"),
                           tr("INT32S"), tr("FP32")});

    auto *fileRow = new QHBoxLayout();
    fileRow->addWidget(m_fileEdit);
    fileRow->addWidget(m_browseButton);

    auto *fileButtonRow = new QHBoxLayout();
    fileButtonRow->addWidget(m_readButton);
    fileButtonRow->addWidget(m_binCheck);
    fileButtonRow->addWidget(m_toFileCheck);
    fileButtonRow->addWidget(m_allCheck);
    fileButtonRow->addStretch();

    auto *fileButtonRow2 = new QHBoxLayout();
    fileButtonRow2->addWidget(m_writeButton);
    fileButtonRow2->addWidget(m_awgnCheck);
    fileButtonRow2->addWidget(m_cycleCheck);
    fileButtonRow2->addWidget(m_hexCheck);
    fileButtonRow2->addStretch();

    auto *showRow = new QHBoxLayout();
    showRow->addWidget(m_showCheck);
    showRow->addWidget(m_verifyCheck);
    showRow->addStretch();

    auto *getSetRow = new QHBoxLayout();
    getSetRow->addWidget(m_getButton);
    getSetRow->addWidget(m_setButton);
    getSetRow->addWidget(m_clearButton);
    getSetRow->addWidget(m_testButton);

    auto *form = new QFormLayout();
    form->addRow(tr("Start"), m_startEdit);
    form->addRow(tr("Length"), m_lengthEdit);
    form->addRow(tr("Transfer"), m_xferEdit);
    form->addRow(tr("Type"), m_typeCombo);

    auto *controlGroup = new QGroupBox(tr("Memory Block Read/Write"), this);
    auto *controlLayout = new QVBoxLayout(controlGroup);
    controlLayout->addLayout(fileRow);
    controlLayout->addLayout(fileButtonRow);
    controlLayout->addLayout(fileButtonRow2);
    controlLayout->addLayout(showRow);
    controlLayout->addLayout(getSetRow);
    controlLayout->addLayout(form);
    controlLayout->addStretch();

    m_dumpEdit = new QPlainTextEdit(this);
    m_dumpEdit->setReadOnly(true);
    m_dumpEdit->setMaximumBlockCount(8192);

    auto *layout = new QHBoxLayout(this);
    layout->addWidget(controlGroup);
    layout->addWidget(m_dumpEdit);

    m_timeoutTimer = new QTimer(this);
    m_timeoutTimer->setSingleShot(true);

    connect(m_browseButton, &QPushButton::clicked, this, &CpMemPane::onBrowseClicked);
    connect(m_readButton, &QPushButton::clicked, this, &CpMemPane::onReadClicked);
    connect(m_writeButton, &QPushButton::clicked, this, &CpMemPane::onWriteClicked);
    connect(m_getButton, &QPushButton::clicked, this, &CpMemPane::onGetClicked);
    connect(m_setButton, &QPushButton::clicked, this, &CpMemPane::onSetClicked);
    connect(m_clearButton, &QPushButton::clicked, this, &CpMemPane::onClearClicked);
    connect(m_testButton, &QPushButton::clicked, this, &CpMemPane::onTestClicked);
    connect(m_typeCombo, &QComboBox::currentIndexChanged, this, &CpMemPane::onTypeOrShowChanged);
    connect(m_hexCheck, &QCheckBox::toggled, this, &CpMemPane::onTypeOrShowChanged);
    connect(m_allCheck, &QCheckBox::toggled, this, &CpMemPane::onTypeOrShowChanged);
    connect(m_timeoutTimer, &QTimer::timeout, this, &CpMemPane::onTimeout);

    loadSettings();
    updateTextDump();

    // CpMemView::OnInitialUpdate(): cpm_init() - register as the CM "CP
    // Memory" client.
    cm_register(CM_ID_CP_MEM_CLI, &CpMemPane::cpmMsgThunk, this, nullptr);
}

CpMemPane::~CpMemPane()
{
    if (g_instance == this)
        g_instance = nullptr;
}

void CpMemPane::loadSettings()
{
    QSettings settings;
    m_fileEdit->setText(settings.value(QStringLiteral("CpMemPane/file")).toString());
    m_startEdit->setText(settings.value(QStringLiteral("CpMemPane/start"), QStringLiteral("0x00000000")).toString());
    m_lengthEdit->setText(settings.value(QStringLiteral("CpMemPane/length"), QStringLiteral("0x00000000")).toString());
    m_typeCombo->setCurrentIndex(settings.value(QStringLiteral("CpMemPane/type"), CFG_INT8U).toInt());
    m_binCheck->setChecked(settings.value(QStringLiteral("CpMemPane/bin"), false).toBool());
    m_hexCheck->setChecked(settings.value(QStringLiteral("CpMemPane/hex"), false).toBool());
    m_allCheck->setChecked(settings.value(QStringLiteral("CpMemPane/all"), false).toBool());
    m_cycleCheck->setChecked(settings.value(QStringLiteral("CpMemPane/cycle"), false).toBool());
    m_awgnCheck->setChecked(settings.value(QStringLiteral("CpMemPane/awgn"), false).toBool());
    m_verifyCheck->setChecked(settings.value(QStringLiteral("CpMemPane/verify"), false).toBool());
    m_showCheck->setChecked(settings.value(QStringLiteral("CpMemPane/show"), false).toBool());
    m_toFileCheck->setChecked(settings.value(QStringLiteral("CpMemPane/toFile"), false).toBool());
}

void CpMemPane::saveSettings()
{
    QSettings settings;
    settings.setValue(QStringLiteral("CpMemPane/file"), m_fileEdit->text());
    settings.setValue(QStringLiteral("CpMemPane/start"), m_startEdit->text());
    settings.setValue(QStringLiteral("CpMemPane/length"), m_lengthEdit->text());
    settings.setValue(QStringLiteral("CpMemPane/type"), m_typeCombo->currentIndex());
    settings.setValue(QStringLiteral("CpMemPane/bin"), m_binCheck->isChecked());
    settings.setValue(QStringLiteral("CpMemPane/hex"), m_hexCheck->isChecked());
    settings.setValue(QStringLiteral("CpMemPane/all"), m_allCheck->isChecked());
    settings.setValue(QStringLiteral("CpMemPane/cycle"), m_cycleCheck->isChecked());
    settings.setValue(QStringLiteral("CpMemPane/awgn"), m_awgnCheck->isChecked());
    settings.setValue(QStringLiteral("CpMemPane/verify"), m_verifyCheck->isChecked());
    settings.setValue(QStringLiteral("CpMemPane/show"), m_showCheck->isChecked());
    settings.setValue(QStringLiteral("CpMemPane/toFile"), m_toFileCheck->isChecked());
}

CpMemPane::ChunkInfo CpMemPane::chunkInfo() const
{
    switch (m_typeCombo->currentIndex()) {
    case CFG_INT16U:
    case CFG_INT16S:
        return {CFG_INT16U, 128};
    case CFG_INT32U:
    case CFG_INT32S:
    case CFG_FP32:
        return {CFG_INT32U, 64};
    default:
        return {CFG_INT8U, 256};
    }
}

void CpMemPane::computeRequestCounts(uint32_t memLen, uint32_t &numCollect, uint32_t &numRequest) const
{
    switch (m_typeCombo->currentIndex()) {
    case CFG_INT16U:
    case CFG_INT16S:
        numCollect = memLen / 2;
        numRequest = (numCollect + 127) / 128;
        if (numRequest * 128 > static_cast<uint32_t>(kMaxInt16u))
            numRequest = kMaxInt16u / 128;
        break;
    case CFG_INT32U:
    case CFG_INT32S:
    case CFG_FP32:
        numCollect = memLen / 4;
        numRequest = (numCollect + 63) / 64;
        if (numRequest * 64 > static_cast<uint32_t>(kMaxInt32u))
            numRequest = kMaxInt32u / 64;
        break;
    default:
        numCollect = memLen;
        numRequest = (numCollect + 255) / 256;
        if (numRequest * 256 > static_cast<uint32_t>(kMaxInt8u))
            numRequest = kMaxInt8u / 256;
        break;
    }
}

void CpMemPane::sendBlockRequest(uint32_t index, bool write)
{
    pcmq_t slot = cm_alloc();
    if (slot == nullptr)
        return;

    const ChunkInfo chunk = chunkInfo();
    const uint32_t memStart = m_startEdit->text().toUInt(nullptr, 16);
    const uint32_t byteOffset = index * 256;

    auto *msg = reinterpret_cast<pcp_block_msg_t>(slot->buf);
    msg->b.type = chunk.cmType;
    msg->b.length = chunk.count;
    msg->b.index = index;
    msg->b.address = memStart + byteOffset;

    if (write) {
        // Byte-for-byte copy of this chunk's 256 bytes - the wire struct
        // only cares about layout, so this replaces CpMemView.cpp's
        // type-punned pointer-cast copy loops for the same effect.
        memcpy(msg->b.data, m_buf.constData() + byteOffset, 256);
    }

    cm_send_t ps = {};
    msg->p.srvid = CM_ID_CP_SRV;
    msg->p.msgid = CP_BLOCK_REQ;
    msg->p.flags = write ? CP_BLOCK_WR : CP_BLOCK_RD;
    msg->p.status = CP_OK;
    ps.msg = reinterpret_cast<pcm_msg_t>(msg);
    ps.dst_cmid = CM_ID_CP_SRV;
    ps.src_cmid = CM_ID_CP_MEM_CLI;
    ps.msglen = write ? sizeof(cp_block_msg_t) : sizeof(cp_block_msg_t) - CP_BLOCK_MAX;
    cm_send(CM_MSG_REQ, &ps);

    m_timeoutTimer->start(1000);
}

void CpMemPane::onGetClicked()
{
    if (m_getting) {
        stopAll(tr("Get"), m_getButton);
        return;
    }

    const uint32_t memLen = m_lengthEdit->text().toUInt(nullptr, 16);
    if (memLen == 0) {
        log(tr("Error : Block Read Length is Zero"), QColor(255, 0, 0));
        return;
    }

    m_xferEdit->clear();
    m_getting = true;
    if (!m_testing)
        m_getButton->setText(tr("Abort"));

    uint32_t numCollect = 0;
    computeRequestCounts(memLen, numCollect, m_numRequest);
    m_memIndex = 0;
    std::fill(m_buf.begin(), m_buf.end(), 0);

    sendBlockRequest(0, false);
}

void CpMemPane::onSetClicked()
{
    if (m_setting) {
        stopAll(tr("Set"), m_setButton);
        return;
    }

    const uint32_t memLen = m_lengthEdit->text().toUInt(nullptr, 16);
    if (memLen == 0) {
        log(tr("Error : Block Write Length is Zero"), QColor(255, 0, 0));
        return;
    }

    m_setting = true;
    m_setButton->setText(tr("Abort"));

    uint32_t numCollect = 0;
    computeRequestCounts(memLen, numCollect, m_numRequest);
    m_memIndex = 0;

    sendBlockRequest(0, true);
}

void CpMemPane::stopAll(const QString &idleButtonText, QPushButton *button)
{
    m_getting = false;
    m_setting = false;
    button->setText(idleButtonText);
    m_timeoutTimer->stop();
}

void CpMemPane::onTimeout()
{
    // CpMemView::OnTimer(ID_TIMER_MSG_TIMEOUT): the MFC app's retry logic
    // here is entirely commented out - it only logs a warning and gives
    // up, so that's what's ported, not the never-executed retry.
    log(tr("Warning : Response Timeout"), QColor(111, 52, 196));
}

uint32_t CpMemPane::cpmMsgThunk(pcm_msg_t msg, void * /*member*/)
{
    if (g_instance) {
        QMetaObject::invokeMethod(g_instance, &CpMemPane::onCmMessage, Qt::QueuedConnection,
                                   reinterpret_cast<quintptr>(msg));
    }
    return CP_OK;
}

void CpMemPane::onCmMessage(quintptr msgPtr)
{
    auto *msg = reinterpret_cast<pcm_msg_t>(msgPtr);

    if (msg->p.status != CM_OK) {
        log(tr("Error : CM Message status %1, srvid %2, msgid %3")
                .arg(msg->p.status, 2, 16, QLatin1Char('0'))
                .arg(msg->p.srvid, 2, 16, QLatin1Char('0'))
                .arg(msg->p.msgid, 2, 16, QLatin1Char('0')),
            QColor(255, 0, 0));
    } else if (MSG(msg->p.srvid, msg->p.msgid) == MSG(CM_ID_CP_SRV, CP_BLOCK_RESP)) {
        m_timeoutTimer->stop();
        auto *resp = reinterpret_cast<pcp_block_msg_t>(msg);
        if (msg->p.flags & CP_BLOCK_RD)
            handleBlockReadResp(resp);
        else if (msg->p.flags & CP_BLOCK_WR)
            handleBlockWriteResp(resp);
    }

    cm_free(msg);
}

void CpMemPane::handleBlockReadResp(pcp_block_msg_t resp)
{
    const uint32_t byteOffset = resp->b.index * 256;
    if (byteOffset + 256 <= static_cast<uint32_t>(kBufferSize))
        memcpy(m_buf.data() + byteOffset, resp->b.data, 256);

    m_memIndex++;

    if (m_memIndex != m_numRequest && m_getting) {
        sendBlockRequest(m_memIndex, false);
    } else {
        m_getting = false;
        m_getButton->setText(tr("Get"));
        if (m_allCheck->isChecked())
            updateTextDump();
        if (m_testing)
            validateTestPattern();
        plotRun();
    }

    m_xferEdit->setText(QStringLiteral("0x%1").arg(m_memIndex * 256, 8, 16, QLatin1Char('0')).toUpper());

    if (!m_allCheck->isChecked() && m_memIndex == 1)
        updateTextDump();
}

void CpMemPane::handleBlockWriteResp(pcp_block_msg_t /*resp*/)
{
    m_memIndex++;

    if (m_memIndex != m_numRequest && m_setting) {
        sendBlockRequest(m_memIndex, true);
    } else {
        m_setting = false;
        m_setButton->setText(tr("Set"));
        if (m_testing)
            onGetClicked();
    }

    m_xferEdit->setText(QStringLiteral("0x%1").arg(m_memIndex * 256, 8, 16, QLatin1Char('0')).toUpper());
}

void CpMemPane::onClearClicked()
{
    std::fill(m_buf.begin(), m_buf.end(), 0);
    updateTextDump();
    m_passNumber = 0;
}

void CpMemPane::onTestClicked()
{
    if (m_testing) {
        m_testing = false;
        m_setting = false;
        m_getting = false;
        m_testButton->setText(tr("Test"));
        m_timeoutTimer->stop();
        return;
    }

    const uint32_t memLen = m_lengthEdit->text().toUInt(nullptr, 16);
    if (memLen == 0) {
        log(tr("Error : Block Write Length is Zero"), QColor(255, 0, 0));
        return;
    }

    m_testing = true;
    m_setting = true;
    m_testButton->setText(tr("Abort"));

    std::fill(m_buf.begin(), m_buf.end(), 0);
    m_passNumber++;
    randSeed(m_passNumber);
    // CpMemView::OnBnClickedCpMemTest(): posts the pass number to
    // MF_SBAR_COUNT, the same status bar pane DAQPane's packetCountChanged
    // updates - matches which pane is actually active, like the MFC app.
    emit testPassChanged(m_passNumber);

    uint32_t numCollect = 0;
    computeRequestCounts(memLen, numCollect, m_numRequest);
    m_memIndex = 0;

    generateTestPattern();
    updateTextDump();
    sendBlockRequest(0, true);
}

void CpMemPane::generateTestPattern()
{
    const uint32_t memLen = m_lengthEdit->text().toUInt(nullptr, 16);
    const int type = m_typeCombo->currentIndex();

    uint32_t numCollect = 0, dummy = 0;
    computeRequestCounts(memLen, numCollect, dummy);

    if (m_awgnCheck->isChecked()) {
        const double mean = kMean[type];
        const double stdev = kStdev[type];
        for (uint32_t i = 0; i < numCollect; ++i) {
            const double val = randNormal(mean, stdev);
            switch (type) {
            case CFG_INT8U:
                if (i < static_cast<uint32_t>(kMaxInt8u))
                    m_buf[i] = static_cast<quint8>(val);
                break;
            case CFG_INT8S:
                if (i < static_cast<uint32_t>(kMaxInt8u))
                    reinterpret_cast<qint8 *>(m_buf.data())[i] = static_cast<qint8>(val);
                break;
            case CFG_INT16U:
                if (i < static_cast<uint32_t>(kMaxInt16u))
                    reinterpret_cast<quint16 *>(m_buf.data())[i] = static_cast<quint16>(val);
                break;
            case CFG_INT16S:
                if (i < static_cast<uint32_t>(kMaxInt16u))
                    reinterpret_cast<qint16 *>(m_buf.data())[i] = static_cast<qint16>(val);
                break;
            case CFG_INT32U:
                if (i < static_cast<uint32_t>(kMaxInt32u))
                    reinterpret_cast<quint32 *>(m_buf.data())[i] = static_cast<quint32>(val);
                break;
            case CFG_INT32S:
                if (i < static_cast<uint32_t>(kMaxInt32u))
                    reinterpret_cast<qint32 *>(m_buf.data())[i] = static_cast<qint32>(val);
                break;
            case CFG_FP32:
                if (i < static_cast<uint32_t>(kMaxInt32u))
                    reinterpret_cast<float *>(m_buf.data())[i] = static_cast<float>(val);
                break;
            }
        }
    } else {
        for (uint32_t i = 0; i < memLen / 2 && i < static_cast<uint32_t>(kMaxInt16u); ++i) {
            reinterpret_cast<quint16 *>(m_buf.data())[i] =
                static_cast<quint16>((static_cast<float>(rand()) / RAND_MAX) * 65535.0f);
        }
    }
}

void CpMemPane::validateTestPattern()
{
    const uint32_t memLen = m_lengthEdit->text().toUInt(nullptr, 16);
    const int type = m_typeCombo->currentIndex();
    const uint32_t memStart = m_startEdit->text().toUInt(nullptr, 16);

    // Regenerate the identical seeded sequence and compare, rather than
    // diffing against a saved copy - matches CpMemView::OnMemValidate().
    srand(m_passNumber);
    randSeed(m_passNumber);

    m_testing = false;
    bool passed = true;

    uint32_t numCollect = 0, dummy = 0;
    computeRequestCounts(memLen, numCollect, dummy);

    if (m_verifyCheck->isChecked()) {
        if (m_awgnCheck->isChecked()) {
            const double mean = kMean[type];
            const double stdev = kStdev[type];
            for (uint32_t i = 0; i < numCollect && passed; ++i) {
                const double val = randNormal(mean, stdev);
                bool mismatch = false;
                quint32 got = 0, wanted = 0;
                switch (type) {
                case CFG_INT8U:
                    got = m_buf[i];
                    wanted = static_cast<quint8>(val);
                    mismatch = (got != wanted);
                    break;
                case CFG_INT8S:
                    got = static_cast<quint8>(reinterpret_cast<const qint8 *>(m_buf.constData())[i]);
                    wanted = static_cast<quint8>(static_cast<qint8>(val));
                    mismatch = (got != wanted);
                    break;
                case CFG_INT16U:
                    got = reinterpret_cast<const quint16 *>(m_buf.constData())[i];
                    wanted = static_cast<quint16>(val);
                    mismatch = (got != wanted);
                    break;
                case CFG_INT16S:
                    got = static_cast<quint16>(reinterpret_cast<const qint16 *>(m_buf.constData())[i]);
                    wanted = static_cast<quint16>(static_cast<qint16>(val));
                    mismatch = (got != wanted);
                    break;
                case CFG_INT32U:
                    got = reinterpret_cast<const quint32 *>(m_buf.constData())[i];
                    wanted = static_cast<quint32>(val);
                    mismatch = (got != wanted);
                    break;
                case CFG_INT32S:
                    got = static_cast<quint32>(reinterpret_cast<const qint32 *>(m_buf.constData())[i]);
                    wanted = static_cast<quint32>(static_cast<qint32>(val));
                    mismatch = (got != wanted);
                    break;
                case CFG_FP32:
                    mismatch = reinterpret_cast<const float *>(m_buf.constData())[i] != static_cast<float>(val);
                    break;
                }
                if (mismatch) {
                    log(tr("Memory Test Error: Pass = %1, Location = 0x%2, read = 0x%3, wrote = 0x%4")
                            .arg(m_passNumber)
                            .arg(memStart + i, 8, 16, QLatin1Char('0'))
                            .arg(got, 8, 16, QLatin1Char('0'))
                            .arg(wanted, 8, 16, QLatin1Char('0')),
                        QColor(255, 0, 0));
                    passed = false;
                }
            }
        } else {
            for (uint32_t i = 0; i < memLen / 2 && passed; ++i) {
                const quint16 wanted = static_cast<quint16>((static_cast<float>(rand()) / RAND_MAX) * 65535.0f);
                const quint16 got = reinterpret_cast<const quint16 *>(m_buf.constData())[i];
                if (got != wanted) {
                    log(tr("Memory Test Error: Pass = %1, Location = 0x%2, read = 0x%3, wrote = 0x%4")
                            .arg(m_passNumber)
                            .arg(memStart + (i << 1), 8, 16, QLatin1Char('0'))
                            .arg(got, 4, 16, QLatin1Char('0'))
                            .arg(wanted, 4, 16, QLatin1Char('0')),
                        QColor(255, 0, 0));
                    passed = false;
                }
            }
        }
    }

    if (m_cycleCheck->isChecked() && passed)
        onTestClicked();
    else {
        m_testing = false;
        m_setting = false;
        m_getting = false;
        m_testButton->setText(tr("Test"));
    }
}

void CpMemPane::onBrowseClicked()
{
    // The explicit "..." button has no equivalent control ID in the
    // original dialog (CpMemView.cpp only pops a dialog from Read/Write
    // when the path is empty) - it's a convenience add, so pick-or-create
    // is the right default here regardless of file mode.
    const QString filter = m_binCheck->isChecked() ? tr("Memory Files (*.bin);;All Files (*.*)")
                                                     : tr("Memory Files (*.txt);;All Files (*.*)");
    const QString file = QFileDialog::getSaveFileName(this, tr("Memory File"), m_fileEdit->text(), filter,
                                                         nullptr, QFileDialog::DontConfirmOverwrite);
    if (!file.isEmpty()) {
        m_fileEdit->setText(file);
        saveSettings();
    }
}

void CpMemPane::onReadClicked()
{
    // CpMemView::OnBnClickedCpMemRead(): an open dialog (file must exist)
    // when the path field is empty.
    if (m_fileEdit->text().isEmpty()) {
        const QString filter = m_binCheck->isChecked() ? tr("Memory Files (*.bin);;All Files (*.*)")
                                                         : tr("Memory Files (*.txt);;All Files (*.*)");
        const QString file = QFileDialog::getOpenFileName(this, tr("Memory File"), QString(), filter);
        if (file.isEmpty())
            return;
        m_fileEdit->setText(file);
        saveSettings();
    }
    readMemData();
}

void CpMemPane::onWriteClicked()
{
    // CpMemView::OnBnClickedCpMemWrite(): a save dialog with an overwrite
    // prompt when the path field is empty.
    if (m_fileEdit->text().isEmpty()) {
        const QString filter = m_binCheck->isChecked() ? tr("Memory Files (*.bin);;All Files (*.*)")
                                                         : tr("Memory Files (*.txt);;All Files (*.*)");
        const QString file = QFileDialog::getSaveFileName(this, tr("Memory File"), QString(), filter);
        if (file.isEmpty())
            return;
        m_fileEdit->setText(file);
        saveSettings();
    }
    writeMemData();
}

void CpMemPane::readMemData()
{
    const uint32_t memLen = m_lengthEdit->text().toUInt(nullptr, 16);
    if (memLen > static_cast<uint32_t>(kBufferSize)) {
        log(tr("Error : Memory Length Exceeds Max Buffer, 0x%1 > 0x%2")
                .arg(memLen, 8, 16, QLatin1Char('0'))
                .arg(kBufferSize, 8, 16, QLatin1Char('0')),
            QColor(255, 0, 0));
        return;
    }

    QFile file(m_fileEdit->text());
    const QIODevice::OpenMode mode = m_binCheck->isChecked() ? QIODevice::ReadOnly
                                                              : (QIODevice::ReadOnly | QIODevice::Text);
    if (!file.open(mode)) {
        log(tr("Error : Unable to Open file %1").arg(m_fileEdit->text()), QColor(255, 0, 0));
        return;
    }

    std::fill(m_buf.begin(), m_buf.end(), 0);

    if (m_binCheck->isChecked()) {
        const QByteArray data = file.read(memLen);
        memcpy(m_buf.data(), data.constData(), std::min<int>(data.size(), kBufferSize));
    } else {
        QTextStream in(&file);
        const int type = m_typeCombo->currentIndex();
        const int base = m_hexCheck->isChecked() ? 16 : 10;
        uint32_t k = 0;
        const uint32_t limit = (type == CFG_INT8U || type == CFG_INT8S) ? kMaxInt8u
                                : (type == CFG_INT16U || type == CFG_INT16S) ? kMaxInt16u
                                                                              : kMaxInt32u;
        while (!in.atEnd() && k < limit) {
            QString line = in.readLine().trimmed();
            if (line.isEmpty() || line.startsWith(QLatin1Char('[')))
                continue;
            bool ok = false;
            const qint64 value = line.toLongLong(&ok, base);
            if (!ok)
                continue;
            switch (type) {
            case CFG_INT8U:
            case CFG_INT8S:
                m_buf[k++] = static_cast<quint8>(value);
                break;
            case CFG_INT16U:
            case CFG_INT16S:
                reinterpret_cast<quint16 *>(m_buf.data())[k++] = static_cast<quint16>(value);
                break;
            default:
                reinterpret_cast<quint32 *>(m_buf.data())[k++] = static_cast<quint32>(value);
                break;
            }
        }
    }

    updateTextDump();
}

void CpMemPane::writeMemData()
{
    const uint32_t memLen = m_lengthEdit->text().toUInt(nullptr, 16);
    if (memLen > static_cast<uint32_t>(kBufferSize)) {
        log(tr("Error : Memory Length Exceeds Max Buffer, 0x%1 > 0x%2")
                .arg(memLen, 8, 16, QLatin1Char('0'))
                .arg(kBufferSize, 8, 16, QLatin1Char('0')),
            QColor(255, 0, 0));
        return;
    }

    QFile file(m_fileEdit->text());
    const QIODevice::OpenMode mode = m_binCheck->isChecked() ? QIODevice::WriteOnly
                                                              : (QIODevice::WriteOnly | QIODevice::Text);
    if (!file.open(mode)) {
        log(tr("Error : Unable to Open file %1").arg(m_fileEdit->text()), QColor(255, 0, 0));
        return;
    }

    if (m_binCheck->isChecked()) {
        file.write(reinterpret_cast<const char *>(m_buf.constData()), memLen);
        return;
    }

    QTextStream out(&file);
    const int type = m_typeCombo->currentIndex();
    const bool hex = m_hexCheck->isChecked();

    switch (type) {
    case CFG_INT8U:
        for (uint32_t i = 0; i < memLen; ++i)
            out << (hex ? QStringLiteral(" 0x%1\n").arg(m_buf[i], 2, 16, QLatin1Char('0')).toUpper()
                        : QStringLiteral(" %1\n").arg(m_buf[i], 4));
        break;
    case CFG_INT8S:
        for (uint32_t i = 0; i < memLen; ++i) {
            const qint8 v = reinterpret_cast<const qint8 *>(m_buf.constData())[i];
            out << (hex ? QStringLiteral(" 0x%1\n").arg(static_cast<quint8>(v), 2, 16, QLatin1Char('0')).toUpper()
                        : QStringLiteral(" %1\n").arg(v, 4));
        }
        break;
    case CFG_INT16U:
        for (uint32_t i = 0; i < memLen / 2; ++i) {
            const quint16 v = reinterpret_cast<const quint16 *>(m_buf.constData())[i];
            out << (hex ? QStringLiteral(" 0x%1\n").arg(v, 4, 16, QLatin1Char('0')).toUpper()
                        : QStringLiteral(" %1\n").arg(v, 6));
        }
        break;
    case CFG_INT16S:
        for (uint32_t i = 0; i < memLen / 2; ++i) {
            const qint16 v = reinterpret_cast<const qint16 *>(m_buf.constData())[i];
            out << (hex ? QStringLiteral(" 0x%1\n").arg(static_cast<quint16>(v), 4, 16, QLatin1Char('0')).toUpper()
                        : QStringLiteral(" %1\n").arg(v, 6));
        }
        break;
    case CFG_INT32U:
        for (uint32_t i = 0; i < memLen / 4; ++i) {
            const quint32 v = reinterpret_cast<const quint32 *>(m_buf.constData())[i];
            out << (hex ? QStringLiteral(" 0x%1\n").arg(v, 8, 16, QLatin1Char('0')).toUpper()
                        : QStringLiteral(" %1\n").arg(v, 12));
        }
        break;
    case CFG_INT32S:
        for (uint32_t i = 0; i < memLen / 4; ++i) {
            const qint32 v = reinterpret_cast<const qint32 *>(m_buf.constData())[i];
            out << (hex ? QStringLiteral(" 0x%1\n").arg(static_cast<quint32>(v), 8, 16, QLatin1Char('0')).toUpper()
                        : QStringLiteral(" %1\n").arg(v, 12));
        }
        break;
    case CFG_FP32:
        for (uint32_t i = 0; i < memLen / 4; ++i)
            out << QStringLiteral(" %1\n").arg(reinterpret_cast<const float *>(m_buf.constData())[i], 12, 'f', 5);
        break;
    }
}

void CpMemPane::onTypeOrShowChanged()
{
    updateTextDump();
}

// Replacement for CpMemView::OnPlot(): builds the whole buffer as a single
// lane for GraphPane, same shape CpMemView.cpp sent to GraphWnd via
// PostMessage(WM_PLOT_MSG, ...).
void CpMemPane::plotRun()
{
    const uint32_t memLen = m_lengthEdit->text().toUInt(nullptr, 16);
    if (memLen > static_cast<uint32_t>(kBufferSize))
        return;

    const int type = m_typeCombo->currentIndex();
    uint32_t len = memLen;
    switch (type) {
    case CFG_INT16U:
    case CFG_INT16S:
        len = memLen / 2;
        break;
    case CFG_INT32U:
    case CFG_INT32S:
    case CFG_FP32:
        len = memLen / 4;
        break;
    default:
        break;
    }
    if (len == 0)
        return;

    PlotData data;
    data.plot = PlotData::CpuMem;
    data.type = type;
    data.lanes[0] = 1;
    data.lanes[1] = 0;
    data.length = static_cast<int>(len);
    // CpMemView.cpp uses m_nPktCnt here only to force a one-time graph
    // rebuild on the very first plot; GraphPane's own lanes/type check
    // already covers that (lanes start at {0,0}), so a fixed non-zero
    // value is equivalent without adding a counter this pane has no other
    // use for.
    data.record = 1;
    data.xMin = 0.0;
    data.xMax = static_cast<double>(len);

    data.dataX.resize(static_cast<int>(len));
    data.dataY.resize(static_cast<int>(len));

    for (uint32_t i = 0; i < len; ++i) {
        data.dataX[i] = static_cast<float>(i);
        switch (type) {
        case CFG_INT8U:
            data.dataY[i] = static_cast<float>(m_buf[i]);
            break;
        case CFG_INT8S:
            data.dataY[i] = static_cast<float>(reinterpret_cast<const qint8 *>(m_buf.constData())[i]);
            break;
        case CFG_INT16U:
            data.dataY[i] = static_cast<float>(reinterpret_cast<const quint16 *>(m_buf.constData())[i]);
            break;
        case CFG_INT16S:
            data.dataY[i] = static_cast<float>(reinterpret_cast<const qint16 *>(m_buf.constData())[i]);
            break;
        case CFG_INT32U:
            data.dataY[i] = static_cast<float>(reinterpret_cast<const quint32 *>(m_buf.constData())[i]);
            break;
        case CFG_INT32S:
            data.dataY[i] = static_cast<float>(reinterpret_cast<const qint32 *>(m_buf.constData())[i]);
            break;
        case CFG_FP32:
            data.dataY[i] = reinterpret_cast<const float *>(m_buf.constData())[i];
            break;
        }
    }

    emit plotReady(data);
}

void CpMemPane::updateTextDump()
{
    if (!m_showCheck->isChecked())
        return;

    uint32_t length = m_lengthEdit->text().toUInt(nullptr, 16);
    if (!m_allCheck->isChecked())
        length = 256;

    if (length != m_lenDel) {
        m_lenDel = length;
        m_dumpEdit->clear();
    }

    const int type = m_typeCombo->currentIndex();
    const bool hex = m_hexCheck->isChecked();
    uint32_t memCount = length;
    int cols = 8;
    int elemSize = 1;
    switch (type) {
    case CFG_INT16U:
    case CFG_INT16S:
        memCount = length / 2;
        elemSize = 2;
        break;
    case CFG_INT32U:
    case CFG_INT32S:
    case CFG_FP32:
        memCount = length / 4;
        cols = 4;
        elemSize = 4;
        break;
    default:
        break;
    }

    QString text;
    for (uint32_t i = 0; i < memCount; i += cols) {
        text += m_allCheck->isChecked() ? QStringLiteral(" %1 ").arg(i * elemSize, 8, 16, QLatin1Char('0')).toUpper()
                                         : QStringLiteral(" %1 ").arg(i * elemSize, 4, 16, QLatin1Char('0')).toUpper();
        for (int j = 0; j < cols && (i + static_cast<uint32_t>(j)) < memCount; ++j) {
            const uint32_t idx = i + j;
            switch (type) {
            case CFG_INT8U:
                text += hex ? QStringLiteral(" %1").arg(m_buf[idx], 2, 16, QLatin1Char('0')).toUpper()
                            : QStringLiteral(" %1").arg(m_buf[idx], 4);
                break;
            case CFG_INT8S:
                text += QStringLiteral(" %1").arg(reinterpret_cast<const qint8 *>(m_buf.constData())[idx], 4);
                break;
            case CFG_INT16U: {
                const quint16 v = reinterpret_cast<const quint16 *>(m_buf.constData())[idx];
                text += hex ? QStringLiteral(" %1").arg(v, 4, 16, QLatin1Char('0')).toUpper()
                            : QStringLiteral(" %1").arg(v, 6);
                break;
            }
            case CFG_INT16S:
                text += QStringLiteral(" %1").arg(reinterpret_cast<const qint16 *>(m_buf.constData())[idx], 6);
                break;
            case CFG_INT32U: {
                const quint32 v = reinterpret_cast<const quint32 *>(m_buf.constData())[idx];
                text += hex ? QStringLiteral(" %1").arg(v, 8, 16, QLatin1Char('0')).toUpper()
                            : QStringLiteral(" %1").arg(v, 12);
                break;
            }
            case CFG_INT32S:
                text += QStringLiteral(" %1").arg(reinterpret_cast<const qint32 *>(m_buf.constData())[idx], 12);
                break;
            case CFG_FP32:
                text += QStringLiteral(" %1").arg(reinterpret_cast<const float *>(m_buf.constData())[idx], 10, 'f', 3);
                break;
            }
        }
        text += QStringLiteral("\n");
    }

    m_dumpEdit->setPlainText(text);
}

void CpMemPane::log(const QString &text, const QColor &color)
{
    emit logMessage(text, color);
}
