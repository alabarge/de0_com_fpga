#include "DAQPane.h"

#include <algorithm>

#include <QCheckBox>
#include <QDateTime>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSettings>
#include <QTextStream>
#include <QTimer>
#include <QVBoxLayout>

#include "appCfg.h"
#include "fifoapi.h"

namespace {
DAQPane *g_instance = nullptr; // for the static cm_register thunk, see daqMsgThunk()

// DAQView.cpp: static cm_sub_t subs[] = {{CM_ID_DAQ_SRV, DAQ_DONE_IND, CM_ID_DAQ_SRV}, {CM_ID_NULL, NULL, NULL}};
cm_sub_t g_daqSubs[] = {{CM_ID_DAQ_SRV, DAQ_DONE_IND, CM_ID_DAQ_SRV}, {CM_ID_NULL, 0, 0}};

constexpr int kAdcBufLen = DAQ_MAX_PIPE_RUN * DAQ_MAX_SAM * DAQ_MAX_CH;

// cm_pipe_daq_t is "FIXED 1024-BYTE DAQ PIPE MESSAGES... SAME SIZE AS SLOTS
// IN WINDOWS DRIVER" per its own comment in daq_msg.h - DAQView.cpp strides
// through consecutive sub-messages with the OPTO-named OPTO_PIPELEN_UINT8
// (1024), which is really just that universal CM pipe slot size, not
// anything OPTO-specific; sizeof() is the same value without pulling in a
// transport header this port doesn't otherwise use.
constexpr size_t kPipeStride = sizeof(cm_pipe_daq_t);
}

DAQPane::DAQPane(QWidget *parent)
    : QWidget(parent)
    , m_running(false)
    , m_pktCount(0)
    , m_blkCount(0)
    , m_xferTotal(0)
    , m_lastStampUs(0)
    , m_adcRate(0)
    , m_adcBuf(kAdcBufLen, 0)
{
    g_instance = this;
    std::fill(std::begin(m_xferLen), std::end(m_xferLen), 0.0);
    std::fill(std::begin(m_deltaT), std::end(m_deltaT), 0.0);

    m_fileEdit = new QLineEdit(this);
    m_browseButton = new QPushButton(tr("..."), this);
    m_browseButton->setMaximumWidth(28);
    m_toFileCheck = new QCheckBox(tr("File"), this);
    m_hexCheck = new QCheckBox(tr("Hex"), this);
    m_rampCheck = new QCheckBox(tr("Ramp"), this);
    m_showCheck = new QCheckBox(tr("Show"), this);
    m_floatCheck = new QCheckBox(tr("Float"), this);
    m_plotCheck = new QCheckBox(tr("Plot"), this);
    m_rawCheck = new QCheckBox(tr("Raw"), this);
    m_packetsEdit = new QLineEdit(this);
    m_startButton = new QPushButton(tr("Start"), this);

    auto *fileRow = new QHBoxLayout();
    fileRow->addWidget(m_fileEdit);
    fileRow->addWidget(m_browseButton);

    auto *checkRow1 = new QHBoxLayout();
    checkRow1->addWidget(m_toFileCheck);
    checkRow1->addWidget(m_floatCheck);
    checkRow1->addWidget(m_hexCheck);

    auto *checkRow2 = new QHBoxLayout();
    checkRow2->addWidget(m_rampCheck);
    checkRow2->addWidget(m_showCheck);
    checkRow2->addWidget(m_plotCheck);

    auto *startRow = new QHBoxLayout();
    startRow->addWidget(m_startButton);
    startRow->addWidget(m_rawCheck);

    auto *packetsRow = new QHBoxLayout();
    packetsRow->addWidget(new QLabel(tr("Packets"), this));
    packetsRow->addWidget(m_packetsEdit);

    auto *controlGroup = new QGroupBox(tr("DAQ Controller"), this);
    auto *controlLayout = new QVBoxLayout(controlGroup);
    controlLayout->addLayout(fileRow);
    controlLayout->addLayout(startRow);
    controlLayout->addLayout(checkRow1);
    controlLayout->addLayout(checkRow2);
    controlLayout->addLayout(packetsRow);
    controlLayout->addStretch();

    m_dumpEdit = new QPlainTextEdit(this);
    m_dumpEdit->setReadOnly(true);
    m_dumpEdit->setMaximumBlockCount(4096);

    auto *layout = new QHBoxLayout(this);
    layout->addWidget(controlGroup);
    layout->addWidget(m_dumpEdit);

    connect(m_browseButton, &QPushButton::clicked, this, &DAQPane::onBrowseClicked);
    connect(m_startButton, &QPushButton::clicked, this, &DAQPane::onStartClicked);

    loadSettings();

    // DAQView::daq_init(): register as the CM DAQ client.
    cm_register(CM_ID_DAQ_CLI, &DAQPane::daqMsgThunk, this, g_daqSubs);
}

DAQPane::~DAQPane()
{
    if (g_instance == this)
        g_instance = nullptr;
}

void DAQPane::loadSettings()
{
    QSettings settings;
    m_fileEdit->setText(settings.value(QStringLiteral("DAQPane/file")).toString());
    m_toFileCheck->setChecked(settings.value(QStringLiteral("DAQPane/toFile"), false).toBool());
    m_hexCheck->setChecked(settings.value(QStringLiteral("DAQPane/hex"), false).toBool());
    m_rampCheck->setChecked(settings.value(QStringLiteral("DAQPane/ramp"), false).toBool());
    m_showCheck->setChecked(settings.value(QStringLiteral("DAQPane/show"), false).toBool());
    m_floatCheck->setChecked(settings.value(QStringLiteral("DAQPane/float"), false).toBool());
    m_plotCheck->setChecked(settings.value(QStringLiteral("DAQPane/plot"), false).toBool());
    m_rawCheck->setChecked(settings.value(QStringLiteral("DAQPane/raw"), false).toBool());
    m_packetsEdit->setText(settings.value(QStringLiteral("DAQPane/packets"), 1).toString());
}

void DAQPane::saveSettings()
{
    QSettings settings;
    settings.setValue(QStringLiteral("DAQPane/file"), m_fileEdit->text());
    settings.setValue(QStringLiteral("DAQPane/toFile"), m_toFileCheck->isChecked());
    settings.setValue(QStringLiteral("DAQPane/hex"), m_hexCheck->isChecked());
    settings.setValue(QStringLiteral("DAQPane/ramp"), m_rampCheck->isChecked());
    settings.setValue(QStringLiteral("DAQPane/show"), m_showCheck->isChecked());
    settings.setValue(QStringLiteral("DAQPane/float"), m_floatCheck->isChecked());
    settings.setValue(QStringLiteral("DAQPane/plot"), m_plotCheck->isChecked());
    settings.setValue(QStringLiteral("DAQPane/raw"), m_rawCheck->isChecked());
    settings.setValue(QStringLiteral("DAQPane/packets"), m_packetsEdit->text());
}

void DAQPane::onBrowseClicked()
{
    const QString file = QFileDialog::getSaveFileName(this, tr("DAQ Capture File"), m_fileEdit->text(),
                                                        tr("DAQ Files (*.txt *.bin);;All Files (*.*)"));
    if (!file.isEmpty())
        m_fileEdit->setText(file);
}

void DAQPane::onStartClicked()
{
    cm_send_t ps = {};

    if (m_running) {
        //
        // STOP
        //
        m_running = false;
        m_startButton->setText(tr("Start"));

        pcmq_t slot = cm_alloc();
        if (slot != nullptr) {
            auto *msg = reinterpret_cast<pdaq_run_msg_t>(slot->buf);
            msg->p.srvid = CM_ID_DAQ_SRV;
            msg->p.msgid = DAQ_RUN_REQ;
            msg->p.flags = DAQ_NO_FLAGS;
            msg->p.status = DAQ_OK;
            msg->b.opcode = DAQ_CMD_STOP;
            ps.msg = reinterpret_cast<pcm_msg_t>(msg);
            ps.dst_cmid = CM_ID_DAQ_SRV;
            ps.src_cmid = CM_ID_DAQ_CLI;
            ps.msglen = sizeof(daq_run_msg_t);
            cm_send(CM_MSG_REQ, &ps);
        }

        // Un-register for pipe messages.
        cm_pipe_reg(CM_ID_DAQ_CLI, 0, 0, CM_DEV_NULL);

        if (m_captureFile.isOpen())
            m_captureFile.close();

        saveSettings();
        return;
    }

    //
    // START
    //
    m_pktCount = 0;
    m_blkCount = 0;
    m_xferTotal = 0;
    m_lastStampUs = 0;
    std::fill(std::begin(m_xferLen), std::end(m_xferLen), 0.0);
    std::fill(std::begin(m_deltaT), std::end(m_deltaT), 0.0);

    // Only FIFO is a real transport in this port (see ConnectPane); the
    // other four fifo_head()-equivalents in DAQView.cpp don't apply here.
    fifo_head();

    if (m_toFileCheck->isChecked()) {
        const QFileInfo info(m_fileEdit->text());
        if (!info.filePath().isEmpty()) {
            const QString stamped = QStringLiteral("%1/%2_%3")
                                         .arg(info.path(),
                                              QDateTime::currentDateTime().toString(QStringLiteral("yyMMdd_HHmmss")),
                                              info.fileName());
            m_captureFile.setFileName(stamped);
            [[maybe_unused]] const bool opened = m_captureFile.open(QIODevice::WriteOnly | QIODevice::Text);
        }
    }

    if (m_toFileCheck->isChecked() && !m_captureFile.isOpen()) {
        log(tr("Error : File %1 did not open for writing").arg(m_fileEdit->text()), QColor(255, 0, 0));
        return;
    }

    pcmq_t slot = cm_alloc();
    if (slot == nullptr)
        return;

    auto *msg = reinterpret_cast<pdaq_run_msg_t>(slot->buf);
    msg->b.opcode = DAQ_CMD_RUN | DAQ_CMD_SEND_IND;
    msg->b.opcode |= m_rampCheck->isChecked() ? DAQ_CMD_RAMP : 0;
    msg->b.opcode |= DAQ_CMD_SCAN;
    msg->b.opcode |= DAQ_CMD_HEAD;
    msg->b.opcode |= DAQ_CMD_CH_ALL;
    msg->b.packets = m_packetsEdit->text().toUInt();

    cm_pipe_reg(CM_ID_DAQ_CLI, CM_PIPE_DAQ_DATA, 1, CM_DEV_WIN);

    msg->p.srvid = CM_ID_DAQ_SRV;
    msg->p.msgid = DAQ_RUN_REQ;
    msg->p.flags = DAQ_NO_FLAGS;
    msg->p.status = DAQ_OK;
    ps.msg = reinterpret_cast<pcm_msg_t>(msg);
    ps.dst_cmid = CM_ID_DAQ_SRV;
    ps.src_cmid = CM_ID_DAQ_CLI;
    ps.msglen = sizeof(daq_run_msg_t);
    cm_send(CM_MSG_REQ, &ps);

    if (m_captureFile.isOpen() && !m_hexCheck->isChecked()) {
        QTextStream out(&m_captureFile);
        out << QStringLiteral("# opCode : %1\n").arg(msg->b.opcode, 8, 16, QLatin1Char('0')).toUpper();
    }

    m_running = true;
    m_startButton->setText(tr("Stop"));
    saveSettings();
}

// cm_register() callback - see CpPane.cpp for why this is a queued Qt
// signal rather than DAQView.cpp's ring buffer/dedicated worker thread.
// Each pipe delivery here is a fixed 1024-byte struct (DAQ_MAX_PIPE_RUN=4
// of them per burst, ~8KB total) at a modest rate, well within what a
// queued connection handles without GUI-thread contention.
uint32_t DAQPane::daqMsgThunk(pcm_msg_t msg, void * /*member*/)
{
    if (g_instance) {
        QMetaObject::invokeMethod(g_instance, &DAQPane::onCmMessage, Qt::QueuedConnection,
                                   reinterpret_cast<quintptr>(msg));
    }
    return DAQ_OK;
}

void DAQPane::onCmMessage(quintptr msgPtr)
{
    auto *msg = reinterpret_cast<pcm_msg_t>(msgPtr);
    auto *pipe = reinterpret_cast<pcm_pipe_t>(msg);

    if (pipe->dst_cmid == CM_ID_PIPE) {
        if (pipe->msgid == CM_PIPE_DAQ_DATA)
            handlePipeData(reinterpret_cast<pcm_pipe_daq_t>(msg));
        // Pipe buffers are reused by the CM layer directly, unlike
        // control messages - DAQView.cpp's daq_msg() never cm_free()s
        // the pipe branch either.
        return;
    }

    if (msg->p.status != CM_OK) {
        log(tr("Error : CM Message status %1, srvid %2, msgid %3")
                .arg(msg->p.status, 2, 16, QLatin1Char('0'))
                .arg(msg->p.srvid, 2, 16, QLatin1Char('0'))
                .arg(msg->p.msgid, 2, 16, QLatin1Char('0')),
            QColor(255, 0, 0));
    } else if (MSG(msg->p.srvid, msg->p.msgid) == MSG(CM_ID_DAQ_SRV, DAQ_DONE_IND)) {
        QTimer::singleShot(20, this, &DAQPane::onDoneIndDelayed);
    }

    cm_free(msg);
}

void DAQPane::onDoneIndDelayed()
{
    // DAQView::OnTimer(ID_TIMER_DONE_IND): auto-stop once the firmware
    // reports the requested packet count is done.
    if (m_running)
        onStartClicked();
}

void DAQPane::handlePipeData(pcm_pipe_daq_t pipe)
{
    if (!m_running)
        return;

    m_xferTotal += static_cast<qint64>(pipe->msglen) * sizeof(uint32_t);
    m_xferLen[m_blkCount % 10] = static_cast<double>(pipe->msglen) * sizeof(uint32_t);
    m_deltaT[m_blkCount % 10] = static_cast<double>(pipe->stamp_us - m_lastStampUs) * 1E-6;
    m_lastStampUs = pipe->stamp_us;
    m_adcRate = pipe->rate;

    QTextStream out(m_captureFile.isOpen() ? &m_captureFile : nullptr);

    for (int i = 0; i < DAQ_MAX_PIPE_RUN; ++i) {
        for (int k = 0, m = 0; k < DAQ_MAX_SAM; ++k) {
            for (int l = 0; l < DAQ_MAX_CH; ++l)
                m_adcBuf[(i * DAQ_MAX_LEN) + m++] = static_cast<qint32>(pipe->samples[(k * DAQ_MAX_CH) + l]);
        }

        if (m_captureFile.isOpen() && !m_rawCheck->isChecked()) {
            for (int l = 0, j = 0; l < DAQ_MAX_LEN; l += DAQ_MAX_CH) {
                const int k = (i * DAQ_MAX_LEN) + l;
                const quint32 sampleIndex = ((m_pktCount + i) * DAQ_MAX_SAM) + j++;
                if (m_hexCheck->isChecked()) {
                    out << QStringLiteral("  %1").arg(sampleIndex, 6, 16, QLatin1Char('0')).toUpper();
                    for (int m = 0; m < DAQ_MAX_CH; ++m)
                        out << QStringLiteral(" %1").arg(static_cast<quint32>(m_adcBuf[k + m]), 6, 16, QLatin1Char('0')).toUpper();
                } else if (m_floatCheck->isChecked()) {
                    out << QStringLiteral("  %1").arg(sampleIndex, 8);
                    for (int m = 0; m < DAQ_MAX_CH; ++m)
                        out << QStringLiteral(" %1").arg(m_adcBuf[k + m] * DAQ_LSB, 0, 'E');
                } else {
                    out << QStringLiteral("  %1").arg(sampleIndex, 8);
                    for (int m = 0; m < DAQ_MAX_CH; ++m)
                        out << QStringLiteral(" %1").arg(m_adcBuf[k + m], 8);
                }
                out << QStringLiteral("\n");
            }
        } else if (m_captureFile.isOpen()) {
            const auto *words = reinterpret_cast<const quint16 *>(pipe) + (i * (kPipeStride / sizeof(quint16)));
            for (size_t l = 0; l < kPipeStride / sizeof(quint16); l += 8) {
                out << QStringLiteral("%1   %2 %3 %4 %5   %6 %7 %8 %9\n")
                           .arg((m_pktCount + i) * kPipeStride + (l * 2), 8, 16, QLatin1Char('0'))
                           .arg(words[l + 0], 4, 16, QLatin1Char('0'))
                           .arg(words[l + 1], 4, 16, QLatin1Char('0'))
                           .arg(words[l + 2], 4, 16, QLatin1Char('0'))
                           .arg(words[l + 3], 4, 16, QLatin1Char('0'))
                           .arg(words[l + 4], 4, 16, QLatin1Char('0'))
                           .arg(words[l + 5], 4, 16, QLatin1Char('0'))
                           .arg(words[l + 6], 4, 16, QLatin1Char('0'))
                           .arg(words[l + 7], 4, 16, QLatin1Char('0'))
                           .toUpper();
            }
        }
    }

    updateTextDump();

    if (m_plotCheck->isChecked())
        plotRun();

    if (m_blkCount > 10) {
        double xferSum = 0.0, deltaTSum = 0.0;
        for (int i = 0; i < 10; ++i) {
            xferSum += m_xferLen[i];
            deltaTSum += m_deltaT[i];
        }
        const double xferRate = deltaTSum != 0.0 ? xferSum / deltaTSum : 0.0;
        QString rateStr;
        if (xferRate > 1E6)
            rateStr = tr("%1 MiB/s").arg(xferRate / 1E6, 6, 'f', 1);
        else if (xferRate > 1E3)
            rateStr = tr("%1 KiB/s").arg(xferRate / 1E3, 6, 'f', 1);
        emit packetCountChanged(m_pktCount);
        Q_UNUSED(rateStr);
    }

    m_blkCount++;
    m_pktCount += DAQ_MAX_PIPE_RUN;
}

// Replacement for DAQView::OnPlotRun(): builds the same channel-major
// X/Y arrays the MFC app sent to GraphWnd via PostMessage(WM_PLOT_MSG, ...),
// but as a plain PlotData value handed to GraphPane through a signal.
void DAQPane::plotRun()
{
    PlotData data;
    data.plot = PlotData::Daq;
    data.type = 1; // "all channels" - the only DAQ plot type this port produces
    data.freq = (m_adcRate != 0) ? (100e6 / m_adcRate) / DAQ_MAX_CH / 16.0
                                  : (100e6 / DAQ_RATE_MAX) / DAQ_MAX_CH / 16.0;
    data.chan = 8;
    data.lanes[0] = DAQ_MAX_CH / 2;
    data.lanes[1] = DAQ_MAX_CH / 2;
    data.length = DAQ_MAX_PIPE_RUN * DAQ_MAX_SAM;
    data.record = m_pktCount;
    data.xMin = 0.0;
    data.xMax = static_cast<double>(DAQ_MAX_PIPE_RUN * DAQ_MAX_SAM) * (1.0 / data.freq);

    data.dataX.resize(DAQ_MAX_CH * data.length);
    data.dataY.resize(DAQ_MAX_CH * data.length);

    const bool ramp = m_rampCheck->isChecked();
    int l = 0, m = 0;
    for (int i = 0; i < DAQ_MAX_CH; ++i) {
        for (int j = 0; j < DAQ_MAX_PIPE_RUN; ++j) {
            for (int k = 0; k < DAQ_MAX_SAM; ++k) {
                const int sampleIdx = i + ((DAQ_MAX_LEN * j) + (k * DAQ_MAX_CH));
                data.dataX[m++] = static_cast<float>(((DAQ_MAX_SAM * j) + k) * (1.0 / data.freq));
                if (ramp) {
                    data.dataY[l++] = static_cast<float>(m_adcBuf[sampleIdx]);
                } else {
                    qint16 adc = static_cast<qint16>(m_adcBuf[sampleIdx]);
                    // Sign extension: DAQ_LSB assumes a 12-bit code (bit 11 is
                    // the sign), but adc is a naive 16-bit reinterpretation of
                    // the raw word - bits 12-15 (unused/tag bits on this ADC)
                    // must be discarded and replaced by sign-extending bit 11,
                    // or a stray 1 up there turns into a multi-volt swing once
                    // scaled below. DAQView.cpp's OnPlotRun() has this same
                    // line, just left commented out - see app/DAQView.cpp:241.
                    if (adc & 0x0800)
                        adc |= 0xF800;
                    else
                        adc &= 0x0FFF;
                    // Convert offset binary Volts
                    data.dataY[l++] = static_cast<float>(adc) * static_cast<float>(DAQ_LSB);
                }
            }
        }
    }

    emit plotReady(data);
}

void DAQPane::updateTextDump()
{
    if (!m_showCheck->isChecked())
        return;

    if ((m_pktCount % 1024) == 0)
        m_dumpEdit->clear();

    QString text;
    if ((m_pktCount % 1024) == 0) {
        text += m_hexCheck->isChecked()
            ? tr("CNT     CH0    CH1    CH2    CH3    CH4    CH5    CH6    CH7\n"
                 "======  ====== ====== ====== ====== ====== ====== ====== ======\n")
            : tr("CNT       CH0      CH1      CH2      CH3      CH4      CH5      CH6      CH7\n"
                 "========  ======== ======== ======== ======== ======== ======== ======== ========\n");
    }

    for (int i = 0; i < DAQ_MAX_PIPE_RUN; ++i) {
        for (int l = 0, j = 0; l < DAQ_MAX_LEN; l += DAQ_MAX_CH) {
            const int k = (i * DAQ_MAX_LEN) + l;
            const quint32 idx = ((m_pktCount + i) * DAQ_MAX_SAM) + j++;
            if (m_hexCheck->isChecked()) {
                text += QStringLiteral("%1 ").arg(idx, 6, 16, QLatin1Char('0')).toUpper();
                for (int m = 0; m < DAQ_MAX_CH; ++m)
                    text += QStringLiteral(" %1").arg(static_cast<quint32>(m_adcBuf[k + m]), 6, 16, QLatin1Char('0')).toUpper();
            } else {
                text += QStringLiteral("%1 ").arg(idx, 8);
                for (int m = 0; m < DAQ_MAX_CH; ++m)
                    text += QStringLiteral(" %1").arg(static_cast<quint32>(m_adcBuf[k + m]), 8);
            }
            text += QStringLiteral("\n");
        }
    }

    m_dumpEdit->appendPlainText(text);
}

void DAQPane::log(const QString &text, const QColor &color)
{
    emit logMessage(text, color);
}
