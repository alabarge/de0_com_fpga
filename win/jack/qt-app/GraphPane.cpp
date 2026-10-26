#include "GraphPane.h"

#include <algorithm>
#include <limits>

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QSettings>
#include <QVBoxLayout>

#include "qcustomplot.h"

namespace {
// GraphWnd.cpp's static clrList[]/chnList[], ported directly so channel
// colors/labels match the MFC app.
constexpr QRgb kColorList[] = {
    0x00dc4f, 0xff4242, 0x3399ff, 0xab848a, 0xffdc00, 0xcd59ff, 0xffb312, 0x002c94,
    0x00cdd4, 0x9a8a74, 0x292929, 0x005b17, 0xb41100, 0xf95bcd, 0x680095, 0x008a8a,
};

QVector<double> toDoubleRange(const QVector<float> &src, int offset, int count)
{
    QVector<double> out(count);
    for (int i = 0; i < count; ++i)
        out[i] = src.value(offset + i);
    return out;
}
}

GraphPane::GraphPane(QWidget *parent)
    : QWidget(parent)
    , m_graphType(PlotData::None)
    , m_xMin(0.0)
    , m_xMax(0.0)
    , m_yMin(0.0)
    , m_yMax(0.0)
    , m_autoScale(true)
    , m_cursorEnabled(false)
    , m_smallPoints(false)
    , m_lastType(-1)
{
    m_lanes[0] = 0;
    m_lanes[1] = 0;

    m_plot = new QCustomPlot(this);
    m_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    m_plot->legend->setVisible(true);
    m_plot->yAxis2->setVisible(false);

    // Axis B's lane - built now but not inserted into m_plot's layout until
    // rebuildGraphs() actually needs it (dualAxis true), so a single-lane
    // plot (CpMemPane, or DAQ with no Axis B channels) still gets the whole
    // chart area to itself.
    m_laneBRect = new QCPAxisRect(m_plot);
    m_laneBRect->axis(QCPAxis::atBottom)->setLabel(tr("Seconds (s)"));
    m_laneBRect->axis(QCPAxis::atLeft)->setLabel(tr("Amplitude (V)"));

    m_settingsButton = new QPushButton(tr("Settings..."), this);
    m_exportButton = new QPushButton(tr("Export..."), this);
    m_titleLabel = new QLabel(tr("None"), this);
    m_cursorLabel = new QLabel(this);

    auto *toolRow = new QHBoxLayout();
    toolRow->addWidget(m_settingsButton);
    toolRow->addWidget(m_exportButton);
    toolRow->addWidget(m_titleLabel);
    toolRow->addStretch();
    toolRow->addWidget(m_cursorLabel);

    auto *layout = new QVBoxLayout(this);
    layout->addLayout(toolRow);
    layout->addWidget(m_plot);

    connect(m_settingsButton, &QPushButton::clicked, this, &GraphPane::onSettingsClicked);
    connect(m_exportButton, &QPushButton::clicked, this, &GraphPane::onExportClicked);
    connect(m_plot, &QCustomPlot::mouseMove, this, [this](QMouseEvent *event) {
        if (!m_cursorEnabled) {
            m_cursorLabel->clear();
            return;
        }
        // Report against whichever lane the cursor is actually over - with
        // Axis B in its own stacked rect, m_plot->yAxis alone would be wrong
        // for anything read off Axis B's lane.
        QCPAxisRect *rect = m_plot->axisRectAt(event->pos());
        if (!rect) {
            m_cursorLabel->clear();
            return;
        }
        const double x = rect->axis(QCPAxis::atBottom)->pixelToCoord(event->pos().x());
        const double y = rect->axis(QCPAxis::atLeft)->pixelToCoord(event->pos().y());
        m_cursorLabel->setText(tr("X: %1  Y: %2").arg(x, 0, 'g', 5).arg(y, 0, 'g', 5));
    });

    loadSettings();
    updateWindowTitle();
}

void GraphPane::loadSettings()
{
    QSettings settings;
    m_graphType = static_cast<PlotData::GraphType>(
        settings.value(QStringLiteral("GraphPane/type"), PlotData::None).toInt());
    m_xMin = settings.value(QStringLiteral("GraphPane/xMin"), 0.0).toDouble();
    m_xMax = settings.value(QStringLiteral("GraphPane/xMax"), 0.0).toDouble();
    m_yMin = settings.value(QStringLiteral("GraphPane/yMin"), 0.0).toDouble();
    m_yMax = settings.value(QStringLiteral("GraphPane/yMax"), 0.0).toDouble();
    m_autoScale = settings.value(QStringLiteral("GraphPane/auto"), true).toBool();
    m_cursorEnabled = settings.value(QStringLiteral("GraphPane/cursor"), false).toBool();
    m_smallPoints = settings.value(QStringLiteral("GraphPane/points"), false).toBool();
}

void GraphPane::saveSettings()
{
    QSettings settings;
    settings.setValue(QStringLiteral("GraphPane/type"), static_cast<int>(m_graphType));
    settings.setValue(QStringLiteral("GraphPane/xMin"), m_xMin);
    settings.setValue(QStringLiteral("GraphPane/xMax"), m_xMax);
    settings.setValue(QStringLiteral("GraphPane/yMin"), m_yMin);
    settings.setValue(QStringLiteral("GraphPane/yMax"), m_yMax);
    settings.setValue(QStringLiteral("GraphPane/auto"), m_autoScale);
    settings.setValue(QStringLiteral("GraphPane/cursor"), m_cursorEnabled);
    settings.setValue(QStringLiteral("GraphPane/points"), m_smallPoints);
}

void GraphPane::updateWindowTitle()
{
    switch (m_graphType) {
    case PlotData::CpuMem:
        m_titleLabel->setText(tr("CPU Memory"));
        break;
    case PlotData::Daq:
        m_titleLabel->setText(tr("DAQ"));
        break;
    default:
        m_titleLabel->setText(tr("None"));
        break;
    }
}

void GraphPane::onSettingsClicked()
{
    // Replacement for CGraphSet (GraphSet.h/.cpp).
    QDialog dlg(this);
    dlg.setWindowTitle(tr("Graph Settings"));

    auto *typeCombo = new QComboBox(&dlg);
    typeCombo->addItems({tr("None"), tr("CPU Memory"), tr("DAQ")});
    typeCombo->setCurrentIndex(static_cast<int>(m_graphType));

    auto *xMinEdit = new QLineEdit(QString::number(m_xMin, 'f', 2), &dlg);
    auto *xMaxEdit = new QLineEdit(QString::number(m_xMax, 'f', 2), &dlg);
    auto *yMinEdit = new QLineEdit(QString::number(m_yMin, 'f', 2), &dlg);
    auto *yMaxEdit = new QLineEdit(QString::number(m_yMax, 'f', 2), &dlg);
    auto *autoCheck = new QCheckBox(tr("Auto"), &dlg);
    autoCheck->setChecked(m_autoScale);
    auto *cursorCheck = new QCheckBox(tr("Cursor"), &dlg);
    cursorCheck->setChecked(m_cursorEnabled);
    auto *pointsCheck = new QCheckBox(tr("Small Points"), &dlg);
    pointsCheck->setChecked(m_smallPoints);

    // GraphSet::OnInitDialog()/OnUpdateGphAuto(): the X/Y range fields are
    // only meaningful (and only enabled) when Auto is off.
    auto updateRangeEnabled = [=]() {
        const bool enabled = !autoCheck->isChecked();
        xMinEdit->setEnabled(enabled);
        xMaxEdit->setEnabled(enabled);
        yMinEdit->setEnabled(enabled);
        yMaxEdit->setEnabled(enabled);
    };
    updateRangeEnabled();
    connect(autoCheck, &QCheckBox::toggled, updateRangeEnabled);

    auto *form = new QFormLayout();
    form->addRow(tr("Type"), typeCombo);
    form->addRow(tr("X Min"), xMinEdit);
    form->addRow(tr("X Max"), xMaxEdit);
    form->addRow(tr("Y Min"), yMinEdit);
    form->addRow(tr("Y Max"), yMaxEdit);
    form->addRow(autoCheck);
    form->addRow(cursorCheck);
    form->addRow(pointsCheck);

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto *layout = new QVBoxLayout(&dlg);
    layout->addLayout(form);
    layout->addWidget(buttons);

    if (dlg.exec() != QDialog::Accepted)
        return;

    m_graphType = static_cast<PlotData::GraphType>(typeCombo->currentIndex());
    m_xMin = xMinEdit->text().toDouble();
    m_xMax = xMaxEdit->text().toDouble();
    m_yMin = yMinEdit->text().toDouble();
    m_yMax = yMaxEdit->text().toDouble();
    m_autoScale = autoCheck->isChecked();
    m_cursorEnabled = cursorCheck->isChecked();
    m_smallPoints = pointsCheck->isChecked();
    saveSettings();
    updateWindowTitle();

    // CGraphWnd::OnSettings()/ResetGraph(): changing the type/points/scale
    // clears the graph and puts up an initial axis range immediately,
    // without waiting for new data - the one real use these Auto/Y Min/
    // Y Max fields have, since every actual plot() call re-scales via
    // ParmsPane's settings instead (see applyAxisScale()).
    m_lastType = -1;
    m_lanes[0] = m_lanes[1] = 0;
    m_plot->clearGraphs();
    m_plot->xAxis->setRange(m_xMin, m_xMax);
    if (!m_autoScale)
        m_plot->yAxis->setRange(m_yMin, m_yMax);
    m_plot->replot();
}

void GraphPane::onExportClicked()
{
    // GraphWnd.cpp's OnPrint() sent the graph to ProEssentials' own print
    // dialog; QCustomPlot's equivalent is exporting an image/PDF directly.
    const QString file = QFileDialog::getSaveFileName(this, tr("Export Graph"), QString(),
                                                         tr("PNG Image (*.png);;PDF Document (*.pdf)"));
    if (file.isEmpty())
        return;

    if (file.endsWith(QStringLiteral(".pdf"), Qt::CaseInsensitive))
        m_plot->savePdf(file);
    else
        m_plot->savePng(file);
}

void GraphPane::plot(const PlotData &data)
{
    // CGraphWnd::PlotGraph(): only draw if this pane is currently set to
    // the matching graph type; otherwise the data is simply not used (the
    // MFC app explicitly deletes pPlot's heap buffers here instead - not
    // needed, PlotData owns its own QVectors).
    if (data.plot != m_graphType)
        return;

    switch (data.plot) {
    case PlotData::CpuMem:
        plotMemory(data);
        break;
    case PlotData::Daq:
        plotDaq(data);
        break;
    default:
        break;
    }
}

void GraphPane::rebuildGraphs(int laneCount, bool dualAxis)
{
    m_plot->clearGraphs();

    // Insert/remove Axis B's lane from the layout to match - take(), not
    // remove(), since remove() would delete m_laneBRect and we reuse the
    // same object the next time dualAxis flips back on.
    if (dualAxis) {
        if (!m_plot->plotLayout()->hasElement(1, 0)) {
            m_plot->plotLayout()->addElement(1, 0, m_laneBRect);
            m_plot->plotLayout()->setRowStretchFactor(0, 1);
            m_plot->plotLayout()->setRowStretchFactor(1, 1);
        }
    } else if (m_plot->plotLayout()->hasElement(1, 0)) {
        m_plot->plotLayout()->take(m_laneBRect);
        m_plot->plotLayout()->simplify();
    }

    const QCPScatterStyle scatter = m_smallPoints
        ? QCPScatterStyle(QCPScatterStyle::ssDisc, 3)
        : QCPScatterStyle(QCPScatterStyle::ssDisc, 6);

    for (int i = 0; i < laneCount; ++i) {
        const bool laneB = dualAxis && i >= m_lanes[0];
        QCPAxis *keyAxis = laneB ? m_laneBRect->axis(QCPAxis::atBottom) : m_plot->xAxis;
        QCPAxis *valueAxis = laneB ? m_laneBRect->axis(QCPAxis::atLeft) : m_plot->yAxis;
        QCPGraph *graph = m_plot->addGraph(keyAxis, valueAxis);
        graph->setPen(QColor(kColorList[i % 16]));
        graph->setScatterStyle(scatter);
        graph->setName(tr("CH%1").arg(i, 2, 10, QLatin1Char('0')));
    }
}

void GraphPane::applyAxisScale(const PlotData &data)
{
    // X range comes from the data (how much time/how many samples were
    // actually captured) - always data-driven, same as GraphWnd.cpp's
    // unconditional per-call X manual-scale.
    m_plot->xAxis->setRange(data.xMin, data.xMax);
    if (m_plot->plotLayout()->hasElement(1, 0))
        m_laneBRect->axis(QCPAxis::atBottom)->setRange(data.xMin, data.xMax);

    // Y range/legend: ParmsPane's cplot.*/qplot.* settings (lane A/B per
    // plot type), read fresh on every call - exactly how CappDoc's ini
    // table fed GraphWnd.cpp's PlotMemory()/PlotDAQ() on every real plot
    // message. GraphPane's own Settings dialog (m_autoScale/m_yMin/m_yMax)
    // only matters for the brief pre-data state right after it's changed -
    // see onSettingsClicked().
    const QString keyPrefix = (data.plot == PlotData::Daq) ? QStringLiteral("qplot") : QStringLiteral("cplot");
    QSettings settings;

    auto scaleAxis = [&](QCPAxis *axis, const QString &laneSuffix, int laneBegin, int laneEnd) {
        const bool autoScale = settings.value(keyPrefix + QStringLiteral(".auto") + laneSuffix, true).toBool();
        if (autoScale) {
            double lo = std::numeric_limits<double>::max();
            double hi = std::numeric_limits<double>::lowest();
            for (int lane = laneBegin; lane < laneEnd; ++lane) {
                for (int i = 0; i < data.length; ++i) {
                    const float v = data.dataY.value(lane * data.length + i);
                    lo = std::min<double>(lo, v);
                    hi = std::max<double>(hi, v);
                }
            }
            if (lo <= hi)
                axis->setRange(lo, hi);
        } else {
            const double yMin = settings.value(keyPrefix + QStringLiteral(".yMin") + laneSuffix, -1.0).toDouble();
            const double yMax = settings.value(keyPrefix + QStringLiteral(".yMax") + laneSuffix, 1.0).toDouble();
            axis->setRange(yMin, yMax);
        }
    };

    scaleAxis(m_plot->yAxis, QStringLiteral("A"), 0, m_lanes[0]);
    if (m_lanes[1] > 0)
        scaleAxis(m_laneBRect->axis(QCPAxis::atLeft), QStringLiteral("B"), m_lanes[0], m_lanes[0] + m_lanes[1]);

    m_plot->legend->setVisible(settings.value(keyPrefix + QStringLiteral(".legend"), false).toBool());
}

void GraphPane::plotMemory(const PlotData &data)
{
    updateWindowTitle();

    // GraphWnd.cpp's PlotMemory(): rebuild when the shape changes (always
    // a single lane for CPU Memory, but the window length can change).
    if (data.record == 0 || m_lanes[0] != data.lanes[0] || m_lanes[1] != data.lanes[1]
        || m_lastType != data.type) {
        m_lanes[0] = data.lanes[0];
        m_lanes[1] = data.lanes[1];
        m_lastType = data.type;
        rebuildGraphs(m_lanes[0] + m_lanes[1], false);
    }

    if (m_plot->graphCount() > 0)
        m_plot->graph(0)->setData(toDoubleRange(data.dataX, 0, data.length),
                                   toDoubleRange(data.dataY, 0, data.length));

    applyAxisScale(data);
    m_plot->replot();
}

void GraphPane::plotDaq(const PlotData &data)
{
    updateWindowTitle();

    const bool dualAxis = data.lanes[1] > 0;

    // GraphWnd.cpp's PlotDAQ(): rebuild when the lane split or window
    // length changes.
    if (m_lanes[0] != data.lanes[0] || m_lanes[1] != data.lanes[1] || m_lastType != data.type) {
        m_lanes[0] = data.lanes[0];
        m_lanes[1] = data.lanes[1];
        m_lastType = data.type;
        rebuildGraphs(m_lanes[0] + m_lanes[1], dualAxis);
    }

    const int laneCount = m_lanes[0] + m_lanes[1];
    for (int lane = 0; lane < laneCount && lane < m_plot->graphCount(); ++lane) {
        m_plot->graph(lane)->setData(toDoubleRange(data.dataX, lane * data.length, data.length),
                                      toDoubleRange(data.dataY, lane * data.length, data.length));
    }

    applyAxisScale(data);
    m_plot->replot();
}
