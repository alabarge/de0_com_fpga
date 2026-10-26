#include "ParmsPane.h"

#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QSettings>
#include <QVBoxLayout>

ParmsPane::ParmsPane(QWidget *parent)
    : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    // appDoc.cpp's iniConst[]: cplot.* defaults to Auto off, +/-80.0.
    PlotGroup cpuMem;
    layout->addWidget(buildGroup(tr("CPU Plot Settings"), this, cpuMem));
    loadGroup(QStringLiteral("cplot"), cpuMem, false, -80.0, 80.0);
    connectGroup(QStringLiteral("cplot"), cpuMem);

    // qplot.* defaults to Auto on, +/-60.0.
    PlotGroup daq;
    layout->addWidget(buildGroup(tr("DAQ Plot Settings"), this, daq));
    loadGroup(QStringLiteral("qplot"), daq, true, -60.0, 60.0);
    connectGroup(QStringLiteral("qplot"), daq);

    // qplot.length: display-only fidelity, genuinely unused elsewhere in
    // the original (see class comment).
    m_qplotLength = new QLineEdit(this);
    QSettings settings;
    m_qplotLength->setText(settings.value(QStringLiteral("qplot.length"), 4080).toString());
    connect(m_qplotLength, &QLineEdit::editingFinished, this, [this]() {
        QSettings().setValue(QStringLiteral("qplot.length"), m_qplotLength->text().toInt());
    });
    auto *lengthForm = new QFormLayout();
    lengthForm->addRow(tr("Length"), m_qplotLength);
    layout->addLayout(lengthForm);

    layout->addStretch();
}

QGroupBox *ParmsPane::buildGroup(const QString &title, QWidget *parent, PlotGroup &outGroup)
{
    auto *group = new QGroupBox(title, parent);
    auto *form = new QFormLayout(group);

    outGroup.legend = new QCheckBox(tr("Legend"), group);
    outGroup.autoA = new QCheckBox(tr("Auto A"), group);
    outGroup.yMinA = new QLineEdit(group);
    outGroup.yMaxA = new QLineEdit(group);
    outGroup.autoB = new QCheckBox(tr("Auto B"), group);
    outGroup.yMinB = new QLineEdit(group);
    outGroup.yMaxB = new QLineEdit(group);

    form->addRow(outGroup.legend);
    form->addRow(outGroup.autoA);
    form->addRow(tr("Y Min A"), outGroup.yMinA);
    form->addRow(tr("Y Max A"), outGroup.yMaxA);
    form->addRow(outGroup.autoB);
    form->addRow(tr("Y Min B"), outGroup.yMinB);
    form->addRow(tr("Y Max B"), outGroup.yMaxB);

    return group;
}

void ParmsPane::loadGroup(const QString &keyPrefix, const PlotGroup &group, bool defaultAuto, double defaultYMin,
                           double defaultYMax)
{
    QSettings settings;
    group.legend->setChecked(settings.value(keyPrefix + QStringLiteral(".legend"), false).toBool());
    group.autoA->setChecked(settings.value(keyPrefix + QStringLiteral(".autoA"), defaultAuto).toBool());
    group.yMinA->setText(settings.value(keyPrefix + QStringLiteral(".yMinA"), defaultYMin).toString());
    group.yMaxA->setText(settings.value(keyPrefix + QStringLiteral(".yMaxA"), defaultYMax).toString());
    group.autoB->setChecked(settings.value(keyPrefix + QStringLiteral(".autoB"), defaultAuto).toBool());
    group.yMinB->setText(settings.value(keyPrefix + QStringLiteral(".yMinB"), defaultYMin).toString());
    group.yMaxB->setText(settings.value(keyPrefix + QStringLiteral(".yMaxB"), defaultYMax).toString());

    // GraphSet::OnUpdateGphAuto()'s enable/disable pattern, applied per lane.
    auto updateEnabled = [](QCheckBox *autoCheck, QLineEdit *minEdit, QLineEdit *maxEdit) {
        const bool manual = !autoCheck->isChecked();
        minEdit->setEnabled(manual);
        maxEdit->setEnabled(manual);
    };
    updateEnabled(group.autoA, group.yMinA, group.yMaxA);
    updateEnabled(group.autoB, group.yMinB, group.yMaxB);
    connect(group.autoA, &QCheckBox::toggled, this,
            [=]() { updateEnabled(group.autoA, group.yMinA, group.yMaxA); });
    connect(group.autoB, &QCheckBox::toggled, this,
            [=]() { updateEnabled(group.autoB, group.yMinB, group.yMaxB); });
}

void ParmsPane::connectGroup(const QString &keyPrefix, const PlotGroup &group)
{
    auto save = [keyPrefix](const QString &suffix, const QVariant &value) {
        QSettings().setValue(keyPrefix + suffix, value);
    };

    connect(group.legend, &QCheckBox::toggled, this,
            [=](bool checked) { save(QStringLiteral(".legend"), checked); });
    connect(group.autoA, &QCheckBox::toggled, this,
            [=](bool checked) { save(QStringLiteral(".autoA"), checked); });
    connect(group.autoB, &QCheckBox::toggled, this,
            [=](bool checked) { save(QStringLiteral(".autoB"), checked); });
    connect(group.yMinA, &QLineEdit::editingFinished, this,
            [=]() { save(QStringLiteral(".yMinA"), group.yMinA->text().toDouble()); });
    connect(group.yMaxA, &QLineEdit::editingFinished, this,
            [=]() { save(QStringLiteral(".yMaxA"), group.yMaxA->text().toDouble()); });
    connect(group.yMinB, &QLineEdit::editingFinished, this,
            [=]() { save(QStringLiteral(".yMinB"), group.yMinB->text().toDouble()); });
    connect(group.yMaxB, &QLineEdit::editingFinished, this,
            [=]() { save(QStringLiteral(".yMaxB"), group.yMaxB->text().toDouble()); });
}
