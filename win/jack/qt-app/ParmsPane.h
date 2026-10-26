#pragma once

#include <QWidget>

class QCheckBox;
class QGroupBox;
class QLineEdit;

// Replacement for CParmsWnd (ParmsWnd.h/.cpp). The MFC version is a
// generic reflective CMFCPropertyGridCtrl over every CappDoc ini entry
// tagged DOC_TAG_PARM - but checking appDoc.cpp's ini table shows that's
// only ever exactly 15 properties, in two fixed groups ("CPU Plot
// Settings"/"DAQ Plot Settings" - the two non-empty DOC_TAG_TITLES
// entries). Nothing else is ever tagged DOC_TAG_PARM, so a generic
// property-grid widget isn't needed - this is just those 15 fields as a
// plain form, the same way CpPane's fixed trace-flag UI isn't a generic
// grid either.
//
// These are exactly the per-lane auto-scale/min/max/legend settings
// GraphWnd.cpp's PlotMemory()/PlotDAQ() reads on every plot call (as
// cplotAutoA/qplotYminB/etc.) - initially mis-identified as dead UI-less
// fields while building GraphPane (Phase 4), since ParmsWnd hadn't been
// looked at yet. GraphPane now reads these same settings (as plain
// QSettings keys matching the original ini key names, e.g. "cplot.yMinA")
// directly in its applyAxisScale(), so this pane only needs to edit and
// persist them - no direct signal/slot coupling to GraphPane required,
// same as the original: CappDoc's ini was the only thing connecting them.
//
// qplot.length is ported for display fidelity even though it's genuinely
// dead in the original too - never read anywhere outside its own
// definition in appDoc.cpp.
class ParmsPane : public QWidget
{
    Q_OBJECT

public:
    explicit ParmsPane(QWidget *parent = nullptr);

private:
    struct PlotGroup {
        QCheckBox *legend;
        QCheckBox *autoA;
        QLineEdit *yMinA;
        QLineEdit *yMaxA;
        QCheckBox *autoB;
        QLineEdit *yMinB;
        QLineEdit *yMaxB;
    };

    QGroupBox *buildGroup(const QString &title, QWidget *parent, PlotGroup &outGroup);
    void loadGroup(const QString &keyPrefix, const PlotGroup &group, bool defaultAuto, double defaultYMin,
                    double defaultYMax);
    void connectGroup(const QString &keyPrefix, const PlotGroup &group);

    QLineEdit *m_qplotLength;
};
