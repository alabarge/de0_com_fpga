#pragma once

#include <QWidget>
#include <QColor>

class QPlainTextEdit;
class QTabWidget;

// Replacement for COutputWnd/CLogEdit (OutputWnd.h/LogEdit.h): a tabbed
// Log/Traffic/Debug console. The MFC version serialized concurrent appends
// with a CRITICAL_SECTION per tab because device threads called LogStr()/
// TrafficStr()/DebugStr() directly; here every append happens on the GUI
// thread already (see DeviceBridge), so no locking is needed.
class OutputPane : public QWidget
{
    Q_OBJECT

public:
    explicit OutputPane(QWidget *parent = nullptr);

public slots:
    void logMessage(const QString &text, const QColor &color = QColor(0, 0, 0));
    void trafficMessage(const QString &text, const QColor &color = QColor(0, 0, 0));
    void debugMessage(const QString &text, const QColor &color = QColor(0, 0, 0));

    // Matches DeviceBridge::cmTraffic until Phase 3 ports the real CM
    // message decoding/formatting from MainFrm::OnCmLog.
    void onCmTraffic(quint8 opCode, quintptr msg);

private:
    void appendTo(QPlainTextEdit *edit, const QString &text, const QColor &color);

    QTabWidget *m_tabs;
    QPlainTextEdit *m_log;
    QPlainTextEdit *m_traffic;
    QPlainTextEdit *m_debug;
};
