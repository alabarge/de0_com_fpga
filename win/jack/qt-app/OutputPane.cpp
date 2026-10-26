#include "OutputPane.h"

#include <QDateTime>
#include <QPlainTextEdit>
#include <QTabWidget>
#include <QTextCursor>
#include <QVBoxLayout>

OutputPane::OutputPane(QWidget *parent)
    : QWidget(parent)
    , m_tabs(new QTabWidget(this))
    , m_log(new QPlainTextEdit(this))
    , m_traffic(new QPlainTextEdit(this))
    , m_debug(new QPlainTextEdit(this))
{
    for (QPlainTextEdit *edit : {m_log, m_traffic, m_debug}) {
        edit->setReadOnly(true);
        edit->setMaximumBlockCount(5000);
    }

    m_tabs->addTab(m_log, tr("Log"));
    m_tabs->addTab(m_traffic, tr("Traffic"));
    m_tabs->addTab(m_debug, tr("Debug"));

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_tabs);
}

void OutputPane::logMessage(const QString &text, const QColor &color)
{
    appendTo(m_log, text, color);
}

void OutputPane::trafficMessage(const QString &text, const QColor &color)
{
    appendTo(m_traffic, text, color);
}

void OutputPane::debugMessage(const QString &text, const QColor &color)
{
    appendTo(m_debug, text, color);
}

void OutputPane::onCmTraffic(quint8 opCode, quintptr msg)
{
    trafficMessage(tr("op_code=0x%1 msg=0x%2")
                        .arg(opCode, 2, 16, QLatin1Char('0'))
                        .arg(msg, 0, 16));
}

void OutputPane::appendTo(QPlainTextEdit *edit, const QString &text, const QColor &color)
{
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("HH:mm:ss.zzz"));

    QTextCursor cursor(edit->document());
    cursor.movePosition(QTextCursor::End);

    QTextCharFormat format;
    format.setForeground(color);
    cursor.insertText(QStringLiteral("%1  %2\n").arg(stamp, text), format);

    edit->moveCursor(QTextCursor::End);
}
