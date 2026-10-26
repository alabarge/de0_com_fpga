#include "ProjectPane.h"

#include <QTreeWidget>
#include <QVBoxLayout>

namespace {
constexpr int PageIdRole = Qt::UserRole;
}

ProjectPane::ProjectPane(QWidget *parent)
    : QWidget(parent)
    , m_tree(new QTreeWidget(this))
{
    m_tree->setHeaderHidden(true);

    auto *connect_ = new QTreeWidgetItem(m_tree, {tr("Connect")});
    connect_->setData(0, PageIdRole, QStringLiteral("connect"));

    auto *cp = new QTreeWidgetItem(m_tree, {tr("CP")});
    cp->setData(0, PageIdRole, QStringLiteral("cp"));

    auto *cpMem = new QTreeWidgetItem(cp, {tr("CP Memory")});
    cpMem->setData(0, PageIdRole, QStringLiteral("cp_mem"));

    auto *daq = new QTreeWidgetItem(m_tree, {tr("DAQ")});
    daq->setData(0, PageIdRole, QStringLiteral("daq"));

    m_tree->expandAll();

    connect(m_tree, &QTreeWidget::currentItemChanged, this,
            [this](QTreeWidgetItem *current, QTreeWidgetItem *) {
                if (current)
                    emit pageSelected(current->data(0, PageIdRole).toString());
            });

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_tree);

    m_tree->setCurrentItem(connect_);
}
