#include "ImagePane.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QToolTip>
#include <QVBoxLayout>

namespace {
// 2DImage.cpp's APP_DEF_IMAGE_BACK and static greyScale[9].
constexpr QRgb kImageBackColor = qRgb(159, 181, 210);
constexpr int kGreyScale[9] = {0, 32, 64, 96, 128, 160, 192, 224, 255};
}

// Replacement for C2DImage: renders into an off-screen QImage on resize
// (same idea as C2DImage::RefreshCtrl()'s cached memory DC, blitted in
// OnPaint()), and answers mouse-move with the actual rendered pixel color
// (C2DImage::OnMouseMove()'s dc.GetPixel(point) tooltip) rather than
// recomputing it analytically - reading back the real rendered buffer is
// what the original did, and it stays correct if this ever grows real
// image content instead of just the calibration strip.
class ImageCanvas : public QWidget
{
public:
    explicit ImageCanvas(QWidget *parent = nullptr)
        : QWidget(parent)
    {
        setMouseTracking(true);
        render();
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);
        painter.drawImage(0, 0, m_image);
    }

    void resizeEvent(QResizeEvent *event) override
    {
        QWidget::resizeEvent(event);
        render();
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        const QPoint pos = event->pos();
        if (m_image.rect().contains(pos)) {
            const QColor c = m_image.pixelColor(pos);
            QToolTip::showText(event->globalPosition().toPoint(),
                                QStringLiteral("R:%1  G:%2  B:%3").arg(c.red()).arg(c.green()).arg(c.blue()),
                                this);
        }
        QWidget::mouseMoveEvent(event);
    }

private:
    // 2DImage.cpp's DrawGreyScale(): nine 16x8 swatches at x=4, stacked
    // vertically starting at y=4.
    void render()
    {
        m_image = QImage(size(), QImage::Format_RGB32);
        m_image.fill(kImageBackColor);

        QPainter painter(&m_image);
        for (int i = 0; i < 9; ++i) {
            const int v = kGreyScale[i];
            painter.fillRect(4, (i * 8) + 4, 16, 8, QColor(v, v, v));
        }

        update();
    }

    QImage m_image;
};

ImagePane::ImagePane(QWidget *parent)
    : QWidget(parent)
{
    m_canvas = new ImageCanvas(this);
    m_settingsButton = new QPushButton(tr("Settings..."), this);

    auto *toolRow = new QHBoxLayout();
    toolRow->addWidget(m_settingsButton);
    toolRow->addStretch();

    auto *layout = new QVBoxLayout(this);
    layout->addLayout(toolRow);
    layout->addWidget(m_canvas);

    connect(m_settingsButton, &QPushButton::clicked, this, &ImagePane::onSettingsClicked);
}

void ImagePane::onSettingsClicked()
{
    // CImageWnd::OnSettings()/CImageSet: the original dialog genuinely has
    // no fields, just OK/Cancel - kept empty rather than inventing settings
    // that were never there.
    QDialog dlg(this);
    dlg.setWindowTitle(tr("Image Settings"));

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    auto *layout = new QVBoxLayout(&dlg);
    layout->addWidget(buttons);

    dlg.exec();
}
