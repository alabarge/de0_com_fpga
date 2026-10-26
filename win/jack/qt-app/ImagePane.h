#pragma once

#include <QWidget>

class QPushButton;
class ImageCanvas;

// Replacement for CImageWnd/CImageSet/C2DImage (ImageWnd.h/.cpp,
// ImageSet.h/.cpp, 2DImage.h/.cpp).
//
// This one turned out to be almost entirely vestigial once actually read:
// - CImageWnd::OnPlot() is declared in the header but never implemented
//   anywhere, and nothing ever calls it - there's no real caller that
//   ever assembles device image data (cpImgType/Rows/Cols/Depth/Data/Auto
//   in appType.h are declared and never referenced anywhere else at all).
// - CImageSet ("Image Settings") is a dialog with OK/Cancel and literally
//   no other controls - app.rc's IDD_IMG_SETTINGS has zero fields.
// - C2DImage::DrawImage()/DrawOrder()/DrawLine()/FillSolidRect()/Reset()/
//   Redraw() are real, working methods - just never called by anything
//   except DrawGreyScale() (the static calibration-strip test pattern)
//   and OnPaint()'s own cached-bitmap blit.
//
// So what's actually ported here is exactly what's actually live: the
// nine-swatch greyscale calibration strip, and the mouse-over pixel-color
// tooltip (C2DImage::OnMouseMove()'s real, if small, feature). The empty
// Settings dialog is kept empty rather than inventing fields that were
// never there, and OnPlot()/DrawImage()/etc. aren't ported at all, same
// as every other genuinely dead code path found across this whole port.
class ImagePane : public QWidget
{
    Q_OBJECT

public:
    explicit ImagePane(QWidget *parent = nullptr);

private slots:
    void onSettingsClicked();

private:
    ImageCanvas *m_canvas;
    QPushButton *m_settingsButton;
};
