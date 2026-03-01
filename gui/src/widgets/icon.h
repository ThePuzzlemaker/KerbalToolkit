//! Based on https://github.com/dridk/QFontIcon, licensed under MIT.
#pragma once

#include <QIconEngine>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>

class FontIconEngine final : public QIconEngine {
 public:
  FontIconEngine(QString fontFamily, QString icon, QColor color)
      : m_fontFamily(std::move(fontFamily)),
        m_icon(std::move(icon)),
        m_color(color) {}

  void paint(QPainter* painter,
             const QRect& rect,
             QIcon::Mode mode,
             QIcon::State state) override;

  QPixmap pixmap(const QSize& size,
                 QIcon::Mode mode,
                 QIcon::State state) override;

  QIconEngine* clone() const override;

 private:
  QString m_fontFamily;
  QString m_icon;
  QColor m_color = QColor(0, 0, 0, 255);
};

QIcon PhosphorIcon(QString icon, const QColor color);

QPushButton* IconButton(const QString& icon, QWidget* parent = nullptr);