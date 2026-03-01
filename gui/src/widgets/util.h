#pragma once

#include <QFont>
#include <QFontMetrics>
#include <QPaintEvent>

#include "colors.h"

inline int getLineHeight() {
  return QFontMetrics(QFont("Inter Variable", 10)).height();
}

class HorizontalSeparator final : public QWidget {
  Q_OBJECT

 public:
  explicit HorizontalSeparator(const QColor color = RADIX_MAUVE_DARK_7_Q,
                               const int width = 1,
                               QWidget* parent = nullptr)
      : QWidget(parent), m_color(color), m_width(width) {
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
  }

  void paintEvent(QPaintEvent* event) override {
    QPainter painter(this);
    painter.setPen(QPen(QBrush(m_color), m_width * 2, Qt::SolidLine,
                        Qt::RoundCap, Qt::RoundJoin));
    painter.drawLine(QPoint(0, 0), QPoint(event->rect().width(), 0));
  }

  QSize sizeHint() const override { return {0, m_width * 2}; }

 private:
  QColor m_color;
  int m_width;
};