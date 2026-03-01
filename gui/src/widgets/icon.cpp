#include "icon.h"

#include "colors.h"
#include "util.h"

void FontIconEngine::paint(QPainter* painter,
                           const QRect& rect,
                           const QIcon::Mode mode,
                           const QIcon::State state) {
  auto font = QFont(m_fontFamily);
  const auto drawSize = qRound(rect.height() * 0.8);
  font.setPixelSize(drawSize);

  painter->save();
  painter->setPen(QPen(m_color));
  painter->setFont(font);
  painter->drawText(rect, Qt::AlignCenter | Qt::AlignVCenter, m_icon);

  painter->restore();
}

QPixmap FontIconEngine::pixmap(const QSize& size,
                               const QIcon::Mode mode,
                               const QIcon::State state) {
  QPixmap pixmap(size);
  pixmap.fill(Qt::transparent);

  QPainter painter(&pixmap);
  paint(&painter, QRect(QPoint(0, 0), size), mode, state);

  return pixmap;
}

QIconEngine* FontIconEngine::clone() const {
  return new FontIconEngine(m_fontFamily, m_icon, m_color);
}

QIcon PhosphorIcon(QString icon, const QColor color) {
  const auto engine = new FontIconEngine("Phosphor", std::move(icon), color);
  return QIcon(engine);
}

QPushButton* IconButton(const QString& icon, QWidget* parent) {
  const auto btn = new QPushButton(parent);
  const auto lineHeight = getLineHeight();
  btn->setIcon(PhosphorIcon(icon, RADIX_MAUVE_DARK_12_Q));
  btn->setStyleSheet("padding: 4px;");
  btn->setIconSize(QSize(lineHeight, lineHeight));
  btn->setFixedWidth(lineHeight + 8);
  btn->setFixedHeight(lineHeight + 11);
  return btn;
}
