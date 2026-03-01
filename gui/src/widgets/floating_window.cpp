#include "floating_window.h"
#include "colors.h"
#include "icon.h"

#include <ranges>

#include <QLabel>
#include <QPropertyAnimation>
#include <QPushButton>
#include <QTimer>

#include "util.h"

class CollapseBtn final : public QWidget {
  Q_OBJECT

 public:
  explicit CollapseBtn(const int size, QWidget* parent = nullptr)
      : QWidget(parent), m_size(size) {
    setMouseTracking(true);
  }

  QSize sizeHint() const override { return {m_size, m_size}; }
  QSize minimumSizeHint() const override { return {m_size, m_size}; }


 signals:
  // ReSharper disable CppParameterNamesMismatch
  void collapsed(bool collapse);
  // ReSharper restore CppParameterNamesMismatch

 protected:
  void mousePressEvent(QMouseEvent* event) override {
    m_collapsed = !m_collapsed;
    if (m_collapsed) {
      animateProgress(0.0f, 1.0f);
    } else {
      animateProgress(1.0f, 0.0f);
    }
    emit collapsed(m_collapsed);
  }

  void paintEvent(QPaintEvent* event) override {
    QPainter painter(this);
    painter.setPen(Qt::NoPen);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setBrush(QBrush(RADIX_MAUVE_DARK_12_Q, Qt::SolidPattern));
    const auto sz = static_cast<float>(m_size) * 0.9;
    painter.translate(sz / 2 + sz * 0.1, sz / 2 + sz * 0.1);
    painter.rotate((1.0 - m_progress) * 90.0f);
    painter.translate(-sz / 2, -sz / 2);
    painter.drawPolygon(QPolygonF(
        {QPointF(0.0f, 0.0f), QPointF(0.0f, sz), QPointF(sz, 0.5f * sz)}));
  }

 private:
  void animateProgress(const float start, const float end) {
    const auto anim = new QPropertyAnimation(this, "progress");
    anim->setDuration(250);
    anim->setEasingCurve(QEasingCurve::OutCubic);
    anim->setStartValue(start);
    anim->setEndValue(end);
    connect(anim, &QPropertyAnimation::valueChanged, this,
            [this](auto _) { this->repaint(); });
    anim->start(QAbstractAnimation::DeleteWhenStopped);
  }

  Q_PROPERTY(float progress MEMBER m_progress)
  float m_progress = 0.0f;
  bool m_collapsed = false;
  int m_size;
};

class TitleBar final : public QFrame {
  Q_OBJECT

 public:
  explicit TitleBar(QString title,
                    const bool closeable,
                    QWidget* parent = nullptr)
      : QFrame(parent), m_title(std::move(title)), m_closeable(closeable) {
    this->setStyleSheet("TitleBar {"
                        "  background-color: " RADIX_PURPLE_DARK_7 ";"
                        "  border-top-left-radius: 8px;"
                        "  border-top-right-radius: 8px;"
                        "}"

                        "TitleBar .title {"
                        "  color: #ffffff;"
                        "  font-weight: bold;"
                        "}");
    this->setMouseTracking(true);
    this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    setContentsMargins(0, 0, 0, 0);

    const auto titleBarLayout = new QGridLayout(this);
    titleBarLayout->setContentsMargins(8, 0, 8, 0);
    this->setFixedHeight(32.0);

    const auto lineHeight = getLineHeight();
    m_collapseBtn = new CollapseBtn(lineHeight);
    titleBarLayout->addWidget(m_collapseBtn, 0, 0, Qt::AlignLeft);
    m_collapseBtn->setFixedSize(lineHeight, lineHeight);
    connect(m_collapseBtn, &CollapseBtn::collapsed, this, &TitleBar::collapsed);

    const auto titleText = new QLabel(m_title);
    titleBarLayout->addWidget(titleText, 0, 1, Qt::AlignHCenter);
    titleText->setAlignment(Qt::AlignCenter | Qt::AlignHCenter);
    titleText->setContentsMargins(0, 0, 0, 0);
    titleText->setProperty("class", "title");

    if (m_closeable) {
      const auto exitBtn = new QPushButton;
      titleBarLayout->addWidget(exitBtn, 0, 2, Qt::AlignRight);
      exitBtn->setStyleSheet("background-color: none; border: none;");
      exitBtn->setFixedSize(lineHeight, lineHeight);
      exitBtn->setIcon(PhosphorIcon("\u{E4F6}", RADIX_MAUVE_DARK_12_Q));
      exitBtn->setIconSize({lineHeight * 3 / 2, lineHeight * 3 / 2});
      connect(exitBtn, &QPushButton::clicked, [&]() { emit closed(); });
    }

    titleBarLayout->setColumnStretch(0, 1);
    titleBarLayout->setColumnStretch(2, 1);
  }

 protected:
  void mousePressEvent(QMouseEvent* e) override {
    if (e->button() == Qt::LeftButton) {
      m_dragging = true;
      m_dragOffset = e->pos();
      emit moved(e->pos() - m_dragOffset, true);
    }
  }

  void mouseMoveEvent(QMouseEvent* e) override {
    if (m_dragging) {
      emit moved(e->pos() - m_dragOffset, false);
    }
  }

  void mouseReleaseEvent(QMouseEvent*) override { m_dragging = false; }

 signals:
  void moved(QPoint screenPos, bool justStarted);
  void closed();
  void collapsed(bool collapse);

 private:
  QString m_title;
  bool m_dragging = false;
  QPoint m_dragOffset;
  CollapseBtn* m_collapseBtn;
  bool m_closeable;
};

FloatingWindow::FloatingWindow(FloatingWindowOptions options, QWidget* parent)
    : QFrame(parent) {
  this->setStyleSheet("FloatingWindow {"
                      "  border: 1px solid " RADIX_MAUVE_DARK_6 ";"
                      "  border-radius: 8px;"
                      "  background-color: " RADIX_MAUVE_DARK_3 ";"
                      "}");
  this->setMouseTracking(true);
  this->setObjectName(options.title);
  this->setSizePolicy(QSizePolicy::MinimumExpanding,
                      QSizePolicy::MinimumExpanding);

  m_baseLayout = new QVBoxLayout(this);
  m_baseLayout->setContentsMargins(0, 0, 0, 0);
  m_baseLayout->setSpacing(0);
  m_baseLayout->setAlignment(Qt::AlignTop);

  const auto titleBar =
      new TitleBar(std::move(options.title), options.closeable);
  m_baseLayout->addWidget(titleBar);
  connect(titleBar, &TitleBar::moved, this,
          [&](auto screenPos, auto justStarted) {
            move(mapToParent(screenPos));
            if (justStarted)
              this->raise();
          });
  connect(titleBar, &TitleBar::closed, this, [&] { this->open(false); });
  connect(titleBar, &TitleBar::collapsed, this, &FloatingWindow::collapse);

  m_body = new QVBoxLayout;
  m_baseLayout->addLayout(m_body);
  m_body->setContentsMargins(8, 8, 8, 8);
  m_body->setSpacing(4);
}

bool FloatingWindowManager::eventFilter(QObject* watched, QEvent* event) {
  if (event->type() == QEvent::MouseButtonPress) {
    // NOLINTNEXTLINE(*-pro-type-static-cast-downcast)
    const auto mouse = static_cast<QMouseEvent*>(event);
    const auto globalPos = mouse->globalPosition().toPoint();

    const auto comp = [](auto win1, auto win2) {
      auto children = win1->parentWidget()->children();
      auto p1Idx = children.indexOf(win1);
      auto p2Idx = children.indexOf(win2);
      return p1Idx > p2Idx;
    };
    std::ranges::sort(windows, comp);
    for (const auto fw : windows) {
      if (fw->geometry().contains(
              fw->parentWidget()->mapFromGlobal(globalPos))) {
        fw->raise();
        fw->activateWindow();
        return false;
      }
    }
  }
  return QObject::eventFilter(watched, event);
}

void FloatingWindow::animateOpen(const bool open) {
  if (m_open == open)
    return;

  m_open = open;

  this->raise();
  this->setVisible(true);
  const auto targetHeight =
      m_collapsed ? titleBarHeight : m_baseLayout->sizeHint().height();
  const auto anim = new QPropertyAnimation(this, "geometry");
  anim->setDuration(250);
  anim->setEasingCurve(QEasingCurve::OutCubic);
  anim->setStartValue(this->geometry());
  anim->setEndValue(
      QRect(this->x(), this->y(), this->width(), m_open ? targetHeight : 0));
  connect(anim, &QPropertyAnimation::finished, this, [this] {
    if (!m_open)
      this->setVisible(false);
  });
  anim->start(QAbstractAnimation::DeleteWhenStopped);
}

void FloatingWindow::animateCollapse(const bool collapse) {
  if (m_collapsed == collapse)
    return;

  m_collapsed = collapse;
  if (!m_open)
    return;

  const auto targetHeight = m_baseLayout->sizeHint().height();
  const auto anim = new QPropertyAnimation(this, "geometry");
  anim->setDuration(250);
  anim->setEasingCurve(QEasingCurve::OutCubic);
  anim->setStartValue(this->geometry());
  anim->setEndValue(QRect(this->x(), this->y(), this->width(),
                          m_collapsed ? titleBarHeight : targetHeight));
  anim->start(QAbstractAnimation::DeleteWhenStopped);
}

#include "floating_window.moc"