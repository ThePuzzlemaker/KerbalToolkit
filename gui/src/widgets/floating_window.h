#pragma once

#include <QFrame>
#include <QVBoxLayout>

struct FloatingWindowOptions final {
  QString title;
  bool closeable = true;
};

class FloatingWindow final : public QFrame {
  Q_OBJECT

 public:
  explicit FloatingWindow(FloatingWindowOptions options,
                          QWidget* parent = nullptr);

  QVBoxLayout* body() const { return m_body; }

  bool isOpen() const { return m_open; }
  bool isCollapsed() const { return m_collapsed; }

 signals:
  void opened(bool open);
  void collapsed(bool collapse);

 public slots:
  void open(const bool open) {
    animateOpen(open);
    emit opened(open);
  };
  void collapse(const bool collapse) {
    animateCollapse(collapse);
    emit collapsed(collapse);
  }

 protected:
  void mousePressEvent(QMouseEvent* event) override { this->raise(); }

  constexpr static int titleBarHeight = 32;

 private:
  void animateOpen(bool open);
  void animateCollapse(bool collapse);

  QVBoxLayout* m_baseLayout;
  QVBoxLayout* m_body;
  bool m_open = true;
  bool m_collapsed = false;
};

class FloatingWindowManager final : public QObject {
  Q_OBJECT

 public:
  explicit FloatingWindowManager(QObject* parent = nullptr) : QObject(parent) {}
  QList<FloatingWindow*> windows;

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override;
};