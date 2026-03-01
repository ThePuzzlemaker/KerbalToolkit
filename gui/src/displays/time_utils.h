#pragma once

#include "../widgets/time.h"

class TimeUtils final : public QWidget {
  Q_OBJECT

 public:
  explicit TimeUtils(QWidget* parent = nullptr);

 private:
  Duration m_t1 = Duration(0);
  Duration m_t2 = Duration(0);
  Q_PROPERTY(bool adding MEMBER m_adding);
  bool m_adding = true;
};