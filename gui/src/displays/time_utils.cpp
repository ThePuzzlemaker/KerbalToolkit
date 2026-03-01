#include "time_utils.h"

#include <QLabel>
#include <QStyle>
#include <QVBoxLayout>

#include "../widgets/util.h"

TimeUtils::TimeUtils(QWidget* parent) : QWidget(parent) {
  this->setStyleSheet("TimeUtils[adding=\"true\"] #addBtn {"
                      "  background-color: " RADIX_PURPLE_DARK_5 ";"
                      "}"

                      "TimeUtils[adding=\"true\"] #addBtn:hover:!pressed {"
                      "  background-color: " RADIX_PURPLE_DARK_6 ";"
                      "}"

                      "TimeUtils[adding=\"true\"] #addBtn:pressed {"
                      "  background-color: " RADIX_PURPLE_DARK_7 ";"
                      "}"

                      "TimeUtils[adding=\"false\"] #subBtn {"
                      "  background-color: " RADIX_PURPLE_DARK_5 ";"
                      "}"

                      "TimeUtils[adding=\"false\"] #subBtn:hover:!pressed {"
                      "  background-color: " RADIX_PURPLE_DARK_6 ";"
                      "}"

                      "TimeUtils[adding=\"false\"] #subBtn:pressed {"
                      "  background-color: " RADIX_PURPLE_DARK_7 ";"
                      "}");
  const auto layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(4);

  layout->addWidget(new QLabel("Reference Vessel: TODO"));
  layout->addWidget(new HorizontalSeparator);

  const auto t1Input = new TimeInput(true);
  layout->addWidget(t1Input);

  const auto btnRow = new QHBoxLayout;
  layout->addLayout(btnRow);
  btnRow->setContentsMargins(0, 0, 0, 0);
  btnRow->setSpacing(4);
  btnRow->setAlignment(Qt::AlignLeft);

  const auto swapBtn = IconButton("\u{E098}");
  btnRow->addWidget(swapBtn);

  const auto addBtn = IconButton("\u{E3D4}");
  btnRow->addWidget(addBtn);
  addBtn->setObjectName("addBtn");

  const auto subBtn = IconButton("\u{E32A}");
  btnRow->addWidget(subBtn);
  subBtn->setObjectName("subBtn");

  const auto t2Input = new TimeInput(true);
  layout->addWidget(t2Input);

  const auto sep = new HorizontalSeparator;
  layout->addWidget(sep);

  const auto t3Output = new TimeInput(true);
  t3Output->setReadOnly(true);
  layout->addWidget(t3Output);

  const auto recompute = [this, t3Output] {
    t3Output->setValue(m_adding ? m_t1 + m_t2 : m_t1 - m_t2);
  };

  connect(swapBtn, &QPushButton::clicked, this, [this, t1Input, t2Input] {
    std::swap(m_t1, m_t2);
    t1Input->setValue(m_t1);
    t2Input->setValue(m_t2);
  });

  connect(addBtn, &QPushButton::clicked, this, [=, this] {
    m_adding = true;
    addBtn->style()->unpolish(addBtn);
    addBtn->style()->polish(addBtn);
    subBtn->style()->unpolish(subBtn);
    subBtn->style()->polish(subBtn);
    recompute();
  });
  connect(subBtn, &QPushButton::clicked, this, [=, this] {
    m_adding = false;
    addBtn->style()->unpolish(addBtn);
    addBtn->style()->polish(addBtn);
    subBtn->style()->unpolish(subBtn);
    subBtn->style()->polish(subBtn);
    recompute();
  });

  connect(t1Input, &TimeInput::valueChanged, this,
          [this, recompute](const auto t1) {
            m_t1 = t1;
            recompute();
          });
  connect(t2Input, &TimeInput::valueChanged, this,
          [this, recompute](const auto t2) {
            m_t2 = t2;
            recompute();
          });
}