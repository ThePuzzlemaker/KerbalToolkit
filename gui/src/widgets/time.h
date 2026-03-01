#pragma once

#include <qcoreevent.h>

#include <QLineEdit>
#include "icon.h"

enum TimeInputKind {
  UT,
  GET,
};

enum TimeDisplayKind {
  DHMS,
  HMS,
  SEC,
};

using Duration = std::chrono::milliseconds;

std::optional<Duration> parseDhmsDuration(const QString& s,
                                          bool allowNegative = false);
std::optional<Duration> parseDhmsTime(const QString& s,
                                      bool allowNegative = false);
std::optional<Duration> parseDecTime(const QString& s,
                                     bool allowNegative = false);

QString toStringDhms(Duration d);
QString toStringHms(Duration d);
QString toStringSec(Duration d);
QString toStringDuration(Duration d);
QString toStringTime(Duration d, TimeDisplayKind kind);

class TimeInput final : public QWidget {
  Q_OBJECT

 public:
  explicit TimeInput(bool allowNegative = false, QWidget* parent = nullptr);

  bool isReadOnly() const { return m_readOnly; }
  void setReadOnly(bool readOnly);

  Duration value() const { return m_value; }
  void setValue(Duration value);

 signals:
  void valueChanged(Duration newValue);

 private:
  Q_PROPERTY(bool valid MEMBER m_valid);
  bool m_allowNegative = false;
  Duration m_value = Duration(0);
  TimeDisplayKind m_displayKind = DHMS;
  QLineEdit* m_edit;
  QPushButton* m_button;
  bool m_disabled = false;
  bool m_valid = true;
  bool m_readOnly = false;
};
