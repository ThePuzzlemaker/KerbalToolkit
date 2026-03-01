#include "./time.h"

#include <QHBoxLayout>
#include <QRegularExpression>
#include <QStyle>

#include "util.h"

std::optional<Duration> parseDhmsDuration(const QString& s,
                                          const bool allowNegative) {
  static QRegularExpression wsRegex("\\s"), dayRegex("d(?:ay(?:s)?)?"),
      hourRegex("h(?:(?:ou)?r(?:s)?)?"), minRegex("m(?:in(?:ute(?:s)?)?)?"),
      secRegex("s(?:ec(?:ond)?(?:s)?)?");
  namespace chrono = std::chrono;

  auto str = s.trimmed();
  if (str.isEmpty())
    return {};

  const auto neg = str.startsWith("-(") && str.endsWith(")");
  if (allowNegative && neg)
    str.slice(2, s.length() - 3);
  else if (!allowNegative && neg)
    return {};

  str.remove(wsRegex);

  QRegularExpressionMatch match;
  auto idx = str.indexOf(dayRegex, 0, &match);
  const auto daysText = idx > 0 ? auto(str).slice(0, idx) : "";
  if (idx > 0)
    str.remove(0, match.capturedEnd(0));

  idx = str.indexOf(hourRegex, 0, &match);
  const auto hrsText = idx > 0 ? auto(str).slice(0, idx) : "";
  if (idx > 0)
    str.remove(0, match.capturedEnd(0));

  idx = str.indexOf(minRegex, 0, &match);
  const auto minsText = idx > 0 ? auto(str).slice(0, idx) : "";
  if (idx > 0)
    str.remove(0, match.capturedEnd(0));

  idx = str.indexOf(secRegex, 0, &match);
  auto secText = idx > 0 ? auto(str).slice(0, idx) : "";
  if (idx > 0)
    str.remove(0, match.capturedEnd(0));
  if (!str.isEmpty())
    return {};

  const auto secSplit = secText.split('.');
  if (secSplit.length() > 2 || secSplit.length() == 0)
    return {};
  secText = secSplit[0];
  QString millisText = "";
  if (secSplit.length() == 2)
    millisText = secSplit[1];

  bool ok = true;
  const auto d =
      chrono::days(daysText.length() > 0 ? daysText.toLongLong(&ok) : 0);
  if (!ok)
    return {};
  const auto h =
      chrono::hours(hrsText.length() > 0 ? hrsText.toLongLong(&ok) : 0);
  if (!ok)
    return {};
  const auto m =
      chrono::minutes(minsText.length() > 0 ? minsText.toLongLong(&ok) : 0);
  if (!ok)
    return {};
  const auto sec =
      chrono::seconds(secText.length() > 0 ? secText.toLongLong(&ok) : 0);

  chrono::milliseconds millis;
  switch (millisText.length()) {
    case 0:
      millis = chrono::milliseconds(0);
      break;
    case 1: {
      millis = chrono::milliseconds(100 * millisText.toLongLong(&ok));
      if (!ok)
        return {};
      break;
    }
    case 2: {
      millis = chrono::milliseconds(10 * millisText.toLongLong(&ok));
      if (!ok)
        return {};
      break;
    }
    case 3: {
      millis = chrono::milliseconds(1 * millisText.toLongLong(&ok));
      if (!ok)
        return {};
      break;
    }
    default:
      return {};
  }

  auto dur = chrono::duration_cast<Duration>(d) +
             chrono::duration_cast<Duration>(h) +
             chrono::duration_cast<Duration>(m) +
             chrono::duration_cast<Duration>(sec) + millis;
  if (neg)
    dur = -dur;
  return dur;
}

std::optional<Duration> parseDhmsTime(const QString& s,
                                      const bool allowNegative) {
  namespace chrono = std::chrono;

  auto str = s.trimmed();
  if (str.isEmpty())
    return {};

  const auto neg = str.startsWith("-(") && str.endsWith(")");
  if (allowNegative && neg)
    str.slice(2, s.length() - 3);
  else if (!allowNegative && neg)
    return {};

  const auto split = str.split('.');
  if (split.length() > 2)
    return {};
  const auto& timeStr = split[0];
  QString millisStr = "";
  if (split.length() == 2)
    millisStr = {split[1]};

  const auto partsStr = timeStr.split(":");
  if (partsStr.length() != 3 && partsStr.length() != 4)
    return {};
  auto parts = QList<qlonglong>(partsStr.length(), Qt::Uninitialized);
  for (auto i = 0; i < partsStr.length(); i++) {
    bool ok = true;
    parts[i] = partsStr[i].toLongLong(&ok);
    if (!ok)
      return {};
  }

  const auto d = chrono::days(parts.length() == 4 ? parts[0] : 0);
  const auto h = chrono::hours(parts.length() == 4 ? parts[1] : parts[0]);
  const auto m = chrono::minutes(parts.length() == 4 ? parts[2] : parts[1]);
  const auto sec = chrono::seconds(parts.length() == 4 ? parts[3] : parts[2]);
  Duration millis;
  bool ok = true;
  switch (millisStr.length()) {
    case 0:
      millis = chrono::milliseconds(0);
      break;
    case 1: {
      millis = chrono::milliseconds(100 * millisStr.toLongLong(&ok));
      if (!ok)
        return {};
      break;
    }
    case 2: {
      millis = chrono::milliseconds(10 * millisStr.toLongLong(&ok));
      if (!ok)
        return {};
      break;
    }
    case 3: {
      millis = chrono::milliseconds(1 * millisStr.toLongLong(&ok));
      if (!ok)
        return {};
      break;
    }
    default:
      return {};
  }

  auto dur = chrono::duration_cast<Duration>(d) +
             chrono::duration_cast<Duration>(h) +
             chrono::duration_cast<Duration>(m) +
             chrono::duration_cast<Duration>(sec) + millis;
  if (neg)
    dur = -dur;
  return dur;
}

std::optional<Duration> parseDecTime(const QString& s, bool allowNegative) {
  namespace chrono = std::chrono;

  auto str = s.trimmed();
  if (str.isEmpty())
    return {};

  const auto neg = str.startsWith("-(") && str.endsWith(")");
  if (allowNegative && neg)
    str.slice(2, s.length() - 3);
  else if (!allowNegative && neg)
    return {};

  const auto split = str.split('.');
  if (split.length() > 2)
    return {};
  bool ok = true;
  const auto timeStr = split[0].toLongLong(&ok);
  if (!ok)
    return {};
  QString millisStr = "";
  if (split.length() == 2)
    millisStr = {split[1]};

  const auto sec = chrono::seconds(timeStr);
  Duration millis;
  switch (millisStr.length()) {
    case 0:
      millis = chrono::milliseconds(0);
      break;
    case 1: {
      millis = chrono::milliseconds(100 * millisStr.toLongLong(&ok));
      if (!ok)
        return {};
      break;
    }
    case 2: {
      millis = chrono::milliseconds(10 * millisStr.toLongLong(&ok));
      if (!ok)
        return {};
      break;
    }
    case 3: {
      millis = chrono::milliseconds(1 * millisStr.toLongLong(&ok));
      if (!ok)
        return {};
      break;
    }
    default:
      return {};
  }

  auto dur = chrono::duration_cast<Duration>(sec) + millis;
  if (neg)
    dur = -dur;
  return dur;
}

QString toStringDhms(Duration d) {
  using namespace std::chrono;
  const auto neg = d < Duration::zero();
  d = Duration(std::abs(d.count()));
  return QString(std::format("{}{:03}:{:02}:{:02}:{:02}.{:03}{}",
                             neg ? "-(" : "", duration_cast<days>(d).count(),
                             duration_cast<hours>(d).count() % 24,
                             duration_cast<minutes>(d).count() % 60,
                             duration_cast<seconds>(d).count() % 60,
                             d.count() % 1000, neg ? ")" : "")
                     .c_str());
}

QString toStringHms(Duration d) {
  using namespace std::chrono;
  const auto neg = d < Duration::zero();
  d = abs(d);
  return QString(std::format("{}{:02}:{:02}:{:02}.{:03}{}", neg ? "-(" : "",
                             duration_cast<hours>(d).count(),
                             duration_cast<minutes>(d).count() % 60,
                             duration_cast<seconds>(d).count() % 60,
                             d.count() % 1000, neg ? ")" : "")
                     .c_str());
}

QString toStringSec(Duration d) {
  using namespace std::chrono;
  QString s = d < Duration::zero() ? "-" : "";
  d = abs(d);
  s.append(QString::number(duration_cast<seconds>(d).count()));
  if (const auto millis = d.count() % 1000; millis != 0) {
    s.append(".");
    s.append(QStringLiteral("%1").arg(millis, 3, 10, QChar('0')));
  } else {
    s.append(".000");
  }
  return s;
}

QString toStringDuration(Duration d) {
  using namespace std::chrono;
  const auto neg = d < Duration::zero();
  d = abs(d);
  QString s = neg ? "-(" : "";
  auto sep = "";
  if (const auto ds = duration_cast<days>(d).count(); ds > 0) {
    s.append(QString::number(ds));
    s.append("d");
    sep = " ";
  }
  if (const auto hr = duration_cast<hours>(d).count() % 24; hr > 0) {
    s.append(sep);
    s.append(QString::number(hr));
    s.append("h");
    sep = " ";
  }
  if (const auto min = duration_cast<minutes>(d).count() % 60; min > 0) {
    s.append(sep);
    s.append(QString::number(min));
    s.append("m");
    sep = " ";
  }
  if (const auto sec = duration_cast<seconds>(d).count() % 60; sec > 0) {
    s.append(sep);
    s.append(QString::number(sec));
    if (const auto millis = d.count() % 1000; millis > 0) {
      s.append(".");
      s.append(QStringLiteral("%1").arg(millis, 3, 10, QChar('0')));
    } else {
      s.append(".000");
    }
    s.append("s");
  }
  if (neg)
    s.append(")");
  return s;
}

QString toStringTime(const Duration d, const TimeDisplayKind kind) {
  switch (kind) {
    case DHMS:
      return toStringDhms(d);
    case HMS:
      return toStringHms(d);
    case SEC:
      return toStringSec(d);
  }
  assert(false);
}

TimeInput::TimeInput(const bool allowNegative, QWidget* parent)
    : QWidget(parent), m_allowNegative(allowNegative) {
  this->setStyleSheet("TimeInput QLineEdit { font-family: monospace; }"
                      "TimeInput[valid=\"false\"] QLineEdit {"
                      "  border: 1px solid " RADIX_RED_DARK_7 ";"
                      "}");
  const auto layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(4);
  layout->setAlignment(Qt::AlignLeft);
  m_edit = new QLineEdit;
  layout->addWidget(m_edit);
  m_edit->setMaximumWidth(224);
  m_edit->setStyleSheet("QLineEdit { font-family: monospace; }");
  m_button = IconButton("\u{E492}");
  layout->addWidget(m_button);

  connect(m_button, &QPushButton::clicked, this, [this] {
    switch (m_displayKind) {
      case DHMS:
        m_displayKind = HMS;
        break;
      case HMS:
        m_displayKind = SEC;
        break;
      case SEC:
        m_displayKind = DHMS;
        break;
      default:
        assert(false);
    }
    if (m_valid)
      m_edit->setText(toStringTime(m_value, m_displayKind));
  });
  connect(m_edit, &QLineEdit::editingFinished, this, [this] {
    if (m_valid)
      m_edit->setText(toStringTime(m_value, m_displayKind));
  });

  connect(m_edit, &QLineEdit::textChanged, this, [this](const auto buffer) {
    const auto parsed = parseDhmsDuration(buffer, m_allowNegative)
                            .or_else([this, buffer] {
                              return parseDhmsTime(buffer, m_allowNegative);
                            })
                            .or_else([this, buffer] {
                              return parseDecTime(buffer, m_allowNegative);
                            });
    if (parsed.has_value()) {
      m_value = *parsed;
      emit valueChanged(*parsed);
    }
    m_valid = parsed.has_value();
    m_edit->style()->unpolish(m_edit);
    m_edit->style()->polish(m_edit);
  });

  emit valueChanged(m_value);
}

void TimeInput::setReadOnly(const bool readOnly) {
  m_edit->setReadOnly(readOnly);
  m_readOnly = readOnly;
}

void TimeInput::setValue(const Duration value) {
  m_value = value;
  m_edit->setText(toStringTime(m_value, m_displayKind));
  emit valueChanged(value);
}

// #include "time_widgets.moc"