#include <QApplication>
#include <QFontDatabase>
#include <QGraphicsEffect>
#include <QLabel>
#include <QMainWindow>

#include "displays/time_utils.h"
#include "widgets/colors.h"
#include "widgets/floating_window.h"
#include "widgets/time.h"

class MainWindow final : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow() {
    qGuiApp->installEventFilter(&fwmgr);
    const auto central = new QWidget;
    setCentralWidget(central);
    setFocusPolicy(Qt::StrongFocus);

    setStyleSheet("MainWindow {"
                  "  background-color: " RADIX_MAUVE_DARK_1 ";"
                  "}"

                  "* {"
                  "  font-family: \"Inter Variable\";"
                  "  font-size: 10pt;"
                  "  color: " RADIX_MAUVE_DARK_12 ";"
                  "}"

                  "QPushButton {"
                  "  background-color: " RADIX_MAUVE_DARK_3 ";"
                  "  border: 1px solid " RADIX_MAUVE_DARK_7 ";"
                  "  border-radius: 4px;"
                  "  padding: 4px;"
                  "}"

                  "QPushButton:hover:!disabled:!pressed {"
                  "  background-color: " RADIX_MAUVE_DARK_4 ";"
                  "}"

                  "QPushButton:pressed {"
                  "  background-color: " RADIX_MAUVE_DARK_5 ";"
                  "}"

                  "QPushButton:disabled {"
                  "  background-color: " RADIX_MAUVE_DARK_3 ";"
                  "  border: 1px solid " RADIX_MAUVE_DARK_5 ";"
                  "}"

                  "QLineEdit {"
                  "  background-color: " RADIX_MAUVE_DARK_1 ";"
                  "  border: 1px solid " RADIX_MAUVE_DARK_7 ";"
                  "  border-radius: 4px;"
                  "  padding: 4px;"
                  "}"

                  "QLineEdit:read-only {"
                  "  background-color: " RADIX_MAUVE_DARK_2 ";"
                  "  border: 1px solid " RADIX_MAUVE_DARK_5 ";"
                  "}"

                  "QLineEdit:focus {"
                  "  border: 1px solid " RADIX_PURPLE_DARK_7 ";"
                  "}");

    const auto p1 = new FloatingWindow(
        FloatingWindowOptions{.title = "Time Utilities", .closeable = true},
        central);
    p1->setMinimumWidth(256);
    p1->setMaximumWidth(384);
    p1->body()->addWidget(new TimeUtils);

    const auto p2 = new FloatingWindow(
        FloatingWindowOptions{.title = "Test 2", .closeable = false}, central);
    p2->setMinimumWidth(256);
    const auto button6 = new QPushButton("Open Window");
    connect(button6, &QPushButton::clicked, this, [p1] { p1->open(true); });
    p2->body()->addWidget(button6);
    const auto timeInput = new TimeInput;
    timeInput->setReadOnly(true);
    p2->body()->addWidget(timeInput);

    fwmgr.windows << p1;
    fwmgr.windows << p2;

    p1->move(60, 60);
    p2->move(340, 80);
  }

 private:
  FloatingWindowManager fwmgr;
};

#include "main.moc"

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  QFontDatabase::addApplicationFont(":/assets/fonts/InterVariable.ttf");
  QFontDatabase::addApplicationFont(":/assets/fonts/InterVariable-Italic.ttf");
  QFontDatabase::addApplicationFont(":/assets/icons/regular/Phosphor.ttf");
  MainWindow w;
  w.setWindowTitle("Qt Testing");
  w.show();
  return QApplication::exec();
}