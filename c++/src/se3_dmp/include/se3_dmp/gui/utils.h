#ifndef UTILS_H
#define UTILS_H

#include <QString>
#include <QMessageBox>
#include <QLineEdit>
#include <QPushButton>

#include <mutex>
#include <condition_variable>

void showErrorMsg(const QString &msg);
void showWarningMsg(const QString &msg);
int showQuestionMsg(const QString &msg);
void showInfoMsg(const QString &msg);

class MyLineEdit: public QLineEdit
{
public:
    MyLineEdit(QWidget *parent=0):QLineEdit(parent) { size_hint = QSize(60,30); }

    void setSizeHint(int w, int h) { size_hint.setWidth(w); size_hint.setHeight(h); }
    QSize sizeHint() const override { return size_hint; }
private:
    QSize size_hint;
};

#endif // UTILS_H
