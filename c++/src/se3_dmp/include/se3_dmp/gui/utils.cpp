#include "utils.h"

void showErrorMsg(const QString &msg)
{
    QMessageBox msg_box;

    msg_box.setText(msg);
    msg_box.setIcon(QMessageBox::Critical);
    msg_box.setStandardButtons(QMessageBox::Ok);
    msg_box.setModal(true);

    msg_box.exec();
}

void showWarningMsg(const QString &msg)
{
    QMessageBox msg_box;

    msg_box.setText(msg);
    msg_box.setIcon(QMessageBox::Warning);
    msg_box.setStandardButtons(QMessageBox::Ok);
    msg_box.setModal(true);

    msg_box.exec();
}

int showQuestionMsg(const QString &msg)
{
    QMessageBox msg_box;

    msg_box.setText(msg);
    msg_box.setIcon(QMessageBox::Question);
    msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    msg_box.setModal(true);

    return msg_box.exec();
}

void showInfoMsg(const QString &msg)
{
    QMessageBox msg_box;

    msg_box.setText(msg);
    msg_box.setIcon(QMessageBox::Information);
    msg_box.setStandardButtons(QMessageBox::Ok);
    msg_box.setModal(true);

    msg_box.exec();
}
