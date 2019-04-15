#ifndef VIEW_POSE_DIALOG_H
#define VIEW_POSE_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QCloseEvent>

#include <functional>
#include <armadillo>
#include <thread>

#include "utils.h"

class ViewPoseDialog : public QDialog
{
    Q_OBJECT

public:
    ViewPoseDialog(std::function<arma::vec()> readToolPos, std::function<arma::vec()> readToolOrient ,QWidget *parent = 0);
    ~ViewPoseDialog();

public slots:
    void launch();
    void stop();

private:
    MyLineEdit *x_le;
    MyLineEdit *y_le;
    MyLineEdit *z_le;
    MyLineEdit *qw_le;
    MyLineEdit *qx_le;
    MyLineEdit *qy_le;
    MyLineEdit *qz_le;

    bool run;
    std::function<arma::vec()> readToolPos;
    std::function<arma::vec()> readToolOrient;

    void updateDialogThread();

    MyLineEdit *createLineEdit();

    void closeEvent(QCloseEvent *event) override;
};

#endif // VIEW_POSE_DIALOG_H
