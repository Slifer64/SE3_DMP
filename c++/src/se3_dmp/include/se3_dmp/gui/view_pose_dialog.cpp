#include "view_pose_dialog.h"

ViewPoseDialog::ViewPoseDialog(std::function<arma::vec()> readToolPos, std::function<arma::vec()> readToolOrient ,QWidget *parent): QDialog(parent)
{
    run = false;

    this->readToolPos = readToolPos;
    this->readToolOrient = readToolOrient;

    this->setWindowTitle("Robot tool pose");

    QLabel *pos_label = new QLabel("Position");
    pos_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");
    QLabel *x_label = new QLabel("x");
    QLabel *y_label = new QLabel("y");
    QLabel *z_label = new QLabel("z");
    QLabel *m_label = new QLabel("m");

    x_le = createLineEdit();
    y_le = createLineEdit();
    z_le = createLineEdit();

    QLabel *orient_label = new QLabel("Orientation\n(Quaternion)");
    orient_label->setStyleSheet("background-color: rgb(245,245,245); color: rgb(0,0,0); font: 75 14pt \"FreeSans\";");
    QLabel *scalar_label = new QLabel("scalar");
    QLabel *vector_label = new QLabel("vector");

    qw_le = createLineEdit();
    qx_le = createLineEdit();
    qy_le = createLineEdit();
    qz_le = createLineEdit();

    QGridLayout *main_layout = new QGridLayout(this);
    // main_layout->setSizeConstraint(QLayout::SetFixedSize);
    main_layout->addWidget(x_label,0,1, Qt::AlignCenter);
    main_layout->addWidget(y_label,0,2, Qt::AlignCenter);
    main_layout->addWidget(z_label,0,3, Qt::AlignCenter);
    main_layout->addWidget(pos_label,1,0, Qt::AlignCenter);
    main_layout->addWidget(x_le,1,1);
    main_layout->addWidget(y_le,1,2);
    main_layout->addWidget(z_le,1,3);
    main_layout->addWidget(m_label,1,4);
    main_layout->addItem(new QSpacerItem(0,20),2,0);
    main_layout->addWidget(orient_label,3,0,2,1, Qt::AlignCenter);
    main_layout->addWidget(scalar_label,3,1, Qt::AlignCenter);
    main_layout->addWidget(vector_label,3,2,1,3, Qt::AlignCenter);
    main_layout->addWidget(qw_le,4,1);
    main_layout->addWidget(qx_le,4,2);
    main_layout->addWidget(qy_le,4,3);
    main_layout->addWidget(qz_le,4,4);

    Qt::ConnectionType connect_type = Qt::AutoConnection;
    MyLineEdit *le_array[] = {x_le, y_le, z_le, qw_le, qx_le, qy_le, qz_le};
    for (int i=0;i<7;i++) QObject::connect(le_array[i], SIGNAL(textChanged(QString)), le_array[i], SLOT(setText(QString)), connect_type);

//    QObject::connect(x_le, SIGNAL(textChanged(QString)), x_le, SLOT(setText(QString)), connect_type);
//    QObject::connect(y_le, SIGNAL(textChanged(QString)), y_le, SLOT(setText(QString)), connect_type);
//    QObject::connect(z_le, SIGNAL(textChanged(QString)), z_le, SLOT(setText(QString)), connect_type);
//    QObject::connect(qw_le, SIGNAL(textChanged(QString)), qw_le, SLOT(setText(QString)), connect_type);
//    QObject::connect(qx_le, SIGNAL(textChanged(QString)), qx_le, SLOT(setText(QString)), connect_type);
//    QObject::connect(qy_le, SIGNAL(textChanged(QString)), qy_le, SLOT(setText(QString)), connect_type);
//    QObject::connect(qz_le, SIGNAL(textChanged(QString)), qz_le, SLOT(setText(QString)), connect_type);

}

ViewPoseDialog::~ViewPoseDialog()
{
    stop();
}

MyLineEdit *ViewPoseDialog::createLineEdit()
{
    MyLineEdit *le = new MyLineEdit;
    le->setSizeHint(60,30);
    le->setReadOnly(true);
    le->setStyleSheet("font: 75 14pt;");
    le->setAlignment(Qt::AlignCenter);

    return le;
}

void ViewPoseDialog::launch()
{
    if (!run)
    {
        run = true;
        std::thread(&ViewPoseDialog::updateDialogThread, this).detach();
        this->show();
    }
}

void ViewPoseDialog::stop()
{
    if (run)
    {
        run = false;
        this->hide();
    }
}

void ViewPoseDialog::updateDialogThread()
{
    while (run)
    {
        arma::vec pos = readToolPos();
        arma::vec quat = readToolOrient();

        emit x_le->textChanged(QString::number(pos(0),'f',2));
        emit y_le->textChanged(QString::number(pos(1),'f',2));
        emit z_le->textChanged(QString::number(pos(2),'f',2));

        emit qw_le->textChanged(QString::number(quat(0),'f',2));
        emit qx_le->textChanged(QString::number(quat(1),'f',2));
        emit qy_le->textChanged(QString::number(quat(2),'f',2));
        emit qz_le->textChanged(QString::number(quat(3),'f',2));

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void ViewPoseDialog::closeEvent(QCloseEvent *event)
{
    stop();
}
