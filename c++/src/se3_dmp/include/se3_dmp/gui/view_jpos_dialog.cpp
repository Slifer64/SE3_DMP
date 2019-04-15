#include "view_jpos_dialog.h"

#include <iostream>

ViewJPosDialog::ViewJPosDialog(const arma::vec &jlow_lim, const arma::vec &jup_lim, std::function<arma::vec()> readJointsPosition, QWidget *parent): QDialog(parent)
{
    run = false;

    this->readJointsPosition = readJointsPosition;
    this->jlow_lim = jlow_lim;
    this->jup_lim = jup_lim;
    N_JOINTS = jlow_lim.size();

    slider_max = 1000;
    slider_min = -1000;

    jnames.resize(N_JOINTS);
    for (int i=0;i<N_JOINTS;i++) jnames[i] = QString("joint " + QString::number(i+1)).toStdString();

    this->setWindowTitle("Robot Joints Position (degrees)");
    this->resize(350,250);

    labels.resize(N_JOINTS);
    sliders.resize(N_JOINTS);
    values.resize(N_JOINTS);

    QGridLayout *main_layout = new QGridLayout(this);
    // main_layout->setSizeConstraint(QLayout::SetFixedSize);

    for (int i=0; i<N_JOINTS; i++)
    {
        labels[i] = new QLabel(jnames[i].c_str());
        labels[i]->setStyleSheet("font: 75 14pt;");

        sliders[i] = new QSlider(Qt::Horizontal);
        sliders[i]->setRange(slider_min, slider_max);

        values[i] = new QLineEdit;
        values[i]->setAlignment(Qt::AlignCenter);
        values[i]->setMaximumWidth(65);
        values[i]->setStyleSheet("font: 75 14pt;");

        QObject::connect(sliders[i], SIGNAL(valueChanged(int)), sliders[i], SLOT(setValue(int)), Qt::AutoConnection);
        QObject::connect(sliders[i], &QSlider::valueChanged, [this,i]() { this->updateSliderText(i); });
        QObject::connect(values[i], SIGNAL(textChanged(QString)), values[i], SLOT(setText(QString)));
        emit sliders[i]->valueChanged(0);

        main_layout->addWidget(labels[i],i,0);
        main_layout->addWidget(sliders[i],i,1);
        main_layout->addWidget(values[i],i,2);
    }
}

ViewJPosDialog::~ViewJPosDialog()
{
    stop();
}

void ViewJPosDialog::setJointNames(const std::vector<std::string> &joint_names)
{
    jnames = joint_names;
    for (int i=0; i<N_JOINTS; i++) labels[i]->setText(jnames[i].c_str());

}

void ViewJPosDialog::launch()
{
    if (!run)
    {
        run = true;
        std::thread(&ViewJPosDialog::updateDialogThread, this).detach();
        this->show();
    }
}

void ViewJPosDialog::stop()
{
    if (run) run = false;
    this->hide();
}

void ViewJPosDialog::updateDialogThread()
{
    arma::vec slider_pos(N_JOINTS);

    while (run)
    {
        arma::vec jpos = readJointsPosition()*180/3.14159;

        for (int i=0; i<N_JOINTS; i++) slider_pos(i) = jointPos2SliderPos(jpos(i), i);
        //= (jpos-jlow_lim)*(slider_max-slider_min)/(jup_lim-jlow_lim) + slider_min + 0.5;

        // std::cerr << "[ViewJPosDialog]::jpos = " << jpos.t() << "\n";

        for (int i=0;i<slider_pos.size();i++) emit sliders[i]->valueChanged((int)(slider_pos(i)));

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void ViewJPosDialog::updateSliderText(int i)
{
    double j_val = sliderPos2JointPos(sliders[i]->sliderPosition(), i);

    //values[i]->setText(QString::number(j_val,'f',1));
    emit values[i]->textChanged(QString::number(j_val,'f',1));
}

double ViewJPosDialog::sliderPos2JointPos(int s_pos, int i) const
{
    double s_min = sliders[i]->minimum();
    double s_max = sliders[i]->maximum();
    double j_min = jlow_lim(i);
    double j_max = jup_lim(i);

    double j_val = (s_pos-s_min)*(j_max-j_min)/(s_max-s_min) + j_min;

    return j_val;
}

double ViewJPosDialog::jointPos2SliderPos(double j_pos, int i) const
{
    double s_min = sliders[i]->minimum();
    double s_max = sliders[i]->maximum();
    double j_min = jlow_lim(i);
    double j_max = jup_lim(i);

    double s_pos = static_cast<int>((j_pos-j_min)*(s_max-s_min)/(j_max-j_min) + s_min + 0.5);

    return s_pos;
}

void ViewJPosDialog::closeEvent(QCloseEvent *event)
{
    stop();
}
