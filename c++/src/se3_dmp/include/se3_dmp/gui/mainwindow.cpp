#include "mainwindow.h"
#include "utils.h"

#include <QDebug>

MainWindow::MainWindow(const Robot *robot, QWidget *parent): QMainWindow(parent)
{
  this->robot = robot;

  mode_name[RUN_CONTROLLER] = "RUN_CONTROLLER";
  mode_name[FREEDRIVE] = "FREEDRIVE";
  mode_name[IDLE] = "IDLE";

  //this->resize(400,350);
  this->setWindowTitle("SE(3) DMP");

  //QToolBar *tool_bar = new QToolBar(this);
  //this->addToolBar(tool_bar);
  status_bar = new QStatusBar(this);
  this->setStatusBar(status_bar);

  central_widget = new QWidget(this);
  this->setCentralWidget(central_widget);

  // ====================================

  createWidgets();

  createLayouts();

  createActions();

  createMenus();

  createConnections();

  goto_start_pose = false;
  current_pose_as_start = false;
  save_data = false;
  clear_data = false;
  is_running = true;

  default_save_data_path = "../data/recorded_data.bin";

  mode = FREEDRIVE;
  setMode(IDLE);
}

MainWindow::~MainWindow()
{

}

MainWindow::Mode MainWindow::getMode() const
{
  return mode;
}

QString MainWindow::getModeName() const
{
  return (mode_name.find(getMode()))->second;
}

void MainWindow::setMode(const Mode &m)
{
  if (getMode() == m) return;

  mode = m;

  idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");

  this->setEnabled(false);

  // updateInterface();
}

void MainWindow::updateInterface()
{
  switch (getMode())
  {
    case FREEDRIVE:
      idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
      freedrive_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
      break;

    case IDLE:
      freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
      idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0) }");

      break;
  }
}

void MainWindow::createActions()
{
  save_act = new QAction(tr("&Save"), this);
  save_act->setShortcut(QKeySequence("Ctrl+S"));
  save_act->setStatusTip(tr("Saves the recorded data to default location."));

  save_as_act = new QAction(tr("Save As..."), this);
  save_as_act->setShortcut(QKeySequence("Shift+Ctrl+S"));
  save_as_act->setStatusTip(tr("Saves the recorded data to a user specified path."));

  clear_act = new QAction(tr("&Clear"), this);
  clear_act->setShortcut(QKeySequence("Ctrl+C"));
  clear_act->setStatusTip(tr("Clears the recorded data."));

  view_pose_act = new QAction(tr("View pose"), this);
  view_pose_act->setStatusTip(tr("Opens a window displaying the robot's end-effector pose."));

  view_joints_act = new QAction(tr("View joints"), this);
  view_joints_act->setStatusTip(tr("Opens a window with sliders displaying the robot's joints position."));
}

void MainWindow::createConnections()
{
  QObject::connect( save_btn, &QPushButton::clicked, this, &MainWindow::saveTriggered );
  QObject::connect( save_act, &QAction::triggered, this, &MainWindow::saveTriggered );

  QObject::connect( save_as_btn, &QPushButton::clicked, this, &MainWindow::saveAsTriggered );
  QObject::connect( save_as_act, &QAction::triggered, this, &MainWindow::saveAsTriggered );

  QObject::connect( clear_btn, &QPushButton::clicked, this, &MainWindow::clearTriggered );
  QObject::connect( clear_act, &QAction::triggered, this, &MainWindow::clearTriggered );

  QObject::connect( view_pose_btn, &QPushButton::clicked, this, &MainWindow::viewPoseTriggered );
  QObject::connect( view_pose_act, &QAction::triggered, this, &MainWindow::viewPoseTriggered );

  QObject::connect( view_jpos_btn, &QPushButton::clicked, this, &MainWindow::viewJointsTriggered );
  QObject::connect( view_joints_act, &QAction::triggered, this, &MainWindow::viewJointsTriggered );

  QObject::connect( freedrive_btn, &QPushButton::clicked, [this](){ this->setMode(FREEDRIVE);} );
  QObject::connect( idle_btn, &QPushButton::clicked, [this](){ this->setMode(IDLE);} );

  QObject::connect( set_start_pose_btn, &QPushButton::clicked, this, &MainWindow::setStartPosePressed );
  QObject::connect( goto_start_pose_btn, &QPushButton::clicked, this, &MainWindow::gotoStartPosePressed );

  QObject::connect( emergency_stop_btn, &QPushButton::clicked, [this](){ const_cast<Robot *>(this->robot)->setExternalStop(true); this->setEnabled(false); } );

  QObject::connect( this, SIGNAL(terminateAppSignal(const QString &)), this, SLOT(terminateAppSlot(const QString &)) );
  QObject::connect( this, SIGNAL(modeChangedSignal()), this, SLOT(modeChangedSlot()) );
  QObject::connect( this, SIGNAL(gotoStartPoseAckSignal(bool , const QString &)), this, SLOT(gotoStartPoseAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(saveAckSignal(bool , const QString &)), this, SLOT(saveAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(clearAckSignal(bool , const QString &)), this, SLOT(clearAckSlot(bool , const QString &)) );
  QObject::connect( this, SIGNAL(setStartPoseAckSignal(bool , const QString &)), this, SLOT(startPoseAckSlot(bool , const QString &)) );
}

void MainWindow::createMenus()
{
  menu_bar = new QMenuBar(this);
  this->setMenuBar(menu_bar);
  menu_bar->setNativeMenuBar(false);

  file_menu = menu_bar->addMenu(tr("&File"));
  file_menu->addAction(save_act);
  file_menu->addAction(save_as_act);
  file_menu->addAction(clear_act);

  edit_menu = menu_bar->addMenu(tr("&Edit"));

  view_menu = menu_bar->addMenu(tr("&View"));
  view_menu->addAction(view_pose_act);
  view_menu->addAction(view_joints_act);
  // view_menu->addSeparator();
}

void MainWindow::createWidgets()
{
  QFont font1("Ubuntu", 13, QFont::DemiBold);
  QFont font2("Ubuntu", 15, QFont::DemiBold);

  view_pose_dialog = new ViewPoseDialog(std::bind(&Robot::getTaskPosition, robot), std::bind(&Robot::getTaskOrientation, robot), this);
  view_jpos_dialog = new ViewJPosDialog(robot->getJointsLowerLimits(), robot->getJointsUpperLimits(), std::bind(&Robot::getJointsPosition, robot), this);
  view_jpos_dialog->setJointNames(robot->getJointNames());

  mode_label = new QLabel;
  mode_label->setText("Robot mode");
  mode_label->setFont(font2);
  mode_label->setStyleSheet("background-color: rgb(250,250,250); color: rgb(0,0,200); font: 75 15pt \"FreeSans\";");
  mode_label->setSizePolicy(QSizePolicy::Preferred,QSizePolicy::Fixed);
  mode_label->setAlignment(Qt::AlignCenter);

  freedrive_btn = new QPushButton;
  freedrive_btn->setText("FREEDRIVE");
  freedrive_btn->setFont(QFont("Ubuntu", 13, QFont::DemiBold));

  idle_btn = new QPushButton;
  idle_btn->setText("IDLE");
  idle_btn->setFont(QFont("Ubuntu", 13, QFont::DemiBold));

  // ===============================

  save_btn = new QPushButton;
  save_btn->setText("save data...");
  save_btn->setFont(font1);

  save_as_btn = new QPushButton;
  save_as_btn->setText("save data as...");
  save_as_btn->setFont(font1);

  clear_btn = new QPushButton;
  clear_btn->setText("clear data");
  clear_btn->setFont(font1);

  // ===============================

  emergency_stop_btn = new QPushButton;
  emergency_stop_btn->setText("EMERGENCY\n     STOP");
  emergency_stop_btn->setMinimumSize(80,80);
  emergency_stop_btn->setIcon(QIcon(":/panic_button_icon"));
  emergency_stop_btn->setIconSize(QSize(50,50));
  emergency_stop_btn->setStyleSheet("color:rgb(255,0,0); background-color:rgba(200, 200, 200, 100);");
  emergency_stop_btn->setFont(QFont("Ubuntu",13,QFont::DemiBold));

  set_start_pose_btn = new QPushButton;
  set_start_pose_btn->setText("set start pose");
  set_start_pose_btn->setFont(font1);

  goto_start_pose_btn = new QPushButton;
  goto_start_pose_btn->setText("goto start pose");
  goto_start_pose_btn->setFont(font1);

  view_jpos_btn = new QPushButton;
  view_jpos_btn->setText("view joint pos");
  view_jpos_btn->setFont(font1);

  view_pose_btn = new QPushButton;
  view_pose_btn->setText("view cart pose");
  view_pose_btn->setFont(font1);
}

void MainWindow::createLayouts()
{
  mode_layout = new QVBoxLayout;
  mode_layout->addWidget(mode_label);
  mode_layout->addWidget(freedrive_btn);
  mode_layout->addWidget(idle_btn);
  mode_layout->addStretch();

  save_layout = new QVBoxLayout;
  save_layout->addWidget(save_btn);
  save_layout->addWidget(save_as_btn);
  save_layout->addWidget(clear_btn);
  save_layout->addStretch();

  btns_layout = new QVBoxLayout;
  btns_layout->addWidget(emergency_stop_btn);
  btns_layout->addWidget(set_start_pose_btn);
  btns_layout->addWidget(goto_start_pose_btn);
  btns_layout->addWidget(view_jpos_btn);
  btns_layout->addWidget(view_pose_btn);
  btns_layout->addStretch();

  main_layout = new QGridLayout(central_widget);
  main_layout->setSizeConstraint(QLayout::SetFixedSize);
  main_layout->addLayout(mode_layout,0,0);
  main_layout->addItem(new QSpacerItem(30,0,QSizePolicy::Fixed,QSizePolicy::Preferred),0,1);
  main_layout->addLayout(save_layout,0,2);
  main_layout->addItem(new QSpacerItem(20,0,QSizePolicy::Fixed,QSizePolicy::Preferred),0,3);
  main_layout->addLayout(btns_layout,0,4,3,1);
  main_layout->addItem(new QSpacerItem(20,10,QSizePolicy::Preferred,QSizePolicy::Fixed),1,0);
}

void MainWindow::saveTriggered()
{
  // if (getMode() != IDLE)
  // {
  //     showWarningMsg("Mode must be set to \"IDLE\" to save the recorded data.");
  //     return;
  // }

  save_data_path = default_save_data_path;
  save_data = true;
  updateInterfaceOnSaveData();
}

void MainWindow::saveAsTriggered()
{
  // if (getMode() != IDLE)
  // {
  //    showWarningMsg("Mode must be set to \"IDLE\" to save the recorded data.");
  //    return;
  // }

  QString save_as_data_path = QFileDialog::getSaveFileName(this, tr("Save Recorded Data"), QDir::currentPath(), "Binary files (*.bin)");
  if (save_as_data_path.isEmpty()) return;

  save_data_path = save_as_data_path.toStdString();
  save_data = true;
  updateInterfaceOnSaveData();
}

void MainWindow::saveAckSlot(bool success, const QString &msg)
{
  save_data = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnSaveData();
}

void MainWindow::updateInterfaceOnSaveData()
{
  bool set = !save_data();

  save_btn->setEnabled(set);
  save_act->setEnabled(set);

  save_as_btn->setEnabled(set);
  save_as_act->setEnabled(set);

  clear_btn->setEnabled(set);
  clear_act->setEnabled(set);
}

void MainWindow::clearTriggered()
{
  clear_data = true;
  updateInterfaceOnClearData();
}

void MainWindow::clearAckSlot(bool success, const QString &msg)
{
  clear_data = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnClearData();
}

void MainWindow::updateInterfaceOnClearData()
{
  bool set = !clear_data();

  save_btn->setEnabled(set);
  save_act->setEnabled(set);

  save_as_btn->setEnabled(set);
  save_as_act->setEnabled(set);

  clear_btn->setEnabled(set);
  clear_act->setEnabled(set);
}

void MainWindow::viewPoseTriggered()
{
  view_pose_dialog->launch();
}

void MainWindow::viewJointsTriggered()
{
  view_jpos_dialog->launch();
}

void MainWindow::gotoStartPosePressed()
{
  // if (record())
  // {
  //     if (showQuestionMsg("Recording is on! Do you want to stop it?") != QMessageBox::Yes) return;
  //     else setMode(IDLE);
  // }

  // if (getMode()==FREEDRIVE) setMode(IDLE);

  goto_start_pose = true;
  updateInterfaceOnGotoStartPose();
}

void MainWindow::gotoStartPoseAckSlot(bool success, const QString &msg)
{
  goto_start_pose = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  updateInterfaceOnGotoStartPose();
}

void MainWindow::updateInterfaceOnGotoStartPose()
{
  bool set = !goto_start_pose();

  freedrive_btn->setEnabled(set);
  idle_btn->setEnabled(set);
  if (set)
  {
    if (getMode()==FREEDRIVE) freedrive_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
    else if (getMode()==IDLE) idle_btn->setStyleSheet("QPushButton { color: rgb(0, 0, 250); background-color: rgb(0, 255, 0); }");
  }
  else
  {
    freedrive_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
    idle_btn->setStyleSheet("QPushButton { color: black; background-color: rgb(225, 225, 225) }");
  }

  goto_start_pose_btn->setEnabled(set);
  set_start_pose_btn->setEnabled(set);

  save_btn->setEnabled(set);
  save_act->setEnabled(set);

  save_as_btn->setEnabled(set);
  save_as_act->setEnabled(set);

  clear_btn->setEnabled(set);
  clear_act->setEnabled(set);
}

void MainWindow::setStartPosePressed()
{
  current_pose_as_start = true;
  set_start_pose_btn->setEnabled(false);
}

void MainWindow::startPoseAckSlot(bool success, const QString &msg)
{
  current_pose_as_start = false;

  success ? showInfoMsg(msg) : showErrorMsg(msg);

  update_gui_sem.notify();

  set_start_pose_btn->setEnabled(true);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  is_running = false;
  const_cast<Robot *>(robot)->setExternalStop(true);
  update_gui_sem.notify(); // unlock possible waits...
  QMainWindow::closeEvent(event);
}

void MainWindow::terminateAppSlot(const QString &msg)
{
  showErrorMsg(msg);
  this->close();
}

void MainWindow::modeChangedSlot()
{
  this->setEnabled(true);
  updateInterface();
  showInfoMsg("Mode changed to \"" + getModeName() + "\"\n");
}
