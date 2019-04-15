#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QMenu>
#include <QPushButton>
#include <QLabel>
#include <QAction>
#include <QWidget>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPalette>
#include <QColor>
#include <QMessageBox>
#include <QFileDialog>

#include <map>

#include "view_pose_dialog.h"
#include "view_jpos_dialog.h"
#include <se3_dmp/utils.h>
#include <se3_dmp/robot/robot.h>

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    enum Mode
    {
      RUN_CONTROLLER,
      FREEDRIVE,
      IDLE
    };

    MainWindow(const Robot *robot, QWidget *parent = 0);
    ~MainWindow();

    void setMode(const Mode &m);
    MainWindow::Mode getMode() const;
    QString getModeName() const;

    bool isRunning() const { return is_running; }

    bool currentPoseAsStart() const { return current_pose_as_start(); }
    void sendSetStartPoseAck(bool success, const QString &msg)  { emit setStartPoseAckSignal(success,msg); update_gui_sem.wait(); }

    bool gotoStartPose() const { return goto_start_pose(); }
    void sendGotoStartPoseAck(bool success, const QString &msg) { emit gotoStartPoseAckSignal(success,msg); update_gui_sem.wait(); }

    bool saveData() const { return save_data(); }
    void sendSaveAck(bool success, const QString &msg) { emit saveAckSignal(success,msg); update_gui_sem.wait(); }
    std::string getSaveDataPath() const { return save_data_path; }

    bool clearData() const { return clear_data(); }
    void sendClearAck(bool success, const QString &msg) { emit clearAckSignal(success,msg); update_gui_sem.wait(); }

signals:
    void terminateAppSignal(const QString &msg);
    void modeChangedSignal();
    void saveAckSignal(bool success, const QString &msg);
    void clearAckSignal(bool success, const QString &msg);
    void gotoStartPoseAckSignal(bool success, const QString &msg);
    void setStartPoseAckSignal(bool success, const QString &msg);

private slots:
    void saveTriggered();
    void saveAsTriggered();
    void clearTriggered();
    void viewPoseTriggered();
    void viewJointsTriggered();
    void gotoStartPosePressed();
    void setStartPosePressed();

    void saveAckSlot(bool success, const QString &msg);
    void clearAckSlot(bool success, const QString &msg);
    void startPoseAckSlot(bool success, const QString &msg);
    void gotoStartPoseAckSlot(bool success, const QString &msg);

    void terminateAppSlot(const QString &msg);
    void modeChangedSlot();

private:
    QWidget *central_widget;
    QStatusBar *status_bar;

    std::map<Mode, QString> mode_name;

    Semaphore update_gui_sem;

    Mode mode;

    bool is_running;

    MtxVar<bool> goto_start_pose;
    MtxVar<bool> current_pose_as_start;
    MtxVar<bool> save_data;
    MtxVar<bool> clear_data;
    std::string save_data_path;
    std::string default_save_data_path;

    const Robot *robot;
    ViewPoseDialog *view_pose_dialog;
    ViewJPosDialog *view_jpos_dialog;

    // ======  menus  ========
    QMenuBar *menu_bar;
    QMenu *file_menu;
    QMenu *edit_menu;
    QMenu *view_menu;

    // ======  actions  ========

    QAction *save_act;
    QAction *save_as_act;
    QAction *clear_act;

    QAction *view_pose_act;
    QAction *view_joints_act;

    // ======  widgets  ========
    QLabel *mode_label;
    QPushButton *freedrive_btn;
    QPushButton *idle_btn;

    QPushButton *save_btn;
    QPushButton *save_as_btn;
    QPushButton *clear_btn;

    QPushButton *emergency_stop_btn;
    QPushButton *set_start_pose_btn;
    QPushButton *goto_start_pose_btn;
    QPushButton *view_jpos_btn;
    QPushButton *view_pose_btn;

    // ======  layouts  ========
    QVBoxLayout *mode_layout;
    QVBoxLayout *save_layout;
    QVBoxLayout *btns_layout;
    QGridLayout *main_layout;

    // ======  functions  ========
    void createWidgets();
    void createLayouts();
    void createConnections();
    void createActions();
    void createMenus();
    void updateInterface();

    void updateInterfaceOnGotoStartPose();
    void updateInterfaceOnSaveData();
    void updateInterfaceOnClearData();

    void closeEvent(QCloseEvent *event) override;
};

#endif // MAINWINDOW_H
