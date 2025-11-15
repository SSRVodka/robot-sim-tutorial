
#pragma once

#include <QtCore/QObject>

#include <sys_stat_if/msg/system_stat.hpp>

#include "ui_main_window.h"


extern "C" void update_my_window(sys_stat_if::msg::SystemStat stat);


class SSMainWindow: public QMainWindow, public Ui::MainWindow {
    Q_OBJECT
public:
    explicit SSMainWindow(QObject *parent = nullptr);
    ~SSMainWindow();

    static SSMainWindow *get_instance();

    void stat_update();
};

