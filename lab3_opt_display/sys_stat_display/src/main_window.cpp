
#include "main_window.h"


sys_stat_if::msg::SystemStat g_sys_stat;

SSMainWindow::SSMainWindow(QObject *)
    : QMainWindow(nullptr) {
    this->setupUi(this);

    this->cpu_percent_p->setRange(0, 100);
    this->mem_percent_p->setRange(0, 100);
    this->disk_percent_p->setRange(0, 100);

    this->cpu_percent_p->setValue(0);
    this->mem_percent_p->setValue(0);
    this->disk_percent_p->setValue(0);
}

SSMainWindow::~SSMainWindow() {}

SSMainWindow *SSMainWindow::get_instance() {
    static SSMainWindow s_instance;
    return &s_instance;
}

void SSMainWindow::stat_update() {
    this->hostname->setText(QString::fromStdString(g_sys_stat.hostname));
    this->cpu_percent_p->setValue(g_sys_stat.cpu_percent);
    this->mem_percent_p->setValue(g_sys_stat.mem_percent);
    this->disk_percent_p->setValue(g_sys_stat.disk_percent);
    this->net_sent->setText(QString::asprintf("%.2f", g_sys_stat.net_sent));
    this->net_recv->setText(QString::asprintf("%.2f", g_sys_stat.net_recv));
}


void update_my_window(sys_stat_if::msg::SystemStat stat) {
    g_sys_stat = stat;
    
    QMetaObject::invokeMethod(
        SSMainWindow::get_instance(),
        std::bind(&SSMainWindow::stat_update, SSMainWindow::get_instance()));
}
