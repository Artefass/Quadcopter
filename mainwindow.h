#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

//#include "quadcopters/quadcopters_group.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_startSimulationButton_clicked();

    void on_pushButton_5_clicked();

private:
    Ui::MainWindow *ui;

//    QuadcoptersGroup Quadcopters;
};

#endif // MAINWINDOW_H
