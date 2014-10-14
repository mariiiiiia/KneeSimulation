/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.3.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "musclelabel.h"
#include "qwt_plot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QSlider *slider_target_dist;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_6;
    QVBoxLayout *verticalLayout_5;
    QGroupBox *group_activation_controls;
    QGridLayout *gridLayout_2;
    MuscleLabel *label_inf_rec;
    MuscleLabel *label_sup_rec;
    QSlider *slider_superior_rectus;
    QPushButton *btn_sup_rec_inc;
    QPushButton *btn_inf_rec_dec;
    QSlider *slider_lateral_rectus;
    QPushButton *btn_lat_rec_inc;
    MuscleLabel *label_sup_obl;
    QSlider *slider_superior_oblique;
    QPushButton *btn_med_rec_inc;
    QPushButton *btn_sup_rec_dec;
    MuscleLabel *label_lat_rec;
    QPushButton *btn_sup_obl_dec;
    QPushButton *btn_lat_rec_dec;
    MuscleLabel *label_med_rec;
    QPushButton *btn_sup_obl_inc;
    QSlider *slider_inferior_rectus;
    QSlider *slider_medial_rectus;
    QPushButton *btn_inf_rec_inc;
    QPushButton *btn_med_rec_dec;
    MuscleLabel *label_inf_obl;
    QPushButton *btn_inf_obl_inc;
    QPushButton *btn_inf_obl_dec;
    QSlider *slider_inferior_oblique;
    QwtPlot *qwtPlot;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_3;
    QScrollArea *scroll_settings;
    QWidget *scrollAreaWidgetContents_2;
    QVBoxLayout *verticalLayout_4;
    QFormLayout *form_settings;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(682, 467);
        MainWindow->setMinimumSize(QSize(0, 467));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        scrollArea = new QScrollArea(centralwidget);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 435, 376));
        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout->addWidget(scrollArea);

        slider_target_dist = new QSlider(centralwidget);
        slider_target_dist->setObjectName(QStringLiteral("slider_target_dist"));
        slider_target_dist->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(slider_target_dist);


        horizontalLayout->addLayout(verticalLayout);

        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        verticalLayout_6 = new QVBoxLayout(tab);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        group_activation_controls = new QGroupBox(tab);
        group_activation_controls->setObjectName(QStringLiteral("group_activation_controls"));
        gridLayout_2 = new QGridLayout(group_activation_controls);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        label_inf_rec = new MuscleLabel(group_activation_controls);
        label_inf_rec->setObjectName(QStringLiteral("label_inf_rec"));
        label_inf_rec->setCursor(QCursor(Qt::PointingHandCursor));
        label_inf_rec->setStyleSheet(QStringLiteral(""));

        gridLayout_2->addWidget(label_inf_rec, 2, 0, 1, 1);

        label_sup_rec = new MuscleLabel(group_activation_controls);
        label_sup_rec->setObjectName(QStringLiteral("label_sup_rec"));
        label_sup_rec->setCursor(QCursor(Qt::PointingHandCursor));
        label_sup_rec->setStyleSheet(QStringLiteral(""));

        gridLayout_2->addWidget(label_sup_rec, 3, 0, 1, 1);

        slider_superior_rectus = new QSlider(group_activation_controls);
        slider_superior_rectus->setObjectName(QStringLiteral("slider_superior_rectus"));
        slider_superior_rectus->setEnabled(true);
        slider_superior_rectus->setMaximum(100);
        slider_superior_rectus->setSingleStep(1);
        slider_superior_rectus->setOrientation(Qt::Horizontal);
        slider_superior_rectus->setTickPosition(QSlider::TicksBelow);
        slider_superior_rectus->setTickInterval(5);

        gridLayout_2->addWidget(slider_superior_rectus, 3, 2, 1, 1);

        btn_sup_rec_inc = new QPushButton(group_activation_controls);
        btn_sup_rec_inc->setObjectName(QStringLiteral("btn_sup_rec_inc"));
        btn_sup_rec_inc->setMaximumSize(QSize(20, 30));
        QFont font;
        font.setPointSize(12);
        font.setBold(true);
        font.setWeight(75);
        font.setKerning(true);
        btn_sup_rec_inc->setFont(font);

        gridLayout_2->addWidget(btn_sup_rec_inc, 3, 3, 1, 1);

        btn_inf_rec_dec = new QPushButton(group_activation_controls);
        btn_inf_rec_dec->setObjectName(QStringLiteral("btn_inf_rec_dec"));
        btn_inf_rec_dec->setMaximumSize(QSize(20, 30));
        btn_inf_rec_dec->setFont(font);

        gridLayout_2->addWidget(btn_inf_rec_dec, 2, 1, 1, 1);

        slider_lateral_rectus = new QSlider(group_activation_controls);
        slider_lateral_rectus->setObjectName(QStringLiteral("slider_lateral_rectus"));
        slider_lateral_rectus->setEnabled(true);
        slider_lateral_rectus->setMaximum(100);
        slider_lateral_rectus->setSingleStep(1);
        slider_lateral_rectus->setOrientation(Qt::Horizontal);
        slider_lateral_rectus->setTickPosition(QSlider::TicksBelow);
        slider_lateral_rectus->setTickInterval(5);

        gridLayout_2->addWidget(slider_lateral_rectus, 1, 2, 1, 1);

        btn_lat_rec_inc = new QPushButton(group_activation_controls);
        btn_lat_rec_inc->setObjectName(QStringLiteral("btn_lat_rec_inc"));
        btn_lat_rec_inc->setMaximumSize(QSize(20, 30));
        btn_lat_rec_inc->setFont(font);

        gridLayout_2->addWidget(btn_lat_rec_inc, 1, 3, 1, 1);

        label_sup_obl = new MuscleLabel(group_activation_controls);
        label_sup_obl->setObjectName(QStringLiteral("label_sup_obl"));
        label_sup_obl->setCursor(QCursor(Qt::PointingHandCursor));
        label_sup_obl->setStyleSheet(QStringLiteral(""));

        gridLayout_2->addWidget(label_sup_obl, 4, 0, 1, 1);

        slider_superior_oblique = new QSlider(group_activation_controls);
        slider_superior_oblique->setObjectName(QStringLiteral("slider_superior_oblique"));
        slider_superior_oblique->setEnabled(true);
        slider_superior_oblique->setMaximum(100);
        slider_superior_oblique->setSingleStep(1);
        slider_superior_oblique->setOrientation(Qt::Horizontal);
        slider_superior_oblique->setTickPosition(QSlider::TicksBelow);
        slider_superior_oblique->setTickInterval(5);

        gridLayout_2->addWidget(slider_superior_oblique, 4, 2, 1, 1);

        btn_med_rec_inc = new QPushButton(group_activation_controls);
        btn_med_rec_inc->setObjectName(QStringLiteral("btn_med_rec_inc"));
        btn_med_rec_inc->setMaximumSize(QSize(20, 30));
        btn_med_rec_inc->setFont(font);

        gridLayout_2->addWidget(btn_med_rec_inc, 0, 3, 1, 1);

        btn_sup_rec_dec = new QPushButton(group_activation_controls);
        btn_sup_rec_dec->setObjectName(QStringLiteral("btn_sup_rec_dec"));
        btn_sup_rec_dec->setMaximumSize(QSize(20, 30));
        btn_sup_rec_dec->setFont(font);

        gridLayout_2->addWidget(btn_sup_rec_dec, 3, 1, 1, 1);

        label_lat_rec = new MuscleLabel(group_activation_controls);
        label_lat_rec->setObjectName(QStringLiteral("label_lat_rec"));
        label_lat_rec->setCursor(QCursor(Qt::PointingHandCursor));
        label_lat_rec->setStyleSheet(QStringLiteral(""));

        gridLayout_2->addWidget(label_lat_rec, 1, 0, 1, 1);

        btn_sup_obl_dec = new QPushButton(group_activation_controls);
        btn_sup_obl_dec->setObjectName(QStringLiteral("btn_sup_obl_dec"));
        btn_sup_obl_dec->setMaximumSize(QSize(20, 30));
        btn_sup_obl_dec->setFont(font);

        gridLayout_2->addWidget(btn_sup_obl_dec, 4, 1, 1, 1);

        btn_lat_rec_dec = new QPushButton(group_activation_controls);
        btn_lat_rec_dec->setObjectName(QStringLiteral("btn_lat_rec_dec"));
        btn_lat_rec_dec->setMaximumSize(QSize(20, 30));
        btn_lat_rec_dec->setFont(font);

        gridLayout_2->addWidget(btn_lat_rec_dec, 1, 1, 1, 1);

        label_med_rec = new MuscleLabel(group_activation_controls);
        label_med_rec->setObjectName(QStringLiteral("label_med_rec"));
        label_med_rec->setCursor(QCursor(Qt::PointingHandCursor));
        label_med_rec->setStyleSheet(QStringLiteral(""));

        gridLayout_2->addWidget(label_med_rec, 0, 0, 1, 1);

        btn_sup_obl_inc = new QPushButton(group_activation_controls);
        btn_sup_obl_inc->setObjectName(QStringLiteral("btn_sup_obl_inc"));
        btn_sup_obl_inc->setMaximumSize(QSize(20, 30));
        btn_sup_obl_inc->setFont(font);

        gridLayout_2->addWidget(btn_sup_obl_inc, 4, 3, 1, 1);

        slider_inferior_rectus = new QSlider(group_activation_controls);
        slider_inferior_rectus->setObjectName(QStringLiteral("slider_inferior_rectus"));
        slider_inferior_rectus->setEnabled(true);
        slider_inferior_rectus->setMaximum(100);
        slider_inferior_rectus->setSingleStep(1);
        slider_inferior_rectus->setOrientation(Qt::Horizontal);
        slider_inferior_rectus->setTickPosition(QSlider::TicksBelow);
        slider_inferior_rectus->setTickInterval(5);

        gridLayout_2->addWidget(slider_inferior_rectus, 2, 2, 1, 1);

        slider_medial_rectus = new QSlider(group_activation_controls);
        slider_medial_rectus->setObjectName(QStringLiteral("slider_medial_rectus"));
        slider_medial_rectus->setEnabled(true);
        slider_medial_rectus->setMaximum(100);
        slider_medial_rectus->setSingleStep(1);
        slider_medial_rectus->setOrientation(Qt::Horizontal);
        slider_medial_rectus->setTickPosition(QSlider::TicksBelow);
        slider_medial_rectus->setTickInterval(5);

        gridLayout_2->addWidget(slider_medial_rectus, 0, 2, 1, 1);

        btn_inf_rec_inc = new QPushButton(group_activation_controls);
        btn_inf_rec_inc->setObjectName(QStringLiteral("btn_inf_rec_inc"));
        btn_inf_rec_inc->setMaximumSize(QSize(20, 30));
        btn_inf_rec_inc->setFont(font);

        gridLayout_2->addWidget(btn_inf_rec_inc, 2, 3, 1, 1);

        btn_med_rec_dec = new QPushButton(group_activation_controls);
        btn_med_rec_dec->setObjectName(QStringLiteral("btn_med_rec_dec"));
        btn_med_rec_dec->setMaximumSize(QSize(20, 30));
        btn_med_rec_dec->setFont(font);

        gridLayout_2->addWidget(btn_med_rec_dec, 0, 1, 1, 1);

        label_inf_obl = new MuscleLabel(group_activation_controls);
        label_inf_obl->setObjectName(QStringLiteral("label_inf_obl"));
        label_inf_obl->setCursor(QCursor(Qt::PointingHandCursor));
        label_inf_obl->setStyleSheet(QStringLiteral(""));

        gridLayout_2->addWidget(label_inf_obl, 5, 0, 1, 1);

        btn_inf_obl_inc = new QPushButton(group_activation_controls);
        btn_inf_obl_inc->setObjectName(QStringLiteral("btn_inf_obl_inc"));
        btn_inf_obl_inc->setMaximumSize(QSize(20, 30));
        btn_inf_obl_inc->setFont(font);

        gridLayout_2->addWidget(btn_inf_obl_inc, 5, 3, 1, 1);

        btn_inf_obl_dec = new QPushButton(group_activation_controls);
        btn_inf_obl_dec->setObjectName(QStringLiteral("btn_inf_obl_dec"));
        btn_inf_obl_dec->setMaximumSize(QSize(20, 30));
        btn_inf_obl_dec->setFont(font);

        gridLayout_2->addWidget(btn_inf_obl_dec, 5, 1, 1, 1);

        slider_inferior_oblique = new QSlider(group_activation_controls);
        slider_inferior_oblique->setObjectName(QStringLiteral("slider_inferior_oblique"));
        slider_inferior_oblique->setEnabled(true);
        slider_inferior_oblique->setMaximum(100);
        slider_inferior_oblique->setSingleStep(1);
        slider_inferior_oblique->setOrientation(Qt::Horizontal);
        slider_inferior_oblique->setTickPosition(QSlider::TicksBelow);
        slider_inferior_oblique->setTickInterval(5);

        gridLayout_2->addWidget(slider_inferior_oblique, 5, 2, 1, 1);


        verticalLayout_5->addWidget(group_activation_controls);


        verticalLayout_6->addLayout(verticalLayout_5);

        qwtPlot = new QwtPlot(tab);
        qwtPlot->setObjectName(QStringLiteral("qwtPlot"));

        verticalLayout_6->addWidget(qwtPlot);

        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        verticalLayout_3 = new QVBoxLayout(tab_2);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        scroll_settings = new QScrollArea(tab_2);
        scroll_settings->setObjectName(QStringLiteral("scroll_settings"));
        scroll_settings->setWidgetResizable(true);
        scrollAreaWidgetContents_2 = new QWidget();
        scrollAreaWidgetContents_2->setObjectName(QStringLiteral("scrollAreaWidgetContents_2"));
        scrollAreaWidgetContents_2->setGeometry(QRect(0, 0, 98, 28));
        verticalLayout_4 = new QVBoxLayout(scrollAreaWidgetContents_2);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        form_settings = new QFormLayout();
        form_settings->setObjectName(QStringLiteral("form_settings"));

        verticalLayout_4->addLayout(form_settings);

        scroll_settings->setWidget(scrollAreaWidgetContents_2);

        verticalLayout_3->addWidget(scroll_settings);

        tabWidget->addTab(tab_2, QString());

        horizontalLayout->addWidget(tabWidget);

        horizontalLayout->setStretch(0, 5);
        horizontalLayout->setStretch(1, 2);
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 682, 21));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
#ifndef QT_NO_TOOLTIP
        slider_target_dist->setToolTip(QApplication::translate("MainWindow", "Adjust visual target distance", 0));
#endif // QT_NO_TOOLTIP
        group_activation_controls->setTitle(QApplication::translate("MainWindow", "Right Extraocular Muscle Activations", 0));
        label_inf_rec->setText(QApplication::translate("MainWindow", "Inferior Rectus", 0));
        label_sup_rec->setText(QApplication::translate("MainWindow", "Superior Rectus", 0));
        btn_sup_rec_inc->setText(QApplication::translate("MainWindow", "+", 0));
        btn_inf_rec_dec->setText(QApplication::translate("MainWindow", "-", 0));
        btn_lat_rec_inc->setText(QApplication::translate("MainWindow", "+", 0));
        label_sup_obl->setText(QApplication::translate("MainWindow", "Superior Oblique", 0));
        btn_med_rec_inc->setText(QApplication::translate("MainWindow", "+", 0));
        btn_sup_rec_dec->setText(QApplication::translate("MainWindow", "-", 0));
        label_lat_rec->setText(QApplication::translate("MainWindow", "Lateral Rectus", 0));
        btn_sup_obl_dec->setText(QApplication::translate("MainWindow", "-", 0));
        btn_lat_rec_dec->setText(QApplication::translate("MainWindow", "-", 0));
        label_med_rec->setText(QApplication::translate("MainWindow", "Medial Rectus", 0));
        btn_sup_obl_inc->setText(QApplication::translate("MainWindow", "+", 0));
        btn_inf_rec_inc->setText(QApplication::translate("MainWindow", "+", 0));
        btn_med_rec_dec->setText(QApplication::translate("MainWindow", "-", 0));
        label_inf_obl->setText(QApplication::translate("MainWindow", "Inferior Oblique", 0));
        btn_inf_obl_inc->setText(QApplication::translate("MainWindow", "+", 0));
        btn_inf_obl_dec->setText(QApplication::translate("MainWindow", "-", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "Right Eye", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Settings", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
