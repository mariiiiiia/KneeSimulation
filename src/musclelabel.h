#ifndef MUSCLELABEL_H
#define MUSCLELABEL_H

#include <QLabel>
#include <QMouseEvent>

class MuscleLabel : public QLabel
{
    Q_OBJECT

signals:
    void clicked();

protected:
    virtual void mousePressEvent(QMouseEvent*) {emit clicked();}

public:
    MuscleLabel(QWidget *parent) : QLabel(parent) {}
};

#endif
