/* ----------------------------------------   CHILD-WINDOW  ----------------------------------------
 * 工具栏子窗口
 */

#ifndef CHILDWINDOWS_H
#define CHILDWINDOWS_H

#include <QDialog>
#include "public.h"
#include "Objects/objects.h"
#include "ui_humanassistance.h"

namespace Ui {
class HumanAssistance;
}
//------------------------------------------------------------    Human Assistance     ------------------------------------------------------------
class HumanAssistance : public QDialog
{
    Q_OBJECT

public:
    explicit HumanAssistance(QWidget *parent = nullptr);
    ~HumanAssistance();
    friend class RomiolSCNT;

private:
    Ui::HumanAssistance *ui;
    bool isSelecting;
private slots:
    void btnSelect();
    void btnModify();
};

#endif // CHILDWINDOWS_H
