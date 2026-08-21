#include "childwindows.h"
using namespace std;
using namespace cv;
//------------------------------------------------------------    Human Assistance     ------------------------------------------------------------
HumanAssistance::HumanAssistance(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HumanAssistance)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/NK.ico"));
    this->setWindowTitle("人工辅助");
    ui->EDIT_HOLD_DIA_UM->setValue(HOLD_DIA_UM);
    ui->EDIT_INJ_DIA_UM->setValue(INJ_DIA_UM);
    ui->EDIT_OOCYTE_DIA_UM->setValue(OOCYTE_DIA_UM);
    ui->EDIT_SOMA_DIA_UM->setValue(SOMA_DIA_UM);
    ui->EDIT_ZP_UM->setValue(ZP_WIDTH);

    ui->EDIT_HOLD_RP->setValue(holdRotateP);
    ui->EDIT_HOLD_P->setValue(holdPressure);
    ui->EDIT_HOLD_BP->setValue(holdBalanceP);
    ui->EDIT_INJ_P->setValue(injReleaseP);
    ui->EDIT_INJ_BP->setValue(injBalanceP);

    connect(ui->BTN_SELECT, SIGNAL(clicked()), this, SLOT(btnSelect()));
    connect(ui->BTN_MODIFY, SIGNAL(clicked()), this, SLOT(btnModify()));
    connect(ui->EDIT_STAGE_HEIGHT,  &QSpinBox::valueChanged, [](int v){petriDishHeight   = v;});
    connect(ui->EDIT_HOLD_DIA_UM,   &QSpinBox::valueChanged, [](int v){HOLD_DIA_UM   = v;});
    connect(ui->EDIT_INJ_DIA_UM,    &QSpinBox::valueChanged, [](int v){INJ_DIA_UM    = v;});
    connect(ui->EDIT_OOCYTE_DIA_UM, &QSpinBox::valueChanged, [](int v){OOCYTE_DIA_UM = v;});
    connect(ui->EDIT_SOMA_DIA_UM,   &QSpinBox::valueChanged, [](int v){SOMA_DIA_UM   = v;});
    connect(ui->EDIT_ZP_UM,         &QSpinBox::valueChanged, [](int v){ZP_WIDTH      = v;});

    connect(ui->EDIT_HOLD_RP,        &QSpinBox::valueChanged, [](int v){holdRotateP  = v;});
    connect(ui->EDIT_HOLD_P,        &QSpinBox::valueChanged, [](int v){holdPressure  = v;});
    connect(ui->EDIT_HOLD_BP,       &QSpinBox::valueChanged, [](int v){holdBalanceP  = v;});
    connect(ui->EDIT_INJ_P,         &QSpinBox::valueChanged, [](int v){injReleaseP   = v;});
    connect(ui->EDIT_INJ_BP,        &QSpinBox::valueChanged, [](int v){injBalanceP   = v;});
}
HumanAssistance::~HumanAssistance()
{
    delete ui;
}
void HumanAssistance::btnSelect()
{
    selectPoints.clear();
    isSelecting = true;
}
void HumanAssistance::btnModify()
{
//    convexHull(selectPoints, selectPoints);
    QString str = ui->CMB_LIST->currentText();
    if(str=="吸持针")
    {
        if(selectPoints.size()==2)
        {
            selectPoints.push_back(Point(0, selectPoints[1].y));
            selectPoints.push_back(Point(0, selectPoints[0].y));
        }
        holdPipette->set(selectPoints);
    }
    else if(str=="GLI")
    {
        injectPipette->xGLI = selectPoints[0].x;
    }
    else if(str=="注射针")
    {
        if(selectPoints.size()==2)
        {
            selectPoints.push_back(Point(WIDTH-1, selectPoints[1].y));
            selectPoints.push_back(Point(WIDTH-1, selectPoints[0].y));
        }
        injectPipette->set(selectPoints);
    }
    else if(str=="卵细胞" && selectPoints.size()==2)
    {
        vector<Rect> oocytesRects;
        Point p1, p2;
        int x0,y0,w,h;
        p1 = selectPoints[0];
        p2 = selectPoints[1];
        x0 = min(p1.x, p2.x);   y0 = min(p1.y,p2.y);    w = abs(p1.x-p2.x); h = abs(p1.y-p2.y);
        Rect rect(x0, y0, w, h);
        oocytesRects.push_back(rect);
        currentOocyte->find(CurrentGray, oocytesRects);
    }
    else if(str=="体细胞")
    {

    }
    isSelecting = false;
}
