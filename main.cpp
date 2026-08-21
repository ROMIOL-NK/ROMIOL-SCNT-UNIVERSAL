#include "romiolscnt.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RomiolSCNT w;
    w.move(690, 20);
    w.show();
    return a.exec();
}
