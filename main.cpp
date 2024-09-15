#include "cs_usv.h"
#include <QCoreApplication>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    CS_USV cs_usv;
    return a.exec();
}
