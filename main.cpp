#include <QApplication>
#include <QMainWindow>
#include "pclQviewer.h"
int main (int argc, char *argv[])
{
    QApplication a (argc, argv);
    pclQviewer w;
    w.show ();
    return a.exec ();
}
