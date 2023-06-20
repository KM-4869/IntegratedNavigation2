#include"consoloe.h"

void SetConsoleCurorXY(int y, int x)
{
    COORD  coord;
    coord.X = x;
    coord.Y = y;
    HANDLE a = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleCursorPosition(a, coord);
    
}