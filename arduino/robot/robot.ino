#include "mecunum.h"

mec::Mecanum mecanum;

void setup() {

}

void loop() {
    mecanum.move(1 , 0, 0);
    delay(10);

}