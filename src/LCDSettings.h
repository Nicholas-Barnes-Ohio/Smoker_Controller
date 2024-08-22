#ifndef LCDSETTINGS_H
#define LCDSETTINGS_H

#include <LCD-I2C.h>

// Initialize the LCD the same way as before
LCD_I2C lcd(0x27, 16, 4);

void initializeLCD() {
    lcd.begin();
    lcd.display();
    lcd.backlight();
}

#endif