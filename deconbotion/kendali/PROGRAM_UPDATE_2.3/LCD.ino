void tampil(uint8_t x,uint8_t y, const char* fmtstr, ...)
{
    char lcd_string[21];
    va_list ap;

    va_start(ap, fmtstr);
    vsprintf(lcd_string, fmtstr, ap);
    va_end(ap);

    lcd.setCursor(x, y);
    lcd.print(lcd_string);
}
