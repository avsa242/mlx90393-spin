{
    --------------------------------------------
    Filename: MLX90393-Demo.spin
    Author: Jesse Burt
    Description: Demo of the MLX90393 driver
    Copyright (c) 2021
    Started Aug 27, 2020
    Updated Mar 14, 2021
    See end of file for terms of use.
    --------------------------------------------
}
CON

    _clkmode    = cfg#_clkmode
    _xinfreq    = cfg#_xinfreq

' -- User-modifiable constants
    LED         = cfg#LED1
    SER_BAUD    = 115_200

    SCL_PIN     = 28
    SDA_PIN     = 29
    I2C_HZ      = 400_000                       ' max is 400_000
    INT_PIN     = 22                            ' required (data ready flag)

    STANDARD    = GAUSS                         ' GAUSS or TESLA
' --

    DAT_X_COL   = 20
    DAT_Y_COL   = DAT_X_COL + 15
    DAT_Z_COL   = DAT_Y_COL + 15
    GAUSS       = 0
    TESLA       = 1

OBJ

    cfg     : "core.con.boardcfg.flip"
    ser     : "com.serial.terminal.ansi"
    time    : "time"
    int     : "string.integer"
    mag     : "sensor.magnetometer.3dof.mlx90393.i2c"

PUB Main{}

    setup{}
    mag.preset_active{}                         ' default settings, but enable
                                                ' sensor acquisition and set
                                                ' scale factor
    case STANDARD
        GAUSS:
            repeat
                ser.position(0, 3)
                maggauss{}

                if ser.rxcheck{} == "c"         ' press the 'c' key in the demo
                    calibrate{}                 ' to calibrate sensor offsets
        TESLA:
            repeat
                ser.position(0, 3)
                magtesla{}

                if ser.rxcheck{} == "c"
                    calibrate{}

    repeat

PUB MagGauss{} | mx, my, mz

    repeat until mag.magdataready{}             ' wait for new sensor data set
    mag.maggauss(@mx, @my, @mz)                 ' read calculated sensor data
    ser.str(string("Mag (Gs):"))
    ser.positionx(DAT_X_COL)
    decimal(mx, 100_000)                        ' data is in 1/100_000's Gauss;
    ser.positionx(DAT_Y_COL)                    ' disp. as if it were a float
    decimal(my, 100_000)
    ser.positionx(DAT_Z_COL)
    decimal(mz, 100_000)
    ser.clearline{}
    ser.newline{}

PUB MagTesla{} | mx, my, mz

    repeat until mag.magdataready{}             ' wait for new sensor data set
    mag.magtesla(@mx, @my, @mz)                 ' read calculated sensor data
    ser.str(string("Mag (uT):"))
    ser.positionx(DAT_X_COL)
    decimal(mx, 100_000)                        ' data is in 1/1000's uTesla;
    ser.positionx(DAT_Y_COL)                    ' disp. as if it were a float
    decimal(my, 100_000)
    ser.positionx(DAT_Z_COL)
    decimal(mz, 100_000)
    ser.clearline{}
    ser.newline{}

PUB Calibrate{} ' XXX TODO

    ser.position(0, 7)
    ser.str(string("Calibrating..."))
    mag.calibratemag{}
    ser.positionx(0)
    ser.clearline{}

PRI Decimal(scaled, divisor) | whole[4], part[4], places, tmp, sign
' Display a scaled up number as a decimal
'   Scale it back down by divisor (e.g., 10, 100, 1000, etc)
    whole := scaled / divisor
    tmp := divisor
    places := 0
    part := 0
    sign := 0
    if scaled < 0
        sign := "-"
    else
        sign := " "

    repeat
        tmp /= 10
        places++
    until tmp == 1
    scaled //= divisor
    part := int.deczeroed(||(scaled), places)

    ser.char(sign)
    ser.dec(||(whole))
    ser.char(".")
    ser.str(part)
    ser.chars(" ", 5)

PUB Setup{}

    ser.start(SER_BAUD)
    time.msleep(30)
    ser.clear{}
    ser.strln(string("Serial terminal started"))
    if mag.startx(SCL_PIN, SDA_PIN, I2C_HZ, INT_PIN)
        ser.strln(string("MLX90393 driver started"))
    else
        ser.strln(string("MLX90393 driver failed to start - halting"))
        time.msleep(5)
        repeat

DAT
{
    --------------------------------------------------------------------------------------------------------
    TERMS OF USE: MIT License

    Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
    associated documentation files (the "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
    following conditions:

    The above copyright notice and this permission notice shall be included in all copies or substantial
    portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
    LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
    SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
    --------------------------------------------------------------------------------------------------------
}
