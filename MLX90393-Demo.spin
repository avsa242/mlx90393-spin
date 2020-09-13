{
    --------------------------------------------
    Filename: MLX90393-Demo.spin
    Author: Jesse Burt
    Description: Demo of the MLX90393 driver
    Copyright (c) 2020
    Started Aud 27, 2020
    Updated Sep 13, 2020
    See end of file for terms of use.
    --------------------------------------------
}
CON

    _clkmode    = cfg#_clkmode
    _xinfreq    = cfg#_xinfreq

' -- User-modifiable constants
    LED         = cfg#LED1
    SER_RX      = 31
    SER_TX      = 30
    SER_BAUD    = 115_200

    I2C_SCL     = 24
    I2C_SDA     = 25
    I2C_HZ      = 400_000
' --

    DAT_X_COL   = 20
    DAT_Y_COL   = DAT_X_COL + 15
    DAT_Z_COL   = DAT_Y_COL + 15

OBJ

    cfg     : "core.con.boardcfg.flip"
    ser     : "com.serial.terminal.ansi"
    time    : "time"
    io      : "io"
    int     : "string.integer"
    mag     : "sensor.magnetometer.3dof.mlx90393.i2c"

VAR

    long _overruns

PUB Main{} | dispmode

    setup{}
    mag.magscale(16)                                        ' 14, 16 (bits)
    mag.magbias(0, 0, 0, 1)

    ser.hidecursor{}
    dispmode := 0

    displaysettings{}

    repeat
        case ser.rxcheck{}
            "q", "Q":                                       ' Quit the demo
                ser.position(0, 15)
                ser.str(string("Halting"))
                mag.stop{}
                time.msleep(5)
                quit
            "c", "C":                                       ' Perform calibration
                calibrate{}
                displaysettings{}
            "r", "R":                                       ' Change display mode: raw/calculated
                ser.position(0, 10)
                repeat 2
                    ser.clearline{}
                    ser.newline{}
                dispmode ^= 1

        case dispmode
            0:
                ser.position(0, 12)
                magraw{}
                tempraw{}
            1:
                ser.position(0, 12)
                magcalc{}
                tempcalc{}

    ser.showcursor{}
    flashled(LED, 100)

PUB MagCalc{} | mx, my, mz
' TODO
    repeat until mag.magdataready{}
    mag.measuremag{}
    mag.maggauss (@mx, @my, @mz)
    ser.str(string("Mag gauss:   "))
    ser.position(DAT_X_COL, 12)
    decimal(mx, 1000)
    ser.position(DAT_Y_COL, 12)
    decimal(my, 1000)
    ser.position(DAT_Z_COL, 12)
    decimal(mz, 1000)
    ser.clearline{}
    ser.newline{}

PUB MagRaw{} | mx, my, mz

    repeat until mag.magdataready{}
    mag.measuremag{}
    mag.magdata (@mx, @my, @mz)
    ser.str(string("Mag raw:  "))
    ser.position(DAT_X_COL, 12)
    ser.str (int.decpadded (mx, 7))
    ser.position(DAT_Y_COL, 12)
    ser.str (int.decpadded (my, 7))
    ser.position(DAT_Z_COL, 12)
    ser.str (int.decpadded (mz, 7))
    ser.clearline{}
    ser.newline{}

PUB TempCalc{} | t
' TODO
    t := mag.temperature{}
    ser.str(string("Temp calc: "))
    ser.position(DAT_X_COL, 13)
    decimal(t, 100)

PUB TempRaw{} | t

    t := mag.tempdata{}
    ser.str(string("Temp raw: "))
    ser.position(DAT_X_COL, 13)
    ser.hex(t, 4)

PUB Calibrate{}
' TODO
    ser.position (0, 21)
    ser.str(string("Calibrating..."))
    mag.calibratemag{}
    ser.position (0, 21)
    ser.str(string("              "))

PUB DisplaySettings{} | axo, ayo, azo, gxo, gyo, gzo, mxo, myo, mzo

    ser.position(0, 3)
    mag.magbias(@mxo, @myo, @mzo, 0)
    ser.str(string("MagBias: "))
    ser.dec(mxo)
    ser.str(string("(x), "))
    ser.dec(myo)
    ser.str(string("(y), "))
    ser.dec(mzo)
    ser.str(string("(z)"))
    ser.newline{}

    ser.str(string("MagScale: "))
    ser.dec(mag.magscale(-2))
    ser.newline{}

PRI Decimal(scaled, divisor) | whole[4], part[4], places, tmp
' Display a fixed-point scaled up number in decimal-dot notation - scale it back down by divisor
'   e.g., Decimal (314159, 100000) would display 3.14159 on the termainl
'   scaled: Fixed-point scaled up number
'   divisor: Divide scaled-up number by this amount
    whole := scaled / divisor
    tmp := divisor
    places := 0

    repeat
        tmp /= 10
        places++
    until tmp == 1
    part := int.deczeroed(||(scaled // divisor), places)

    ser.dec (whole)
    ser.char (".")
    ser.str (part)

PUB Setup{}

    repeat until ser.startrxtx(SER_RX, SER_TX, 0, SER_BAUD)
    time.msleep(30)
    ser.clear{}
    ser.strln(string("Serial terminal started"))
    if mag.startx(I2C_SCL, I2C_SDA, I2C_HZ, -1)
        mag.defaults{}
        ser.strln(string("MLX90393 driver started (I2C)"))
    else
        ser.strln(string("MLX90393 driver failed to start - halting"))
        mag.stop{}
        time.msleep(5)
        flashled(LED, 500)


#include "lib.utility.spin"

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
