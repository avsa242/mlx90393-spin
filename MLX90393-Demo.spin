{
    --------------------------------------------
    Filename: MLX90393-Demo.spin
    Author: Jesse Burt
    Description: Demo of the MLX90393 driver
    Copyright (c) 2022
    Started Aug 27, 2020
    Updated Oct 16, 2022
    See end of file for terms of use.
    --------------------------------------------
}
CON

    _clkmode    = cfg#_clkmode
    _xinfreq    = cfg#_xinfreq

' -- User-modifiable constants
    LED         = cfg#LED1
    SER_BAUD    = 115_200

    SCL_PIN     = 13
    SDA_PIN     = 14
    I2C_HZ      = 400_000                       ' max is 400_000
    INT_PIN     = 15                            ' required (data ready flag)

    STANDARD    = GAUSS                         ' GAUSS or TESLA
' --

    DAT_X_COL   = 20
    DAT_Y_COL   = DAT_X_COL + 15
    DAT_Z_COL   = DAT_Y_COL + 15
    GAUSS       = 0
    TESLA       = 1

OBJ

    cfg     : "boardcfg.flip"
    ser     : "com.serial.terminal.ansi"
    time    : "time"
    sensor  : "sensor.magnetometer.3dof.mlx90393"

PUB main{}

    setup{}
    sensor.preset_active{}                      ' default settings, but enable
                                                ' sensor acquisition and set
                                                ' scale factor
    case STANDARD
        GAUSS:
            repeat
                ser.position(0, 3)
                show_mag_data{}

                if (ser.rxcheck{} == "c")     ' press the 'c' key in the demo
                    cal_mag{}                 ' to calibrate sensor offsets
        TESLA:
            repeat
                ser.position(0, 3)
                magtesla{}

                if (ser.rxcheck{} == "c")
                    calibrate{}

    repeat

PUB setup{}

    ser.start(SER_BAUD)
    time.msleep(30)
    ser.clear{}
    ser.strln(string("Serial terminal started"))
    if sensor.startx(SCL_PIN, SDA_PIN, I2C_HZ, INT_PIN)
        ser.strln(string("MLX90393 driver started"))
    else
        ser.strln(string("MLX90393 driver failed to start - halting"))
        repeat

#include "magdemo.common.spinh"

DAT
{
Copyright 2022 Jesse Burt

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
}

