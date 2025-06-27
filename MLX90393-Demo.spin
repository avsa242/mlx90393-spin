{
----------------------------------------------------------------------------------------------------
    Filename:       MLX90393-Demo.spin
    Description:    Demo of the MLX90393 driver
    Author:         Jesse Burt
    Started:        Aug 27, 2020
    Updated:        Oct 7, 2024
    Copyright (c) 2024 - See end of file for terms of use.
----------------------------------------------------------------------------------------------------
}

CON

    _clkmode    = xtal1+pll16x
    _xinfreq    = 5_000_000


OBJ

    time:   "time"
    ser:    "com.serial.terminal.ansi" | SER_BAUD=115_200
    sensor: "input.encoder.mlx90393" | INT=25, SCL=28, SDA=29, I2C_FREQ=100_000


PUB main() | m[sensor.MAG_DOF], sign, axis

    setup()

    repeat
        ser.pos_xy(0, 3)
        repeat until sensor.mag_data_rdy()
        sensor.mag_gauss(@m[sensor.X_AXIS], @m[sensor.Y_AXIS], @m[sensor.Z_AXIS])
        ser.str(@"Mag (Gs):  ")
        repeat axis from sensor.X_AXIS to sensor.Z_AXIS
            if ( m[axis] < 0 )
                sign := "-"
            else
                sign := " "
            ser.printf(@"%c%d.%06.6d     ", sign, ...
                                            abs(m[axis] / 1_000_000), ...
                                            abs(m[axis] // 1_000_000) )
        ser.newline()

        if ( ser.getchar_noblock == "c" )       ' press the 'c' key in the demo
            cal_mag()                           ' to calibrate sensor offsets


PUB cal_mag()
' Calibrate the magnetometer
    ser.pos_xy(0, 5)
    ser.str(@"Calibrating magnetometer...")
    sensor.calibrate_mag()
    ser.pos_xy(0, 5)
    ser.clear_ln()


PUB setup()

    ser.start()
    time.msleep(30)
    ser.clear()
    ser.strln(@"Serial terminal started")

    if ( sensor.start() )
        ser.strln(@"MLX90393 driver started")
    else
        ser.strln(@"MLX90393 driver failed to start - halting")
        repeat

    sensor.preset_active()                      ' default settings, but enable sensor acquisition
                                                ' and set scale factor


DAT
{
Copyright 2025 Jesse Burt

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

