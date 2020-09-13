{
    --------------------------------------------
    Filename: sensor.magnetometer.3dof.mlx90393.i2c.spin
    Author: Jesse Burt
    Description: Driver for the Melexis MLX90393 3DoF magnetometer
    Copyright (c) 2020
    Started Aug 27, 2020
    Updated Sep 13, 2020
    See end of file for terms of use.
    --------------------------------------------
}

CON

    SLAVE_WR            = core#SLAVE_ADDR
    SLAVE_RD            = core#SLAVE_ADDR|1

    DEF_SCL             = 28
    DEF_SDA             = 29
    DEF_HZ              = 100_000
    I2C_MAX_FREQ        = core#I2C_MAX_FREQ

' Measurement selection bits
    Z                   = 3
    Y                   = 2
    X                   = 1
    T                   = 0

' Measurement modes
    CONT                = 0
    SINGLE              = 1

    X_AXIS              = 0
    Y_AXIS              = 1
    Z_AXIS              = 2

VAR

    long _last_temp
    byte _INT_PIN
    byte _axes_enabled

OBJ

    i2c : "com.i2c"                                     ' PASM I2C Driver
    core: "core.con.mlx90393.spin"                      ' low-level constants
    time: "time"                                        ' timing

PUB Null{}
' This is not a top-level object

PUB Start{}: okay
' Standard Propeller I2C pins and 100kHz, no INT pin
    okay := startx (DEF_SCL, DEF_SDA, DEF_HZ, -1)

PUB Startx(SCL_PIN, SDA_PIN, I2C_HZ, INT_PIN): okay

    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31)
        if I2C_HZ =< core#I2C_MAX_FREQ
            if okay := i2c.setupx (SCL_PIN, SDA_PIN, I2C_HZ)' I2C Object Started?
                time.usleep(core#TPOR)
                if lookdown(INT_PIN: 0..31)                 ' Valid INT_PIN?
                    _INT_PIN := INT_PIN                     ' Copy it to hub var
                    dira[_INT_PIN] := 0
                if i2c.present(SLAVE_WR)                    ' Test bus presence
                    return

    return FALSE                                            ' Something above failed

PUB Stop{}
' Put any other housekeeping code here required/recommended by your device before shutting down
    i2c.terminate

PUB Defaults{}
' Set factory defaults

PUB CalibrateMag{}
' TODO

PUB LastTemp{}
' Last temperature reading
'   Returns: Raw temperature word, s16
    return ~~_last_temp

PUB MagAxisEnabled(xyz_mask): curr_mask 'TODO
' Enable magnetometer axis per bitmask
'   Valid values:
'       Bits %210 (xyz):
'           0: Disable axis
'           1: Enable axis
    curr_mask := _axes_enabled
    case xyz_mask & %111
        %000..%111:                                 ' LSB is temp sensor
            xyz_mask ><= 3                          ' rev; order in reg is %zyx
            _axes_enabled &= %0001
            _axes_enabled |= (xyz_mask << 1)
        other:
            return

PUB MagBias(ptr_x, ptr_y, ptr_z, rw)
' TODO

PUB MagData(ptr_x, ptr_y, ptr_z) | tmp[2]
' Read magnetometer data
'   NOTE: For efficiency, the temperature data is read in as well,
'       and stored in a hub variable
    readreg(core#READ_MEAS | core#ALL, 8, @tmp)

    longfill(ptr_x, 0, 3)

    long[ptr_x] := ~~tmp.word[X_AXIS]
    long[ptr_y] := ~~tmp.word[Y_AXIS]
    long[ptr_z] := ~~tmp.word[Z_AXIS]
    _last_temp := tmp.word[3]                       ' Read in the temp, too

PUB MagDataRate(Hz): curr_rate
' TODO

PUB MagDataReady{}: flag

    return true

PUB MagGauss(ptr_x, ptr_y_, ptr_z)
' TODO

PUB MagScale(gauss): curr_scl
' TODO

PUB MeasureMag{}: status
' Perform a measurement
'   NOTE: This method only applies to single-shot measurement mode
'       and will stop continuous measurement mode, if called
    status := writereg(core#START_SINGLE_MEAS | core#ALL, 0, 0)

PUB Reset{}: status
' Reset the device
'   NOTE: A mandatory 2ms delay is waited after resetting
    exit{}
    writereg(core#RESET, 0, 0)
    time.usleep(core#TPOR)

PUB TempData{}: temp_raw
' Read temperature data
'   Returns: Raw temperature word, s16
    writereg(core#START_SINGLE_MEAS | core#T, 0, 0)
    readreg(core#READ_MEAS | core#T, 2, @temp_raw)
    _last_temp := ~~temp_raw

PUB Temperature{}: temp
' TODO

PUB TempScale(scale): curr_scl
' TODO

PRI exit{}: status
' Exit mode
    status := writereg(core#EXIT_MODE, 0, 0)

PRI readReg(reg_nr, nr_bytes, ptr_buff): status | cmd_pkt, tmp
' Read nr_bytes from device
    status := 0
    case reg_nr & $f0
        core#READ_MEAS:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := reg_nr

            i2c.start
            repeat tmp from 0 to 1
                i2c.write(cmd_pkt.byte[tmp])

            i2c.start
            i2c.write(SLAVE_RD)
            status := i2c.read(i2c#ACK)

' How many bytes are ready to read? Make sure it matches what's requested:
            if nr_bytes == (2 * (status & core#D_BITS) + 2)
                repeat tmp from nr_bytes-1 to 0
                    byte[ptr_buff][tmp] := i2c.read(tmp == 0)
                i2c.stop
            else
                i2c.stop{}
                return false

        other:
            return

PRI writeReg(reg_nr, nr_bytes, ptr_buff): status | tmp, cmd_pkt
' Write nr_bytes to device
    case reg_nr & $f0
        core#START_SINGLE_MEAS, core#EXIT_MODE:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := reg_nr

            i2c.start
            repeat tmp from 0 to 1
                i2c.write(cmd_pkt.byte[tmp])

            i2c.start
            i2c.write(SLAVE_RD)
            status := i2c.read(i2c#NAK)
            i2c.stop
            time.msleep(2)                          ' wait for measurement
        core#RESET:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := reg_nr

            i2c.start
            repeat tmp from 0 to 1
                i2c.write(cmd_pkt.byte[tmp])
            i2c.stop{}
        other:
            return false

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
