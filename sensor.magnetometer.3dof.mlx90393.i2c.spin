{
    --------------------------------------------
    Filename: sensor.magnetometer.3dof.mlx90393.i2c.spin
    Author: Jesse Burt
    Description: Driver for the Melexis MLX90393 3DoF magnetometer
    Copyright (c) 2020
    Started Aug 27, 2020
    Updated Nov 16, 2020
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

    SENS_XY_00          = 0_196
    SENS_Z_00           = 0_316
    SENS_XY_0C          = 0_150
    SENS_Z_0C           = 0_242

' Measurement selection bits
    Z                   = 3
    Y                   = 2
    X                   = 1
    T                   = 0

' Measurement modes
    SINGLE              = 0
    IDLE                = 0                     ' alias for SINGLE
    WOC                 = 2
    CONT                = 4

    X_AXIS              = 0
    Y_AXIS              = 1
    Z_AXIS              = 2

' Temperature scales
    C                   = 0
    F                   = 1

VAR

    long _last_temp
    long _mres[3]
    word _adcoffset
    byte _INT_PIN
    byte _axes_enabled
    byte _status
    byte _temp_scale

OBJ

    i2c : "com.i2c"                             ' PASM I2C Driver
    core: "core.con.mlx90393"                   ' HW-specific constants
    time: "time"                                ' timing

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

PUB MagADCRes(adcres): curr_res | opmode_orig
' Set magnetometer ADC resolution, in bits
'   Valid values: 16..19
'   Any other value polls the chip and returns the current setting
    opmode_orig := magopmode(-2)                ' store user op. mode
    magopmode(IDLE)                             ' must be in idle to read regs
    curr_res := 0
    readreg(core#CFG2, 2, @curr_res)
    case adcres
        16..19:
            ' map adcres (bits) 19, 18, 17, 16 to register vals 0, 1, 2, 3
            adcres := lookdownz(adcres: 19, 18, 17, 16)
            _adcoffset := lookupz(adcres: 0, 0, 32768, 16384)
            adcres := (adcres << core#RES_X) | (adcres << core#RES_Y) |{
                    } (adcres << core#RES_Z)    ' set all three axes the same
        other:
            magopmode(opmode_orig)              ' restore user op. mode
            curr_res := ((curr_res >> core#RES_X) & core#RES_X_BITS)
            return lookupz(curr_res: 19, 18, 17, 16)

    adcres := ((curr_res & core#RES_MASK) | adcres) & core#CFG2_MASK
    writereg(core#CFG2, 2, @adcres)
    magopmode(opmode_orig)

PUB MagAxisEnabled(xyz_mask): curr_mask 'TODO
' Enable magnetometer axis per bitmask
'   Valid values:
'       Bits %210 (xyz):
'           0: Disable axis
'           1: Enable axis
    curr_mask := _axes_enabled
    case xyz_mask & %111
        %000..%111:                             ' LSB is temp sensor
            xyz_mask ><= 3                      ' rev; order in reg is %zyx
            _axes_enabled &= %0001
            _axes_enabled |= (xyz_mask << 1)
        other:
            return

PUB MagBias(ptr_x, ptr_y, ptr_z, rw)
' TODO

PUB MagData(ptr_x, ptr_y, ptr_z): status | tmp[2]
' Read magnetometer data
'   NOTE: For efficiency, the temperature data is read in as well,
'       and stored in a hub variable
    status := command(core#READ_MEAS, core#ALL, 8, @tmp)
    longfill(ptr_x, 0, 3)

    long[ptr_x] := ~~tmp.word[2] - _adcoffset
    long[ptr_y] := ~~tmp.word[1] - _adcoffset
    long[ptr_z] := ~~tmp.word[0] - _adcoffset
    _last_temp := tmp.word[3]                   ' Read in the temp, too

PUB MagDataRate(rate): curr_rate | opmode_orig
' Set magnetometer data rate, in Hz
'   Valid values: 0..50, *876
'   Any other value polls the chip and returns the current setting
'   NOTE: Values 0..50 are approximated
    opmode_orig := magopmode(-2)                ' store user op. mode
    magopmode(IDLE)                             ' must be in idle to read regs
    curr_rate := 0
    readreg(core#CFG1, 2, @curr_rate)
    case rate
        1..50, 876:
            rate := 1000 / (rate * 20)
        0:                                      ' ~0.8Hz
            rate := 63
        other:
            magopmode(opmode_orig)              ' restore user op. mode
            curr_rate &= core#BURST_DRATE_BITS
            case curr_rate
                0:
                    return 876
                1..63:
                    return 1000 / (curr_rate * 20)

    rate := ((curr_rate & core#BURST_DRATE_MASK) | rate) & core#CFG1_MASK
    writereg(core#CFG1, 2, @rate)
    magopmode(opmode_orig)                      ' restore user op. mode

PUB MagDataReady{}: flag
' Flag indicating magnetometer data is ready
'   Returns: TRUE (-1) if data ready, FALSE (0) otherwise
    return ina[_INT_PIN] == 1

PUB MagGauss(ptr_x, ptr_y, ptr_z) | tmp[3]
' TODO
    magdata(@tmp[X_AXIS], @tmp[Y_AXIS], @tmp[Z_AXIS])

    tmp[X_AXIS] *= _mres[X_AXIS]
    tmp[Y_AXIS] *= _mres[Y_AXIS]
    tmp[Z_AXIS] *= _mres[Z_AXIS]

    longmove(ptr_x, @tmp, 3)                    ' copy local vars to pointers

PUB MagOpMode(mode): curr_mode
' Set magnetometer operating mode
'   Valid values:
'       SINGLE (0): Single-shot measurement
'       IDLE (0): alias for SINGLE
'       WOC (2): Wake-On-Change
'       CONT (4): Continuous measurement
'   Any other value polls the chip and returns the current setting
    curr_mode := (readstatus{} >> core#MODE) & core#MODE_BITS
    case mode
        SINGLE:
            if curr_mode <> SINGLE
                command(core#EXIT_MODE, 0, 0, 0)
        WOC:
            if curr_mode <> WOC
                command(core#EXIT_MODE, 0, 0, 0)
                command(core#START_WAKE_ON_CHANGE, core#ALL, 0, 0)
        CONT:
            if curr_mode <> CONT
                command(core#START_BURST_MODE, core#ALL, 0, 0)
                time.msleep(10) 'xxx only about 2ms until first ready pulse
        other:
            return curr_mode

PUB MagScale(scale): curr_scl | opmode_orig, adcres
' Set magnetometer full-scale range 'XXX units
'   Valid values: TBD
'   Any other value polls the chip and returns the current setting
    opmode_orig := magopmode(-2)                ' store user's opmode
    magopmode(IDLE)                             ' switch to idle to read/write
    curr_scl := 0
    readreg(core#CFG0, 2, @curr_scl)
    case scale
        0..7:
' scale     1      1000   1000         1
' calc =    raw * [gain * sens_axis * (1 << res_axis)]
            _mres[X_AXIS] := lookupz(scale: 1_000, 1_333, 1_666,{
                                        } 2_000, 2_500, 3_000, 4_000, 5_000)
            _mres[Y_AXIS] := lookupz(scale: 1_000, 1_333, 1_666,{
                                        } 2_000, 2_500, 3_000, 4_000, 5_000)
            _mres[Z_AXIS] := lookupz(scale: 1_000, 1_333, 1_666,{
                                        } 2_000, 2_500, 3_000, 4_000, 5_000)
            adcres := 1 << (3 - (magadcres(-2)-16)) ' map 19..16bits to 0..3
            _mres[X_AXIS] *= (SENS_XY_0C * adcres)
            _mres[Y_AXIS] *= (SENS_XY_0C * adcres)
            _mres[Z_AXIS] *= (SENS_Z_0C * adcres)
            scale <<= core#GAIN_SEL
        other:
            magopmode(opmode_orig)              ' restore user's opmode
            return ((curr_scl >> core#GAIN_SEL) & core#GAIN_SEL_BITS)

    scale := ((curr_scl & core#GAIN_SEL_MASK) | scale) & core#CFG0_MASK
    writereg(core#CFG0, 2, @scale)
    magopmode(opmode_orig)

PUB MeasureMag{}: status
' Perform a measurement
'   NOTE: This method only applies to single-shot measurement mode
'       and will stop continuous measurement mode, if called
    status := command(core#START_SINGLE_MEAS, core#ALL, 0, 0)

PUB Reset{}: status
' Reset the device
'   NOTE: A mandatory 2ms delay is waited after resetting
    exit{}
    command(core#RESET, 0, 0, 0)
    time.usleep(core#TPOR)

PUB TempData{}: temp_raw
' Read temperature data
'   Returns: Raw temperature word, s16 (sign-extended)
    return ~~_last_temp

PUB Temperature{}: temp
' Temperature, in hundredths of a degree
    return calctemp(tempdata{})

PUB TempScale(scale): curr_scl
' Set temperature scale used by Temperature method
'   Valid values:
'      *C (0): Celsius
'       F (1): Fahrenheit
'   Any other value returns the current setting
    case scale
        C, F:
            _temp_scale := scale
        other:
            return _temp_scale

PRI calcTemp(temp_word): temp_cal
' Calculate temperature, using temperature word
'   Returns: temperature, in hundredths of a degree, in chosen scale
    temp_cal := ((((temp_word & $FFFF) * 1_000) - 46244_000) / 45_2) + 25_00
    case _temp_scale
        C:
            return
        F:
            return ((temp_cal * 90) / 50) + 32_00
        other:
            return FALSE

PRI command(cmd, arg, nr_rdbytes, ptr_buff): status | cmd_pkt, tmp
' Send command with arg to device
    status := 0
    case cmd
        core#READ_MEAS:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := cmd | arg        ' cmd[7..4] | arg[3..0]

            i2c.start
            repeat tmp from 0 to 1
                i2c.write(cmd_pkt.byte[tmp])

            i2c.start                           ' first byte read is always
            i2c.write(SLAVE_RD)                 '   the status byte
            _status := status := i2c.read(i2c#ACK)

' How many bytes are ready to read? Make sure it matches what's requested:
            if nr_rdbytes == (2 * (status & core#D_BITS) + 2)
                repeat tmp from nr_rdbytes-1 to 0
                    byte[ptr_buff][tmp] := i2c.read(tmp == 0)
                i2c.stop
            else
                i2c.stop{}
                return false
        core#START_SINGLE_MEAS, core#START_BURST_MODE, core#EXIT_MODE, core#NOOP:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := cmd | arg

            i2c.start
            repeat tmp from 0 to 1
                i2c.write(cmd_pkt.byte[tmp])

            i2c.start
            i2c.write(SLAVE_RD)
            status := i2c.read(i2c#NAK)
            i2c.stop
            time.msleep(2)                      ' wait for measurement
        core#RESET:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := cmd

            i2c.start
            repeat tmp from 0 to 1
                i2c.write(cmd_pkt.byte[tmp])
            i2c.stop{}
        other:
            return

PRI exit{}: status
' Exit mode
    status := command(core#EXIT_MODE, 0, 0, 0)

PUB readReg(reg_nr, nr_bytes, ptr_buff): status | cmd_pkt, tmp
' Read nr_bytes from device
    status := 0
    case reg_nr
        $00..$3c:                               ' read RAM registers
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := core#READ_REG    ' RR command
            cmd_pkt.byte[2] := reg_nr << 2      ' chip requires reg be shifted

            i2c.start
            repeat tmp from 0 to 2
                i2c.write(cmd_pkt.byte[tmp])

            i2c.start                           ' first byte read is always
            i2c.write(SLAVE_RD)                 '   the status byte
            _status := i2c.read(i2c#ACK)
            repeat tmp from 1 to 0              ' now the data
                byte[ptr_buff][tmp] := i2c.read(tmp == 0)
            i2c.stop
        other:
            return

PRI readStatus{}: status
' Read status byte
    _status := status := command(core#NOOP, 0, 0, 0)

PRI writeReg(reg_nr, nr_bytes, ptr_buff): status | tmp, cmd_pkt[2]
' Write nr_bytes to device
    case reg_nr
        $00..$09:                               ' writable RAM reg locations
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := core#WRITE_REG   ' WR command
            cmd_pkt.byte[2] := byte[ptr_buff][1]' data first...
            cmd_pkt.byte[3] := byte[ptr_buff][0]'   ...
            cmd_pkt.byte[4] := reg_nr << 2      ' _now_ the register #

            i2c.start
            repeat tmp from 0 to 4
                i2c.write(cmd_pkt.byte[tmp])

            i2c.start
            i2c.write(SLAVE_RD)
            _status := i2c.read(i2c#NAK)        ' update the status byte
            i2c.stop
            return
        other:
            return

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
