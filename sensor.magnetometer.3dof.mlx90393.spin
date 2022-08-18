{
    --------------------------------------------
    Filename: sensor.magnetometer.3dof.mlx90393.spin
    Author: Jesse Burt
    Description: Driver for the Melexis MLX90393 3DoF magnetometer
    Copyright (c) 2021
    Started Aug 27, 2020
    Updated Aug 11, 2021
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

' Indicate to user apps how many Degrees of Freedom each sub-sensor has
'   (also imply whether or not it has a particular sensor)
    ACCEL_DOF           = 0
    GYRO_DOF            = 0
    MAG_DOF             = 3
    BARO_DOF            = 0
    DOF                 = ACCEL_DOF + GYRO_DOF + MAG_DOF + BARO_DOF

' Bias adjustment (AccelBias(), GyroBias(), MagBias()) read or write
    R                       = 0
    W                       = 1

' Axis-specific sensitivity settings
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

    i2c : "com.i2c"                             ' PASM I2C engine
    core: "core.con.mlx90393"                   ' HW-specific constants
    time: "time"                                ' timing

PUB Null{}
' This is not a top-level object

PUB Start{}: status
' Start using "standard" Propeller I2C pins and 100kHz, no INT pin
    return startx(DEF_SCL, DEF_SDA, DEF_HZ, -1)

PUB Startx(SCL_PIN, SDA_PIN, I2C_HZ, INT_PIN): status
' Start using custom I/O settings
    if lookdown(SCL_PIN: 0..31) and lookdown(SDA_PIN: 0..31) and {
}   I2C_HZ =< core#I2C_MAX_FREQ
        if (status := i2c.init(SCL_PIN, SDA_PIN, I2C_HZ))
            time.usleep(core#TPOR)              ' wait for device to be ready
            if lookdown(INT_PIN: 0..31)         ' Valid INT_PIN?
                _INT_PIN := INT_PIN             ' Copy it to hub var
                dira[_INT_PIN] := 0
            if i2c.present(SLAVE_WR)            ' test device bus presencxe
                return
    ' if this point is reached, something above failed
    ' Double check I/O pin assignments, connections, power
    ' Lastly - make sure you have at least one free core/cog
    return FALSE

PUB Stop{}
' Put any other housekeeping code here required/recommended by your device before shutting down
    i2c.deinit{}

PUB Defaults{}
' Set factory defaults
    reset{}

PUB Preset_Active{}
' Like defaults, but
'   * enables sensor acquisition
    reset{}
    magopmode(CONT)
    magscale(7)
    tempscale(C)

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
            ' map adcres (bits) 16, 17, 18, 19 to register vals 0, 1, 2, 3
            adcres := lookdownz(adcres: 16, 17, 18, 19)
            _adcoffset := lookupz(adcres: 0, 0, 32768, 16384)
            adcres := (adcres << core#RES_X) | (adcres << core#RES_Y) |{
}           (adcres << core#RES_Z)              ' set all three axes the same
        other:
            magopmode(opmode_orig)              ' restore user op. mode
            curr_res := ((curr_res >> core#RES_X) & core#RES_X_BITS)
            return lookupz(curr_res: 16, 17, 18, 19)

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

PUB MagBias(ptr_x, ptr_y, ptr_z, rw) | tmp[2], opmode_orig
' Read or write/manually set Magnetometer calibration offset values
'   Valid values:
'       rw:
'           R (0), W (1)
'       ptr_x, ptr_y, ptr_z:
'           -32768..32767
'   NOTE: When rw is set to READ, ptr_x, ptr_y and ptr_z must be addresses
'       of respective variables to hold the returned calibration offset values
    opmode_orig := magopmode(-2)                ' store user's opmode
    magopmode(IDLE)                             ' switch to idle to read/write
    case rw
        R:
            readreg(core#OFFSET_X, 2, @tmp)     ' read the current offsets
            readreg(core#OFFSET_Y, 2, @tmp.byte[2])
            readreg(core#OFFSET_Z, 2, @tmp.byte[4])
            ' offsets are centered around 32768
            long[ptr_x] := tmp.word[X_AXIS]-32768
            long[ptr_y] := tmp.word[Y_AXIS]-32768
            long[ptr_z] := tmp.word[Z_AXIS]-32768
        W:
            case ptr_x
                -32768..32767:
                other:
                    return
            case ptr_y
                -32768..32767:
                other:
                    return
            case ptr_z
                -32768..32767:
                other:
                    return
            ptr_x += 32768
            ptr_y += 32768
            ptr_z += 32768
            writereg(core#OFFSET_X, 2, @ptr_x)
            writereg(core#OFFSET_Y, 2, @ptr_y)
            writereg(core#OFFSET_Z, 2, @ptr_z)
    magopmode(opmode_orig)                      ' restore user op. mode

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

PUB MagGauss(ptr_x, ptr_y, ptr_z) | tmp[MAG_DOF]
' Read magnetometer data, in hundred-thousandths of a Gauss
'   e.g., 50_228 = 0.50228Gs
    magdata(@tmp[X_AXIS], @tmp[Y_AXIS], @tmp[Z_AXIS])

    tmp[X_AXIS] *= _mres[X_AXIS]
    tmp[Y_AXIS] *= _mres[Y_AXIS]
    tmp[Z_AXIS] *= _mres[Z_AXIS]

    longmove(ptr_x, @tmp, MAG_DOF)              ' copy local vars to pointers

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

PUB MagScale(scale): curr_scl | opmode_orig, adcres, axis
' Set magnetometer full-scale range 'XXX units
'   Valid values: TBD
'   Any other value polls the chip and returns the current setting
    opmode_orig := magopmode(-2)                ' store user's opmode
    magopmode(IDLE)                             ' switch to idle to read/write
    curr_scl := 0
    readreg(core#CFG0, 2, @curr_scl)
    case scale
        0..7:
            adcres := 1 << (magadcres(-2)-16)   ' map 16..19bits to 0..3
            repeat axis from X_AXIS to Y_AXIS
                _mres[axis] := lookupz(scale: 0_751, 0_601, 0_451, {
}               0_376, 0_300, 0_250, 0_200, 0_150)
                _mres[axis] := (_mres[axis] * SENS_XY_0C * adcres) / 1000

            ' Z-axis sensitivity is different from X and Y:
            _mres[Z_AXIS] := lookupz(scale: 1_210, 0_968, 0_726, {
}           0_605, 0_484, 0_403, 0_323, 0_242)
            _mres[Z_AXIS] := (_mres[Z_AXIS] * SENS_Z_0C * adcres) / 1000
            scale <<= core#GAIN_SEL
        other:
            magopmode(opmode_orig)              ' restore user's opmode
            return ((curr_scl >> core#GAIN_SEL) & core#GAIN_SEL_BITS)

    scale := ((curr_scl & core#GAIN_SEL_MASK) | scale) & core#CFG0_MASK
    writereg(core#CFG0, 2, @scale)
    magopmode(opmode_orig)

PUB MagTesla(ptr_x, ptr_y, ptr_z) | tmp[MAG_DOF]
' Read magnetometer data, in hundred-thousandths of a micro-Tesla
'   e.g., 50_228 = 50.228uT
    maggauss(@tmp[X_AXIS], @tmp[Y_AXIS], @tmp[Z_AXIS])

    tmp[X_AXIS] *= 100
    tmp[Y_AXIS] *= 100
    tmp[Z_AXIS] *= 100

    longmove(ptr_x, @tmp, MAG_DOF)              ' copy local vars to pointers

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

PUB TempCompensation(state): curr_state | opmode_orig
' Enable on-chip temperature compensation for magnetometer readings
'   Valid values: TRUE (-1 or 1) or FALSE
'   Any other value polls the chip and returns the current setting
    opmode_orig := magopmode(-2)                ' store user op. mode
    magopmode(IDLE)                             ' must be in idle to read regs
    curr_state := 0
    readreg(core#CFG1, 1, @curr_state)
    case ||(state)
        0, 1:
            state := ||(state) << core#TCMP_EN
        other:
            return (((curr_state >> core#TCMP_EN) & 1) == 1)

    state := ((curr_state & core#TCMP_EN_MASK) | state)
    writereg(core#CFG1, 1, @state)
    magopmode(opmode_orig)                      ' restore user's opmode

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

            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.start{}                         ' first byte read is always
            i2c.write(SLAVE_RD)                 '   the status byte
            _status := status := i2c.read(i2c#ACK)

' How many bytes are ready to read? Make sure it matches what's requested:
            if nr_rdbytes == (2 * (status & core#D_BITS) + 2)
                i2c.rdblock_msbf(ptr_buff, nr_rdbytes, i2c#NAK)
                i2c.stop{}
            else
                i2c.stop{}
                return false
        core#START_SINGLE_MEAS, core#START_BURST_MODE, core#EXIT_MODE, core#NOOP:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := cmd | arg

            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)
            i2c.start{}
            i2c.write(SLAVE_RD)
            status := i2c.read(i2c#NAK)
            i2c.stop{}
            time.msleep(2)                      ' wait for measurement
        core#RESET:
            cmd_pkt.byte[0] := SLAVE_WR
            cmd_pkt.byte[1] := cmd

            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 2)
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

            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 3)
            i2c.start{}                         ' first byte read is always
            i2c.write(SLAVE_RD)                 '   the status byte
            _status := i2c.read(i2c#ACK)
            i2c.rdblock_msbf(ptr_buff, 2, i2c#NAK)
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

            i2c.start{}
            i2c.wrblock_lsbf(@cmd_pkt, 5)
            i2c.start{}
            i2c.write(SLAVE_RD)
            _status := i2c.read(i2c#NAK)        ' update the status byte
            i2c.stop
            return
        other:
            return

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

