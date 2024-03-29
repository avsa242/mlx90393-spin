{
    --------------------------------------------
    Filename: core.con.mlx90393.spin
    Author: Jesse Burt
    Description: MLX90393-specific constants
    Copyright (c) 2020
    Started Aug 27, 2020
    Updated Nov 16, 2020
    See end of file for terms of use.
    --------------------------------------------
}

CON

    I2C_MAX_FREQ        = 400_000
    SLAVE_ADDR          = $0c << 1
    TPOR                = 2_000     ' uSec

' Commands
    NOOP                = $00
    START_BURST_MODE    = $10
    START_WAKE_ON_CHANGE= $20
    START_SINGLE_MEAS   = $30
    READ_MEAS           = $40
    READ_REG            = $50
    WRITE_REG           = $60
    EXIT_MODE           = $80
    MEM_RECALL          = $d0
    MEM_STORE           = $e0
    RESET               = $f0

    X                   = %1000
    Y                   = %0100
    Z                   = %0010
    T                   = %0001
    ALL_MAG             = X | Y | Z
    ALL                 = X | Y | Z | T ' mask for reading all axes, and temp

' Status byte
    BURST_MODE          = 7
    WOC_MODE            = 6
    SM_MODE             = 5
    MODE                = 5
    ERROR               = 4
    SED                 = 3
    RS                  = 2
    D                   = 0
    MODE_BITS           = %111
    D_BITS              = %11


' RAM registers
    CFG0                = $00
    CFG0_MASK           = $01FF
        BIST            = 8
        ZSERIES         = 7
        GAIN_SEL        = 4
        HALLCONF        = 0
        GAIN_SEL_BITS   = %111
        HALLCONF_BITS   = %1111
        BIST_MASK       = 1 ^ CFG0_MASK
        ZSERIES_MASK    = (1 << ZSERIES) ^ CFG0_MASK
        GAIN_SEL_MASK   = (GAIN_SEL_BITS << GAIN_SEL) ^ CFG0_MASK
        HALLCONF_MASK   = (HALLCONF_BITS << HALLCONF) ^ CFG0_MASK

    CFG1                = $01
    CFG1_MASK           = $FFFF
        TRIG_INT        = 15
        COMM_MODE       = 13
        WOC_DIFF        = 12
        EXT_TRG         = 11
        TCMP_EN         = 10
        BURST_SEL       = 6
        BURST_SEL_ZYX   = 7
        BURST_SEL_T     = 6
        BURST_DRATE     = 0
        COMM_MODE_BITS  = %11
        BURST_SEL_BITS  = %1111
        BURSTSEL_ZYXBITS= %111
        BURST_DRATE_BITS= %111111
        TRIG_INT_MASK   = (1 << TRIG_INT) ^ CFG1_MASK
        COMM_MODE_MASK  = (COMM_MODE_BITS << COMM_MODE) ^ CFG1_MASK
        WOC_DIFF_MASK   = (1 << WOC_DIFF) ^ CFG1_MASK
        EXT_TRG_MASK    = (1 << EXT_TRG) ^ CFG1_MASK
        TCMP_EN_MASK    = (1 << TCMP_EN) ^ CFG1_MASK
        BURST_SEL_MASK  = (BURST_SEL_BITS << BURST_SEL) ^ CFG1_MASK
        BURSTSEL_ZYXMASK= (BURSTSEL_ZYXBITS << BURST_SEL_ZYX) ^ CFG1_MASK
        BURST_DRATE_MASK= BURST_DRATE_BITS ^ CFG1_MASK

    CFG2                = $02
    CFG2_MASK           = $1FFF
        OSR2            = 11
        RES_Z           = 9
        RES_Y           = 7
        RES_X           = 5
        DIG_FILT        = 2
        OSR             = 0
        OSR2_BITS       = %11
        RES_Z_BITS      = %11
        RES_Y_BITS      = %11
        RES_X_BITS      = %11
        RES_BITS        = %11_11_11
        DIG_FILT_BITS   = %111
        OSR_BITS        = %11
        OSR2_MASK       = (OSR2_BITS << OSR2) ^ CFG2_MASK
        RES_Z_MASK      = (RES_Z_BITS << RES_Z) ^ CFG2_MASK
        RES_Y_MASK      = (RES_Y_BITS << RES_Y) ^ CFG2_MASK
        RES_X_MASK      = (RES_X_BITS << RES_X) ^ CFG2_MASK
        RES_MASK        = (RES_BITS << RES_X) ^ CFG2_MASK
        DIG_FILT_MASK   = (DIG_FILT_BITS << DIG_FILT) ^ CFG2_MASK
        OSR_MASK        = (OSR_BITS << OSR) ^ CFG2_MASK

    SENS_TC             = $03   ' SENS_TC_LT:SENS_TC_HT

    OFFSET_X            = $04
    OFFSET_Y            = $05
    OFFSET_Z            = $06

    WOXY_THRESHOLD      = $07
    WOZ_THRESHOLD       = $08
    WOT_THRESHOLD       = $09

    FREE_START          = $0A
    FREE_END            = $1F


PUB null{}
' This is not a top-level object

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

