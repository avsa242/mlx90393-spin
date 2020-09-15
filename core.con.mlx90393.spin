{
    --------------------------------------------
    Filename: core.con.mlx90393.spin
    Author: Jesse Burt
    Description: Low-level constants
    Copyright (c) 2020
    Started Aug 27, 2020
    Updated Sep 13, 2020
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
    ERROR               = 4
    SED                 = 3
    RS                  = 2
    D                   = 0
    D_BITS              = %11


' RAM registers
    CFG0                = $00
    CFG0_MASK           = $FF01
        ZSERIES         = 15
        GAIN_SEL        = 12
        HALLCONF        = 8
        BIST            = 0
        GAIN_SEL_BITS   = %111
        HALLCONF_BITS   = %1111
        ZSERIES_MASK    = (1 << ZSERIES) ^ CFG0_MASK
        GAIN_SEL_MASK   = (GAIN_SEL_BITS << GAIN_SEL) ^ CFG0_MASK
        HALLCONF_MASK   = (HALLCONF_BITS << HALLCONF) ^ CFG0_MASK
        BIST_MASK       = 1 ^ CFG0_MASK

    CFG1                = $01
    CFG1_MASK           = $FFFF
        BURST_SEL1      = 14
        BURST_DRATE     = 8
        TRIG_INT        = 7
        COMM_MODE       = 5
        WOC_DIFF        = 4
        EXT_TRG         = 3
        TCMP_EN         = 2
        BURST_SEL0      = 0
        BURST_SEL1_BITS = %11
        BURST_DRATE_BITS= %111111
        COMM_MODE_BITS  = %11
        BURST_SEL0_BITS = %11

    CFG2                = $02
    CFG2_MASK           = $FF1F
        RES_Y1          = 15
        RES_X           = 13
        DIG_FILT        = 10
        OSR             = 8
        OSR2            = 3
        RES_Z           = 1
        RES_Y0          = 0

    SENS_TC             = $03   ' SENS_TC_LT:SENS_TC_HT

    OFFSET_X            = $04
    OFFSET_Y            = $05
    OFFSET_Z            = $06

    WOXY_THRESHOLD      = $07
    WOZ_THRESHOLD       = $08
    WOT_THRESHOLD       = $09

    FREE_START          = $0A
    FREE_END            = $1F


#ifndef __propeller2__
PUB Null
'' This is not a top-level object
#endif
