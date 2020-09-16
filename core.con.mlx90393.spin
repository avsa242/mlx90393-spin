{
    --------------------------------------------
    Filename: core.con.mlx90393.spin
    Author: Jesse Burt
    Description: Low-level constants
    Copyright (c) 2020
    Started Aug 27, 2020
    Updated Sep 16, 2020
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
        TRIG_INT        = 15
        COMM_MODE       = 13
        WOC_DIFF        = 12
        EXT_TRG         = 11
        TCMP_EN         = 10
        BURST_SEL       = 6
        BURST_DRATE     = 0
        COMM_MODE_BITS  = %11
        BURST_SEL_BITS  = %1111
        BURST_DRATE_BITS= %111111
        TRIG_INT_MASK   = (1 << TRIG_INT) ^ CFG1_MASK
        COMM_MODE_MASK  = (COMM_MODE_BITS << COMM_MODE) ^ CFG1_MASK
        WOC_DIFF_MASK   = (1 << WOC_DIFF) ^ CFG1_MASK
        EXT_TRG_MASK    = (1 << EXT_TRG) ^ CFG1_MASK
        TCMP_EN_MASK    = (1 << TCMP_EN) ^ CFG1_MASK
        BURST_SEL_MASK  = (BURST_SEL_BITS << BURST_SEL) ^ CFG1_MASK
        BURST_DRATE_MASK= (BURST_DRATE_BITS << BURST_DRATE) ^ CFG1_MASK

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
        DIG_FILT_BITS   = %111
        OSR_BITS        = %11
        OSR2_MASK       = (OSR2_BITS << OSR2) ^ CFG2_MASK
        RES_Z_MASK      = (RES_Z_BITS << RES_Z) ^ CFG2_MASK
        RES_Y_MASK      = (RES_Y_BITS << RES_Y) ^ CFG2_MASK
        RES_X_MASK      = (RES_X_BITS << RES_X) ^ CFG2_MASK
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


#ifndef __propeller2__
PUB Null
'' This is not a top-level object
#endif
