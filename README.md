# mlx90393-spin

---------------

This is a P8X32A/Propeller, ~~P2X8C4M64P/Propeller 2~~ driver object for the Melexis MLX90393 3DoF magnetometer

**IMPORTANT**: This software is meant to be used with the [spin-standard-library](https://github.com/avsa242/spin-standard-library) (P8X32A) ~~or [p2-spin-standard-library](https://github.com/avsa242/p2-spin-standard-library) (P2X8C4M64P)~~. Please install the applicable library first before attempting to use this code, otherwise you will be missing several files required to build the project.

## Salient Features

* I2C connection at up to 400kHz
* Read magnetometer, temperature sensor data

## Requirements

P1/SPIN1:

* spin-standard-library
* 1 extra core/cog for the PASM I2C engine

~~P2/SPIN2:~~

* ~~p2-spin-standard-library~~

## Compiler Compatibility

* P1/SPIN1: OpenSpin (tested with 1.00.81), FlexSpin (tested with 5.2.1-beta)
* ~~P2/SPIN2: FlexSpin (tested with 4.3.1)~~ _(not implemented yet)_
* ~~BST~~ (incompatible - no preprocessor)
* ~~Propeller Tool~~ (incompatible - no preprocessor)
* ~~PNut~~ (incompatible - no preprocessor)

## Limitations

* Very early in development - may malfunction, or outright fail to build

## TODO

Implement these methods:

- [ ] MagBias()
- [x] MagDataRate()
- [x] MagGauss()
- [x] MagTesla()
- [ ] MagScale() - WIP
- [x] Temperature()
- [x] TempScale()

Other:

- [ ] Port to P2/SPIN2
