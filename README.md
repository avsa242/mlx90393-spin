# mlx90393-spin
---------------

This is a P8X32A/Propeller, P2X8C4M64P/Propeller 2 driver object for the Melexis MLX90393 Triaxis magnetic node

**IMPORTANT**: This software is meant to be used with the [spin-standard-library](https://github.com/avsa242/spin-standard-library) (P8X32A) or [p2-spin-standard-library](https://github.com/avsa242/p2-spin-standard-library) (P2X8C4M64P). Please install the applicable library first before attempting to use this code, otherwise you will be missing several files required to build the project.


## Salient Features

* I2C connection at up to 400kHz
* Read magnetometer, temperature sensor data


## Requirements

P1/SPIN1:
* spin-standard-library
* 1 extra core/cog for the PASM I2C engine
* `sensor.magnetometer.common.spinh` (source: spin-standard-library)
* `sensor.temp.common.spinh` (source: spin-standard-library)

P2/SPIN2:
* p2-spin-standard-library
* `sensor.magnetometer.common.spin2h` (source: p2-spin-standard-library)
* `sensor.temp.common.spin2h` (source: p2-spin-standard-library)


## Compiler Compatibility

| Processor | Language | Compiler               | Backend      | Status                |
|-----------|----------|------------------------|--------------|-----------------------|
| P1        | SPIN1    | FlexSpin (7.7.0)       | Bytecode     | OK                    |
| P1        | SPIN1    | FlexSpin (7.7.0)       | Native/PASM  | OK                    |
| P2        | SPIN2    | FlexSpin (7.7.0)       | NuCode       | Runtime issues        |
| P2        | SPIN2    | FlexSpin (7.7.0)       | Native/PASM2 | OK                    |

(other versions or toolchains not listed are __not supported__, and _may or may not_ work)


## Limitations

* Very early in development - may malfunction, or outright fail to build

