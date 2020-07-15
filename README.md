# `bmp280`

> no_std driver for the bmp280 (pressure sensor).

[![Build Status](https://travis-ci.org/copterust/bmp280.svg?branch=master)](https://travis-ci.org/copterust/bmp280)

## What works

- Pressure sensor

## Supported chips

- `BMP280`;

## Basic usage

Include [library](https://crates.io/crates/bmp280) as a dependency in your Cargo.toml
[![crates.io](http://meritbadge.herokuapp.com/bmp280?style=flat-square)](https://crates.io/crates/bmp280):

```toml
[dependencies.bmp280-ehal]
version = "<version>"
```

Use embedded-hal implementation to get I2C handle and delay then create bmp280 handle:

```rust
extern crate bmp280_ehal; // or just use bmp280; if 2018 edition is used.

// to create sensor with default configuration:
let mut lsm = bmp280_ehal::BMP280::new(i2c)?;
// to get pressure:
let pres = ps.pressure();
println!("{:?}", pres);
```

## More examples

Number of examples can be found in [proving-ground](https://github.com/copterust/proving-ground) repo.

## Documentation

API Docs available on [docs.rs](https://docs.rs/bmp280).

## Technical data

| Parameter                           | Technical Data                                    | Environment & Settings                          |
| ----------------------------------- | ------------------------------------------------- | ----------------------------------------------- |
| **Operation range**                 | `300..1100 hPa`.                                  | `-40..+85 °C`                                   |
| **Absolute accuracy**               | `± 1 hPa`                                         | `0..+65 °C`                                     |
| **Relative accuracy**               | `±0.12 hPa` (typical), equivalent to `±1 m`.      | `+25..+40 °C` @ `700..900 hPa`                  |
| **Absolute accuracy pressure**      | `300..1100 hPa` (typical)                         | `0..+65 °C`                                     |
| **Relative accuracy pressure**      | `900..1100 hPa` (typical)                         | `25..+40 °C`                                    |
| **Average current consumption**     | `3.4 μA` (typical)                                | Standard power mode @ `1 Hz` data refresh rate  |
| **Average current consumption**     | `2.74 μA` (typical)                               | Ultra-low power mode @ `1 Hz` data refresh rate |
| **Average current consumption**     | `0.1 μA` (typical)                                | Sleep mode @ `0 Hz` data refresh rate           |
| **Average measurement time**        | `5.5 msec`                                        | Ultra-low power mode                            |
| **Supply voltage VDDIO**            | `1.2..3.6 V`                                      |                                                 |
| **Supply voltage VDD**              | `1.71..3.6 V`                                     |                                                 |
| **Data resolution**                 | `0.01 hPa` (equivalent to `< 10 cm`),  `0.01 °C`. |                                                 |
| **Temperature coefficient offset**  | `± 0.12 hPa` (typical) equivalent to `±1 m`       | `+25°..+40 °C` @ `900 hPa`                      |
| **Interface**                       | `I²C` & `SPI`                                     |                                                 |
| **Package dimensions**              | 8-Pin LGA with metal: `2.0 x 2.5 x 0.95 mm³`      |                                                 |

(For more information refer to the sensor's [product page](https://www.bosch-sensortec.com/bst/products/all_products/bmp280))

## License

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
