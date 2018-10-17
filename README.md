# `bmp280`

> no_std driver for the bmp280 (pressure sensor).

[![Build Status](https://travis-ci.org/copterust/bmp280.svg?branch=master)](https://travis-ci.org/copterust/bmp280)

## What works

- Pressure sensor

## Supported chips

* `BMP280`;


## Basic usage

Include [library](https://crates.io/crates/bmp280) as a dependency in your Cargo.toml
[![crates.io](http://meritbadge.herokuapp.com/bmp280?style=flat-square)](https://crates.io/crates/bmp280):

```
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

## License

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
