# bmp388

> no_std driver for the bmp388 (pressure sensor).

## What works

- Pressure sensor

## Supported chips

* `BMP388`;


## Basic usage

Include [library](https://crates.io/crates/bmp280) as a dependency in your Cargo.toml
[![crates.io](http://meritbadge.herokuapp.com/bmp280?style=flat-square)](https://crates.io/crates/bmp280):

```
[dependencies.bmp388]
version = "<version>"
```

Use embedded-hal implementation to get I2C handle and delay then create bmp280 handle:

```rust
extern crate bmp388; // or just use bmp388; if 2018 edition is used.

// to create sensor with default configuration:
let mut lsm = bmp388::BMP388::new(i2c)?;
// to get pressure:
let pres = ps.pressure();
println!("{:?}", pres);
```

## Documentation

API Docs available on [docs.rs](todo).

## ToDo:

* Implement FiFo Buffer support

#Origin

Based on the [bmp280 crate](https://github.com/copterust/bmp280) by Roma Sokolov and Alexander Zhuravlev.

## License

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
