# GNSS Multi-System NMEA Parser

The module provides data structures and logic for parsing NMEA sentences from multiple GNSS systems
(GPS, GLONASS, GALILEO, BEIDOU). It supports extracting satellite information, position, DOP values,
and fusing positions from different systems for improved accuracy.  

# GNSS Multi-System NMEA Parser Example

The binary reads NMEA sentences from a serial port, parses them using the GNSS multi-system parser,
and prints individual GNSS system data as well as a fused position estimate.  


```markdown
# nema-parser

A Rust library for parsing NMEA sentences, commonly used in GPS and marine navigation systems.

## Features

- Fast and efficient NMEA sentence parsing
- Easy integration with Rust projects
- Extensible for custom sentence types
- Supports common NMEA sentence types (e\.g\., GGA, RMC, GSA)
- Error handling for invalid or malformed sentences
- Lightweight and dependency-free

## Installation

Add the following to your `Cargo\.toml`:

```toml
[dependencies]
nema-parser = "0.1"
```

## Usage

```rust
use nema_parser::NmeaParser;

fn main() {
    let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    let result = NmeaParser::parse(sentence);
    println!("{:?}", result);
}
```

## Examples

### Parsing a GGA Sentence

```rust
use nema_parser::NmeaParser;

fn main() {
    let gga_sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    let parsed = NmeaParser::parse(gga_sentence).unwrap();
    println!("Parsed GGA: {:?}", parsed);
}
```

### Handling Invalid Sentences

```rust
use nema_parser::NmeaParser;

fn main() {
    let invalid_sentence = "$INVALID,NMEA,SENTENCE";
    match NmeaParser::parse(invalid_sentence) {
        Ok(parsed) => println!("Parsed: {:?}", parsed),
        Err(e) => println!("Error: {}", e),
    }
}
```

## Building

```sh
cargo build
```

## Running Tests

```sh
cargo test
```

## Documentation

Generate and view the documentation locally:

```sh
cargo doc --open
```

## Contributing

Pull requests are welcome\. For major changes, please open an issue first to discuss what you would like to change\.

## Roadmap

- Add support for additional NMEA sentence types
- Improve error messages for invalid sentences
- Add async support for streaming NMEA data
- Provide more detailed examples and tutorials

## License

Distributed under the MIT License\. See [LICENSE](LICENSE.md) for more information\.

## Acknowledgments

- Inspired by the need for efficient NMEA parsing in Rust
- Contributions from the open-source community
```

