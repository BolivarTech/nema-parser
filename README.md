# GNSS Multi-System NMEA Parser

This crate provides fast and efficient parsing of NMEA sentences from multiple GNSS systems (GPS, GLONASS, GALILEO, BEIDOU). It extracts satellite information, position, DOP values, and can fuse positions from different systems for improved accuracy.

## Features

- Fast and efficient NMEA sentence parsing
- Easy integration with Rust projects
- Extensible for custom sentence types
- Supports common NMEA sentence types (e.g., GGA, RMC, GSA)
- Error handling for invalid or malformed sentences
- Lightweight and dependency-free

## Supported Sentence Types

- GGA: Global Positioning System Fix Data
- RMC: Recommended Minimum Specific GNSS Data
- GSA: GNSS DOP and Active Satellites
- Additional types can be added via extension

## Installation

Add the following to your `Cargo.toml`:

```toml
[dependencies]
nema-parser = "0.1"
```

## Usage

Basic usage example:

```rust
use nema_parser::{parse_nmea_sentence, NmeaSentence};

fn main() {
    let sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    match parse_nmea_sentence(sentence) {
        Ok(parsed) => {
            println!("Parsed sentence: {:?}", parsed);
            if let NmeaSentence::GGA(gga) = parsed {
                println!("Latitude: {}", gga.latitude);
                println!("Longitude: {}", gga.longitude);
            }
        }
        Err(e) => println!("Error parsing sentence: {}", e),
    }
}
```

## Examples

### Parsing a GGA Sentence

```rust
use nema_parser::{parse_nmea_sentence, NmeaSentence};

fn main() {
    let gga_sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    match parse_nmea_sentence(gga_sentence) {
        Ok(NmeaSentence::GGA(gga)) => {
            println!("Latitude: {}", gga.latitude);
            println!("Longitude: {}", gga.longitude);
        }
        Ok(other) => println!("Parsed other sentence: {:?}", other),
        Err(e) => println!("Error: {}", e),
    }
}
```

### Handling Invalid Sentences

```rust
use nema_parser::parse_nmea_sentence;

fn main() {
    let invalid_sentence = "$INVALID,NMEA,SENTENCE";
    match parse_nmea_sentence(invalid_sentence) {
        Ok(parsed) => println!("Parsed: {:?}", parsed),
        Err(e) => println!("Error: {}", e),
    }
}
```

## Usage Notes

- The parser returns an enum for supported sentence types.
- For unsupported or malformed sentences, an error is returned.
- Extend support by implementing additional sentence parsing logic.

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

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## Roadmap

- Add support for additional NMEA sentence types
- Improve error messages for invalid sentences
- Add async support for streaming NMEA data
- Provide more detailed examples and tutorials

## License

Distributed under the MIT License. See [LICENSE](LICENSE.md) for more information.

## Acknowledgments

- Inspired by the need for efficient NMEA parsing in Rust
- Contributions from the open-source community
