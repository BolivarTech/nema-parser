//! GNSS Multi-System NMEA Parser Example
//!
//! This binary reads NMEA sentences from a serial port, parses them using the GNSS multi-system parser,
//! and prints individual GNSS system data as well as a fused position estimate.
//!
//! # Usage
//!
//! Configure the serial port name and baud rate as needed. The program will continuously read and process
//! NMEA data, displaying parsed results to the console.

use nema_parser::gnss_multignss_parser::GnssData;
use std::io::{self, Write};
use std::time::Duration;

/// Entry point for the GNSS NMEA parser example.
///
/// Opens the configured serial port, reads NMEA sentences, parses them, and prints GNSS system and fused position data.
/// The loop continues until a serial port error occurs or the program is terminated.
fn main() {
    let port_name = "COM12";
    let baud_rate = 9600;
    let mut gnss = GnssData::new();

    // Attempt to open the serial port with specified settings.
    let serial_result = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(1000))
        .data_bits(serialport::DataBits::Eight)
        .open();

    let mut port = match serial_result {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Failed to open serial port: {}", e);
            return;
        }
    };

    let mut buffer = [0u8; 1024];
    loop {
        // Read data from the serial port into the buffer.
        match port.read(&mut buffer) {
            Ok(bytes_read) => {
                if bytes_read == 0 {
                    continue;
                }
                let data = String::from_utf8_lossy(&buffer[..bytes_read]);
                io::stdout().flush().unwrap();
                for line in data.lines() {
                    // Process only lines that start with '$' (NMEA sentences).
                    if line.starts_with('$') {
                        gnss.feed_nmea(line);

                        // Calculate fused position after processing NMEA data.
                        //gnss.calculate_fused_position();
                        gnss.calculate_advanced_fused_position();

                        // Print individual system data.
                        for (system, sys_data) in &gnss.systems {
                            let sat_count = sys_data.satellites_info.len();
                            println!("System: {} | Satellites: {}", system, sat_count);
                            if let (Some(lat), Some(lon)) = (sys_data.latitude, sys_data.longitude) {
                                print!("System: {} | Lat: {:.6}, Lon: {:.6}", system, lat, lon);
                                if let Some(alt) = sys_data.altitude {
                                    print!(" | Alt: {:.1}m", alt);
                                }
                                println!();
                            } else {
                                println!("System: {} | Coordinates not available", system);
                            }
                            if let Some(hdop) = sys_data.hdop {
                                print!("System: {} | HDOP: {:.2}", system, hdop);
                                if let Some(vdop) = sys_data.vdop {
                                    print!(" | VDOP: {:.2}", vdop);
                                }
                                if let Some(pdop) = sys_data.pdop {
                                    print!(" | PDOP: {:.2}", pdop);
                                }
                                let acc = sys_data.accuracy;
                                print!(" | Sys. Acc: {:.2}m", acc);
                                println!();
                            }
                        }

                        // Print comprehensive fused position data.
                        if let Some(fused) = &gnss.fused_position {
                            println!("┌─ FUSED POSITION DATA ─────────────────────────────────────────┐");
                            println!("│ Latitude:         {:.7}°", fused.latitude);
                            println!("│ Longitude:        {:.7}°", fused.longitude);
                            println!("│ Altitude:         {:.2} m", fused.altitude);
                            println!("│ Horizontal Acc:   {:.2} m", fused.estimated_accuracy);
                            println!("│ Altitude Acc:     {:.2} m", fused.altitude_accuracy);
                            println!("│ Contributing:     {:?}", fused.contributing_systems);
                            println!("└───────────────────────────────────────────────────────────────┘");
                        } else {
                            println!("FUSED | Position not available");
                        }
                        println!("---");
                    }
                }
            }
            // Handle serial port timeout errors gracefully.
            Err(ref e) if e.kind() == io::ErrorKind::TimedOut => continue,
            // Handle other serial port errors and exit.
            Err(e) => {
                eprintln!("Serial port error: {}", e);
                break;
            }
        }
    }
}
