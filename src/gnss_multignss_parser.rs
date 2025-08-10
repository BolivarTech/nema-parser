//! GNSS Multi-System NMEA Parser
//!
//! This module provides data structures and logic for parsing NMEA sentences from multiple GNSS systems
//! (GPS, GLONASS, GALILEO, BEIDOU). It supports extracting satellite information, position, DOP values,
//! and fusing positions from different systems for improved accuracy.
//!
//! # Features
//! - Parses GGA, RMC, VTG, GSA, GSV, and GLL sentences for supported systems
//! - Tracks satellite info and usage per system
//! - Calculates fused position using weighted averaging and advanced filtering
//! - Provides utility functions for latitude/longitude parsing
//!
//! # Usage
//!
//! ```rust
//! use crate::gnss_multignss_parser::GnssData;
//! let mut gnss = GnssData::new();
//! gnss.feed_nmea("$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
//! gnss.calculate_fused_position();
//! if let Some(fused) = &gnss.fused_position {
//!     println!("Fused position: {}, {}", fused.latitude, fused.longitude);
//! }
//! ```

use std::collections::HashMap;

/// Information about a single satellite, including PRN, elevation, azimuth, and SNR.
#[derive(Debug, Default, Clone)]
pub struct SatelliteInfo {
    /// Pseudo-Random Noise number (satellite identifier)
    pub prn: u16,
    /// Elevation angle in degrees
    pub elevation: Option<u8>,
    /// Azimuth angle in degrees
    pub azimuth: Option<u16>,
    /// Signal-to-noise ratio in dBHz
    pub snr: Option<u8>,
}

/// Data for a single GNSS system (GPS, GLONASS, GALILEO, BEIDOU).
#[derive(Debug, Default, Clone)]
pub struct GnssSystemData {
    /// List of satellites used for position fix
    pub satellites_used: Vec<u16>,
    /// Information about all tracked satellites
    pub satellites_info: HashMap<u16, SatelliteInfo>,
    /// Position Dilution of Precision
    pub pdop: Option<f64>,
    /// Horizontal Dilution of Precision
    pub hdop: Option<f64>,
    /// Vertical Dilution of Precision
    pub vdop: Option<f64>,
    /// Latitude in decimal degrees
    pub latitude: Option<f64>,
    /// Longitude in decimal degrees
    pub longitude: Option<f64>,
}

/// Main GNSS data structure holding parsed information and fused position.
#[derive(Debug, Default, Clone)]
pub struct GnssData {
    /// UTC time from NMEA sentence
    pub time: Option<String>,
    /// Latitude in decimal degrees
    pub latitude: Option<f64>,
    /// Longitude in decimal degrees
    pub longitude: Option<f64>,
    /// Fix quality indicator
    pub fix_quality: Option<u8>,
    /// Number of satellites used for fix
    pub num_satellites: Option<u8>,
    /// Altitude above mean sea level in meters
    pub altitude: Option<f64>,
    /// Speed over ground in knots
    pub speed_knots: Option<f64>,
    /// Track angle in degrees
    pub track_angle: Option<f64>,
    /// Date in DDMMYY format
    pub date: Option<String>,
    /// Data for each GNSS system
    pub systems: HashMap<&'static str, GnssSystemData>,
    /// Fused position calculated from available systems
    pub fused_position: Option<FusedPosition>,
}

/// Fused position result from multiple GNSS systems.
#[derive(Debug, Clone)]
pub struct FusedPosition {
    /// Fused latitude in decimal degrees
    pub latitude: f64,
    /// Fused longitude in decimal degrees
    pub longitude: f64,
    /// Estimated accuracy in meters
    pub estimated_accuracy: f64,
    /// List of contributing GNSS systems
    pub contributing_systems: Vec<String>,
}

impl GnssData {
    /// Creates a new `GnssData` instance with all supported GNSS systems initialized.
    ///
    /// # Example
    /// ```
    /// let gnss = GnssData::new();
    /// ```
    pub fn new() -> Self {
        let mut systems = HashMap::new();
        systems.insert("GPS", GnssSystemData::default());
        systems.insert("GLONASS", GnssSystemData::default());
        systems.insert("GALILEO", GnssSystemData::default());
        systems.insert("BEIDOU", GnssSystemData::default());
        Self { systems, ..Default::default() }
    }

    /// Parses and updates GNSS data from a GGA sentence.
    fn update_gga(&mut self, parts: &[&str]) {
        let lat = parse_lat(parts.get(2), parts.get(3));
        let lon = parse_lon(parts.get(4), parts.get(5));
        self.time = parts.get(1).map(|s| s.to_string());
        self.latitude = lat;
        self.longitude = lon;
        self.fix_quality = parts.get(6).and_then(|s| s.parse().ok());
        self.num_satellites = parts.get(7).and_then(|s| s.parse().ok());
        self.altitude = parts.get(9).and_then(|s| s.parse().ok());

        // Update coordinates for all systems that have satellites
        for (_, system_data) in self.systems.iter_mut() {
            if system_data.satellites_info.len() > 0 {
                system_data.latitude = lat;
                system_data.longitude = lon;
            } else {
                system_data.latitude = None;
                system_data.longitude = None;
            }
        }
    }

    /// Parses and updates GNSS data from an RMC sentence.
    fn update_rmc(&mut self, parts: &[&str]) {
        let lat = parse_lat(parts.get(3), parts.get(4));
        let lon = parse_lon(parts.get(5), parts.get(6));
        self.time = parts.get(1).map(|s| s.to_string());
        self.latitude = lat;
        self.longitude = lon;
        self.speed_knots = parts.get(7).and_then(|s| s.parse().ok());
        self.track_angle = parts.get(8).and_then(|s| s.parse().ok());
        self.date = parts.get(9).map(|s| s.to_string());

        // Update coordinates for all systems that have satellites
        for (_, system_data) in self.systems.iter_mut() {
            if system_data.satellites_info.len() > 0 {
                system_data.latitude = lat;
                system_data.longitude = lon;
            } else {
                system_data.latitude = None;
                system_data.longitude = None;
            }
        }
    }

    /// Parses and updates GNSS data from a VTG sentence.
    fn update_vtg(&mut self, parts: &[&str]) {
        self.speed_knots = parts.get(5).and_then(|s| s.parse().ok());
    }

    /// Parses and updates GNSS system data from a GSA sentence.
    fn update_gsa(&mut self, parts: &[&str]) {
        let mut gps_ids = Vec::new();
        for i in 3..=14 {
            if let Some(Ok(prn)) = parts.get(i).map(|s| s.parse()) {
                gps_ids.push(prn);
            }
        }

        // Find DOP values by searching for the first three non-empty numeric fields after satellites
        let mut dop_values = Vec::new();
        for i in 15..parts.len() {
            if let Some(part) = parts.get(i) {
                let clean_part = part.split('*').next().unwrap_or(part);
                if !clean_part.is_empty() {
                    if let Ok(value) = clean_part.parse::<f64>() {
                        dop_values.push(value);
                        if dop_values.len() == 3 {
                            break;
                        }
                    }
                }
            }
        }

        let pdop = dop_values.get(0).copied();
        let hdop = dop_values.get(1).copied();
        let vdop = dop_values.get(2).copied();

        let mut updated_systems = Vec::new();
        for prn in &gps_ids {
            match prn {
                1..=32 => {
                    self.systems.get_mut("GPS").unwrap().satellites_used.push(*prn as u16);
                    if !updated_systems.contains(&"GPS") {
                        updated_systems.push("GPS");
                    }
                },
                65..=96 => {
                    self.systems.get_mut("GLONASS").unwrap().satellites_used.push(*prn as u16);
                    if !updated_systems.contains(&"GLONASS") {
                        updated_systems.push("GLONASS");
                    }
                },
                201..=236 => {
                    self.systems.get_mut("BEIDOU").unwrap().satellites_used.push(*prn as u16);
                    if !updated_systems.contains(&"BEIDOU") {
                        updated_systems.push("BEIDOU");
                    }
                },
                301..=336 => {
                    self.systems.get_mut("GALILEO").unwrap().satellites_used.push(*prn as u16);
                    if !updated_systems.contains(&"GALILEO") {
                        updated_systems.push("GALILEO");
                    }
                },
                _ => {}
            }
        }
        // Only update error values for systems that received satellites in this GSA sentence
        for sys_name in updated_systems {
            if let Some(sys) = self.systems.get_mut(sys_name) {
                sys.pdop = pdop;
                sys.hdop = hdop;
                sys.vdop = vdop;
            }
        }
    }

    /// Parses and updates satellite information from a GSV sentence for the specified system.
    fn update_gsv(&mut self, parts: &[&str], system: &str) {
        if let Some(sys_data) = self.systems.get_mut(system) {
            let mut i = 4;
            while i + 3 < parts.len() {
                if let Some(Ok(prn)) = parts.get(i).map(|s| s.parse()) {
                    let elevation = parts.get(i + 1).and_then(|s| s.parse().ok());
                    let azimuth = parts.get(i + 2).and_then(|s| s.parse().ok());
                    let snr = parts.get(i + 3).and_then(|s| s.trim_end_matches('*').parse().ok());
                    sys_data.satellites_info.insert(
                        prn,
                        SatelliteInfo {
                            prn,
                            elevation,
                            azimuth,
                            snr,
                        },
                    );
                }
                i += 4;
            }
        }
    }

    /// Parses and updates latitude/longitude from a GLL sentence for the specified system.
    fn update_gll(&mut self, parts: &[&str], system: &str) {
        let lat = parse_lat(parts.get(1), parts.get(2));
        let lon = parse_lon(parts.get(3), parts.get(4));
        self.latitude = lat;
        self.longitude = lon;
        if let Some(sys) = self.systems.get_mut(system) {
            if sys.satellites_info.len() > 0 {
                sys.latitude = lat;
                sys.longitude = lon;
            } else {
                sys.latitude = None;
                sys.longitude = None;
            }
        }
    }

    /// Feeds a single NMEA sentence to the parser and updates internal state.
    ///
    /// # Arguments
    /// * `sentence` - A string slice containing the NMEA sentence.
    ///
    /// # Example
    /// ```
    /// gnss.feed_nmea("$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
    /// ```
    pub fn feed_nmea(&mut self, sentence: &str) {
        let sentence = sentence.trim_start_matches('$');
        let parts: Vec<&str> = sentence.split(',').collect();
        match parts.first().filter(|s| s.len() >= 5).map(|s| &s[0..5]) {
            Some("GNGGA") => self.update_gga(&parts),
            Some("GNRMC") => self.update_rmc(&parts),
            Some("GNVTG") => self.update_vtg(&parts),
            Some("GNGSA") => self.update_gsa(&parts),
            Some("GPGSV") => self.update_gsv(&parts, "GPS"),
            Some("GLGSV") => self.update_gsv(&parts, "GLONASS"),
            Some("GAGSV") => self.update_gsv(&parts, "GALILEO"),
            Some("BDGSV") => self.update_gsv(&parts, "BEIDOU"),
            Some("GPGLL") => self.update_gll(&parts, "GPS"),
            Some("GLGLL") => self.update_gll(&parts, "GLONASS"),
            Some("GAGLL") => self.update_gll(&parts, "GALILEO"),
            Some("BDGLL") => self.update_gll(&parts, "BEIDOU"),
            _ => {}
        }
    }

    /// Calculates a fused position from all available GNSS systems using weighted averaging.
    ///
    /// The fused position is stored in `self.fused_position`.
    ///
    /// # Example
    /// ```
    /// gnss.calculate_fused_position();
    /// if let Some(fused) = &gnss.fused_position {
    ///     println!("Fused position: {}, {}", fused.latitude, fused.longitude);
    /// }
    /// ```
    pub fn calculate_fused_position(&mut self) {
        let mut valid_positions = Vec::new();

        for (system_name, system_data) in &self.systems {
            if let (Some(lat), Some(lon), Some(hdop)) = (system_data.latitude, system_data.longitude, system_data.hdop) {
                if system_data.satellites_info.len() >= 4 { // Minimum satellites for 3D fix
                    valid_positions.push((system_name.to_string(), lat, lon, hdop));
                }
            }
        }

        if valid_positions.is_empty() {
            self.fused_position = None;
            return;
        }

        if valid_positions.len() == 1 {
            let (system, lat, lon, hdop) = &valid_positions[0];
            self.fused_position = Some(FusedPosition {
                latitude: *lat,
                longitude: *lon,
                estimated_accuracy: hdop * 3.0, // Rough accuracy estimate in meters
                contributing_systems: vec![system.clone()],
            });
            return;
        }

        // Weighted average using inverse HDOP as weights
        let mut weighted_lat = 0.0;
        let mut weighted_lon = 0.0;
        let mut total_weight = 0.0;
        let mut contributing_systems = Vec::new();

        for (system, lat, lon, hdop) in &valid_positions {
            let weight = 1.0 / (hdop + 0.1); // Add small value to avoid division by zero
            weighted_lat += lat * weight;
            weighted_lon += lon * weight;
            total_weight += weight;
            contributing_systems.push(system.clone());
        }

        if total_weight > 0.0 {
            let fused_lat = weighted_lat / total_weight;
            let fused_lon = weighted_lon / total_weight;

            // Calculate estimated accuracy based on weighted HDOP
            let weighted_hdop: f64 = valid_positions.iter()
                .map(|(_, _, _, hdop)| {
                    let weight = 1.0 / (hdop + 0.1);
                    hdop * weight
                })
                .sum::<f64>() / total_weight;

            self.fused_position = Some(FusedPosition {
                latitude: fused_lat,
                longitude: fused_lon,
                estimated_accuracy: weighted_hdop * 2.0, // Improved accuracy estimate
                contributing_systems,
            });
        } else {
            self.fused_position = None;
        }
    }

    /// Calculates an advanced fused position using a Kalman-like filtering approach.
    ///
    /// The fused position is stored in `self.fused_position`.
    pub fn calculate_advanced_fused_position(&mut self) {
        let mut valid_positions = Vec::new();

        for (system_name, system_data) in &self.systems {
            if let (Some(lat), Some(lon), Some(hdop), Some(pdop)) =
                (system_data.latitude, system_data.longitude, system_data.hdop, system_data.pdop) {
                if system_data.satellites_info.len() >= 4 {
                    valid_positions.push((system_name.to_string(), lat, lon, hdop, pdop));
                }
            }
        }

        if valid_positions.is_empty() {
            self.fused_position = None;
            return;
        }

        // Kalman-like filtering approach
        let mut weighted_lat = 0.0;
        let mut weighted_lon = 0.0;
        let mut total_weight = 0.0;
        let mut contributing_systems = Vec::new();

        for (system, lat, lon, hdop, pdop) in &valid_positions {
            // Combined weight using both HDOP and PDOP
            let combined_dop = (hdop * hdop + pdop * pdop).sqrt();
            let weight = 1.0 / (combined_dop + 0.1);

            weighted_lat += lat * weight;
            weighted_lon += lon * weight;
            total_weight += weight;
            contributing_systems.push(system.clone());
        }

        if total_weight > 0.0 {
            let fused_lat = weighted_lat / total_weight;
            let fused_lon = weighted_lon / total_weight;

            // Calculate confidence interval
            let variance: f64 = valid_positions.iter()
                .map(|(_, lat, lon, hdop, _)| {
                    let weight = 1.0 / (*hdop + 0.1);
                    let lat_diff = lat - fused_lat;
                    let lon_diff = lon - fused_lon;
                    weight * (lat_diff * lat_diff + lon_diff * lon_diff)
                })
                .sum::<f64>() / total_weight;

            let estimated_accuracy = variance.sqrt() * 111000.0; // Convert to meters roughly

            self.fused_position = Some(FusedPosition {
                latitude: fused_lat,
                longitude: fused_lon,
                estimated_accuracy: estimated_accuracy.max(1.0), // Minimum 1 meter accuracy
                contributing_systems,
            });
        } else {
            self.fused_position = None;
        }
    }
}

/// Parses latitude from NMEA format to decimal degrees.
///
/// # Arguments
/// * `value` - Latitude value as string (DDMM.MMMM)
/// * `hemi` - Hemisphere ("N" or "S")
///
/// # Returns
/// * `Option<f64>` - Latitude in decimal degrees
fn parse_lat(value: Option<&&str>, hemi: Option<&&str>) -> Option<f64> {
    let val = value?.parse::<f64>().ok()?;
    let deg = (val / 100.0).floor();
    let min = val % 100.0;
    let mut result = deg + min / 60.0;
    if hemi? == &"S" { result *= -1.0; }
    Some(result)
}

/// Parses longitude from NMEA format to decimal degrees.
///
/// # Arguments
/// * `value` - Longitude value as string (DDDMM.MMMM)
/// * `hemi` - Hemisphere ("E" or "W")
///
/// # Returns
/// * `Option<f64>` - Longitude in decimal degrees
fn parse_lon(value: Option<&&str>, hemi: Option<&&str>) -> Option<f64> {
    let val = value?.parse::<f64>().ok()?;
    let deg = (val / 100.0).floor();
    let min = val % 100.0;
    let mut result = deg + min / 60.0;
    if hemi? == &"W" { result *= -1.0; }
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gnssdata_new() {
        let gnss = GnssData::new();
        assert!(gnss.systems.contains_key("GPS"));
        assert!(gnss.systems.contains_key("GLONASS"));
        assert!(gnss.systems.contains_key("GALILEO"));
        assert!(gnss.systems.contains_key("BEIDOU"));
    }

    #[test]
    fn test_feed_nmea_gga() {
        let mut gnss = GnssData::new();
        let gga = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        gnss.feed_nmea(gga);
        assert_eq!(gnss.time, Some("123519".to_string()));
        assert!(gnss.latitude.is_some());
        assert!(gnss.longitude.is_some());
        assert_eq!(gnss.fix_quality, Some(1));
        assert_eq!(gnss.num_satellites, Some(8));
        assert_eq!(gnss.altitude, Some(545.4));
    }

    #[test]
    fn test_feed_nmea_gps_gsv() {
        let mut gnss = GnssData::new();
        let gsv = "$GPGSV,2,1,08,01,40,083,41,02,17,308,43,03,13,172,42,04,09,020,39*7C";
        gnss.feed_nmea(gsv);
        let gps_info = &gnss.systems["GPS"].satellites_info;
        assert_eq!(gps_info.len(), 4);
        assert!(gps_info.contains_key(&1));
        assert!(gps_info.contains_key(&2));
        assert!(gps_info.contains_key(&3));
        assert!(gps_info.contains_key(&4));
        let sat1 = gps_info.get(&1).unwrap();
        assert_eq!(sat1.prn, 1);
        assert_eq!(sat1.elevation, Some(40));
        assert_eq!(sat1.azimuth, Some(83));
        assert_eq!(sat1.snr, Some(41));
    }

    #[test]
    fn test_feed_nmea_glonass_gsv() {
        let mut gnss = GnssData::new();
        let gsv = "$GLGSV,2,1,08,67,14,186,09,68,49,228,26,69,42,308,,77,15,064,17*61";
        gnss.feed_nmea(gsv);
        let glonass_info = &gnss.systems["GLONASS"].satellites_info;
        assert_eq!(glonass_info.len(), 4);
        assert!(glonass_info.contains_key(&67));
        assert!(glonass_info.contains_key(&68));
        assert!(glonass_info.contains_key(&69));
        assert!(glonass_info.contains_key(&77));
        let sat67 = glonass_info.get(&67).unwrap();
        assert_eq!(sat67.prn, 67);
        assert_eq!(sat67.elevation, Some(14));
        assert_eq!(sat67.azimuth, Some(186));
        assert_eq!(sat67.snr, Some(9));
    }

    #[test]
    fn test_feed_nmea_galileo_gsv() {
        let mut gnss = GnssData::new();
        let gsv = "$GAGSV,1,1,04,301,45,123,35,302,30,045,40,303,60,234,45,304,25,156,38*XX";
        gnss.feed_nmea(gsv);
        let galileo_info = &gnss.systems["GALILEO"].satellites_info;
        assert_eq!(galileo_info.len(), 4);
        assert!(galileo_info.contains_key(&301));
        assert!(galileo_info.contains_key(&302));
        assert!(galileo_info.contains_key(&303));
        assert!(galileo_info.contains_key(&304));
        let sat301 = galileo_info.get(&301).unwrap();
        assert_eq!(sat301.prn, 301);
        assert_eq!(sat301.elevation, Some(45));
        assert_eq!(sat301.azimuth, Some(123));
        assert_eq!(sat301.snr, Some(35));
    }

    #[test]
    fn test_feed_nmea_beidou_gsv() {
        let mut gnss = GnssData::new();
        let gsv = "$BDGSV,1,1,04,201,45,123,35,202,30,045,40,203,60,234,45,204,25,156,38*XX";
        gnss.feed_nmea(gsv);
        let beidou_info = &gnss.systems["BEIDOU"].satellites_info;
        assert_eq!(beidou_info.len(), 4);
        assert!(beidou_info.contains_key(&201));
        assert!(beidou_info.contains_key(&202));
        assert!(beidou_info.contains_key(&203));
        assert!(beidou_info.contains_key(&204));
        let sat201 = beidou_info.get(&201).unwrap();
        assert_eq!(sat201.prn, 201);
        assert_eq!(sat201.elevation, Some(45));
        assert_eq!(sat201.azimuth, Some(123));
        assert_eq!(sat201.snr, Some(35));
    }

    #[test]
    fn test_feed_nmea_gsa_gps() {
        let mut gnss = GnssData::new();
        let gsa = "$GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.2,0.9,2.1*39";
        gnss.feed_nmea(gsa);
        let gps_used = &gnss.systems["GPS"].satellites_used;
        assert!(gps_used.contains(&1));
        assert!(gps_used.contains(&2));
        assert!(gps_used.contains(&3));
        assert!(gps_used.contains(&4));
        assert_eq!(gnss.systems["GPS"].pdop, Some(1.2));
        assert_eq!(gnss.systems["GPS"].hdop, Some(0.9));
        assert_eq!(gnss.systems["GPS"].vdop, Some(2.1));
    }

    #[test]
    fn test_feed_nmea_gsa_glonass() {
        let mut gnss = GnssData::new();
        let gsa = "$GNGSA,A,3,67,68,69,77,78,79,86,87,88,,,,,1.8,1.1,1.4*3F";
        gnss.feed_nmea(gsa);

        // Debug output
        println!("GLONASS satellites_used: {:?}", gnss.systems["GLONASS"].satellites_used);
        println!("GLONASS pdop: {:?}", gnss.systems["GLONASS"].pdop);
        println!("GLONASS hdop: {:?}", gnss.systems["GLONASS"].hdop);
        println!("GLONASS vdop: {:?}", gnss.systems["GLONASS"].vdop);

        let glonass_used = &gnss.systems["GLONASS"].satellites_used;
        assert!(glonass_used.contains(&67));
        assert!(glonass_used.contains(&68));
        assert!(glonass_used.contains(&69));
        assert!(glonass_used.contains(&77));
        assert_eq!(gnss.systems["GLONASS"].pdop, Some(1.8));
        assert_eq!(gnss.systems["GLONASS"].hdop, Some(1.1));
        assert_eq!(gnss.systems["GLONASS"].vdop, Some(1.4));
    }

    #[test]
    fn test_feed_nmea_gsa_galileo() {
        let mut gnss = GnssData::new();
        let gsa = "$GNGSA,A,3,301,302,303,304,305,306,,,,,,,2.1,1.3,1.6*XX";
        gnss.feed_nmea(gsa);
        let galileo_used = &gnss.systems["GALILEO"].satellites_used;
        assert!(galileo_used.contains(&301));
        assert!(galileo_used.contains(&302));
        assert!(galileo_used.contains(&303));
        assert!(galileo_used.contains(&304));
        assert_eq!(gnss.systems["GALILEO"].pdop, Some(2.1));
        assert_eq!(gnss.systems["GALILEO"].hdop, Some(1.3));
        assert_eq!(gnss.systems["GALILEO"].vdop, Some(1.6));
    }

    #[test]
    fn test_feed_nmea_gsa_beidou() {
        let mut gnss = GnssData::new();
        let gsa = "$GNGSA,A,3,201,202,203,204,205,206,,,,,,,1.5,0.8,1.2*XX";
        gnss.feed_nmea(gsa);
        let beidou_used = &gnss.systems["BEIDOU"].satellites_used;
        assert!(beidou_used.contains(&201));
        assert!(beidou_used.contains(&202));
        assert!(beidou_used.contains(&203));
        assert!(beidou_used.contains(&204));
        assert_eq!(gnss.systems["BEIDOU"].pdop, Some(1.5));
        assert_eq!(gnss.systems["BEIDOU"].hdop, Some(0.8));
        assert_eq!(gnss.systems["BEIDOU"].vdop, Some(1.2));
    }

    #[test]
    fn test_coordinates_update_for_systems_with_satellites() {
        let mut gnss = GnssData::new();

        // Add GPS satellites
        let gps_gsv = "$GPGSV,1,1,04,01,40,083,41,02,17,308,43,03,13,172,42,04,09,020,39*XX";
        gnss.feed_nmea(gps_gsv);

        // Add GLONASS satellites
        let glonass_gsv = "$GLGSV,1,1,04,67,14,186,09,68,49,228,26,69,42,308,,77,15,064,17*XX";
        gnss.feed_nmea(glonass_gsv);

        // Update coordinates via GGA
        let gga = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        gnss.feed_nmea(gga);

        // Systems with satellites should have coordinates
        assert!(gnss.systems["GPS"].latitude.is_some());
        assert!(gnss.systems["GPS"].longitude.is_some());
        assert!(gnss.systems["GLONASS"].latitude.is_some());
        assert!(gnss.systems["GLONASS"].longitude.is_some());

        // Systems without satellites should not have coordinates
        assert!(gnss.systems["GALILEO"].latitude.is_none());
        assert!(gnss.systems["GALILEO"].longitude.is_none());
        assert!(gnss.systems["BEIDOU"].latitude.is_none());
        assert!(gnss.systems["BEIDOU"].longitude.is_none());
    }

    #[test]
    fn test_fused_position_calculation() {
        let mut gnss = GnssData::new();

        // Add GPS satellites and coordinates
        let gps_gsv = "$GPGSV,1,1,04,01,40,083,41,02,17,308,43,03,13,172,42,04,09,020,39*XX";
        gnss.feed_nmea(gps_gsv);
        let gps_gsa = "$GNGSA,A,3,01,02,03,04,05,06,07,08,,,,,1.2,0.9,2.1*39";
        gnss.feed_nmea(gps_gsa);

        // Add GLONASS satellites and coordinates
        let glonass_gsv = "$GLGSV,1,1,04,67,14,186,09,68,49,228,26,69,42,308,,77,15,064,17*XX";
        gnss.feed_nmea(glonass_gsv);
        let glonass_gsa = "$GNGSA,A,3,67,68,69,77,78,79,86,87,,,,,1.8,1.1,1.4*3F";
        gnss.feed_nmea(glonass_gsa);

        // Update coordinates
        let gga = "$GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        gnss.feed_nmea(gga);

        // Calculate fused position
        gnss.calculate_fused_position();

        assert!(gnss.fused_position.is_some());
        let fused = gnss.fused_position.as_ref().unwrap();
        assert!(fused.contributing_systems.contains(&"GPS".to_string()));
        assert!(fused.contributing_systems.contains(&"GLONASS".to_string()));
        assert!(fused.estimated_accuracy > 0.0);
    }

    #[test]
    fn test_parse_lat_lon() {
        let lat = parse_lat(Some(&"4807.038"), Some(&"N"));
        let lon = parse_lon(Some(&"01131.000"), Some(&"E"));
        assert!(lat.is_some());
        assert!(lon.is_some());
        let lat_val = lat.unwrap();
        let lon_val = lon.unwrap();
        assert!((lat_val - 48.1173).abs() < 0.0001);
        assert!((lon_val - 11.5166667).abs() < 0.0001);
    }
}
