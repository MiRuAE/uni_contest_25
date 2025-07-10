use anyhow::{Error, Result};
use rclrs::*;
use sensor_msgs::msg::LaserScan;
use std::sync::{Arc, Mutex};
use std_msgs::msg::String as StringMsg;

// Define mission states
#[derive(Debug, Clone, Copy, PartialEq)]
enum MissionType {
    MissionA, // Camera mission
    MissionB, // Lidar mission (in narrow walls)
    MissionC, // Odometry mission
}

impl MissionType {
    fn as_str(&self) -> &'static str {
        match self {
            MissionType::MissionA => "MISSION_A",
            MissionType::MissionB => "MISSION_B",
            MissionType::MissionC => "MISSION_C",
        }
    }
}

// Configuration constants - More lenient conditions for better detection
const CLOSE_THRESHOLD: f32 = 1.0; // Below 1.0m is considered close
const MEDIUM_THRESHOLD: f32 = 2.0; // Below 2.0m is considered medium distance
const B_DETECTION_THRESHOLD: i32 = 3; // Consecutive detections needed for A->B transition (reduced from 5)
const C_DETECTION_THRESHOLD: i32 = 5; // Consecutive detections needed for B->C transition (reduced from 10)

// Shared state for mission transitions
#[derive(Debug)]
struct MissionState {
    current_mission: MissionType,
    current_mission_str: String,
    consecutive_b_detections: i32,
    consecutive_c_detections: i32,
}

impl Default for MissionState {
    fn default() -> Self {
        Self {
            current_mission: MissionType::MissionA,
            current_mission_str: String::new(),
            consecutive_b_detections: 0,
            consecutive_c_detections: 0,
        }
    }
}

struct MissionLauncher {
    _scan_subscription: Subscription<LaserScan>,
    mission_publisher: Publisher<StringMsg>,
    mission_state: Arc<Mutex<MissionState>>,
}

impl MissionLauncher {
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("mission_launcher")?;

        let mission_publisher = node.create_publisher::<StringMsg>("current_mission")?;
        let publisher_clone = mission_publisher.clone();

        let mission_state = Arc::new(Mutex::new(MissionState::default()));
        let state_clone = mission_state.clone();

        let _scan_subscription =
            node.create_subscription::<LaserScan, _>("/scan", move |msg: LaserScan| {
                if let Err(e) = Self::lidar_callback(msg, &publisher_clone, &state_clone) {
                    eprintln!("Error during scan process: {}", e);
                }
            })?;

        // Publish initial mission state
        let mut initial_msg = StringMsg::default();
        initial_msg.data = MissionType::MissionA.as_str().to_string();
        mission_publisher.publish(&initial_msg).ok();

        println!("Mission Launcher initialized - Starting in MISSION_A");

        Ok(Self {
            _scan_subscription,
            mission_publisher,
            mission_state,
        })
    }

    fn filter_valid_ranges(ranges: &[f32]) -> Vec<f32> {
        ranges
            .iter()
            .filter(|&&range| range.is_finite() && range > 0.0 && range < 10.0) // Exclude ranges above 10m
            .copied()
            .collect()
    }

    fn calculate_percentages(ranges: &[f32]) -> (f32, f32, f32) {
        let total = ranges.len() as f32;

        let close_count = ranges.iter().filter(|&&r| r < CLOSE_THRESHOLD).count() as f32;
        let medium_count = ranges
            .iter()
            .filter(|&&r| r >= CLOSE_THRESHOLD && r < MEDIUM_THRESHOLD)
            .count() as f32;
        let far_count = total - close_count - medium_count;

        (
            (close_count / total) * 100.0,
            (medium_count / total) * 100.0,
            (far_count / total) * 100.0,
        )
    }

    // Simple and effective narrow passage detection logic
    fn detect_narrow_passage(close_percent: f32, medium_percent: f32, mean_range: f32) -> bool {
        // Condition 1: High percentage of close and medium distance points
        let proximity_condition = (close_percent + medium_percent) > 50.0;
        
        // Condition 2: Short average distance
        let mean_condition = mean_range < 1.5;
        
        // Condition 3: Significant percentage of close points
        let close_condition = close_percent > 20.0;

        println!(
            "Narrow passage check - Close: {:.1}%, Medium: {:.1}%, Mean: {:.2}m",
            close_percent, medium_percent, mean_range
        );
        println!(
            "Conditions - Proximity: {}, Mean: {}, Close: {}",
            proximity_condition, mean_condition, close_condition
        );

        proximity_condition && (mean_condition || close_condition)
    }

    // Improved corridor exit detection logic
    fn detect_corridor_exit(close_percent: f32, medium_percent: f32, far_percent: f32, mean_range: f32) -> bool {
        // Condition 1: High percentage of far distance points
        let far_condition = far_percent > 40.0;
        
        // Condition 2: Increased average distance
        let mean_condition = mean_range > 2.5;
        
        // Condition 3: Reduced percentage of close points
        let close_reduction = close_percent < 25.0;

        println!(
            "Exit check - Far: {:.1}%, Mean: {:.2}m, Close: {:.1}%",
            far_percent, mean_range, close_percent
        );
        println!(
            "Exit conditions - Far: {}, Mean: {}, Close reduction: {}",
            far_condition, mean_condition, close_reduction
        );

        (far_condition && mean_condition) || (far_condition && close_reduction)
    }

    fn publish_mission(
        mission_name: &str,
        publisher: &Publisher<StringMsg>,
        state: &Arc<Mutex<MissionState>>,
    ) -> Result<(), Error> {
        let mut mission_state = state.lock().unwrap();

        if mission_name != mission_state.current_mission_str {
            let mut message = StringMsg::default();
            message.data = mission_name.to_string();
            publisher.publish(&message)?;
            mission_state.current_mission_str = mission_name.to_string();

            println!("Mission changed to: {}", mission_name);
        }
        Ok(())
    }

    fn lidar_callback(
        scan_msg: LaserScan,
        mission_publisher: &Publisher<StringMsg>,
        mission_state: &Arc<Mutex<MissionState>>,
    ) -> Result<(), Error> {
        if scan_msg.ranges.is_empty() {
            eprintln!("Empty scan message received");
            return Ok(());
        }

        // Filter valid ranges
        let valid_ranges = Self::filter_valid_ranges(&scan_msg.ranges);
        if valid_ranges.is_empty() {
            eprintln!("No valid range measurements");
            return Ok(());
        }

        // Calculate overall statistics
        let mean_range = valid_ranges.iter().sum::<f32>() / valid_ranges.len() as f32;
        let (close_percent, medium_percent, far_percent) =
            Self::calculate_percentages(&valid_ranges);

        // Mission state transitions
        {
            let mut state = mission_state.lock().unwrap();

            match state.current_mission {
                MissionType::MissionA => {
                    if Self::detect_narrow_passage(close_percent, medium_percent, mean_range) {
                        state.consecutive_b_detections += 1;
                        println!(
                            "Narrow passage detected ({}/{})",
                            state.consecutive_b_detections, B_DETECTION_THRESHOLD
                        );

                        if state.consecutive_b_detections >= B_DETECTION_THRESHOLD {
                            println!("Transitioning: MISSION_A -> MISSION_B");
                            state.current_mission = MissionType::MissionB;
                            state.consecutive_b_detections = 0;
                            drop(state); // Release lock before calling publish
                            Self::publish_mission("MISSION_B", mission_publisher, mission_state)?;
                        }
                    } else {
                        // Gradually reduce counter if conditions are not met
                        if state.consecutive_b_detections > 0 {
                            state.consecutive_b_detections -= 1;
                        }
                    }
                }

                MissionType::MissionB => {
                    if Self::detect_corridor_exit(close_percent, medium_percent, far_percent, mean_range) {
                        state.consecutive_c_detections += 1;
                        println!(
                            "Corridor exit detected ({}/{})",
                            state.consecutive_c_detections, C_DETECTION_THRESHOLD
                        );

                        if state.consecutive_c_detections >= C_DETECTION_THRESHOLD {
                            println!("*** CORRIDOR EXIT CONFIRMED ***");
                            println!("Transitioning: MISSION_B -> MISSION_C");
                            state.current_mission = MissionType::MissionC;
                            state.consecutive_c_detections = 0;
                            drop(state); // Release lock before calling publish
                            Self::publish_mission("MISSION_C", mission_publisher, mission_state)?;
                        }
                    } else {
                        // Gradually reduce counter if conditions are not met
                        if state.consecutive_c_detections > 0 {
                            state.consecutive_c_detections -= 1;
                        }
                    }
                }

                MissionType::MissionC => {
                    // Stay in Mission C - No further transitions from Mission C
                    println!("Staying in MISSION_C");
                }
            }
        }

        // Enhanced logging
        let current_mission_str = {
            let state = mission_state.lock().unwrap();
            match state.current_mission {
                MissionType::MissionA => "A (Camera)",
                MissionType::MissionB => "B (LiDAR)",
                MissionType::MissionC => "C (Odometry)",
            }
        };

        println!(
            "Scan: {} pts | Mean: {:.2}m | Close(<{:.1}m): {:.1}% | Medium({:.1}-{:.1}m): {:.1}% | Far(>{:.1}m): {:.1}% | Mission: {}",
            valid_ranges.len(),
            mean_range,
            CLOSE_THRESHOLD,
            close_percent,
            CLOSE_THRESHOLD,
            MEDIUM_THRESHOLD,
            medium_percent,
            MEDIUM_THRESHOLD,
            far_percent,
            current_mission_str
        );

        Ok(())
    }
}

fn main() -> Result<(), RclrsError> {
    println!("Rust Mission Launcher Node - Enhanced Version");
    println!("Mission Transition Conditions:");
    println!("   A->B: Close+Medium > 50% AND (Mean < 1.5m OR Close > 20%)");
    println!("   B->C: Far > 40% AND (Mean > 2.5m OR Close < 25%)");

    let mut executor = Context::default_from_env()?.create_basic_executor();
    let _launcher = MissionLauncher::new(&executor)?;
    executor.spin(SpinOptions::default()).first_error()
}