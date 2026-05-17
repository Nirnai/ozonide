use std::env;
use std::fs;
use std::path::Path;

fn main() {
    println!("cargo:rerun-if-changed=configs/");

    let config_name = env::var("BOARD_CONFIG").unwrap_or_else(|_| "default".to_string());
    let config_path = format!("configs/{}.toml", config_name);
    let config_content = fs::read_to_string(&config_path)
        .expect(&format!("Failed to read {}", config_path));
    let config: toml::Value = toml::from_str(&config_content)
        .expect("Failed to parse config TOML");

    // Emit feature flag from config
    let board_name = config["board"]["name"]
        .as_str()
        .expect("Missing board.name in config");
    println!("cargo:rustc-cfg=feature=\"board-{}\"", board_name);

    // Generate config
    let generated = generate_config_code(&config, &config_name);
    let out_dir = env::var("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("generated_config.rs");
    fs::write(dest_path, generated).unwrap();

    println!("cargo:warning=Using board: {}", board_name);
    println!("cargo:warning=Using configuration: {}.toml", config_name);
}

fn generate_config_code(config: &toml::Value, config_name: &str) -> String {
    let mut output = String::new();

    output.push_str(&format!("// Auto-generated from configs/{}.toml\n", config_name));
    output.push_str("// DO NOT EDIT - changes will be overwritten\n\n");

    // Board module (keep existing logic for non-sensor config)
    if let Some(board) = config.get("board") {
        output.push_str(&generate_module("board", board, 0));
    }

    // Sensor list
    output.push_str(&generate_sensor_list(config));

    output
}

fn generate_sensor_list(config: &toml::Value) -> String {
    let mut code = String::new();

    code.push_str("pub const SENSORS: &[SensorConfig] = &[\n");

    if let Some(sensors) = config.get("sensors").and_then(|s| s.as_table()) {
        for (role, sensor_config) in sensors {
            let name = sensor_config.get("name")
                .and_then(|v| v.as_str())
                .unwrap_or_else(|| panic!("Missing name for sensor '{}'", role));

            let interface = sensor_config.get("interface")
                .and_then(|v| v.as_str())
                .unwrap_or("");

            code.push_str(&format!("    SensorConfig {{\n"));
            code.push_str(&format!("        role: \"{}\",\n", role));
            code.push_str(&format!("        name: \"{}\",\n", name));
            code.push_str(&format!("        interface: \"{}\",\n", interface));

            // Collect pins
            code.push_str("        pins: &[\n");
            if let Some(pins) = sensor_config.get("pins").and_then(|p| p.as_table()) {
                for (pin_name, pin_value) in pins {
                    if let Some(pin_str) = pin_value.as_str() {
                        code.push_str(&format!(
                            "            PinConfig {{ name: \"{}\", pin: \"{}\" }},\n",
                            pin_name, pin_str
                        ));
                    }
                }
            }
            code.push_str("        ],\n");

            // Collect all non-structural fields as params
            code.push_str("        params: &[\n");
            if let Some(table) = sensor_config.as_table() {
                for (key, value) in table {
                    match key.as_str() {
                        "name" | "interface" | "pins" => continue,
                        _ => {
                            if let Some(val_str) = value.as_str() {
                                code.push_str(&format!(
                                    "            ParamConfig {{ name: \"{}\", value: \"{}\" }},\n",
                                    key, val_str
                                ));
                            }
                        }
                    }
                }
            }
            code.push_str("        ],\n");

            code.push_str("    },\n");
        }
    }

    code.push_str("];\n");
    code
}

fn generate_module(name: &str, value: &toml::Value, depth: usize) -> String {
    let mut output = String::new();
    let indent = "    ".repeat(depth);
    
    match value {
        toml::Value::Table(table) => {
            output.push_str(&format!("{}pub mod {} {{\n", indent, name));
            
            for (key, val) in table {
                if val.is_table() {
                    // Recursively handle nested tables
                    output.push_str(&generate_module(key, val, depth + 1));
                } else {
                    // Generate constants for leaf values
                    output.push_str(&generate_const(key, val, depth + 1));
                }
            }
            
            output.push_str(&format!("{}}}\n\n", indent));
        }
        _ => {
            // Top-level primitives become constants (shouldn't happen with your structure)
            output.push_str(&generate_const(name, value, depth));
        }
    }
    
    output
}

fn generate_const(name: &str, value: &toml::Value, depth: usize) -> String {
    let indent = "    ".repeat(depth);
    let const_name = name.to_uppercase();
    
    match value {
        toml::Value::String(s) => {
            format!("{}pub const {}: &str = \"{}\";\n", indent, const_name, s)
        }
        toml::Value::Integer(i) => {
            format!("{}pub const {}: i64 = {};\n", indent, const_name, i)
        }
        toml::Value::Float(f) => {
            format!("{}pub const {}: f64 = {};\n", indent, const_name, f)
        }
        toml::Value::Boolean(b) => {
            format!("{}pub const {}: bool = {};\n", indent, const_name, b)
        }
        toml::Value::Array(arr) => {
            if arr.iter().all(|v| v.is_integer()) {
                let values: Vec<String> = arr.iter()
                    .filter_map(|v| v.as_integer().map(|i| i.to_string()))
                    .collect();
                format!("{}pub const {}: &[i64] = &[{}];\n", 
                    indent, const_name, values.join(", "))
            } else if arr.iter().all(|v| v.is_str()) {
                let values: Vec<String> = arr.iter()
                    .filter_map(|v| v.as_str().map(|s| format!("\"{}\"", s)))
                    .collect();
                format!("{}pub const {}: &[&str] = &[{}];\n", 
                    indent, const_name, values.join(", "))
            } else {
                format!("{}// Skipping heterogeneous array {}\n", indent, name)
            }
        }
        _ => String::new(),
    }
}