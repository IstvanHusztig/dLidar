# Spec Delta

## ADDED Requirements

### Requirement: Button settings
The `[button]` section SHALL accept `gpio`, the line number of the button (0 to 27, default 17).

#### Scenario: Other pin
- **WHEN** the file sets `[button] gpio = 27`
- **THEN** the button is read on GPIO 27

### Requirement: LED settings
The `[led]` section SHALL accept `gpio`, the line number of the status LED (0 to 27, default 27). It SHALL be rejected if it equals the button's line.

#### Scenario: Same pin as the button
- **WHEN** the file sets `[led] gpio = 17` and the button uses GPIO 17
- **THEN** dLidar exits with an error that the LED and the button need different lines

### Requirement: Mid-360S IMU settings
The `[livox_mid360s]` section SHALL accept `imu_rate_hz` (50, 100, 200 or 500; default 500), `imu_accel_range_g` (4, 8, 16 or 32; default 8) and `imu_gyro_range_dps` (2000, 1000, 500, 250, 125, 62.5, 31.25 or 15.625; default 2000). Any other value SHALL be rejected with the allowed values.

#### Scenario: Unsupported rate
- **WHEN** the file sets `imu_rate_hz = 400`
- **THEN** dLidar exits with an error listing 50, 100, 200 and 500

### Requirement: No chunk limit setting
The `target_chunks` key SHALL be rejected with an error saying that recordings are now started and stopped with the button.

#### Scenario: Old settings file
- **WHEN** a settings file still contains `[device] target_chunks = 1500`
- **THEN** dLidar exits with an error naming the line and explaining the button
