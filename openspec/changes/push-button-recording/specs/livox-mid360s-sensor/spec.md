# Spec Delta

## ADDED Requirements

### Requirement: IMU rate and range set at start-up
Every Mid-360S start-up SHALL request the configured IMU output rate and accelerometer and gyroscope ranges. The request SHALL be best effort: without a confirming answer, a warning SHALL say that the sensor keeps its current IMU setting, and the start-up SHALL continue.

#### Scenario: Firmware supports it
- **WHEN** the sensor confirms the request for 500 Hz
- **THEN** the recording's IMU CSV has about 500 rows per second, and `timingNNNN.json` shows an IMU period of about 2 ms

#### Scenario: Older firmware
- **WHEN** the sensor does not answer the request
- **THEN** a warning is printed and the recording runs with the sensor's IMU rate (200 Hz from the factory)

### Requirement: No packets kept while idle
Between recordings the Mid-360S SHALL stay open, and its packets SHALL be dropped, so memory use does not grow while the device waits. Every recording SHALL run the full start-up again.

#### Scenario: Idle for an hour
- **WHEN** the service waits in Idle for an hour with the Mid-360S connected and streaming
- **THEN** memory use stays constant, and the next recording starts with the start-up sequence and holds no packets from before it
