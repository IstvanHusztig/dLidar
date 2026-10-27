# sensor-integration Specification

## Purpose

Defines how a LiDAR/IMU sensor plugs into the dLidar core, so the time alignment, chunking, recording and recovery logic is reused unchanged when the sensor model changes.

## Requirements

### Requirement: Sensor-independent core
The core (domain and use cases) SHALL build and run without any vendor sensor SDK, vendor protocol header, point-cloud file library or OS device API. Only adapters MAY depend on them.

#### Scenario: Core builds alone
- **WHEN** the core and its tests are built without the Unitree SDK, LASzip or vendor headers on the include path
- **THEN** the build succeeds and the core tests pass

#### Scenario: Forbidden include is rejected
- **WHEN** a core source includes a vendor, LASzip or adapter header
- **THEN** the build or the architecture check run by the test suite fails

### Requirement: Sensor delivers points in a common sensor frame
A sensor SHALL deliver each point in its own sensor frame, in metres, with intensity, measured range, time relative to a time zero given by the core (seconds), and the per-point fields of the current recording format. Fields a sensor cannot provide SHALL be 0.

#### Scenario: Sensor without scan angles
- **WHEN** a sensor does not report raw range or scan angles
- **THEN** every point is recorded with those fields set to 0

#### Scenario: Point time relative to the core's zero
- **WHEN** the core requests the points of a packet with a given time zero
- **THEN** each point time equals the point's sensor-clock time minus that zero, in seconds

### Requirement: Sensor delivers IMU samples in the same frame and clock
A sensor SHALL deliver IMU samples in the same frame as its points, with acceleration in m/s², angular velocity in rad/s, the sample's sensor stamp in ns and a sequence number. A sensor whose IMU is mounted differently from its LiDAR SHALL rotate the IMU into the point frame itself. A packet MAY carry several samples.

#### Scenario: IMU and points share the frame
- **WHEN** the device is rotated about one axis
- **THEN** the recorded gyro shows the rotation about the same output axis about which the point cloud rotates

### Requirement: Sensor reports packet timing and host arrival
For every packet, a sensor SHALL report the host arrival time (monotonic and wall clock) and the sensor stamps of the packet's first and last sample. The packet log SHALL record every IMU and point packet.

#### Scenario: Packet log
- **WHEN** a recording is made with any sensor
- **THEN** `packetsNNNN.bin` holds one record per IMU and point packet, with host arrival times and sensor stamps

### Requirement: Core owns mount, units, time zero and point filter
The core SHALL apply the device's mounting rotation identically to points and IMU, convert acceleration to g, set the time zero from the first point packet that has points, and apply the range/intensity filter. The mounting rotation SHALL be a proper rotation, and the filter values come from the sensor's defaults unless configured.

#### Scenario: Mirror mount rejected
- **WHEN** the configured mount is a reflection (determinant −1)
- **THEN** the program refuses to start with an error naming the mount

#### Scenario: Vertical L2 mount
- **WHEN** the L2 is mounted with its +Z forward and its cable outlet up
- **THEN** the output is +X front, +Y left, +Z up for points and IMU, and gravity reads about +1 g on Z at rest

### Requirement: Optional identity, health and diagnostics
A sensor SHALL provide identity and version strings for the recording metadata. It MAY provide a lens-dirtiness reading and per-packet diagnostic rows with a header. The core SHALL record and report whatever is provided, and skip the rest.

#### Scenario: Sensor without diagnostics
- **WHEN** a sensor provides no dirtiness reading and no diagnostics
- **THEN** the recording completes without errors, there is no dirtiness warning, and no packet state file is written

### Requirement: Adding a sensor needs no core change
Supporting a new sensor SHALL require only a new adapter implementing the sensor contract and selecting it in the startup wiring.

#### Scenario: Simulated sensor
- **WHEN** a test supplies a simulated sensor implementing the contract
- **THEN** the unchanged core produces a complete recording from it

### Requirement: Unitree L2 recordings unchanged
With the Unitree L2 adapter, the program SHALL produce byte-identical recordings, and the same console messages, as the version before this change for the same input.

#### Scenario: Golden replay
- **WHEN** the same packet log is replayed through the base revision and through the restructured version, both built with the same compiler and floating-point flags on the Pi
- **THEN** every output file (`imu`, `lidar`, `timing`, `packets`, `packetstate`) is byte-identical

#### Scenario: Negative zero and NaN preserved
- **WHEN** an IMU or point component is −0.0 or NaN before mounting
- **THEN** the recorded value has the same sign and text as before this change

#### Scenario: Live capture for dMapper
- **WHEN** a recording is made on the device with the L2
- **THEN** dMapper processes it with its configuration unchanged
