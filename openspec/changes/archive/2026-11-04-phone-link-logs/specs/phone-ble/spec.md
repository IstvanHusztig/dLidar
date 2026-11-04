# Spec Delta

## MODIFIED Requirements

### Requirement: Messages over writes and notifications
Over BLE the protocol's byte stream SHALL map as follows:
- dLidar joins the phone's writes in order into one stream (a request may span writes, a write may hold several);
- dLidar splits each message into notifications of at most MTU−3 bytes of this connection, and at most 512 bytes (Android's longest), each message starting in a fresh notification;
- until the phone's first write reports the MTU, dLidar assumes the minimum MTU of 23.

#### Scenario: Request in pieces at MTU 517
- **WHEN** a bonded phone with MTU 517 writes `{"v":1,"id":7,"cmd":"status.get"}\n` in two writes
- **THEN** it receives one response with id 7, in notifications of at most 512 bytes, the first of which starts with `{`

#### Scenario: Large message
- **WHEN** a response of 3000 bytes is sent on a connection with MTU 247
- **THEN** it arrives in 13 notifications of at most 244 bytes, and the next message starts in a new notification

#### Scenario: A full page of the log at MTU 517
- **WHEN** a `log.get` answer of 16 KiB is sent on a connection with MTU 517
- **THEN** every notification is at most 512 bytes, and the phone joins them into the complete message
