# IMU Cube V2 — RS-485 Protocol Reference

Interface specification for the ADCS master to command and receive telemetry from the IMU Cube module over RS-485.

---

## Physical Layer

| Parameter | Value |
|-----------|-------|
| Interface | RS-485 half-duplex |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Byte order | Big-endian for CRC, **little-endian for payload floats** |

### Bus Turnaround

After the master sends a command, the IMU Cube reads all sensors before replying. The master must wait **at least 500 ms** for a response before timing out.

---

## Frame Format

All frames (command and response) share the same structure:

```
+-------+--------+----------+-------------+-----------------+----------+
| Start | Mod ID | Msg Type | Payload Len |     Payload     |  CRC-16  |
| 1 B   | 1 B    | 1 B      | 1 B         | 0–128 B         | 2 B      |
+-------+--------+----------+-------------+-----------------+----------+
  0xAA                                                        MSB  LSB
```

| Field | Size | Description |
|-------|------|-------------|
| Start | 1 | Always `0xAA` |
| Module ID | 1 | Target module: `0x04` (IMU Cube) or `0xFF` (broadcast) |
| Msg Type | 1 | Command code (master → slave) or response code (slave → master) |
| Payload Len | 1 | Number of payload bytes (0–128) |
| Payload | 0–128 | Command/response data |
| CRC-16 | 2 | CRC16-CCITT over bytes 0 through (3 + Payload Len), big-endian |

Maximum frame size: 4 (header) + 128 (payload) + 2 (CRC) = **134 bytes**.

---

## CRC-16 CCITT

- Polynomial: `0x1021`
- Initial value: `0xFFFF`
- Input: all bytes from Start through end of Payload (header + payload)
- Output: 16-bit CRC, transmitted **MSB first**

```c
uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc <<= 1;
        }
    }
    return crc;
}
```

---

## Commands (Master → IMU Cube)

All commands have **0 bytes payload** unless noted.

| Msg Type | Name | Payload | Response | Description |
|----------|------|---------|----------|-------------|
| `0x01` | PING | 0 B | ACK | Check if module is alive |
| `0x02` | GET_STATUS | 0 B | NACK (not implemented) | Reserved for future use |
| `0x03` | GET_TELEMETRY | 0 B | ALL_SENSORS (88 B) | Request all sensor data |
| `0x04` | RESET | 0 B | ACK, then reboot | Soft-reset the MCU |

### Broadcast Rules

- Broadcast commands use Module ID = `0xFF`
- The IMU Cube **will not reply** to broadcast commands (no ACK/NACK)
- Broadcast RESET is silently ignored

---

## Responses (IMU Cube → Master)

| Msg Type | Name | Payload | Description |
|----------|------|---------|-------------|
| `0x80` | ACK | 4 B (`AckResponse_t`) | Command accepted |
| `0x81` | NACK | 4 B (`AckResponse_t`) | Command rejected / unknown |
| `0x90` | ALL_SENSORS | 88 B (`IMUCube_AllSensorsPayload_t`) | Full telemetry |
| `0x91` | STATUS | Reserved | Not implemented |

---

## Payload Structs

### AckResponse_t (4 bytes)

Used for both ACK (`0x80`) and NACK (`0x81`) responses.

```c
typedef struct __attribute__((packed)) {
    uint8_t ack_type;      // 0x80 = ACK, 0x81 = NACK
    uint8_t original_cmd;  // echo of the command that was received
    uint8_t status_code;   // 0x00 = OK, 0xFF = error/unknown
    uint8_t reserved;      // 0x00
} AckResponse_t;
```

| Offset | Size | Field | Values |
|--------|------|-------|--------|
| 0 | 1 | ack_type | `0x80` ACK, `0x81` NACK |
| 1 | 1 | original_cmd | `0x01`–`0x04` |
| 2 | 1 | status_code | `0x00` = OK, `0xFF` = error |
| 3 | 1 | reserved | `0x00` |

### IMUCube_AllSensorsPayload_t (88 bytes)

All fields are **32-bit IEEE 754 floats, little-endian**. Values are raw sensor counts cast to float (no unit conversion on-board). The master must apply the conversion factors listed below.

```c
typedef struct __attribute__((packed)) {
    float rm3100_x, rm3100_y, rm3100_z;                    // bytes  0–11
    float iis2mdc_x, iis2mdc_y, iis2mdc_z;                 // bytes 12–23
    float icm_accel_x, icm_accel_y, icm_accel_z;            // bytes 24–35
    float icm_gyro_x, icm_gyro_y, icm_gyro_z;               // bytes 36–47
    float sch16t_accel_x, sch16t_accel_y, sch16t_accel_z;   // bytes 48–59
    float sch16t_gyro_x, sch16t_gyro_y, sch16t_gyro_z;      // bytes 60–71
    float i3g_gyro_x, i3g_gyro_y, i3g_gyro_z;               // bytes 72–83
    float mcp_temp_val;                                      // bytes 84–87
} IMUCube_AllSensorsPayload_t;
```

#### Byte Map

| Offset | Size | Field | Sensor | Raw Unit |
|--------|------|-------|--------|----------|
| 0 | 4 | rm3100_x | RM3100 Magnetometer X | counts |
| 4 | 4 | rm3100_y | RM3100 Magnetometer Y | counts |
| 8 | 4 | rm3100_z | RM3100 Magnetometer Z | counts |
| 12 | 4 | iis2mdc_x | IIS2MDC Magnetometer X | counts |
| 16 | 4 | iis2mdc_y | IIS2MDC Magnetometer Y | counts |
| 20 | 4 | iis2mdc_z | IIS2MDC Magnetometer Z | counts |
| 24 | 4 | icm_accel_x | ICM-45686 Accelerometer X | counts |
| 28 | 4 | icm_accel_y | ICM-45686 Accelerometer Y | counts |
| 32 | 4 | icm_accel_z | ICM-45686 Accelerometer Z | counts |
| 36 | 4 | icm_gyro_x | ICM-45686 Gyroscope X | counts |
| 40 | 4 | icm_gyro_y | ICM-45686 Gyroscope Y | counts |
| 44 | 4 | icm_gyro_z | ICM-45686 Gyroscope Z | counts |
| 48 | 4 | sch16t_accel_x | SCH16T Accelerometer X | counts |
| 52 | 4 | sch16t_accel_y | SCH16T Accelerometer Y | counts |
| 56 | 4 | sch16t_accel_z | SCH16T Accelerometer Z | counts |
| 60 | 4 | sch16t_gyro_x | SCH16T Gyroscope X | counts |
| 64 | 4 | sch16t_gyro_y | SCH16T Gyroscope Y | counts |
| 68 | 4 | sch16t_gyro_z | SCH16T Gyroscope Z | counts |
| 72 | 4 | i3g_gyro_x | I3G4250D Gyroscope X | counts |
| 76 | 4 | i3g_gyro_y | I3G4250D Gyroscope Y | counts |
| 80 | 4 | i3g_gyro_z | I3G4250D Gyroscope Z | counts |
| 84 | 4 | mcp_temp_val | MCP9808 Temperature | degC |

---

## Unit Conversion Table

All values except MCP9808 are transmitted as raw sensor counts. Apply these conversions on the master side:

| Sensor | Measurement | Raw to Engineering | Unit | Source |
|--------|------------|-------------------|------|--------|
| RM3100 | Magnetic field | raw / 75.0 | uT | Datasheet, CC=200 default |
| IIS2MDC | Magnetic field | raw * 0.15 | uT | Datasheet, fixed 1.5 mgauss/LSB |
| ICM-45686 | Acceleration | raw / 16384.0 | g | FS_SEL=000, +/-2g |
| ICM-45686 | Angular rate | raw / 131.072 | dps | FS_SEL=000, +/-250 dps |
| SCH16T | Acceleration | raw / 1962.0 | g | Murata datasheet typical |
| SCH16T | Angular rate | raw / 50.0 | dps | Murata datasheet typical |
| I3G4250D | Angular rate | raw * 0.00875 | dps | FS=00, +/-245 dps, 8.75 mdps/digit |
| MCP9808 | Temperature | no conversion | degC | Firmware converts before sending |

To convert g to m/s2, multiply by 9.80665.

---

## Pre-computed Command Bytes

Ready-to-send hex strings (header + CRC):

| Command | Hex Bytes |
|---------|-----------|
| PING | `AA 04 01 00 E9 EC` |
| GET_STATUS | `AA 04 02 00 BC BF` |
| GET_TELEMETRY | `AA 04 03 00 8F 8E` |
| RESET | `AA 04 04 00 5A 5D` |

---

## Transaction Examples

### PING

```
Master TX:  AA 04 01 00 E9 EC
IMU RX:     AA 04 80 04 80 01 00 00 [CRC_H] [CRC_L]
```

Response payload: `80 01 00 00` = ACK for PING, status OK.

### GET_TELEMETRY

```
Master TX:  AA 04 03 00 8F 8E
            (wait ~500 ms for sensor reads)
IMU RX:     AA 04 90 58 [88 bytes payload] [CRC_H] [CRC_L]
```

Total response: 94 bytes. Msg type `0x90`, payload length `0x58` (88 decimal).

### RESET

```
Master TX:  AA 04 04 00 5A 5D
IMU RX:     AA 04 80 04 80 04 00 00 [CRC_H] [CRC_L]
            (MCU reboots after 10 ms)
```

### Unknown Command

```
Master TX:  AA 04 FF 00 [CRC_H] [CRC_L]
IMU RX:     AA 04 81 04 81 FF FF 00 [CRC_H] [CRC_L]
```

Response: NACK, original_cmd=`0xFF`, status=`0xFF`.

---

## Timing

| Event | Duration |
|-------|----------|
| Sensor read cycle (all sensors) | ~300 ms |
| Max response time after command | ~500 ms |
| Recommended master timeout | 1000 ms |
| Bus turnaround (DE/RE switch) | ~1 ms |
| Post-RESET reboot time | ~1 s |

---

## Module Addressing

| ID | Description |
|----|-------------|
| `0x04` | IMU Cube (unicast) |
| `0xFF` | Broadcast (no reply) |

The IMU Cube ignores frames addressed to other module IDs. Broadcast frames are processed but never generate a reply.
