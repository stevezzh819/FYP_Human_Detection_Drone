# Crazyflie 2.1 + ESP32-C3 SuperMini System Analysis

> Update 2026-03-29: the legacy ESP32 <-> Crazyflie I2C bridge files discussed in this report were removed from the current workspace. The active communication path remains the custom UART bridge.

## 1. Scope and Source of Truth

This document analyzes the current workspace under `/Users/zhangzehua/Desktop/fyp` as it exists on 2026-03-28. The analysis is code-driven: every conclusion below is based on the actual implementation in the repository, not on a generic Crazyflie architecture description.

Main code areas used in this report:

- `espcontrol/crazyflie_esp/`: ESP32-C3 firmware
- `crazyflie/crazyflie-firmware/`: Crazyflie firmware
- `tools/radio_wall_follow_dashboard.py`: Python dashboard
- `tools/radio_wall_follow_mission.py`: shared radio helper utilities used by the dashboard

This report focuses on three system boundaries:

1. ESP32-C3 <-> Crazyflie 2.1 communication
2. Crazyflie mission logic that consumes ESP32 data
3. Python dashboard logic that talks to the Crazyflie and visualizes telemetry

## 2. Executive Summary

The current FYP system uses **two distinct communication layers**:

1. **ESP32-C3 <-> Crazyflie 2.1**
   - Active path: **custom UART protocol**
   - Inactive legacy path: **I2C telemetry bridge**
   - Unused but present in repo: **CPX over UART2**

2. **Dashboard <-> Crazyflie**
   - Uses **CRTP** through `cflib` over a Crazyradio link discovered with `cflib.crtp.scan_interfaces()`
   - The dashboard does **not** talk to the ESP32 directly
   - The dashboard reads Crazyflie log variables, including variables that are ultimately derived from ESP32 packets

In the active runtime path:

1. ESP32-C3 runs thermal inference from the AMG8833 sensor.
2. ESP32 packs a human-detection UART packet.
3. Crazyflie receives raw bytes on `USART2`, validates the frame, and parses the payload.
4. Crazyflie stores the latest ESP32 sample in `esp_uart_bridge.c`.
5. `wall_following.c` reads the parsed data through the Crazyflie log subsystem (`espUart.*`) and turns it into mission state.
6. The dashboard reads `app.*`, `range.*`, `motion.*`, `stateEstimate.*`, and `pm.*` logs from the Crazyflie over radio.

## 3. Communication Architecture

### 3.1 Protocol inventory

| Protocol / Mechanism | Code location | Status in this project | Direction | Purpose |
| --- | --- | --- | --- | --- |
| Custom UART frame | `espcontrol/crazyflie_esp/main/uart_cf_comm.*`, `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c` | Active | Bidirectional | ESP32 human-detection packets to Crazyflie, plus control/test packets |
| I2C bridge | `crazyflie/crazyflie-firmware/src/modules/src/esp_bridge.c`, `espcontrol/crazyflie_esp/main/legacy_cf_i2c_bridge_disabled.c` | Legacy / disabled | Crazyflie -> ESP32 | Older telemetry bridge over deck I2C |
| CPX on UART2 | `src/modules/src/cpx/*`, `src/deck/drivers/src/cpx-host-on-uart2.c` | Present in repo, not used here | Bidirectional | General-purpose CPX transport; explicitly disabled to avoid conflict with the ESP UART bridge |
| CRTP via `cflib` | `tools/radio_wall_follow_dashboard.py` | Active | Dashboard <-> Crazyflie | Connect, scan, telemetry logging, mission start/stop |

### 3.2 Evidence that UART is the active ESP32/Crazyflie path

The ESP32 component build uses the UART communication module:

```cmake
idf_component_register(SRCS "crazyflie_esp_main.c"
                            "amg8833.c"
                            "uart_cf_comm.c"
                       PRIV_REQUIRES driver esp_timer spi_flash
                       INCLUDE_DIRS "")
```

Source: `espcontrol/crazyflie_esp/main/CMakeLists.txt`

Explanation:

- `uart_cf_comm.c` is part of the ESP32 firmware build.
- There is no active I2C bridge source in the ESP32 build list.
- This is strong evidence that the active ESP32 communication channel is UART.

The Crazyflie app config explicitly reserves UART2 for the ESP bridge and disables CPX-on-UART2:

```text
CONFIG_APP_ENABLE=y
CONFIG_APP_PRIORITY=1
CONFIG_APP_STACKSIZE=500
# Reserve UART2 for the ESP32 bridge; CPX on UART2 conflicts with it.
# CONFIG_ENABLE_CPX_ON_UART2 is not set
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/app-config`

Explanation:

- The current application build intentionally dedicates UART2 to the custom ESP32 bridge.
- CPX is acknowledged in the repo, but not used in this mission configuration.

The Crazyflie stabilizer startup code shows the old I2C bridge is disabled and the UART bridge is the active path:

```c
/*
 * Legacy I2C communication disabled:
 * if (espBridgeInit()) {
 *   STATIC_MEM_TASK_CREATE(espBridgeTask, espBridgeTask, "espBridge", NULL, ESP_BRIDGE_TASK_PRI);
 * } else {
 *   DEBUG_PRINT("ESP bridge init failed\n");
 * }
 */
if (espUartBridgeInit()) {
  STATIC_MEM_TASK_CREATE(espUartBridgeTask, espUartBridgeTask, "espUartBridge", NULL, ESP_UART_BRIDGE_TASK_PRI);
} else {
  DEBUG_PRINT("ESP UART bridge init failed\n");
}
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/stabilizer.c:197-209`

Explanation:

- The legacy I2C bridge startup is commented out.
- The active boot path initializes `espUartBridgeInit()` and starts `espUartBridgeTask`.
- This is the runtime switch point that selects the current communication mechanism.

## 4. ESP32-C3 Side: Active UART Communication

### 4.1 ESP32 UART protocol definitions

The ESP32 UART frame constants and packet type definitions are:

```c
#define UART_CF_HEADER_BYTE0       0xAAU
#define UART_CF_HEADER_BYTE1       0x55U
#define UART_CF_MAX_DATA_LEN       64U

#define UART_CF_TYPE_HUMAN_DETECT  0x01U
#define UART_CF_TYPE_COMMAND       0x10U
#define UART_CF_TYPE_TEST_TEXT     0x7EU

#define UART_CF_CMD_SET_THRESHOLD  0x01U
#define UART_CF_CMD_TAKEOFF        0x20U
#define UART_CF_CMD_LAND           0x21U
```

Source: `espcontrol/crazyflie_esp/main/uart_cf_comm.h:11-21`

Explanation:

- `0xAA 0x55` are the two sync/header bytes for every custom UART frame.
- `UART_CF_TYPE_HUMAN_DETECT` is the active payload type used by the ESP32 thermal application.
- `UART_CF_TYPE_COMMAND` is a control packet type for commands such as threshold updates or flight commands.
- `UART_CF_TYPE_TEST_TEXT` exists for diagnostics.

### 4.2 ESP32 UART hardware configuration

The ESP32 application configures UART1 with fixed pins and baud rate:

```c
#define CF_UART_PORT                 UART_NUM_1
#define CF_UART_TX_PIN               GPIO_NUM_20
#define CF_UART_RX_PIN               GPIO_NUM_21
#define CF_UART_BAUDRATE             57600
```

Source: `espcontrol/crazyflie_esp/main/crazyflie_esp_main.c:34-37`

And initializes it in `app_main()`:

```c
ESP_ERROR_CHECK(uart_cf_init(
    CF_UART_PORT,
    CF_UART_TX_PIN,
    CF_UART_RX_PIN,
    CF_UART_BAUDRATE));
```

Source: `espcontrol/crazyflie_esp/main/crazyflie_esp_main.c:440-444`

Explanation:

- The ESP32-C3 uses `UART_NUM_1`.
- It runs at `57600` baud, 8 data bits, no parity, 1 stop bit.
- These settings must match the Crazyflie `uart2Init()` call on the STM32 side.

### 4.3 ESP32 UART frame sender

The actual frame construction happens in `uart_cf_send_packet()`:

```c
frame[offset++] = UART_CF_HEADER_BYTE0;
frame[offset++] = UART_CF_HEADER_BYTE1;
frame[offset++] = type;
frame[offset++] = length;

if ((length > 0U) && (data != NULL)) {
    memcpy(&frame[offset], data, length);
}
offset += length;

const uint16_t crc = uart_cf_crc16_ccitt(&frame[2], (size_t)2U + length);
frame[offset++] = (uint8_t)(crc & 0xFFU);
frame[offset++] = (uint8_t)((crc >> 8U) & 0xFFU);

const int written = uart_write_bytes(s_uart_port, (const char *)frame, offset);
```

Source: `espcontrol/crazyflie_esp/main/uart_cf_comm.c:191-206`

Explanation:

- The transport is a fixed pattern:
  1. two-byte header
  2. one-byte type
  3. one-byte payload length
  4. payload bytes
  5. CRC16-CCITT of `type + length + payload`
- The CRC does not include the two header bytes.
- This exact framing is mirrored by the Crazyflie parser in `esp_uart_bridge.c`.

### 4.4 ESP32 UART frame receiver and parser

The ESP32 also contains a byte-wise receive state machine:

```c
switch (s_rx_state) {
    case rxWaitHeader0:
        if (byte == UART_CF_HEADER_BYTE0) {
            s_rx_state = rxWaitHeader1;
        }
        break;

    case rxWaitHeader1:
        if (byte == UART_CF_HEADER_BYTE1) {
            s_rx_state = rxWaitType;
        } else if (byte == UART_CF_HEADER_BYTE0) {
            s_rx_state = rxWaitHeader1;
        } else {
            s_rx_state = rxWaitHeader0;
        }
        break;

    case rxWaitType:
        s_rx_type = byte;
        s_rx_state = rxWaitLength;
        break;
```

Source: `espcontrol/crazyflie_esp/main/uart_cf_comm.c:63-84`

The CRC check and packet finalization are:

```c
const uint16_t rx_crc = (uint16_t)s_rx_crc_lo | ((uint16_t)byte << 8U);

uint8_t crc_buf[2 + UART_CF_MAX_DATA_LEN] = {0};
crc_buf[0] = s_rx_type;
crc_buf[1] = s_rx_length;
if (s_rx_length > 0U) {
    memcpy(&crc_buf[2], s_rx_payload, s_rx_length);
}
const uint16_t calc_crc = uart_cf_crc16_ccitt(crc_buf, (size_t)2U + s_rx_length);

if (rx_crc == calc_crc) {
    packet->type = s_rx_type;
    packet->length = s_rx_length;
    packet->crc = rx_crc;
    if (s_rx_length > 0U) {
        memcpy(packet->data, s_rx_payload, s_rx_length);
    }
    uart_cf_parser_reset();
    return true;
}
```

Source: `espcontrol/crazyflie_esp/main/uart_cf_comm.c:110-133`

Explanation:

- The ESP32 receive path is symmetric with the Crazyflie UART receive path.
- This allows bidirectional use of the same framing protocol.
- In the current application, the ESP32 mainly uses the receive path to accept command packets from the Crazyflie.

### 4.5 ESP32 command decoding

After a frame is assembled, the ESP32 only handles `UART_CF_TYPE_COMMAND`:

```c
if (packet->type != UART_CF_TYPE_COMMAND) {
    return;
}

if (packet->length < 1U) {
    return;
}

const uint8_t cmd = packet->data[0];

if ((cmd == UART_CF_CMD_SET_THRESHOLD) && (packet->length >= 3U)) {
    const int16_t threshold_x10 = (int16_t)((uint16_t)packet->data[1] | ((uint16_t)packet->data[2] << 8U));
    const float threshold_c = ((float)threshold_x10) / 10.0f;
    ...
}
```

Source: `espcontrol/crazyflie_esp/main/uart_cf_comm.c:239-259`

Explanation:

- The current ESP32 firmware does not process test-text packets.
- It only processes command packets, and the only implemented command in this file is threshold update.
- This is important for system explanation: the bidirectional protocol exists, but the active application behavior is still asymmetric.

### 4.6 ESP32 thermal inference to UART payload

The ESP32 computes thermal inference in `run_human_inference()`:

```c
result.human_detected = human_present;
result.max_temp_c = max_temp;
result.avg_temp_c = sum_temp / 64.0f;
result.ambient_c = amb;
result.blob_size = b.size;
result.human_dir = 0;

if (b.size > 0) {
    const float blob_mean = b.t_sum / (float)b.size;
    float score = ((b.t_max - amb) * 18.0f) + ((blob_mean - amb) * 12.0f);
    if (b.size >= 6) {
        score += 10.0f;
    }
    if (score < 0.0f) score = 0.0f;
    if (score > 100.0f) score = 100.0f;
    result.confidence = (uint8_t)lrintf(score);
```

Source: `espcontrol/crazyflie_esp/main/crazyflie_esp_main.c:377-392`

Explanation:

- The ESP32 is not forwarding raw 8x8 thermal arrays to the Crazyflie.
- Instead, it compresses the thermal processing result into a small semantic packet:
  - detected / not detected
  - confidence
  - human direction
  - hottest pixel temperature
  - thermistor temperature
  - source timestamp

The packet assembly is:

```c
const uint32_t timestamp_ms = (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
const int16_t max_temp_x100 = (int16_t)lrintf(result->max_temp_c * 100.0f);
const int16_t thermistor_x100 = (int16_t)lrintf(thermistor_c * 100.0f);

uint8_t payload[11] = {0};
payload[0] = result->human_detected ? 1U : 0U;
payload[1] = result->confidence;
payload[2] = (uint8_t)result->human_dir;
payload[3] = (uint8_t)(max_temp_x100 & 0xFF);
payload[4] = (uint8_t)((max_temp_x100 >> 8) & 0xFF);
payload[5] = (uint8_t)(thermistor_x100 & 0xFF);
payload[6] = (uint8_t)((thermistor_x100 >> 8) & 0xFF);
payload[7] = (uint8_t)(timestamp_ms & 0xFF);
payload[8] = (uint8_t)((timestamp_ms >> 8) & 0xFF);
payload[9] = (uint8_t)((timestamp_ms >> 16) & 0xFF);
payload[10] = (uint8_t)((timestamp_ms >> 24) & 0xFF);

const esp_err_t err = uart_cf_send_packet(UART_CF_TYPE_HUMAN_DETECT, payload, sizeof(payload));
```

Source: `espcontrol/crazyflie_esp/main/crazyflie_esp_main.c:413-430`

Explanation:

- The payload is compact and fixed-width.
- The temperature fields are scaled by `x100` to preserve two decimal digits while avoiding floating-point transmission.
- The timestamp is produced on the ESP32 side and later stored alongside the Crazyflie receive timestamp.

### 4.7 ESP32 main loop communication behavior

The ESP32 main loop both receives and transmits:

```c
while (true) {
    uart_cf_packet_t rx_packet = {0};
    while (uart_cf_receive_packet(&rx_packet, 0)) {
        uart_cf_parse_message(&rx_packet);
    }

    ...

    const float thermistor_c = amg_read_thermistor();
    const inference_result_t result = run_human_inference(pix, bg);

    send_detection_to_crazyflie(&result, thermistor_c);
    status_led_update(result.human_detected, now_ms);
    ...
}
```

Source: `espcontrol/crazyflie_esp/main/crazyflie_esp_main.c:478-536`

Explanation:

- The ESP32 does two communication jobs in one loop:
  - parse any incoming Crazyflie command frames
  - send the latest human-detection result to the Crazyflie
- This loop is the firmware-level bridge between the AMG8833 sensor and the Crazyflie mission logic.

## 5. Crazyflie Side: Active UART Bridge

### 5.1 Public bridge interface

The public data structure exposed by the Crazyflie bridge is:

```c
typedef struct {
  bool detected;
  uint8_t confidence;
  int8_t direction;
  int16_t maxTempX100;
  int16_t thermistorX100;
  uint32_t sourceTimestampMs;
  uint32_t rxLocalTimeMs;
} espUartHumanDetection_t;
```

Source: `crazyflie/crazyflie-firmware/src/modules/interface/esp_uart_bridge.h:13-21`

Explanation:

- This is the decoded representation of the ESP32 human-detection packet inside the Crazyflie.
- `sourceTimestampMs` is the timestamp generated by the ESP32.
- `rxLocalTimeMs` is the local Crazyflie tick when the packet was accepted.

The corresponding getters are:

```c
bool espUartBridgeGetLatestHumanDetectionSample(espUartHumanDetection_t *sample);

bool espUartBridgeGetLatestHumanDetection(bool *detected,
                                          uint8_t *confidence,
                                          uint32_t *timestamp_ms);
```

Source: `crazyflie/crazyflie-firmware/src/modules/interface/esp_uart_bridge.h:26-30`

Explanation:

- The module supports both a full-sample getter and a reduced convenience getter.
- In the current mission implementation, `wall_following.c` does not call these getters directly; it uses the `espUart.*` log group instead.

### 5.2 Crazyflie UART2 low-level driver initialization

The STM32-side UART2 driver sets up GPIO, USART, interrupts, and stream buffers:

```c
GPIO_InitStructure.GPIO_Pin   = UART2_GPIO_RX_PIN;
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
GPIO_Init(UART2_GPIO_PORT, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin   = UART2_GPIO_TX_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
GPIO_Init(UART2_GPIO_PORT, &GPIO_InitStructure);

USART_InitStructure.USART_BaudRate            = baudrate;
USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
USART_InitStructure.USART_StopBits            = USART_StopBits_1;
USART_InitStructure.USART_Parity              = USART_Parity_No ;
USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART_Init(UART2_TYPE, &USART_InitStructure);
```

Source: `crazyflie/crazyflie-firmware/src/drivers/src/uart2.c:117-140`

Explanation:

- Crazyflie uses `USART2` on pins `PA2`/`PA3` for the active ESP32 bridge.
- The configuration matches the ESP32 side: 57600 baud, 8N1, no flow control.

The symbolic mapping is defined in:

```c
#define UART2_TYPE             USART2
#define UART2_GPIO_TX_PIN      GPIO_Pin_2
#define UART2_GPIO_RX_PIN      GPIO_Pin_3
```

Source: `crazyflie/crazyflie-firmware/src/drivers/interface/uart2.h:35-54`

Explanation:

- This is the actual MCU peripheral and pins used by the active bridge.

### 5.3 Crazyflie UART interrupt receive path

The raw byte receive path is interrupt-driven:

```c
void __attribute__((used)) USART2_IRQHandler(void)
{
  uint32_t status = UART2_TYPE->SR;
  if ((UART2_TYPE->SR & USART_FLAG_RXNE) != 0)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t rxData = USART_ReceiveData(UART2_TYPE) & 0x00FF;
    xStreamBufferSendFromISR(rxStream, &rxData, 1, &xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
```

Source: `crazyflie/crazyflie-firmware/src/drivers/src/uart2.c:296-305`

Explanation:

- The ISR only moves bytes from the UART data register into the FreeRTOS stream buffer.
- Parsing does not happen inside the ISR.
- This keeps interrupt work small and pushes frame assembly into a task context.

The task-facing byte API is:

```c
bool uart2GetCharWithTimeout(uint8_t *c, const uint32_t timeoutTicks)
{
  if (uart2GetDataWithTimeout(1, c, timeoutTicks) > 0)
  {
    return true;
  }

  *c = 0;
  return false;
}
```

Source: `crazyflie/crazyflie-firmware/src/drivers/src/uart2.c:229-237`

Explanation:

- `espUartBridgeTask` consumes one byte at a time from this API.
- This creates a clear separation between the hardware driver and the bridge protocol parser.

### 5.4 Crazyflie UART bridge task

The bridge task is the active protocol engine:

```c
void espUartBridgeTask(void *param)
{
  ...
  while (true) {
    uint8_t byte = 0;
    if (uart2GetCharWithTimeout(&byte, pdMS_TO_TICKS(10))) {
      if (consumeByte(byte, &packet)) {
        handlePacket(&packet);
      }
    } else {
      taskYIELD();
    }

    ...

    if ((now - lastTestTx) >= pdMS_TO_TICKS(ESP_UART_TEST_PERIOD_MS)) {
      const char *testMsg = ESP_UART_TEST_FROM_CRAZYFLIE;
      const uint8_t testLen = (uint8_t)strlen(testMsg);
      if (espUartBridgeSendPacket(ESP_UART_TYPE_TEST_TEXT, (const uint8_t *)testMsg, testLen)) {
        testTxCountLog++;
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c:343-384`

Explanation:

- `espUartBridgeTask` is the top-level bridge runtime.
- It continuously:
  - reads raw bytes from UART2
  - assembles complete frames with `consumeByte()`
  - dispatches decoded packets to `handlePacket()`
  - periodically sends a test text packet from Crazyflie to ESP32
- This means the protocol is **bidirectional**, even though the human-detection application is mainly ESP32 -> Crazyflie.

### 5.5 Crazyflie UART frame parser

The Crazyflie parser uses the same state machine pattern as the ESP32:

```c
typedef enum {
  rxWaitHeader0 = 0,
  rxWaitHeader1,
  rxWaitType,
  rxWaitLength,
  rxWaitPayload,
  rxWaitCrcLo,
  rxWaitCrcHi,
} espUartRxState_t;
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c:46-54`

And the byte-wise parser is:

```c
case rxWaitHeader0:
  if (byte == ESP_UART_HEADER_BYTE0) {
    rxState = rxWaitHeader1;
  }
  break;

case rxWaitHeader1:
  if (byte == ESP_UART_HEADER_BYTE1) {
    rxState = rxWaitType;
  } else if (byte == ESP_UART_HEADER_BYTE0) {
    rxState = rxWaitHeader1;
  } else {
    rxState = rxWaitHeader0;
  }
  break;

case rxWaitType:
  rxType = byte;
  rxState = rxWaitLength;
  break;
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c:112-132`

The CRC verification and completed-packet handoff are:

```c
const uint16_t rxCrc = (uint16_t)rxCrcLo | ((uint16_t)byte << 8U);
uint8_t crcBuf[2 + ESP_UART_MAX_PAYLOAD] = {0};
crcBuf[0] = rxType;
crcBuf[1] = rxLength;
if (rxLength > 0U) {
  memcpy(&crcBuf[2], rxPayload, rxLength);
}
const uint16_t calcCrc = crc16_ccitt(crcBuf, (uint32_t)2U + rxLength);

if (rxCrc == calcCrc) {
  packet->type = rxType;
  packet->length = rxLength;
  if (rxLength > 0U) {
    memcpy(packet->payload, rxPayload, rxLength);
  }
  resetParser();
  return true;
}
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c:159-177`

Explanation:

- The Crazyflie validates the exact same frame contract used by the ESP32 sender.
- If the CRC fails, the packet is dropped.
- Only fully reconstructed packets reach the semantic parser in `handlePacket()`.

### 5.6 Crazyflie packet decoding: command, test, human detection

`handlePacket()` branches by packet type:

```c
if (packet->type == ESP_UART_TYPE_COMMAND) {
    ...
    const uint8_t cmd = packet->payload[0];
    if ((cmd == ESP_UART_CMD_TAKEOFF) || (cmd == ESP_UART_CMD_LAND)) {
        ...
        if (cmd == ESP_UART_CMD_TAKEOFF) {
          result = crtpCommanderHighLevelTakeoffWithVelocity(height_m, velocity_m_s, true);
        } else {
          result = crtpCommanderHighLevelLandWithVelocity(height_m, velocity_m_s, true);
        }
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c:195-255`

Explanation:

- The bridge can directly invoke high-level Crazyflie takeoff and land behavior from command packets received over UART.
- This shows that external communication can influence flight behavior directly, not just through logging.

The human-detection parse path is:

```c
if (packet->type != ESP_UART_TYPE_HUMAN_DETECT) {
  return;
}

if (packet->length < 10U) {
  DEBUG_PRINT("ESP UART RX: human packet too short (%u)\n", packet->length);
  return;
}

espUartHumanDetection_t sample = {0};
sample.detected = (packet->payload[0] != 0U);
sample.confidence = packet->payload[1];
sample.rxLocalTimeMs = T2M(xTaskGetTickCount());

if (packet->length >= 11U) {
  sample.direction = (int8_t)packet->payload[2];
  sample.maxTempX100 = (int16_t)((uint16_t)packet->payload[3] |
                                 ((uint16_t)packet->payload[4] << 8U));
  sample.thermistorX100 = (int16_t)((uint16_t)packet->payload[5] |
                                    ((uint16_t)packet->payload[6] << 8U));
  sample.sourceTimestampMs = ((uint32_t)packet->payload[7]) |
                             ((uint32_t)packet->payload[8] << 8U) |
                             ((uint32_t)packet->payload[9] << 16U) |
                             ((uint32_t)packet->payload[10] << 24U);
}
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c:273-307`

Explanation:

- This is the exact code that turns the ESP32 byte stream into typed Crazyflie fields.
- The bridge supports both a current 11-byte payload and an older 10-byte compatibility layout.
- The active ESP32 firmware uses the 11-byte layout.

### 5.7 Crazyflie bridge state storage and log export

Once parsed, the sample is stored and mirrored to log variables:

```c
if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
  latestHumanSample = sample;
  hasHumanSample = true;
  humanDetectedLog = sample.detected ? 1U : 0U;
  humanConfidenceLog = sample.confidence;
  humanDirectionLog = sample.direction;
  humanMaxTempLog = sample.maxTempX100;
  humanThermistorLog = sample.thermistorX100;
  humanTimestampMsLog = sample.sourceTimestampMs;
  humanRxLocalTimeMsLog = sample.rxLocalTimeMs;
  xSemaphoreGive(stateMutex);
}
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c:309-320`

And the log export is:

```c
LOG_GROUP_START(espUart)
LOG_ADD(LOG_UINT8, detected, &humanDetectedLog)
LOG_ADD(LOG_UINT8, confidence, &humanConfidenceLog)
LOG_ADD(LOG_INT8, direction, &humanDirectionLog)
LOG_ADD(LOG_INT16, maxTemp, &humanMaxTempLog)
LOG_ADD(LOG_INT16, therm, &humanThermistorLog)
LOG_ADD(LOG_UINT32, timestampMs, &humanTimestampMsLog)
LOG_ADD(LOG_UINT32, rxLocalMs, &humanRxLocalTimeMsLog)
...
LOG_GROUP_STOP(espUart)
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_uart_bridge.c:475-487`

Explanation:

- The bridge stores the latest semantic sample in `latestHumanSample`.
- It also publishes a log group called `espUart`.
- The current wall-following mission reads this log group instead of calling the getter API directly.

## 6. Legacy / Inactive ESP32-Crazyflie Communication Paths

### 6.1 Disabled I2C bridge on the Crazyflie side

The Crazyflie repo still contains a deck I2C telemetry bridge:

```c
 * ESP bridge: streams a compact telemetry frame over the deck I2C bus to an
 * external ESP32-C3 that listens as a slave at address 0x28.
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_bridge.c:10-12`

Its frame construction is:

```c
frame[0] = 1;  // proto_ver
uint8_t flags = 0;
...
const uint32_t timestamp = (uint32_t)usecTimestamp();
le32(&frame[2], timestamp);
...
for (size_t i = 0; i < 5; ++i) {
  le16(&frame[12 + (i * 2U)], ranges_mm[i]);
}

const uint16_t crc = crc16_ccitt(frame, ESP_BRIDGE_PAYLOAD_SIZE_BYTES);
le16(&frame[22], crc);

const bool writeOk = i2cdevWrite(I2C1_DEV, ESP_BRIDGE_I2C_ADDR, ESP_BRIDGE_FRAME_SIZE_BYTES, frame);
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/esp_bridge.c:172-215`

Explanation:

- This is a valid older communication path.
- It is **not active** in the current build because the startup hook is commented out in `stabilizer.c`.

### 6.2 Disabled I2C bridge on the ESP32 side

The ESP32 still contains a matching disabled slave implementation:

```c
/* Legacy I2C communication disabled: ESP32<->Crazyflie I2C slave bridge kept for reference. */
#if 0
/*
 * Crazyflie telemetry I2C slave bridge for ESP32-C3.
 *
 * Wiring (ESP32-C3 ↔ Crazyflie 2.x):
 *   - SDA  (GPIO8)  ←→ SDA
 *   - SCL  (GPIO9)  ←→ SCL
 *   - GND  ↔ GND (shared)
 *   Bus speed: 100 kHz. Crazyflie acts as I²C master, ESP32-C3 as slave at address 0x28.
 */
```

Source: `espcontrol/crazyflie_esp/main/legacy_cf_i2c_bridge_disabled.c:1-10`

It parses a fixed 24-byte frame:

```c
static void cf_handle_frame(const uint8_t *frame)
{
    const uint16_t crc_expected = read_le_u16(&frame[CF_PAYLOAD_LEN_BYTES]);
    const uint16_t crc_actual = cf_crc16_ccitt(frame, CF_PAYLOAD_LEN_BYTES);
    ...
    cf_telemetry_t sample = {
        .proto_ver = frame[0],
        .flags = frame[1],
        .time_us = read_le_u32(&frame[2]),
        .yaw_mrad = read_le_i16(&frame[6]),
        .vx_mmps = read_le_i16(&frame[8]),
        .vy_mmps = read_le_i16(&frame[10]),
    };
```

Source: `espcontrol/crazyflie_esp/main/legacy_cf_i2c_bridge_disabled.c:129-156`

Explanation:

- The repository preserves a full older I2C bridge pair on both ends.
- For your FYP report, this should be described as a **legacy design retained for reference**, not as the active communication architecture.

### 6.3 CPX is present but intentionally unused

The application configuration explicitly says CPX on UART2 is disabled because it conflicts with the ESP32 bridge:

```text
# Reserve UART2 for the ESP32 bridge; CPX on UART2 conflicts with it.
# CONFIG_ENABLE_CPX_ON_UART2 is not set
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/app-config:4-5`

The build system also includes the custom bridge and leaves the old I2C bridge disabled:

```make
# obj-y += esp_bridge.o
obj-y += esp_uart_bridge.o
```

Source: `crazyflie/crazyflie-firmware/src/modules/src/Kbuild`

Explanation:

- CPX exists in the upstream Crazyflie firmware tree.
- In this project, it is not the FYP communication path.
- The report should treat CPX as a potential alternative transport that was deliberately not used to avoid UART2 conflicts.

## 7. How ESP32 Data Influences Flight Logic

### 7.1 `wall_following.c` reads ESP32-derived data through `espUart.*`

The mission app resolves the bridge log IDs:

```c
EspUartLogIds espUartIds = {
  .detected = logGetVarId("espUart", "detected"),
  .confidence = logGetVarId("espUart", "confidence"),
  .direction = logGetVarId("espUart", "direction"),
  .maxTemp = logGetVarId("espUart", "maxTemp"),
  .therm = logGetVarId("espUart", "therm"),
  .rxLocalMs = logGetVarId("espUart", "rxLocalMs"),
};
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/src/wall_following.c:213-220`

Explanation:

- The app does not directly include `esp_uart_bridge.h`.
- Instead, it uses the Crazyflie log subsystem as an internal data access layer.
- This is functional, though less direct than using the bridge getter API.

### 7.2 Freshness and perception filtering

The app filters stale ESP32 samples before using them:

```c
humanMaxTempX100 = (int16_t)logGetInt(ids->maxTemp);
humanThermistorX100 = (int16_t)logGetInt(ids->therm);

const uint32_t rxLocalMs = logGetUint(ids->rxLocalMs);
if (rxLocalMs == 0U) {
  return;
}

humanAgeMs = nowMs - rxLocalMs;

if (humanAgeMs > HUMAN_PACKET_STALE_TIMEOUT_MS) {
  return;
}

humanPacketFresh = 1U;
humanConfidence = (uint8_t)logGetUint(ids->confidence);
humanDir = (int8_t)logGetInt(ids->direction);
if (logGetUint(ids->detected) != 0U) {
  humanDetected = 1U;
}
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/src/wall_following.c:179-198`

Explanation:

- `humanMaxTempX100` and `humanThermistorX100` are copied first.
- Then the app checks whether the packet has ever been received (`rxLocalMs != 0`).
- Then it enforces freshness with a 300 ms stale timeout.
- Only fresh packets can assert `humanDetected`.

### 7.3 Mission activation by dashboard

The app is activated through a parameter:

```c
PARAM_GROUP_START(app)
PARAM_ADD(PARAM_UINT8, active, &appActive)
...
PARAM_GROUP_STOP(app)
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/src/wall_following.c:526-532`

And the main loop waits for it:

```c
if (stateOuterLoop == idle) {
  if (appActive && positioningInit && multirangerInit) {
    stateOuterLoop = unlocked;
    resetWallFollower();
    missionState = mission_reacquire_wall;
    ...
    DEBUG_PRINT("App activated\n");
  }
}
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/src/wall_following.c:245-258`

Explanation:

- The dashboard does not send velocity commands directly.
- It only toggles `app.active`.
- Once that parameter becomes `1`, `wall_following.c` takes control of mission behavior.

### 7.4 Mission state machine

The current mission states are:

```c
typedef enum
{
  mission_reacquire_wall,
  mission_wallfollow,
  mission_scan,
  mission_approach,
  mission_bob,
  mission_track,
  mission_transition
} MissionState;
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/src/wall_following.c:100-109`

Explanation:

- This is the mission-level state machine used after `app.active=1`.
- Unlike the earlier signaling/landing-only version, the current workspace uses:
  - wall reacquisition
  - wall following
  - scan
  - human approach
  - bob motion
  - track behavior

### 7.5 Human-detection-triggered transitions

Human detection influences flight behavior in `mission_wallfollow` and `mission_scan`:

```c
if (humanDetected) {
  humanSeenCount++;

  if (humanSeenCount >= 3U) {
    missionTransition(mission_approach, timeNow);
  }
} else {
  humanSeenCount = 0U;
}
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/src/wall_following.c:359-367`

Explanation:

- The app does not transition to human approach on a single packet.
- It requires multiple detections (`humanSeenCount >= 3`) before switching mission state.
- This is a second filtering stage above the UART packet freshness check.

The human-approach behavior uses direction and front range:

```c
cmdYawRateDeg = 20.0f * (float)humanDir;

if (frontRange > (humanStandOff + margin)) {
  cmdVelX = 0.15f;
} else if (frontRange < (humanStandOff - margin)) {
  cmdVelX = -0.1f;
} else {
  cmdVelX = 0.0f;
  missionTransition(mission_bob, timeNow);
}
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/src/wall_following.c:412-420`

Explanation:

- `humanDir` from the ESP32 packet directly affects yaw command.
- The multiranger `frontRange` determines approach / retreat relative to `humanStandOff`.
- This is the clearest end-to-end example of ESP32 data changing flight behavior.

### 7.6 Mission-generated setpoint to flight controller

The app ultimately publishes a Crazyflie setpoint:

```c
setVelocitySetpoint(&setpoint, cmdVelX, cmdVelY, cmdHeight, cmdYawRateDeg);
commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
```

Source: `crazyflie/crazyflie-firmware/examples/demos/app_wall_following_demo/src/wall_following.c:521-522`

Explanation:

- `wall_following.c` is the mission/behavior layer.
- It does not drive motors directly.
- It sends desired body-frame velocity, height, and yaw-rate commands into the Crazyflie commander stack.

## 8. Dashboard Architecture: `tools/radio_wall_follow_dashboard.py`

### 8.1 High-level structure

The dashboard has three major architectural layers:

1. **Transport / backend threads**
   - `RadioSession`
   - `ScanSession`

2. **UI state and widgets**
   - `DashboardApp`
   - `MissionSnapshot`, `FlowSnapshot`, `RangeSnapshot`

3. **Event bridge between backend and UI**
   - shared `event_queue`
   - `_emit(...)` in worker threads
   - `_pump_events()` in the Tk main loop

The data snapshots are:

```python
@dataclass
class MissionSnapshot:
    timestamp_ms: int = 0
    outer: int = 0
    mission: int = 0
    z: float = 0.0
    human: int = 0
    fresh: int = 0
    stable: int = 0
    hold: float = 0.0
    conf: int = 0
    direction: int = 0
    max_temp_c: float = 0.0
    therm_c: float = 0.0
```

Source: `tools/radio_wall_follow_dashboard.py:69-82`

Explanation:

- The dashboard stores the latest decoded backend event into snapshot dataclasses.
- These snapshots are then rendered onto Tk widgets.

### 8.2 RadioSession backend thread

The radio thread owns the Crazyflie connection:

```python
class RadioSession(threading.Thread):
    def __init__(...):
        super().__init__(name="cf-radio-session", daemon=True)
        ...
        self.command_queue: "queue.Queue[tuple[str, Optional[int]]]" = queue.Queue()
        self.stop_event = threading.Event()
        self.cf: Optional[Crazyflie] = None
```

Source: `tools/radio_wall_follow_dashboard.py:221-236`

Explanation:

- `RadioSession` runs outside the Tk main thread.
- It is responsible for:
  - discovering/connecting to the Crazyflie
  - subscribing to logs
  - sending `app.active`
  - forwarding telemetry and console text into the UI event queue

The thread entry point is:

```python
def run(self) -> None:
    cflib.crtp.init_drivers()

    try:
        uri = discover_uri(self.preferred_uri)
    except RuntimeError as exc:
        self._emit("status", level="error", message=str(exc))
        self._emit("disconnected")
        return
```

Source: `tools/radio_wall_follow_dashboard.py:253-260`

Explanation:

- It initializes the `cflib` transport layer.
- It resolves a URI using `discover_uri()` imported from `radio_wall_follow_mission.py`.
- If discovery fails, it reports failure back to the UI instead of blocking the main thread.

### 8.3 Dashboard telemetry callbacks

The three telemetry callbacks are:

```python
def on_mission(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
    self._emit(
        "mission",
        timestamp_ms=ts,
        outer=int(data["app.stateOuter"]),
        mission=int(data["app.mission"]),
        human=int(data["app.human"]),
        fresh=int(data["app.humanFresh"]),
        stable=int(data["app.humanStable"]),
        hold=float(data["app.humanHold"]),
        conf=int(data["app.humanConf"]),
        direction=int(data["app.humanDir"]),
        z=float(data["stateEstimate.z"]),
        max_temp_c=int(data["app.humanMax"]) / 100.0,
        therm_c=int(data["app.humanTherm"]) / 100.0,
    )
```

Source: `tools/radio_wall_follow_dashboard.py:281-296`

Explanation:

- The mission callback combines app-level mission state with ESP32-derived thermal values and estimator height.
- Temperature values are decoded back from centi-degrees to degrees C.

The flow callback is:

```python
def on_flow(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
    self._emit(
        "flow",
        timestamp_ms=ts,
        delta_x=int(data["motion.deltaX"]),
        delta_y=int(data["motion.deltaY"]),
        shutter=int(data["motion.shutter"]),
        vel_x=float(data["stateEstimate.vx"]),
        vel_y=float(data["stateEstimate.vy"]),
        cmd_x=float(data["app.cmdVelX"]),
        cmd_y=float(data["app.cmdVelY"]),
    )
```

Source: `tools/radio_wall_follow_dashboard.py:298-309`

Explanation:

- This mixes raw optical-flow deck measurements with state-estimator velocity and mission command outputs.
- It is useful for comparing what the app commands versus what the estimator measures.

The range callback is:

```python
def on_range(ts: int, data: Dict[str, object], _logconf: LogConfig) -> None:
    self._emit(
        "range",
        timestamp_ms=ts,
        front=int(data["range.front"]),
        left=int(data["range.left"]),
        right=int(data["range.right"]),
        back=int(data["range.back"]),
        up=int(data["range.up"]),
        vbat=float(data["pm.vbat"]),
    )
```

Source: `tools/radio_wall_follow_dashboard.py:311-321`

Explanation:

- This callback packages multiranger distances and battery voltage for the range panel and mission panel.

### 8.4 Dashboard connection and log subscription

The radio session establishes the link and configures logs here:

```python
with SyncCrazyflie(uri, cf=cf) as scf:
    scf.wait_for_params()

    mission_cfg = LogConfig(name="mission", period_in_ms=self.log_period_ms)
    ...
    flow_cfg = LogConfig(name="flow", period_in_ms=self.log_period_ms)
    ...
    range_cfg = LogConfig(name="range", period_in_ms=self.log_period_ms)
    ...
    for cfg in logconfs:
        cfg.start()

    cf.console.receivedChar.add_callback(on_console)
    set_active_with_retry(0)
    self._emit("connected", uri=uri)
```

Source: `tools/radio_wall_follow_dashboard.py:345-394`

Explanation:

- `SyncCrazyflie` is the connection scope.
- `wait_for_params()` ensures the parameter TOC is available before trying to write `app.active`.
- The dashboard subscribes to three log blocks and to the Crazyflie console stream.
- It explicitly clears `app.active` to `0` on connect.

### 8.5 Mission start/stop transport

The dashboard sends mission commands indirectly by writing `app.active`:

```python
def set_active_with_retry(active: int) -> None:
    ...
    if cf.param.toc.get_element("app", "active") is None:
        time.sleep(0.2)
        continue
    cf.param.set_value("app.active", str(active))
    return
```

Source: `tools/radio_wall_follow_dashboard.py:323-331`

Explanation:

- This is the only control command used by the dashboard for mission start/stop.
- The dashboard does not inject velocity commands or CRTP commander packets directly.

## 9. Python Dashboard Features and Functions

### 9.1 Scan

The scan worker is:

```python
class ScanSession(threading.Thread):
    def run(self) -> None:
        cflib.crtp.init_drivers()

        try:
            links = cflib.crtp.scan_interfaces()
        except Exception as exc:
            ok, radio_message = check_crazyradio_access()
            ...
            self._emit("scan_result", interfaces=[], selected_uri=None)
            return

        interfaces = [uri for uri, _ in links]
        selected_uri = None
        for uri in interfaces:
            if uri.startswith("radio://"):
                selected_uri = uri
                break
```

Source: `tools/radio_wall_follow_dashboard.py:425-456`

Explanation:

- Scan uses `cflib.crtp.scan_interfaces()` to discover available Crazyflie links.
- It prefers a `radio://` URI if found.
- The result is posted back to the UI through the event queue.

Implementation note:

- In the current workspace, `check_crazyradio_access()` is called here but only `discover_uri` is imported at the top of the file.
- `check_crazyradio_access()` is defined in `tools/radio_wall_follow_mission.py`, so the current dashboard scan error path will raise `NameError` unless that function is imported as well.

### 9.2 Connect

The connect button handler is:

```python
def connect(self) -> None:
    if self.ui_state == self.STATE_CONNECTED:
        self._disconnect_session("Disconnect requested")
    elif self.ui_state == self.STATE_CONNECTING:
        self._disconnect_session("Cancelling connection")
    elif self.ui_state == self.STATE_SCANNING:
        return
    else:
        self._begin_connection()
```

Source: `tools/radio_wall_follow_dashboard.py:728-736`

The actual connection startup is:

```python
def _begin_connection(self) -> None:
    ...
    preferred_uri = self.uri_var.get().strip() or None
    ...
    self.worker = RadioSession(
        preferred_uri=preferred_uri,
        cache_dir=self.args.cache,
        log_period_ms=self.args.log_period_ms,
        event_queue=self.event_queue,
    )
    self.worker.start()
```

Source: `tools/radio_wall_follow_dashboard.py:899-927`

Explanation:

- The button is stateful:
  - disconnected -> connect
  - connecting -> cancel
  - connected -> disconnect
- `_begin_connection()` spawns a background `RadioSession`, keeping the UI responsive.

### 9.3 Start Mission

The Start Mission button handler is:

```python
def start_mission(self) -> None:
    if not self.worker or not self.connected:
        self.pending_start = True
        self._append_console("Start requested without an active link. Reconnecting first.")
        self._begin_connection()
        return

    self.pending_start = False
    self.worker.request_active(True)
    self._append_console("Start button pressed")
```

Source: `tools/radio_wall_follow_dashboard.py:754-763`

Explanation:

- If already connected, it queues `set_active=1`.
- If disconnected, it reconnects first and sets `pending_start`.
- When `_handle_connected()` later runs, it automatically sends `app.active=1`.

The reconnect auto-start logic is:

```python
if self.pending_start and self.worker:
    self.pending_start = False
    self.status_var.set("Link restored. Sending app.active=1")
    self.worker.request_active(True)
    self._append_console("Auto-starting mission after reconnect")
```

Source: `tools/radio_wall_follow_dashboard.py:801-805`

Explanation:

- This avoids losing a start request if the link is down when the button is pressed.

### 9.4 Stop Mission

The Stop Mission button handler is:

```python
def stop_mission(self) -> None:
    self.pending_start = False
    if not self.worker or not self.connected:
        return
    self.worker.request_active(False)
    self._append_console("Stop button pressed")
```

Source: `tools/radio_wall_follow_dashboard.py:765-770`

Explanation:

- Stop simply writes `app.active=0`.
- That causes `wall_following.c` to leave the unlocked mission state and output a zero setpoint.

## 10. Dashboard UI Logic and Data Mapping

### 10.1 Event-driven UI update architecture

The UI main thread processes worker events in `_pump_events()`:

```python
def _pump_events(self) -> None:
    while True:
        try:
            event_type, payload = self.event_queue.get_nowait()
        except queue.Empty:
            break

        if event_type == "status":
            self._handle_status(payload)
        elif event_type == "connected":
            self._handle_connected(payload)
        elif event_type == "disconnected":
            self._handle_disconnected()
        elif event_type == "scan_result":
            self._handle_scan_result(payload)
        elif event_type == "mission":
            self.mission = MissionSnapshot(**payload)
            self._render_mission()
            self._render_human()
        elif event_type == "flow":
            self.flow = FlowSnapshot(**payload)
            self._render_flow()
        elif event_type == "range":
            self.range = RangeSnapshot(**payload)
            self._render_mission()
            self._render_range()
```

Source: `tools/radio_wall_follow_dashboard.py:929-958`

Explanation:

- Workers never touch Tk widgets directly.
- They only push typed events into `event_queue`.
- The Tk main thread polls the queue every 60 ms and updates the UI.

### 10.2 Mission panel

The mission panel widgets are created here:

```python
self.outer_value = self._value_pair(body, 0, "Outer State")
self.mission_value = self._value_pair(body, 1, "Mission State")
self.z_value = self._value_pair(body, 2, "Height")
self.battery_value = self._value_pair(body, 3, "Battery")
self.max_temp_mission_value = self._value_pair(body, 4, "Max Temp")
self.therm_mission_value = self._value_pair(body, 5, "Thermistor")
self.hold_value = self._value_pair(body, 6, "Human Hold")
self.conf_value = self._value_pair(body, 7, "Confidence")
self.dir_value = self._value_pair(body, 8, "Direction")
```

Source: `tools/radio_wall_follow_dashboard.py:661-669`

The render logic is:

```python
self.outer_value.configure(text=OUTER_NAMES.get(self.mission.outer, str(self.mission.outer)))
self.mission_value.configure(text=MISSION_NAMES.get(self.mission.mission, str(self.mission.mission)))
self.z_value.configure(text=f"{self.mission.z:.2f} m")
self.battery_value.configure(text=f"{self.range.vbat:.2f} V" if self.range.vbat else "--")
self.max_temp_mission_value.configure(text=f"{self.mission.max_temp_c:.2f} C")
self.therm_mission_value.configure(text=f"{self.mission.therm_c:.2f} C")
self.hold_value.configure(text=f"{self.mission.hold:.2f} s")
self.conf_value.configure(text=f"{self.mission.conf} %")
self.dir_value.configure(text=DIR_NAMES.get(self.mission.direction, str(self.mission.direction)))
```

Source: `tools/radio_wall_follow_dashboard.py:966-975`

Panel mapping:

| UI element | Data source | Origin |
| --- | --- | --- |
| Outer State | `app.stateOuter` | `wall_following.c` |
| Mission State | `app.mission` | `wall_following.c` |
| Height | `stateEstimate.z` | Crazyflie estimator |
| Battery | `pm.vbat` | Crazyflie power management |
| Max Temp | `app.humanMax` | ESP32 packet -> `espUart` -> `wall_following.c` |
| Thermistor | `app.humanTherm` | ESP32 packet -> `espUart` -> `wall_following.c` |
| Human Hold | `app.humanHold` | `wall_following.c` derived logic |
| Confidence | `app.humanConf` | ESP32 packet -> `wall_following.c` |
| Direction | `app.humanDir` | ESP32 packet -> `wall_following.c` |

### 10.3 Human Detection panel

The widgets are created here:

```python
self.fresh_value = tk.Label(flags_row, text="Fresh: --", ...)
self.stable_value = tk.Label(flags_row, text="Stable: --", ...)

self.human_led_canvas.create_oval(..., tags="led")
self.human_led_canvas.create_text(..., tags="led_text")

self.human_status = tk.Label(body, text="Waiting for telemetry", ...)
self.max_temp_value = tk.Label(body, text="Max Temp: --", ...)
self.therm_value = tk.Label(body, text="Thermistor: --", ...)
```

Source: `tools/radio_wall_follow_dashboard.py:675-692`

The update logic is:

```python
led_color = human_led_color(self.mission)
human_text = "HUMAN" if self.mission.human else "NO\nHUMAN"
status_text = "Detected" if self.mission.human else "No human"
if self.mission.human and self.mission.stable:
    status_text = "Stable detection"
elif self.mission.human and self.mission.fresh:
    status_text = "Fresh detection"

self.human_led_canvas.itemconfigure("led", fill=led_color)
self.human_led_canvas.itemconfigure("led_text", text=human_text)
self.human_status.configure(text=status_text)
self.fresh_value.configure(text=f"Fresh: {self.mission.fresh}")
self.stable_value.configure(text=f"Stable: {self.mission.stable}")
self.max_temp_value.configure(text=f"Max Temp: {self.mission.max_temp_c:.2f} C")
self.therm_value.configure(text=f"Thermistor: {self.mission.therm_c:.2f} C")
```

Source: `tools/radio_wall_follow_dashboard.py:977-992`

Explanation:

- This panel is purely driven by `MissionSnapshot`.
- The LED color is derived from `human`, `fresh`, and `stable`.
- The dashboard does not calculate detection state itself; it trusts the Crazyflie app-level logs.

### 10.4 Flow panel

The Flow panel widgets are:

```python
self.delta_value = self._value_pair(body, 0, "Delta")
self.shutter_value = self._value_pair(body, 1, "Shutter")
self.vel_value = self._value_pair(body, 2, "Velocity")
self.cmd_value = self._value_pair(body, 3, "Command")
self.flow_age_value = self._value_pair(body, 4, "Timestamp")
```

Source: `tools/radio_wall_follow_dashboard.py:698-702`

Render logic:

```python
self.delta_value.configure(text=f"({self.flow.delta_x}, {self.flow.delta_y})")
self.shutter_value.configure(text=str(self.flow.shutter))
self.vel_value.configure(text=f"({self.flow.vel_x:.2f}, {self.flow.vel_y:.2f}) m/s")
self.cmd_value.configure(text=f"({self.flow.cmd_x:.2f}, {self.flow.cmd_y:.2f}) m/s")
self.flow_age_value.configure(text=f"{self.flow.timestamp_ms} ms")
```

Source: `tools/radio_wall_follow_dashboard.py:994-999`

Panel mapping:

| UI element | Data source | Origin |
| --- | --- | --- |
| Delta | `motion.deltaX`, `motion.deltaY` | Flow deck motion sensor |
| Shutter | `motion.shutter` | Flow deck |
| Velocity | `stateEstimate.vx`, `stateEstimate.vy` | Crazyflie estimator |
| Command | `app.cmdVelX`, `app.cmdVelY` | `wall_following.c` mission output |
| Timestamp | `LogConfig` timestamp | `cflib` callback timestamp |

### 10.5 Range Sector panel

The canvas layout is constructed with one rectangle per direction:

```python
self.range_items = {
    "front_rect": self.range_canvas.create_rectangle(...),
    "front_text": self.range_canvas.create_text(..., text="Front\n--"),
    "left_rect": self.range_canvas.create_rectangle(...),
    "left_text": self.range_canvas.create_text(..., text="Left\n--"),
    "right_rect": self.range_canvas.create_rectangle(...),
    "right_text": self.range_canvas.create_text(..., text="Right\n--"),
    "back_rect": self.range_canvas.create_rectangle(...),
    "back_text": self.range_canvas.create_text(..., text="Back\n--"),
    "center": self.range_canvas.create_oval(...),
    "center_text": self.range_canvas.create_text(..., text="CF"),
}
```

Source: `tools/radio_wall_follow_dashboard.py:710-720`

Render logic:

```python
for direction, distance in sectors.items():
    self.range_canvas.itemconfigure(self.range_items[f"{direction}_rect"], fill=range_fill(distance))
    self.range_canvas.itemconfigure(
        self.range_items[f"{direction}_text"],
        text=f"{direction.title()}\n{pretty_distance(distance)}",
    )

self.up_value.configure(text=f"Up: {pretty_distance(self.range.up)}")
self.range_age_value.configure(text=f"Timestamp: {self.range.timestamp_ms} ms")
```

Source: `tools/radio_wall_follow_dashboard.py:1001-1016`

Explanation:

- The range panel visualizes raw multiranger readings.
- `range_fill()` converts measured distance into a color band.
- `pretty_distance()` formats millimeters into centimeters or `--` if the reading is invalid / saturated.

### 10.6 Status Log panel

The status log is built with a `tk.Text` widget:

```python
self.console_text = tk.Text(
    console_frame,
    height=7,
    bg=PALETTE["canvas_bg"],
    fg=PALETTE["text_main"],
    ...
    state="disabled",
)
```

Source: `tools/radio_wall_follow_dashboard.py:644-654`

It is updated by:

```python
def _append_console(self, line: str) -> None:
    timestamp = time.strftime("%H:%M:%S")
    self.console_lines.append(f"[{timestamp}] {line}")
    self.console_lines = self.console_lines[-16:]
    self.console_text.configure(state="normal")
    self.console_text.delete("1.0", "end")
    self.console_text.insert("end", "\n".join(self.console_lines))
    self.console_text.see("end")
    self.console_text.configure(state="disabled")
```

Source: `tools/radio_wall_follow_dashboard.py:778-786`

Explanation:

- It stores the most recent 16 status lines.
- It displays both local UI messages and Crazyflie console lines forwarded by `cf.console.receivedChar`.

## 11. End-to-End Data Flow

### 11.1 Human-detection data flow: ESP32 to dashboard

1. **ESP32 sensor processing**
   - `crazyflie_esp_main.c`
   - `run_human_inference()` produces `human_detected`, `confidence`, `human_dir`, temperatures

2. **ESP32 UART transmission**
   - `send_detection_to_crazyflie()`
   - `uart_cf_send_packet(UART_CF_TYPE_HUMAN_DETECT, payload, 11)`

3. **Crazyflie UART receive**
   - `USART2_IRQHandler()` -> `rxStream`
   - `espUartBridgeTask()` -> `consumeByte()` -> `handlePacket()`

4. **Crazyflie semantic storage**
   - `latestHumanSample`
   - `espUart.*` log group

5. **Crazyflie mission-level consumption**
   - `wall_following.c`
   - `updateHumanPerception()` copies `espUart.*` into `app.*` state

6. **Crazyflie radio telemetry**
   - dashboard `RadioSession` subscribes to `app.human*`

7. **Dashboard UI**
   - `MissionSnapshot` + `Human Detection` panel update

### 11.2 Start Mission control flow: dashboard to Crazyflie

1. User clicks **Start Mission**
2. `DashboardApp.start_mission()` queues `request_active(True)`
3. `RadioSession` receives `("set_active", 1)`
4. `cf.param.set_value("app.active", "1")`
5. `wall_following.c` sees `appActive == 1`
6. If the required decks are initialized, `stateOuterLoop` changes from `idle` to `unlocked`
7. `wall_following.c` begins generating setpoints
8. `commanderSetSetpoint()` feeds the Crazyflie flight stack

## 12. Conclusions

### 12.1 What is actually active now

The active FYP architecture is:

- **ESP32-C3 -> Crazyflie**: custom UART protocol on UART1 (ESP32) to UART2 (Crazyflie)
- **Crazyflie -> dashboard**: CRTP telemetry over Crazyradio using `cflib`
- **Mission trigger**: dashboard writes `app.active`
- **Flight logic**: `wall_following.c`

### 12.2 What is not active

- The **I2C bridge** exists in the repo but is intentionally disabled.
- **CPX on UART2** exists in the repo but is intentionally disabled for this mission build because it conflicts with the ESP UART bridge.

### 12.3 Most important architectural insight for presentation

The dashboard does not command the drone directly.

Instead:

1. The dashboard enables the mission with `app.active`.
2. `wall_following.c` becomes the mission brain.
3. ESP32-derived human-detection packets modify the mission state machine through `espUart.*` -> `app.*`.
4. The Crazyflie controller/estimator stack converts those mission setpoints into actual flight.

This separation is important for explaining the system to professors:

- **ESP32-C3**: sensing and semantic detection
- **Crazyflie firmware**: communication bridge, mission logic, stabilization
- **Python dashboard**: operator interface, telemetry visualization, mission arming/disarming

## 13. Implementation Notes / Issues Found During Analysis

1. The dashboard scan error path uses `check_crazyradio_access()` but does not import it from `radio_wall_follow_mission.py`.
   - Current file imports only `discover_uri`.
   - This is a real code issue in the current workspace.

2. `wall_following.c` currently reads ESP32 data via the Crazyflie log subsystem instead of directly using `esp_uart_bridge.h`.
   - This works, but it is a string-based internal dependency.
   - A cleaner embedded design would call `espUartBridgeGetLatestHumanDetectionSample()` directly.

3. The dashboard depends on the `app.*` log names staying stable.
   - If `wall_following.c` renames or removes `app.human*`, `app.cmdVel*`, `app.stateOuter`, or `app.mission`, the dashboard will lose data.
