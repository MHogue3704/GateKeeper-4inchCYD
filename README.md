# ESP32 Gate Monitor Display

A 4-inch TFT display system for monitoring gate status with MQTT integration for Home Assistant.

## Features

- Real-time gate status display (Open/Closed/No Signal)
- Weather information display
- ESP32-CAM live feed integration
- Touch screen interface with menu system
- MQTT status publishing for Home Assistant integration
- Web interface for logs and status
- Automatic screen dimming (25% after 5 minutes of gate closed)
- Event logging to SD card

## Hardware

- ESP32 with 4" TFT display (480x320)
- Touch screen (resistive)
- RGB LED indicators
- SD card for logging
- ESP32-CAM (optional, for camera feed)

## MQTT Integration

### MQTT Topics

The device publishes to two MQTT topics:

#### 1. `gate/status` (Retained)
Published whenever the gate state changes or on reconnection.

**Payload Format (JSON):**
```json
{
  "gate": "CLOSED",
  "gateState": 0,
  "wifi": "connected",
  "mqtt": "connected",
  "ip": "192.168.0.123",
  "uptime": 3600,
  "rssi": -45,
  "weather": "Clear, 72°F",
  "time": "2026-02-16 14:30:45"
}
```

**Field Definitions:**
- `gate`: Gate status as text (`OPEN`, `CLOSED`, or `NO_SIGNAL`)
- `gateState`: Numeric state (0=Closed, 1=Open, 2=No Signal)
- `wifi`: WiFi connection status (`connected` or `disconnected`)
- `mqtt`: MQTT connection status (`connected` or `disconnected`)
- `ip`: Device IP address
- `uptime`: Device uptime in seconds
- `rssi`: WiFi signal strength in dBm
- `weather`: Current weather string
- `time`: Timestamp of status update

#### 2. `gate/log`
Published when gate events occur (open, closed, signal lost).

**Payload Format (Plain Text):**
```
2026-02-16 14:30:45: CLOSED
```

### Home Assistant Configuration

Add the following to your Home Assistant `configuration.yaml`:

#### Binary Sensor (Gate State)

```yaml
mqtt:
  binary_sensor:
    - name: "Front Gate"
      state_topic: "gate/status"
      value_template: "{{ value_json.gate }}"
      payload_on: "OPEN"
      payload_off: "CLOSED"
      device_class: door
      unique_id: front_gate_status
```

#### Sensors (Additional Information)

```yaml
mqtt:
  sensor:
    # Gate state as sensor (for history/graphs)
    - name: "Gate State Text"
      state_topic: "gate/status"
      value_template: "{{ value_json.gate }}"
      icon: mdi:gate
      unique_id: gate_state_text
    
    # Device IP address
    - name: "Gate Monitor IP"
      state_topic: "gate/status"
      value_template: "{{ value_json.ip }}"
      icon: mdi:ip-network
      unique_id: gate_monitor_ip
    
    # WiFi signal strength
    - name: "Gate Monitor WiFi Signal"
      state_topic: "gate/status"
      value_template: "{{ value_json.rssi }}"
      unit_of_measurement: "dBm"
      device_class: signal_strength
      unique_id: gate_monitor_rssi
    
    # Device uptime
    - name: "Gate Monitor Uptime"
      state_topic: "gate/status"
      value_template: "{{ (value_json.uptime | int / 3600) | round(1) }}"
      unit_of_measurement: "hours"
      icon: mdi:clock-outline
      unique_id: gate_monitor_uptime
    
    # Weather info
    - name: "Gate Location Weather"
      state_topic: "gate/status"
      value_template: "{{ value_json.weather }}"
      icon: mdi:weather-partly-cloudy
      unique_id: gate_location_weather
    
    # Connection status
    - name: "Gate Monitor Status"
      state_topic: "gate/status"
      value_template: >
        {% if value_json.mqtt == 'connected' and value_json.wifi == 'connected' %}
          Online
        {% else %}
          Offline
        {% endif %}
      icon: mdi:monitor
      unique_id: gate_monitor_status
```

#### Event Log Sensor

```yaml
mqtt:
  sensor:
    - name: "Gate Last Event"
      state_topic: "gate/log"
      icon: mdi:clipboard-text-clock
      unique_id: gate_last_event
```

### Example Automations

#### Notify when gate opens

```yaml
automation:
  - alias: "Gate Opened Notification"
    trigger:
      - platform: mqtt
        topic: "gate/status"
    condition:
      - condition: template
        value_template: "{{ trigger.payload_json.gate == 'OPEN' }}"
    action:
      - service: notify.mobile_app
        data:
          title: "Gate Alert"
          message: "Front gate is now OPEN"
          data:
            priority: high
```

#### Notify when gate loses signal

```yaml
automation:
  - alias: "Gate Signal Lost"
    trigger:
      - platform: mqtt
        topic: "gate/status"
    condition:
      - condition: template
        value_template: "{{ trigger.payload_json.gate == 'NO_SIGNAL' }}"
    action:
      - service: notify.mobile_app
        data:
          title: "Gate Alert"
          message: "Gate monitor has lost signal"
          data:
            tag: "gate_no_signal"
```

#### Turn on lights when gate opens at night

```yaml
automation:
  - alias: "Gate Open - Lights On"
    trigger:
      - platform: mqtt
        topic: "gate/status"
    condition:
      - condition: template
        value_template: "{{ trigger.payload_json.gate == 'OPEN' }}"
      - condition: sun
        after: sunset
      - condition: sun
        before: sunrise
    action:
      - service: light.turn_on
        target:
          entity_id: light.driveway_lights
        data:
          brightness_pct: 100
```

### Lovelace Dashboard Card Example

```yaml
type: vertical-stack
cards:
  - type: entities
    title: Front Gate Monitor
    entities:
      - entity: binary_sensor.front_gate
        name: Gate Status
      - entity: sensor.gate_monitor_status
        name: Monitor
      - entity: sensor.gate_monitor_wifi_signal
        name: WiFi Signal
      - entity: sensor.gate_last_event
        name: Last Event
  
  - type: history-graph
    title: Gate Activity (24h)
    entities:
      - entity: binary_sensor.front_gate
    hours_to_show: 24
    refresh_interval: 60
```

## Configuration

Edit the following constants in `src/main.cpp`:

```cpp
const char* ssid = "YourWiFiSSID";
const char* password = "YourWiFiPassword";
const char* mqttHost = "192.168.0.19";  // Your MQTT broker IP
const uint16_t mqttPort = 1883;
const char* mqttUser = "your_mqtt_username";    // Required for most HAOS Mosquitto setups
const char* mqttPassword = "your_mqtt_password";
```

If your broker allows anonymous access, leave `mqttUser` and `mqttPassword` empty (`""`).

### MQTT Troubleshooting (HAOS)

- Ensure Home Assistant Mosquitto add-on is running and listening on `1883`
- Use a dedicated user from Home Assistant (`Settings -> People -> Users`) with MQTT permissions
- Confirm the broker IP is reachable from the ESP32 WiFi network
- Check Serial output for MQTT state messages (e.g. `Bad credentials`, `Unauthorized`, `Broker unavailable`)

## Web Interface

The device hosts a web interface accessible at `http://<device-ip>/`:

- `/` - Status overview
- `/status` - JSON status endpoint
- `/log` - View event log
- `/erase` - Erase log (POST request)

## UDP Control

The device listens on UDP port 4210 for gate status updates:

- Send `OPEN` to indicate gate opened
- Send `CLOSED` to indicate gate closed
- If no packet received for 15 seconds, status changes to "NO SIGNAL"

## Screen Behavior

- **Normal operation**: Full brightness (255)
- **Auto-dim**: After 5 minutes of "GATE CLOSED" without state changes, screen dims to 25% brightness (64)
- **Wake on change**: Any gate state change or touch interaction returns screen to full brightness
- **Menu timeout**: Menu and camera views return to main status after 30 seconds of inactivity

## Touch Interface

- **Menu button** (top right): Access settings and options
- **Camera button** (top right, second): View live ESP32-CAM feed
- Touch anywhere to wake screen from dim state

## Building and Uploading

This project uses PlatformIO:

```bash
# Build
pio run

# Upload
pio run --target upload

# Clean
pio run --target clean
```

## Dependencies

- TFT_eSPI
- ArduinoJson
- PubSubClient (MQTT)
- TJpg_Decoder
- PNGdec

## License

This project is for personal use.

## Author

MHogue Tech
