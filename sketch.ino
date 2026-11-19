/**
 * @file sketch.ino
 * @brief EcoFlow MicroGrid Node M1 - embedded application for the ESP32 (Wokwi prototype).
 * @details Compact IoT backup and local load management controller for clean-energy home
 *          cabinets. The application follows the Modest IoT Nano Framework architecture
 *          (Device, Sensor, Actuator, Event/EventHandler, Command/CommandHandler):
 *          sensors raise events, the device reacts to them and dispatches commands to
 *          actuators. Business rules live in a hardware-independent controller.
 *
 *          Hardware:
 *          - Reservoir Level Sensor (HC-SR04): coolant/thermal electrolyte reservoir level.
 *          - Enclosure Proximity Sensor (HC-SR04): technician/user presence at <= 15 cm.
 *          - Servo motor: mechanical ventilation/bypass valve towards the auxiliary bus.
 *          - Load cell amplifier (HX711, 5 kg): auxiliary battery module weight in grams.
 *          - LCD 16x2 I2C: reservoir level (%), module weight (g) and "Empty" state.
 *
 * @author Jefferson Ernesto Castro Pariona
 * @date 2026-11-20
 * @version 1.0.0
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <HX711.h>
#include <ArduinoJson.h>

// =============================================================================
// SECTION: Configuration constants (no magic numbers / magic strings)
// =============================================================================

/**
 * @namespace DeviceConfig
 * @brief Centralized configuration: identity, pins, thresholds, timing and texts.
 */
namespace DeviceConfig {
  // ---- Company and device identity ----
  constexpr const char* COMPANY_NAME = "EcoFlow";
  constexpr const char* COMPANY_DESCRIPTION =
      "Smart portable energy storage, residential solar and home backup solutions";
  constexpr const char* COMPANY_GLOBAL_URL = "https://www.ecoflow.com/global";
  constexpr const char* COMPANY_REGIONAL_URL = "https://www.ecoflow.com/us";
  constexpr const char* COMPANY_SUPPORT_URL = "https://www.ecoflow.com/us/support/download/index";
  constexpr const char* COMPANY_DEVELOPER_URL = "https://developer.ecoflow.com/us";
  constexpr const char* DEVICE_NAME = "EcoFlow MicroGrid Node M1";
  constexpr const char* DEVICE_DESCRIPTION =
      "Compact IoT backup and local load management controller for home clean-energy cabinets";
  constexpr const char* FIRMWARE_VERSION = "1.0.0";
  constexpr const char* DEVELOPER_NAME = "Jefferson Ernesto Castro Pariona";
  constexpr const char* DEVELOPER_ROLE = "IoT Embedded Developer - Team Member";

  // ---- Serial ----
  constexpr unsigned long SERIAL_BAUD_RATE = 115200;
  constexpr size_t SERIAL_COMMAND_BUFFER_SIZE = 32;
  constexpr char LINE_FEED = '\n';
  constexpr char CARRIAGE_RETURN = '\r';
  constexpr char STRING_TERMINATOR = '\0';

  // ---- GPIO pins ----
  constexpr uint8_t RESERVOIR_TRIGGER_PIN = 5;
  constexpr uint8_t RESERVOIR_ECHO_PIN = 18;
  constexpr uint8_t PROXIMITY_TRIGGER_PIN = 32;
  constexpr uint8_t PROXIMITY_ECHO_PIN = 33;
  constexpr uint8_t SERVO_PIN = 13;
  constexpr uint8_t LOAD_CELL_DATA_PIN = 16;
  constexpr uint8_t LOAD_CELL_CLOCK_PIN = 4;
  constexpr uint8_t I2C_SDA_PIN = 21;
  constexpr uint8_t I2C_SCL_PIN = 22;

  // ---- Ultrasonic sensor (HC-SR04) ----
  constexpr unsigned int TRIGGER_SETTLE_US = 2;
  constexpr unsigned int TRIGGER_PULSE_US = 10;
  constexpr unsigned long ECHO_TIMEOUT_US = 30000UL;
  constexpr unsigned long NO_ECHO_DURATION = 0UL;
  constexpr float SOUND_SPEED_CM_PER_US = 0.0343f;
  constexpr float ROUND_TRIP_DIVISOR = 2.0f;

  // ---- Reservoir (coolant / thermal electrolyte) ----
  constexpr float RESERVOIR_HEIGHT_CM = 35.0f;          ///< Total reservoir height.
  constexpr float RESERVOIR_FULL_DISTANCE_CM = 5.0f;    ///< Sensor reading when full.
  constexpr float RESERVOIR_CAPACITY_ML = 3000.0f;      ///< Maximum capacity (3.0 L).
  constexpr float RESERVOIR_EMPTY_LEVEL_PERCENT = 0.0f; ///< At or below this level -> "Empty".
  constexpr float PERCENT_MIN = 0.0f;
  constexpr float PERCENT_MAX = 100.0f;

  // ---- Enclosure proximity ----
  constexpr float PROXIMITY_THRESHOLD_CM = 15.0f;       ///< Presence when distance <= 15 cm.

  // ---- Load cell (HX711, 5 kg) ----
  constexpr float LOAD_CELL_SCALE_FACTOR = 0.42f;       ///< Wokwi 5 kg cell: 2100 raw = 5000 g.
  constexpr unsigned long LOAD_CELL_READY_TIMEOUT_MS = 100UL;
  constexpr uint8_t LOAD_CELL_SAMPLES = 1;
  constexpr float MIN_VALID_WEIGHT_G = 0.0f;
  constexpr float LOAD_NOMINAL_CAPACITY_G = 1000.0f;    ///< Holder nominal capacity (1 kg).
  constexpr float LOW_LOAD_RATIO = 0.20f;               ///< 20 % -> coupling demand.
  constexpr float TARGET_LOAD_RATIO = 0.80f;            ///< 80 % -> coupling target.
  constexpr float RECOMMENDED_MAX_LOAD_RATIO = 0.80f;   ///< Manufacturer recommended payload.
  constexpr float LOW_LOAD_THRESHOLD_G = LOAD_NOMINAL_CAPACITY_G * LOW_LOAD_RATIO;           // 200 g
  constexpr float TARGET_LOAD_G = LOAD_NOMINAL_CAPACITY_G * TARGET_LOAD_RATIO;               // 800 g
  constexpr float RECOMMENDED_MAX_LOAD_G = LOAD_NOMINAL_CAPACITY_G * RECOMMENDED_MAX_LOAD_RATIO; // 800 g

  // ---- Servo (bypass / coupling valve) ----
  constexpr int SERVO_CLOSED_ANGLE = 0;
  constexpr int SERVO_OPEN_ANGLE = 90;
  constexpr int SERVO_PWM_FREQUENCY_HZ = 50;
  constexpr int SERVO_MIN_PULSE_US = 500;
  constexpr int SERVO_MAX_PULSE_US = 2400;

  // ---- LCD 16x2 I2C ----
  constexpr uint8_t LCD_I2C_ADDRESS = 0x27;
  constexpr uint8_t LCD_COLUMNS = 16;
  constexpr uint8_t LCD_ROWS = 2;
  constexpr uint8_t LCD_FIRST_ROW = 0;
  constexpr uint8_t LCD_SECOND_ROW = 1;
  constexpr uint8_t LCD_FIRST_COLUMN = 0;
  constexpr size_t LCD_LINE_BUFFER_SIZE = LCD_COLUMNS + 1;
  constexpr const char* LCD_SPLASH_TOP = "EcoFlow";
  constexpr const char* LCD_SPLASH_BOTTOM = "MicroGrid M1";
  constexpr const char* LCD_LEVEL_FORMAT = "Level: %5.1f %%";
  constexpr const char* LCD_LEVEL_EMPTY = "Level: Empty";
  constexpr const char* LCD_LEVEL_ERROR = "Level: Error";
  constexpr const char* LCD_WEIGHT_FORMAT = "Weight:%7.1f g";
  constexpr const char* LCD_WEIGHT_ERROR = "Weight: Error";
  constexpr const char* LCD_PADDED_LINE_FORMAT = "%-16s";
  constexpr unsigned long SPLASH_DURATION_MS = 1500UL;

  // ---- Scheduling (FreeRTOS) ----
  constexpr uint32_t CONTROL_CYCLE_MS = 250;
  constexpr unsigned long TELEMETRY_PERIOD_MS = 5000UL;
  constexpr uint32_t DEVICE_TASK_STACK_SIZE = 8192;
  constexpr unsigned int DEVICE_TASK_PRIORITY = 1;
  constexpr const char* DEVICE_TASK_NAME = "MicroGridNodeTask";

  // ---- Remote serial commands ----
  constexpr const char* COMMAND_STATUS = "STATUS";
  constexpr const char* COMMAND_METRICS = "METRICS";
  constexpr const char* COMMAND_HELP = "HELP";

  // ---- Component (event source) names ----
  constexpr const char* RESERVOIR_SENSOR_NAME = "ReservoirLevelSensor";
  constexpr const char* PROXIMITY_SENSOR_NAME = "EnclosureProximitySensor";
  constexpr const char* LOAD_CELL_NAME = "LoadCellAmplifier";
  constexpr const char* REMOTE_LISTENER_NAME = "RemoteCommandListener";
  constexpr float NO_VALUE = 0.0f;
}

/**
 * @namespace TelemetryKey
 * @brief JSON keys and values used in tracking records for the Device Maker.
 */
namespace TelemetryKey {
  constexpr const char* TYPE = "type";
  constexpr const char* UPTIME_MS = "uptimeMs";
  constexpr const char* FROM = "from";
  constexpr const char* TO = "to";
  constexpr const char* MODE = "mode";
  constexpr const char* SOURCE = "source";
  constexpr const char* STATE = "state";
  constexpr const char* DETAIL = "detail";
  constexpr const char* LEVEL_PERCENT = "levelPercent";
  constexpr const char* VOLUME_ML = "volumeMl";
  constexpr const char* RESERVOIR_STATE = "reservoirState";
  constexpr const char* DISTANCE_CM = "distanceCm";
  constexpr const char* TECHNICIAN_NEARBY = "technicianNearby";
  constexpr const char* LOAD_GRAMS = "loadGrams";
  constexpr const char* LOAD_BAND = "loadBand";
  constexpr const char* LOAD_PATH = "loadPath";
  constexpr const char* ACTIVATION_CYCLES = "activationCycles";
  constexpr const char* BLOCKED_ACTIVATIONS = "blockedActivations";
  constexpr const char* PROXIMITY_EVENTS = "proximityEvents";
  constexpr const char* MODE_TRANSITIONS = "modeTransitions";
  constexpr const char* SENSOR_FAULTS = "sensorFaults";
  constexpr const char* EMPTY_EVENTS = "reservoirEmptyEvents";

  constexpr const char* TYPE_MODE_TRANSITION = "MODE_TRANSITION";
  constexpr const char* TYPE_ACTIVATION_BLOCKED = "ACTIVATION_BLOCKED";
  constexpr const char* TYPE_PROXIMITY = "PROXIMITY_EVENT";
  constexpr const char* TYPE_RESERVOIR = "RESERVOIR_STATE";
  constexpr const char* TYPE_SENSOR_HEALTH = "SENSOR_HEALTH";
  constexpr const char* TYPE_LOAD_BAND = "LOAD_BAND";
  constexpr const char* TYPE_TELEMETRY = "TELEMETRY";
  constexpr const char* TYPE_METRICS = "METRICS";
  constexpr const char* TYPE_COMMAND_ERROR = "COMMAND_ERROR";

  constexpr const char* VALUE_EMPTY = "Empty";
  constexpr const char* VALUE_AVAILABLE = "Available";
  constexpr const char* VALUE_REFILLED = "Refilled";
  constexpr const char* VALUE_UNKNOWN = "Unknown";
  constexpr const char* VALUE_DETECTED = "Detected";
  constexpr const char* VALUE_CLEARED = "Cleared";
  constexpr const char* VALUE_FAULT = "Fault";
  constexpr const char* VALUE_RECOVERED = "Recovered";
  constexpr const char* VALUE_OPEN = "Open";
  constexpr const char* VALUE_CLOSED = "Closed";
  constexpr const char* DETAIL_EMPTY_LOCK = "High-current load actuator locked: reservoir Empty";
  constexpr const char* DETAIL_UNKNOWN_COMMAND = "Unknown remote command. Type HELP";
}

// =============================================================================
// SECTION: Modest IoT style base abstractions
// =============================================================================

/** @brief Event identifiers raised by sensors. */
namespace EventId {
  constexpr int SENSOR_FAULT_DETECTED = 100;
  constexpr int SENSOR_RECOVERED = 101;
  constexpr int RESERVOIR_EMPTY = 110;
  constexpr int RESERVOIR_REFILLED = 111;
  constexpr int PROXIMITY_DETECTED = 120;
  constexpr int PROXIMITY_CLEARED = 121;
  constexpr int REMOTE_STATUS_REQUESTED = 130;
  constexpr int REMOTE_METRICS_REQUESTED = 131;
  constexpr int REMOTE_HELP_REQUESTED = 132;
  constexpr int REMOTE_COMMAND_UNKNOWN = 133;
}

/** @brief Command identifiers dispatched to actuators. */
namespace CommandId {
  constexpr int OPEN_LOAD_PATH = 200;
  constexpr int CLOSE_LOAD_PATH = 201;
  constexpr int REFRESH_DISPLAY = 210;
}

/**
 * @struct Event
 * @brief Notification raised by a sensor.
 */
struct Event {
  int id;              ///< Event identifier (see EventId).
  const char* source;  ///< Name of the component that raised the event.
  float value;         ///< Optional measured value associated with the event.
};

/**
 * @struct Command
 * @brief Instruction dispatched to an actuator.
 */
struct Command {
  int id;  ///< Command identifier (see CommandId).
};

/**
 * @class EventHandler
 * @brief Interface for objects that react to events.
 */
class EventHandler {
 public:
  virtual ~EventHandler() {}
  /**
   * @brief Handles an incoming event.
   * @param event Event raised by a sensor.
   */
  virtual void on(const Event& event) = 0;
};

/**
 * @class CommandHandler
 * @brief Interface for objects that execute commands.
 */
class CommandHandler {
 public:
  virtual ~CommandHandler() {}
  /**
   * @brief Executes a command.
   * @param command Command to execute.
   */
  virtual void handle(const Command& command) = 0;
};

/**
 * @class Sensor
 * @brief Base class for input components that publish events to a handler.
 */
class Sensor {
 public:
  /**
   * @brief Creates a sensor.
   * @param name Component name used as event source.
   * @param handler Event handler that receives the notifications.
   */
  Sensor(const char* name, EventHandler* handler) : name_(name), handler_(handler), healthy_(true) {}
  virtual ~Sensor() {}

  /** @brief Configures the hardware resources of the sensor. */
  virtual void begin() = 0;

  /** @brief Reads the sensor and raises events when its state changes. */
  virtual void update() = 0;

  /**
   * @brief Indicates whether the last reading was valid.
   * @return true when the sensor is healthy.
   */
  bool isHealthy() const { return healthy_; }

  /**
   * @brief Gets the component name.
   * @return Sensor name.
   */
  const char* getName() const { return name_; }

 protected:
  /**
   * @brief Sends an event to the registered handler.
   * @param event Event to publish.
   */
  void notify(const Event& event) {
    if (handler_ != nullptr) {
      handler_->on(event);
    }
  }

  /**
   * @brief Updates the health state and raises fault/recovery events on transitions.
   * @param valid true when the latest reading is valid.
   */
  void reportHealth(bool valid) {
    if (valid == healthy_) {
      return;
    }
    healthy_ = valid;
    const int eventId = valid ? EventId::SENSOR_RECOVERED : EventId::SENSOR_FAULT_DETECTED;
    notify(Event{eventId, name_, DeviceConfig::NO_VALUE});
  }

 private:
  const char* name_;
  EventHandler* handler_;
  bool healthy_;
};

/**
 * @class Actuator
 * @brief Base class for output components that execute commands.
 */
class Actuator : public CommandHandler {
 public:
  virtual ~Actuator() {}
  /** @brief Configures the hardware resources of the actuator. */
  virtual void begin() = 0;
};

/**
 * @class Device
 * @brief Base IoT device: event handler, command dispatcher and FreeRTOS task owner.
 * @details start() initializes the device and launches its own task, so the Arduino
 *          loop() only needs to be inhibited (loop-less, event-driven design).
 */
class Device : public EventHandler, public CommandHandler {
 public:
  virtual ~Device() {}

  /** @brief Initializes components and launches the device task. */
  void start() {
    initialize();
    xTaskCreate(&Device::runTask, DeviceConfig::DEVICE_TASK_NAME, DeviceConfig::DEVICE_TASK_STACK_SIZE,
                this, DeviceConfig::DEVICE_TASK_PRIORITY, nullptr);
  }

  /** @brief Deletes the Arduino loop task; all behavior runs in the device task. */
  static void inhibitMainLoop() { vTaskDelete(nullptr); }

 protected:
  /** @brief Initializes sensors, actuators and services. */
  virtual void initialize() = 0;

  /** @brief Executes one control cycle. */
  virtual void runCycle() = 0;

  /**
   * @brief Gets the control cycle period.
   * @return Period in milliseconds.
   */
  virtual uint32_t getCyclePeriodMs() const = 0;

 private:
  /**
   * @brief FreeRTOS task entry point.
   * @param context Pointer to the Device instance.
   */
  static void runTask(void* context) {
    Device* device = static_cast<Device*>(context);
    for (;;) {
      device->runCycle();
      vTaskDelay(pdMS_TO_TICKS(device->getCyclePeriodMs()));
    }
  }
};

// =============================================================================
// SECTION: Sensors
// =============================================================================

/**
 * @class UltrasonicDistanceSensor
 * @brief HC-SR04 ultrasonic distance sensor (Template Method for derived sensors).
 */
class UltrasonicDistanceSensor : public Sensor {
 public:
  /**
   * @brief Creates an ultrasonic sensor.
   * @param name Component name.
   * @param handler Event handler.
   * @param triggerPin Trigger GPIO.
   * @param echoPin Echo GPIO.
   */
  UltrasonicDistanceSensor(const char* name, EventHandler* handler, uint8_t triggerPin, uint8_t echoPin)
      : Sensor(name, handler), triggerPin_(triggerPin), echoPin_(echoPin), distanceCm_(DeviceConfig::NO_VALUE) {}

  /** @brief Configures trigger and echo pins. */
  void begin() override {
    pinMode(triggerPin_, OUTPUT);
    pinMode(echoPin_, INPUT);
    digitalWrite(triggerPin_, LOW);
  }

  /** @brief Measures distance, reports health and delegates interpretation. */
  void update() override {
    float measuredCm = DeviceConfig::NO_VALUE;
    const bool valid = measureDistanceCm(measuredCm);
    reportHealth(valid);
    if (valid) {
      distanceCm_ = measuredCm;
      onDistanceMeasured(measuredCm);
    } else {
      onReadingFailed();
    }
  }

  /**
   * @brief Gets the last valid distance.
   * @return Distance in centimeters.
   */
  float getDistanceCm() const { return distanceCm_; }

 protected:
  /**
   * @brief Interprets a valid distance reading.
   * @param distanceCm Measured distance in centimeters.
   */
  virtual void onDistanceMeasured(float distanceCm) = 0;

  /** @brief Reacts to a missing echo (timeout). */
  virtual void onReadingFailed() {}

 private:
  /**
   * @brief Fires a trigger pulse and converts the echo duration.
   * @param distanceCm Output distance in centimeters.
   * @return true if an echo was received before the timeout.
   */
  bool measureDistanceCm(float& distanceCm) {
    digitalWrite(triggerPin_, LOW);
    delayMicroseconds(DeviceConfig::TRIGGER_SETTLE_US);
    digitalWrite(triggerPin_, HIGH);
    delayMicroseconds(DeviceConfig::TRIGGER_PULSE_US);
    digitalWrite(triggerPin_, LOW);
    const unsigned long echoDurationUs = pulseIn(echoPin_, HIGH, DeviceConfig::ECHO_TIMEOUT_US);
    if (echoDurationUs == DeviceConfig::NO_ECHO_DURATION) {
      return false;
    }
    distanceCm = (echoDurationUs * DeviceConfig::SOUND_SPEED_CM_PER_US) / DeviceConfig::ROUND_TRIP_DIVISOR;
    return true;
  }

  uint8_t triggerPin_;
  uint8_t echoPin_;
  float distanceCm_;
};

/**
 * @class ReservoirLevelSensor
 * @brief Top-mounted ultrasonic sensor that measures the reservoir level.
 */
class ReservoirLevelSensor : public UltrasonicDistanceSensor {
 public:
  /**
   * @brief Creates the reservoir level sensor.
   * @param handler Event handler.
   */
  explicit ReservoirLevelSensor(EventHandler* handler)
      : UltrasonicDistanceSensor(DeviceConfig::RESERVOIR_SENSOR_NAME, handler,
                                 DeviceConfig::RESERVOIR_TRIGGER_PIN, DeviceConfig::RESERVOIR_ECHO_PIN),
        levelPercent_(DeviceConfig::PERCENT_MIN), empty_(false) {}

  /**
   * @brief Converts a distance into a level percentage.
   * @param distanceCm Distance from the sensor to the liquid surface.
   * @return Level between 0 % and 100 %.
   */
  static float calculateLevelPercent(float distanceCm) {
    const float usableHeightCm = DeviceConfig::RESERVOIR_HEIGHT_CM - DeviceConfig::RESERVOIR_FULL_DISTANCE_CM;
    const float levelPercent =
        (DeviceConfig::RESERVOIR_HEIGHT_CM - distanceCm) / usableHeightCm * DeviceConfig::PERCENT_MAX;
    return constrain(levelPercent, DeviceConfig::PERCENT_MIN, DeviceConfig::PERCENT_MAX);
  }

  /** @return Level percentage of the last valid reading. */
  float getLevelPercent() const { return levelPercent_; }

  /** @return Estimated volume in milliliters. */
  float getVolumeMilliliters() const {
    return levelPercent_ / DeviceConfig::PERCENT_MAX * DeviceConfig::RESERVOIR_CAPACITY_ML;
  }

  /** @return true when the reservoir is "Empty". */
  bool isEmpty() const { return empty_; }

 protected:
  /**
   * @brief Updates level and raises Empty/Refilled events on transitions.
   * @param distanceCm Measured distance in centimeters.
   */
  void onDistanceMeasured(float distanceCm) override {
    levelPercent_ = calculateLevelPercent(distanceCm);
    const bool nowEmpty = levelPercent_ <= DeviceConfig::RESERVOIR_EMPTY_LEVEL_PERCENT;
    if (nowEmpty != empty_) {
      empty_ = nowEmpty;
      notify(Event{empty_ ? EventId::RESERVOIR_EMPTY : EventId::RESERVOIR_REFILLED, getName(), levelPercent_});
    }
  }

 private:
  float levelPercent_;
  bool empty_;
};

/**
 * @class EnclosureProximitySensor
 * @brief Front-frame ultrasonic sensor that detects a technician at <= 15 cm.
 */
class EnclosureProximitySensor : public UltrasonicDistanceSensor {
 public:
  /**
   * @brief Creates the proximity sensor.
   * @param handler Event handler.
   */
  explicit EnclosureProximitySensor(EventHandler* handler)
      : UltrasonicDistanceSensor(DeviceConfig::PROXIMITY_SENSOR_NAME, handler,
                                 DeviceConfig::PROXIMITY_TRIGGER_PIN, DeviceConfig::PROXIMITY_ECHO_PIN),
        nearby_(false) {}

  /** @return true when someone is within the proximity threshold. */
  bool isTechnicianNearby() const { return nearby_; }

 protected:
  /**
   * @brief Evaluates presence and raises Detected/Cleared events on transitions.
   * @param distanceCm Measured distance in centimeters.
   */
  void onDistanceMeasured(float distanceCm) override {
    setNearby(distanceCm <= DeviceConfig::PROXIMITY_THRESHOLD_CM, distanceCm);
  }

  /** @brief Fail-safe: an invalid reading is treated as no presence. */
  void onReadingFailed() override { setNearby(false, DeviceConfig::NO_VALUE); }

 private:
  /**
   * @brief Updates presence state and notifies changes.
   * @param nearby New presence state.
   * @param distanceCm Associated distance.
   */
  void setNearby(bool nearby, float distanceCm) {
    if (nearby == nearby_) {
      return;
    }
    nearby_ = nearby;
    notify(Event{nearby_ ? EventId::PROXIMITY_DETECTED : EventId::PROXIMITY_CLEARED, getName(), distanceCm});
  }

  bool nearby_;
};

/**
 * @class LoadCellAmplifier
 * @brief HX711 load cell amplifier (5 kg) under the auxiliary battery module.
 */
class LoadCellAmplifier : public Sensor {
 public:
  /**
   * @brief Creates the load cell amplifier.
   * @param handler Event handler.
   */
  explicit LoadCellAmplifier(EventHandler* handler)
      : Sensor(DeviceConfig::LOAD_CELL_NAME, handler), weightGrams_(DeviceConfig::NO_VALUE) {}

  /** @brief Starts the HX711 and applies the gram calibration factor. */
  void begin() override {
    scale_.begin(DeviceConfig::LOAD_CELL_DATA_PIN, DeviceConfig::LOAD_CELL_CLOCK_PIN);
    scale_.set_scale(DeviceConfig::LOAD_CELL_SCALE_FACTOR);
  }

  /** @brief Reads the weight; a not-ready amplifier is reported as a fault. */
  void update() override {
    const bool ready = scale_.wait_ready_timeout(DeviceConfig::LOAD_CELL_READY_TIMEOUT_MS);
    if (ready) {
      const float measured = scale_.get_units(DeviceConfig::LOAD_CELL_SAMPLES);
      weightGrams_ = measured < DeviceConfig::MIN_VALID_WEIGHT_G ? DeviceConfig::MIN_VALID_WEIGHT_G : measured;
    }
    reportHealth(ready);
  }

  /** @return Last valid weight in grams. */
  float getWeightGrams() const { return weightGrams_; }

 private:
  HX711 scale_;
  float weightGrams_;
};

/**
 * @class RemoteCommandListener
 * @brief Serial console listener that turns remote commands into events.
 */
class RemoteCommandListener : public Sensor {
 public:
  /**
   * @brief Creates the listener.
   * @param handler Event handler.
   */
  explicit RemoteCommandListener(EventHandler* handler)
      : Sensor(DeviceConfig::REMOTE_LISTENER_NAME, handler), length_(0) {
    buffer_[0] = DeviceConfig::STRING_TERMINATOR;
  }

  /** @brief No hardware setup required (Serial is started by the device). */
  void begin() override {}

  /** @brief Reads pending characters without blocking and parses full lines. */
  void update() override {
    while (Serial.available() > 0) {
      const char received = static_cast<char>(Serial.read());
      if (received == DeviceConfig::LINE_FEED || received == DeviceConfig::CARRIAGE_RETURN) {
        processLine();
      } else if (length_ < DeviceConfig::SERIAL_COMMAND_BUFFER_SIZE - 1) {
        buffer_[length_++] = static_cast<char>(toupper(static_cast<unsigned char>(received)));
      }
    }
  }

 private:
  /** @brief Maps the buffered line to an event and clears the buffer. */
  void processLine() {
    if (length_ == 0) {
      return;
    }
    buffer_[length_] = DeviceConfig::STRING_TERMINATOR;
    int eventId = EventId::REMOTE_COMMAND_UNKNOWN;
    if (strcmp(buffer_, DeviceConfig::COMMAND_STATUS) == 0) {
      eventId = EventId::REMOTE_STATUS_REQUESTED;
    } else if (strcmp(buffer_, DeviceConfig::COMMAND_METRICS) == 0) {
      eventId = EventId::REMOTE_METRICS_REQUESTED;
    } else if (strcmp(buffer_, DeviceConfig::COMMAND_HELP) == 0) {
      eventId = EventId::REMOTE_HELP_REQUESTED;
    }
    notify(Event{eventId, getName(), DeviceConfig::NO_VALUE});
    length_ = 0;
    buffer_[0] = DeviceConfig::STRING_TERMINATOR;
  }

  char buffer_[DeviceConfig::SERIAL_COMMAND_BUFFER_SIZE];
  size_t length_;
};

// =============================================================================
// SECTION: Actuators
// =============================================================================

/**
 * @class ServoMotor
 * @brief Mechanical ventilation / bypass valve that couples load to the auxiliary bus.
 */
class ServoMotor : public Actuator {
 public:
  /** @brief Creates the servo in the closed position. */
  ServoMotor() : open_(false) {}

  /** @brief Attaches the servo and forces the closed (safe) position. */
  void begin() override {
    servo_.setPeriodHertz(DeviceConfig::SERVO_PWM_FREQUENCY_HZ);
    servo_.attach(DeviceConfig::SERVO_PIN, DeviceConfig::SERVO_MIN_PULSE_US, DeviceConfig::SERVO_MAX_PULSE_US);
    servo_.write(DeviceConfig::SERVO_CLOSED_ANGLE);
  }

  /**
   * @brief Opens or closes the valve; ignores redundant commands.
   * @param command OPEN_LOAD_PATH or CLOSE_LOAD_PATH.
   */
  void handle(const Command& command) override {
    bool requestedOpen = open_;
    if (command.id == CommandId::OPEN_LOAD_PATH) {
      requestedOpen = true;
    } else if (command.id == CommandId::CLOSE_LOAD_PATH) {
      requestedOpen = false;
    } else {
      return;
    }
    if (requestedOpen == open_) {
      return;
    }
    open_ = requestedOpen;
    servo_.write(open_ ? DeviceConfig::SERVO_OPEN_ANGLE : DeviceConfig::SERVO_CLOSED_ANGLE);
  }

  /** @return true when the load path is open. */
  bool isOpen() const { return open_; }

 private:
  Servo servo_;
  bool open_;
};

/**
 * @struct DisplayFrame
 * @brief Data rendered on the LCD.
 */
struct DisplayFrame {
  bool levelValid;
  bool reservoirEmpty;
  float levelPercent;
  bool loadValid;
  float loadGrams;
};

/**
 * @class StatusDisplay
 * @brief LCD 16x2 I2C that shows reservoir level (%), weight (g) and "Empty".
 */
class StatusDisplay : public Actuator {
 public:
  /** @brief Creates the display with the configured address and size. */
  StatusDisplay()
      : lcd_(DeviceConfig::LCD_I2C_ADDRESS, DeviceConfig::LCD_COLUMNS, DeviceConfig::LCD_ROWS),
        frame_(DisplayFrame{false, false, DeviceConfig::NO_VALUE, false, DeviceConfig::NO_VALUE}) {
    lastTop_[0] = DeviceConfig::STRING_TERMINATOR;
    lastBottom_[0] = DeviceConfig::STRING_TERMINATOR;
  }

  /** @brief Starts I2C and the LCD, then shows a splash screen. */
  void begin() override {
    Wire.begin(DeviceConfig::I2C_SDA_PIN, DeviceConfig::I2C_SCL_PIN);
    lcd_.init();
    lcd_.backlight();
    writeLine(DeviceConfig::LCD_FIRST_ROW, DeviceConfig::LCD_SPLASH_TOP);
    writeLine(DeviceConfig::LCD_SECOND_ROW, DeviceConfig::LCD_SPLASH_BOTTOM);
    delay(DeviceConfig::SPLASH_DURATION_MS);
  }

  /**
   * @brief Sets the data to render on the next refresh.
   * @param frame Display data.
   */
  void setFrame(const DisplayFrame& frame) { frame_ = frame; }

  /**
   * @brief Renders the current frame (only rewrites lines that changed).
   * @param command REFRESH_DISPLAY.
   */
  void handle(const Command& command) override {
    if (command.id != CommandId::REFRESH_DISPLAY) {
      return;
    }
    char top[DeviceConfig::LCD_LINE_BUFFER_SIZE];
    char bottom[DeviceConfig::LCD_LINE_BUFFER_SIZE];
    formatLevelLine(top, sizeof(top));
    formatWeightLine(bottom, sizeof(bottom));
    if (strcmp(top, lastTop_) != 0) {
      writeLine(DeviceConfig::LCD_FIRST_ROW, top);
      strncpy(lastTop_, top, sizeof(lastTop_));
    }
    if (strcmp(bottom, lastBottom_) != 0) {
      writeLine(DeviceConfig::LCD_SECOND_ROW, bottom);
      strncpy(lastBottom_, bottom, sizeof(lastBottom_));
    }
  }

 private:
  /**
   * @brief Builds the reservoir line ("Level: xx.x %", "Level: Empty" or "Level: Error").
   * @param out Output buffer.
   * @param size Buffer size.
   */
  void formatLevelLine(char* out, size_t size) const {
    if (!frame_.levelValid) {
      snprintf(out, size, "%s", DeviceConfig::LCD_LEVEL_ERROR);
    } else if (frame_.reservoirEmpty) {
      snprintf(out, size, "%s", DeviceConfig::LCD_LEVEL_EMPTY);
    } else {
      snprintf(out, size, DeviceConfig::LCD_LEVEL_FORMAT, frame_.levelPercent);
    }
  }

  /**
   * @brief Builds the weight line ("Weight: xxx.x g" or "Weight: Error").
   * @param out Output buffer.
   * @param size Buffer size.
   */
  void formatWeightLine(char* out, size_t size) const {
    if (!frame_.loadValid) {
      snprintf(out, size, "%s", DeviceConfig::LCD_WEIGHT_ERROR);
    } else {
      snprintf(out, size, DeviceConfig::LCD_WEIGHT_FORMAT, frame_.loadGrams);
    }
  }

  /**
   * @brief Writes a full, space-padded line.
   * @param row LCD row.
   * @param text Text to write.
   */
  void writeLine(uint8_t row, const char* text) {
    char padded[DeviceConfig::LCD_LINE_BUFFER_SIZE];
    snprintf(padded, sizeof(padded), DeviceConfig::LCD_PADDED_LINE_FORMAT, text);
    lcd_.setCursor(DeviceConfig::LCD_FIRST_COLUMN, row);
    lcd_.print(padded);
  }

  LiquidCrystal_I2C lcd_;
  DisplayFrame frame_;
  char lastTop_[DeviceConfig::LCD_LINE_BUFFER_SIZE];
  char lastBottom_[DeviceConfig::LCD_LINE_BUFFER_SIZE];
};

// =============================================================================
// SECTION: Domain (hardware-independent business rules)
// =============================================================================

/** @brief Operation modes of the MicroGrid Node. */
enum class OperationMode : uint8_t { STANDBY, COUPLING, EMPTY_LOCKOUT, SENSOR_FAULT };

/** @brief Load bands relative to the holder nominal capacity. */
enum class LoadBand : uint8_t { UNKNOWN, BELOW_MINIMUM, NOMINAL, ABOVE_RECOMMENDED, OVER_CAPACITY };

/**
 * @brief Converts an operation mode into text.
 * @param mode Operation mode.
 * @return Mode name.
 */
const char* toString(OperationMode mode) {
  switch (mode) {
    case OperationMode::STANDBY:       return "STANDBY";
    case OperationMode::COUPLING:      return "COUPLING";
    case OperationMode::EMPTY_LOCKOUT: return "EMPTY_LOCKOUT";
    case OperationMode::SENSOR_FAULT:  return "SENSOR_FAULT";
  }
  return TelemetryKey::VALUE_UNKNOWN;
}

/**
 * @brief Converts a load band into text.
 * @param band Load band.
 * @return Band name.
 */
const char* toString(LoadBand band) {
  switch (band) {
    case LoadBand::BELOW_MINIMUM:     return "BELOW_MINIMUM";
    case LoadBand::NOMINAL:           return "NOMINAL";
    case LoadBand::ABOVE_RECOMMENDED: return "ABOVE_RECOMMENDED";
    case LoadBand::OVER_CAPACITY:     return "OVER_CAPACITY";
    case LoadBand::UNKNOWN:           break;
  }
  return TelemetryKey::VALUE_UNKNOWN;
}

/**
 * @struct NodeSnapshot
 * @brief Consistent view of all sensor readings for one control cycle.
 */
struct NodeSnapshot {
  bool reservoirValid;
  float reservoirLevelPercent;
  float reservoirVolumeMl;
  bool reservoirEmpty;
  bool proximityValid;
  bool technicianNearby;
  float proximityDistanceCm;
  bool loadValid;
  float loadGrams;
};

/**
 * @struct ControlDecision
 * @brief Result of evaluating a snapshot.
 */
struct ControlDecision {
  OperationMode previousMode;
  OperationMode mode;
  bool modeChanged;
  bool activationBlocked;  ///< Coupling demanded but locked because the reservoir is Empty.
  bool loadPathOpen;       ///< Requested servo state.
};

/**
 * @class MicroGridController
 * @brief State machine with the load coupling and Empty lockout rules.
 * @details Rules:
 *          - Invalid reservoir or load reading -> SENSOR_FAULT (fail-safe, valve closed).
 *          - Reservoir Empty -> EMPTY_LOCKOUT (high-current actuator blocked).
 *          - Load < 20 % (200 g) and proximity <= 15 cm -> COUPLING (valve open).
 *          - COUPLING continues until load >= 80 % (800 g) -> STANDBY.
 */
class MicroGridController {
 public:
  /**
   * @brief Creates the controller.
   * @param lowLoadGrams Load that triggers coupling demand.
   * @param targetLoadGrams Load that ends coupling.
   * @param recommendedMaxGrams Manufacturer recommended payload.
   * @param nominalCapacityGrams Holder nominal capacity.
   */
  MicroGridController(float lowLoadGrams, float targetLoadGrams, float recommendedMaxGrams, float nominalCapacityGrams)
      : lowLoadGrams_(lowLoadGrams), targetLoadGrams_(targetLoadGrams),
        recommendedMaxGrams_(recommendedMaxGrams), nominalCapacityGrams_(nominalCapacityGrams),
        mode_(OperationMode::STANDBY), demandBlocked_(false) {}

  /**
   * @brief Evaluates a snapshot and updates the operation mode.
   * @param snapshot Current readings.
   * @return Decision with the new mode and actuator request.
   */
  ControlDecision evaluate(const NodeSnapshot& snapshot) {
    ControlDecision decision;
    decision.previousMode = mode_;
    mode_ = resolveMode(snapshot);
    decision.mode = mode_;
    decision.modeChanged = decision.previousMode != mode_;
    const bool blockedNow = snapshot.reservoirValid && snapshot.reservoirEmpty && isCouplingDemanded(snapshot);
    decision.activationBlocked = blockedNow && !demandBlocked_;
    demandBlocked_ = blockedNow;
    decision.loadPathOpen = mode_ == OperationMode::COUPLING;
    return decision;
  }

  /**
   * @brief Classifies a load value.
   * @param valid Whether the reading is valid.
   * @param grams Load in grams.
   * @return Load band.
   */
  LoadBand classifyLoad(bool valid, float grams) const {
    if (!valid) return LoadBand::UNKNOWN;
    if (grams < lowLoadGrams_) return LoadBand::BELOW_MINIMUM;
    if (grams > nominalCapacityGrams_) return LoadBand::OVER_CAPACITY;
    if (grams > recommendedMaxGrams_) return LoadBand::ABOVE_RECOMMENDED;
    return LoadBand::NOMINAL;
  }

  /** @return Current operation mode. */
  OperationMode getMode() const { return mode_; }

 private:
  /**
   * @brief Checks the coupling demand condition.
   * @param s Snapshot.
   * @return true when load < 20 % and a technician is within 15 cm.
   */
  bool isCouplingDemanded(const NodeSnapshot& s) const {
    return s.loadValid && s.loadGrams < lowLoadGrams_ && s.proximityValid && s.technicianNearby;
  }

  /**
   * @brief Computes the next mode.
   * @param s Snapshot.
   * @return Next operation mode.
   */
  OperationMode resolveMode(const NodeSnapshot& s) const {
    if (!s.reservoirValid || !s.loadValid) return OperationMode::SENSOR_FAULT;
    if (s.reservoirEmpty) return OperationMode::EMPTY_LOCKOUT;
    if (mode_ == OperationMode::COUPLING) {
      return s.loadGrams >= targetLoadGrams_ ? OperationMode::STANDBY : OperationMode::COUPLING;
    }
    return isCouplingDemanded(s) ? OperationMode::COUPLING : OperationMode::STANDBY;
  }

  float lowLoadGrams_;
  float targetLoadGrams_;
  float recommendedMaxGrams_;
  float nominalCapacityGrams_;
  OperationMode mode_;
  bool demandBlocked_;
};

// =============================================================================
// SECTION: Services (operation tracking for the Device Maker)
// =============================================================================

/**
 * @class OperationTracker
 * @brief Prints the startup banner and JSON tracking records (cycles, reservoir state,
 *        proximity events, mode transitions, sensor faults) for the Device Maker.
 */
class OperationTracker {
 public:
  /** @brief Creates the tracker with zeroed metrics. */
  OperationTracker()
      : activationCycles_(0), blockedActivations_(0), proximityEvents_(0),
        modeTransitions_(0), sensorFaults_(0), reservoirEmptyEvents_(0) {}

  /** @brief Prints company, device and developer information. */
  void printStartupBanner() const {
    Serial.println(F("=================================================================="));
    Serial.printf("Company     : %s\n", DeviceConfig::COMPANY_NAME);
    Serial.printf("Description : %s\n", DeviceConfig::COMPANY_DESCRIPTION);
    Serial.printf("Global site : %s\n", DeviceConfig::COMPANY_GLOBAL_URL);
    Serial.printf("Regional    : %s\n", DeviceConfig::COMPANY_REGIONAL_URL);
    Serial.printf("Support     : %s\n", DeviceConfig::COMPANY_SUPPORT_URL);
    Serial.printf("Developers  : %s\n", DeviceConfig::COMPANY_DEVELOPER_URL);
    Serial.println(F("------------------------------------------------------------------"));
    Serial.printf("Device      : %s (firmware %s)\n", DeviceConfig::DEVICE_NAME, DeviceConfig::FIRMWARE_VERSION);
    Serial.printf("Purpose     : %s\n", DeviceConfig::DEVICE_DESCRIPTION);
    Serial.printf("Developer   : %s - %s\n", DeviceConfig::DEVELOPER_NAME, DeviceConfig::DEVELOPER_ROLE);
    Serial.println(F("------------------------------------------------------------------"));
    Serial.printf("Rules       : coupling when load < %.0f g and proximity <= %.0f cm, until %.0f g\n",
                  DeviceConfig::LOW_LOAD_THRESHOLD_G, DeviceConfig::PROXIMITY_THRESHOLD_CM,
                  DeviceConfig::TARGET_LOAD_G);
    Serial.printf("              reservoir Empty locks the high-current actuator (%.0f mL max)\n",
                  DeviceConfig::RESERVOIR_CAPACITY_ML);
    Serial.println(F("Commands    : STATUS | METRICS | HELP"));
    Serial.println(F("=================================================================="));
  }

  /**
   * @brief Records a mode transition (and an activation cycle when entering COUPLING).
   * @param from Previous mode.
   * @param to New mode.
   * @param s Snapshot at transition time.
   */
  void recordModeTransition(OperationMode from, OperationMode to, const NodeSnapshot& s) {
    ++modeTransitions_;
    if (to == OperationMode::COUPLING) {
      ++activationCycles_;
    }
    JsonDocument doc;
    doc[TelemetryKey::TYPE] = TelemetryKey::TYPE_MODE_TRANSITION;
    doc[TelemetryKey::FROM] = toString(from);
    doc[TelemetryKey::TO] = toString(to);
    doc[TelemetryKey::LOAD_GRAMS] = s.loadGrams;
    doc[TelemetryKey::LEVEL_PERCENT] = s.reservoirLevelPercent;
    doc[TelemetryKey::ACTIVATION_CYCLES] = activationCycles_;
    emit(doc);
  }

  /**
   * @brief Records an activation rejected by the Empty lockout.
   * @param s Snapshot at rejection time.
   */
  void recordBlockedActivation(const NodeSnapshot& s) {
    ++blockedActivations_;
    JsonDocument doc;
    doc[TelemetryKey::TYPE] = TelemetryKey::TYPE_ACTIVATION_BLOCKED;
    doc[TelemetryKey::DETAIL] = TelemetryKey::DETAIL_EMPTY_LOCK;
    doc[TelemetryKey::LOAD_GRAMS] = s.loadGrams;
    doc[TelemetryKey::BLOCKED_ACTIVATIONS] = blockedActivations_;
    emit(doc);
  }

  /**
   * @brief Records a proximity transition.
   * @param detected true when presence is detected.
   * @param distanceCm Measured distance.
   */
  void recordProximityEvent(bool detected, float distanceCm) {
    if (detected) {
      ++proximityEvents_;
    }
    JsonDocument doc;
    doc[TelemetryKey::TYPE] = TelemetryKey::TYPE_PROXIMITY;
    doc[TelemetryKey::STATE] = detected ? TelemetryKey::VALUE_DETECTED : TelemetryKey::VALUE_CLEARED;
    doc[TelemetryKey::DISTANCE_CM] = distanceCm;
    doc[TelemetryKey::PROXIMITY_EVENTS] = proximityEvents_;
    emit(doc);
  }

  /**
   * @brief Records a reservoir state transition.
   * @param empty true when the reservoir became Empty.
   * @param levelPercent Level at transition time.
   */
  void recordReservoirState(bool empty, float levelPercent) {
    if (empty) {
      ++reservoirEmptyEvents_;
    }
    JsonDocument doc;
    doc[TelemetryKey::TYPE] = TelemetryKey::TYPE_RESERVOIR;
    doc[TelemetryKey::STATE] = empty ? TelemetryKey::VALUE_EMPTY : TelemetryKey::VALUE_REFILLED;
    doc[TelemetryKey::LEVEL_PERCENT] = levelPercent;
    if (empty) {
      doc[TelemetryKey::DETAIL] = TelemetryKey::DETAIL_EMPTY_LOCK;
    }
    emit(doc);
  }

  /**
   * @brief Records a sensor fault or recovery.
   * @param source Sensor name.
   * @param recovered true on recovery, false on fault.
   */
  void recordSensorHealth(const char* source, bool recovered) {
    if (!recovered) {
      ++sensorFaults_;
    }
    JsonDocument doc;
    doc[TelemetryKey::TYPE] = TelemetryKey::TYPE_SENSOR_HEALTH;
    doc[TelemetryKey::SOURCE] = source;
    doc[TelemetryKey::STATE] = recovered ? TelemetryKey::VALUE_RECOVERED : TelemetryKey::VALUE_FAULT;
    emit(doc);
  }

  /**
   * @brief Records a load band transition (warnings above 80 % / over 100 %).
   * @param band New band.
   * @param grams Load in grams.
   */
  void recordLoadBand(LoadBand band, float grams) const {
    JsonDocument doc;
    doc[TelemetryKey::TYPE] = TelemetryKey::TYPE_LOAD_BAND;
    doc[TelemetryKey::LOAD_BAND] = toString(band);
    doc[TelemetryKey::LOAD_GRAMS] = grams;
    emit(doc);
  }

  /**
   * @brief Publishes a full telemetry record.
   * @param s Snapshot.
   * @param mode Current mode.
   * @param band Current load band.
   * @param loadPathOpen Servo state.
   */
  void publishTelemetry(const NodeSnapshot& s, OperationMode mode, LoadBand band, bool loadPathOpen) const {
    JsonDocument doc;
    doc[TelemetryKey::TYPE] = TelemetryKey::TYPE_TELEMETRY;
    doc[TelemetryKey::MODE] = toString(mode);
    doc[TelemetryKey::RESERVOIR_STATE] = !s.reservoirValid ? TelemetryKey::VALUE_UNKNOWN
                                         : (s.reservoirEmpty ? TelemetryKey::VALUE_EMPTY : TelemetryKey::VALUE_AVAILABLE);
    doc[TelemetryKey::LEVEL_PERCENT] = s.reservoirLevelPercent;
    doc[TelemetryKey::VOLUME_ML] = s.reservoirVolumeMl;
    doc[TelemetryKey::TECHNICIAN_NEARBY] = s.technicianNearby;
    doc[TelemetryKey::DISTANCE_CM] = s.proximityDistanceCm;
    doc[TelemetryKey::LOAD_GRAMS] = s.loadGrams;
    doc[TelemetryKey::LOAD_BAND] = toString(band);
    doc[TelemetryKey::LOAD_PATH] = loadPathOpen ? TelemetryKey::VALUE_OPEN : TelemetryKey::VALUE_CLOSED;
    doc[TelemetryKey::ACTIVATION_CYCLES] = activationCycles_;
    emit(doc);
  }

  /** @brief Prints accumulated operation metrics. */
  void printMetrics() const {
    JsonDocument doc;
    doc[TelemetryKey::TYPE] = TelemetryKey::TYPE_METRICS;
    doc[TelemetryKey::ACTIVATION_CYCLES] = activationCycles_;
    doc[TelemetryKey::BLOCKED_ACTIVATIONS] = blockedActivations_;
    doc[TelemetryKey::PROXIMITY_EVENTS] = proximityEvents_;
    doc[TelemetryKey::MODE_TRANSITIONS] = modeTransitions_;
    doc[TelemetryKey::SENSOR_FAULTS] = sensorFaults_;
    doc[TelemetryKey::EMPTY_EVENTS] = reservoirEmptyEvents_;
    emit(doc);
  }

  /** @brief Reports an unknown remote command. */
  void recordCommandError() const {
    JsonDocument doc;
    doc[TelemetryKey::TYPE] = TelemetryKey::TYPE_COMMAND_ERROR;
    doc[TelemetryKey::DETAIL] = TelemetryKey::DETAIL_UNKNOWN_COMMAND;
    emit(doc);
  }

 private:
  /**
   * @brief Adds the uptime and prints the record as a JSON line.
   * @param doc Record to print.
   */
  void emit(JsonDocument& doc) const {
    doc[TelemetryKey::UPTIME_MS] = millis();
    serializeJson(doc, Serial);
    Serial.println();
  }

  uint32_t activationCycles_;
  uint32_t blockedActivations_;
  uint32_t proximityEvents_;
  uint32_t modeTransitions_;
  uint32_t sensorFaults_;
  uint32_t reservoirEmptyEvents_;
};

// =============================================================================
// SECTION: Concrete device
// =============================================================================

/**
 * @class MicroGridNodeDevice
 * @brief EcoFlow MicroGrid Node M1: wires sensors, actuators, controller and tracker.
 */
class MicroGridNodeDevice : public Device {
 public:
  /** @brief Builds all components and registers the device as their event handler. */
  MicroGridNodeDevice()
      : reservoirSensor_(this), proximitySensor_(this), loadCell_(this), remoteListener_(this),
        controller_(DeviceConfig::LOW_LOAD_THRESHOLD_G, DeviceConfig::TARGET_LOAD_G,
                    DeviceConfig::RECOMMENDED_MAX_LOAD_G, DeviceConfig::LOAD_NOMINAL_CAPACITY_G),
        lastLoadBand_(LoadBand::UNKNOWN), lastTelemetryMs_(0),
        snapshot_(NodeSnapshot{false, DeviceConfig::NO_VALUE, DeviceConfig::NO_VALUE, false, false, false,
                               DeviceConfig::NO_VALUE, false, DeviceConfig::NO_VALUE}) {}

  /**
   * @brief Reacts to sensor events.
   * @param event Event raised by a sensor.
   */
  void on(const Event& event) override {
    switch (event.id) {
      case EventId::RESERVOIR_EMPTY:          tracker_.recordReservoirState(true, event.value); break;
      case EventId::RESERVOIR_REFILLED:       tracker_.recordReservoirState(false, event.value); break;
      case EventId::PROXIMITY_DETECTED:       tracker_.recordProximityEvent(true, event.value); break;
      case EventId::PROXIMITY_CLEARED:        tracker_.recordProximityEvent(false, event.value); break;
      case EventId::SENSOR_FAULT_DETECTED:    tracker_.recordSensorHealth(event.source, false); break;
      case EventId::SENSOR_RECOVERED:         tracker_.recordSensorHealth(event.source, true); break;
      case EventId::REMOTE_STATUS_REQUESTED:  publishTelemetry(); break;
      case EventId::REMOTE_METRICS_REQUESTED: tracker_.printMetrics(); break;
      case EventId::REMOTE_HELP_REQUESTED:    printHelp(); break;
      case EventId::REMOTE_COMMAND_UNKNOWN:   tracker_.recordCommandError(); break;
      default: break;
    }
  }

  /**
   * @brief Dispatches commands to the matching actuator.
   * @param command Command to dispatch.
   */
  void handle(const Command& command) override {
    switch (command.id) {
      case CommandId::OPEN_LOAD_PATH:
      case CommandId::CLOSE_LOAD_PATH:
        bypassServo_.handle(command);
        break;
      case CommandId::REFRESH_DISPLAY:
        display_.handle(command);
        break;
      default:
        break;
    }
  }

 protected:
  /** @brief Starts Serial, prints the banner and initializes components. */
  void initialize() override {
    Serial.begin(DeviceConfig::SERIAL_BAUD_RATE);
    tracker_.printStartupBanner();
    reservoirSensor_.begin();
    proximitySensor_.begin();
    loadCell_.begin();
    remoteListener_.begin();
    bypassServo_.begin();
    display_.begin();
  }

  /** @brief One control cycle: read, evaluate, actuate, display and track. */
  void runCycle() override {
    updateSensors();
    snapshot_ = captureSnapshot();
    applyDecision(controller_.evaluate(snapshot_));
    trackLoadBand();
    refreshDisplay();
    publishPeriodicTelemetry();
  }

  /** @return Control cycle period in milliseconds. */
  uint32_t getCyclePeriodMs() const override { return DeviceConfig::CONTROL_CYCLE_MS; }

 private:
  /** @brief Updates every sensor (events are raised through on()). */
  void updateSensors() {
    remoteListener_.update();
    reservoirSensor_.update();
    proximitySensor_.update();
    loadCell_.update();
  }

  /** @return Snapshot of the current sensor values. */
  NodeSnapshot captureSnapshot() const {
    NodeSnapshot s;
    s.reservoirValid = reservoirSensor_.isHealthy();
    s.reservoirLevelPercent = reservoirSensor_.getLevelPercent();
    s.reservoirVolumeMl = reservoirSensor_.getVolumeMilliliters();
    s.reservoirEmpty = reservoirSensor_.isEmpty();
    s.proximityValid = proximitySensor_.isHealthy();
    s.technicianNearby = proximitySensor_.isTechnicianNearby();
    s.proximityDistanceCm = proximitySensor_.getDistanceCm();
    s.loadValid = loadCell_.isHealthy();
    s.loadGrams = loadCell_.getWeightGrams();
    return s;
  }

  /**
   * @brief Applies the controller decision to the actuators and tracker.
   * @param decision Control decision.
   */
  void applyDecision(const ControlDecision& decision) {
    if (decision.activationBlocked) {
      tracker_.recordBlockedActivation(snapshot_);
    }
    if (decision.modeChanged) {
      tracker_.recordModeTransition(decision.previousMode, decision.mode, snapshot_);
    }
    handle(Command{decision.loadPathOpen ? CommandId::OPEN_LOAD_PATH : CommandId::CLOSE_LOAD_PATH});
  }

  /** @brief Tracks load band transitions (low, nominal, above recommended, over capacity). */
  void trackLoadBand() {
    const LoadBand band = controller_.classifyLoad(snapshot_.loadValid, snapshot_.loadGrams);
    if (band != lastLoadBand_) {
      lastLoadBand_ = band;
      tracker_.recordLoadBand(band, snapshot_.loadGrams);
    }
  }

  /** @brief Sends the latest values to the LCD. */
  void refreshDisplay() {
    display_.setFrame(DisplayFrame{snapshot_.reservoirValid, snapshot_.reservoirEmpty,
                                   snapshot_.reservoirLevelPercent, snapshot_.loadValid, snapshot_.loadGrams});
    handle(Command{CommandId::REFRESH_DISPLAY});
  }

  /** @brief Publishes telemetry every TELEMETRY_PERIOD_MS. */
  void publishPeriodicTelemetry() {
    const unsigned long now = millis();
    if (now - lastTelemetryMs_ >= DeviceConfig::TELEMETRY_PERIOD_MS) {
      lastTelemetryMs_ = now;
      publishTelemetry();
    }
  }

  /** @brief Publishes the current telemetry record. */
  void publishTelemetry() const {
    tracker_.publishTelemetry(snapshot_, controller_.getMode(), lastLoadBand_, bypassServo_.isOpen());
  }

  /** @brief Prints the remote command help. */
  void printHelp() const {
    Serial.println(F("Remote commands:"));
    Serial.printf("  %-8s -> current telemetry (mode, level, load, valve)\n", DeviceConfig::COMMAND_STATUS);
    Serial.printf("  %-8s -> accumulated operation metrics for the Device Maker\n", DeviceConfig::COMMAND_METRICS);
    Serial.printf("  %-8s -> this help\n", DeviceConfig::COMMAND_HELP);
  }

  ReservoirLevelSensor reservoirSensor_;
  EnclosureProximitySensor proximitySensor_;
  LoadCellAmplifier loadCell_;
  RemoteCommandListener remoteListener_;
  ServoMotor bypassServo_;
  StatusDisplay display_;
  MicroGridController controller_;
  OperationTracker tracker_;
  LoadBand lastLoadBand_;
  unsigned long lastTelemetryMs_;
  NodeSnapshot snapshot_;
};

// =============================================================================
// SECTION: Application entry point
// =============================================================================

/** @brief Device instance (EcoFlow MicroGrid Node M1). */
MicroGridNodeDevice microGridNode;

/** @brief Arduino setup: starts the device. */
void setup() {
  microGridNode.start();
}

/** @brief Arduino loop: inhibited, all behavior runs in the device task. */
void loop() {
  Device::inhibitMainLoop();
}
