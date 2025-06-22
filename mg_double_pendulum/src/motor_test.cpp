#include <Arduino.h>
#include <SimpleFOC.h>
#include <WiFi.h>
#include <joybridge_receiver.h>

#include <atomic>

#include "encoders/calibrated/CalibratedSensor.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include "pins_arduino.h"  // Include our custom pins for AXIS board

#define MAX_SPEED 10  // Maximum speed in rad/s

// motor parameters
int pole_pairs = 11;
float phase_resistance = 9.5 / 2.0;
float kv = 100;

// Setup the motor and driver
BLDCMotor motor = BLDCMotor(pole_pairs);
BLDCDriver6PWM driver =
    BLDCDriver6PWM(CH0_UH, CH0_UL, CH0_VH, CH0_VL, CH0_WH, CH0_WL);

// make encoder for simplefoc
SPIClass hspi = SPIClass(HSPI);
MagneticSensorMT6701SSI encoder0(CH0_ENC_CS);

// calibrated sensor object from simplefoc
CalibratedSensor sensor_calibrated = CalibratedSensor(encoder0);

// atomic variable for thread-safe velocity control
std::atomic<float> target = 0;

// FOC loop thread
TaskHandle_t loop_foc_task;
void loop_foc_thread(void *pvParameters)
{
  while (1)
  {
    motor.move(target.load());
    motor.loopFOC();
  }
}

// Activate joybridge controller
JoyBridge::JoyBridgeReceiver receiver;

void setup()
{
  Serial.begin(115200);
  delay(5000);

  pinMode(LED_BUILTIN, OUTPUT);  // BLUE LED 44
  pinMode(43, OUTPUT);           // GREEN LED 43
  digitalWrite(43, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  hspi.begin(ENC_SCL, ENC_SDA, ENC_MOSI);
  delay(1000);

  encoder0.init(&hspi);
  motor.linkSensor(&encoder0);

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12;
  driver.init();

  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 5.;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;

  motor.PID_velocity.P = 0.9;
  motor.PID_velocity.I = 0.05;
  motor.PID_velocity.D = 0;
  motor.PID_velocity.output_ramp = 100;
  motor.PID_velocity.limit = 50;
  motor.LPF_velocity.Tf = 0.025;
  motor.controller = MotionControlType::velocity;

  motor.init();

  // Sensor calibration stuff
  // Calibrate encoders/motors if flag is set
  sensor_calibrated.voltage_calibration = 4;
  sensor_calibrated.calibrate(motor, "slider");
  //   sensor_calibrated.loadCalibrationData(motor, "slider");

  // Link the calibrated sensor to the motor
  motor.linkSensor(&sensor_calibrated);

  motor.initFOC();

  motor.enable();

  xTaskCreatePinnedToCore(loop_foc_thread, "loop_foc", 10000, NULL, 0,
                          &loop_foc_task, 1);

  if (!receiver.begin("slider"))
  {
    Serial.println("Receiver initialization failed!");
    while (true) delay(1000);
  }
}

// Global speed variable (setpoint)
float speed = 0;

void loop()
{
  // Joystick example code
  receiver.loop();

  if (receiver.isConnected())
  {
    JoyBridge::JoystickData data = receiver.getJoystickData();
    target.store(data.left_x * MAX_SPEED);
  }
  else
  {
    target.store(0);
    Serial.println("Joystick not connected.");
  }

  delay(50);
}
