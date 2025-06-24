#include <Arduino.h>
#include <SimpleFOC.h>
#include <WiFi.h>
#include <joybridge_receiver.h>

#include <atomic>

#include "encoders/calibrated/CalibratedSensor.h"
#include "encoders/mt6701/MagneticSensorMT6701SSI.h"
#include "pins_arduino.h"  // Include our custom pins for AXIS board

#define MAX_SPEED 50  // Maximum speed in rad/s

std::atomic<bool> enabled(false);
std::atomic<bool> disable_flag(false);
std::atomic<bool> enable_flag(false);

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

// Input low pass filter for target speed
LowPassFilter target_filter(0.1f);  // 0.1s time constant

struct MinJerkTrajectory
{
  float delta_pos;         // Change in position
  float total_time;        // Total time for the trajectory
  float total_time_cubed;  // Total time cubed for normalization
  float start_pos;         // Starting position
  float end_pos;           // Ending position
};

MinJerkTrajectory create_min_jerk_trajectory(float start_pos, float end_pos,
                                             float total_time)
{
  MinJerkTrajectory traj;
  traj.start_pos = start_pos;
  traj.end_pos = end_pos;
  traj.total_time = total_time;
  traj.total_time_cubed = total_time * total_time * total_time;
  traj.delta_pos = end_pos - start_pos;

  return traj;
}

float get_target_velocity(MinJerkTrajectory &traj,
                          unsigned long elapsed_time_micro)
{
  float elapsed_time =
      elapsed_time_micro / 1.0e6;  // Convert microseconds to seconds

  // Calculate the position at the current time using a cubic polynomial
  float t = elapsed_time / traj.total_time;  // Normalized time [0, 1]
  if (t > 1.0) t = 1.0;                      // Clamp t to [0, 1]

  // Cubic polynomial for minimum jerk trajectory
  float target = traj.delta_pos / traj.total_time *
                 (30.0 * (t * t) - 60.0 * (t * t * t) + 30.0 * (t * t * t * t));

  // Return the target velocity
  return target;
}

// FOC loop thread
TaskHandle_t loop_foc_task;
void loop_foc_thread(void *pvParameters)
{
  while (1)
  {
    // Serial.println(motor.shaft_angle, 8);
    if (enabled.load() && disable_flag.load())
    {
      motor.disable();

      //   Disable motor and  reset flags
      disable_flag.store(false);
      enabled.store(false);
    }
    else if (!enabled.load() && enable_flag.load())
    {
      motor.enable();

      //   Enable motor and reset flags
      enable_flag.store(false);
      enabled.store(true);
    }

    motor.move(target_filter(target.load()));
    motor.loopFOC();
  }
}

// Activate joybridge controller
JoyBridge::JoyBridgeReceiver receiver;
MinJerkTrajectory trajectory;

unsigned long start;
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

  motor.PID_velocity.P = 0.7;
  motor.PID_velocity.I = 0.05;
  motor.PID_velocity.D = 0.003;
  motor.PID_velocity.output_ramp = 10000;
  motor.PID_velocity.limit = MAX_SPEED;
  motor.LPF_velocity.Tf = 0.03;
  motor.controller = MotionControlType::velocity;

  motor.init();

  // Sensor calibration stuff
  // Calibrate encoders/motors if flag is set
  sensor_calibrated.voltage_calibration = 4;
  //   sensor_calibrated.calibrate(motor, "slider");
  sensor_calibrated.loadCalibrationData(motor, "slider");

  // Link the calibrated sensor to the motor
  motor.linkSensor(&sensor_calibrated);

  motor.initFOC();

  enable_flag.store(true);

  xTaskCreatePinnedToCore(loop_foc_thread, "loop_foc", 10000, NULL, 1,
                          &loop_foc_task, 1);

  //   if (!receiver.begin("slider"))
  //   {
  //     Serial.println("Receiver initialization failed!");
  //     while (true) delay(1000);
  //   }
  delay(1000);

  trajectory = create_min_jerk_trajectory(motor.shaft_angle, 10, 4.0);

  start = micros();
}

// Global speed variable (setpoint)
float speed_command = 0;

void loop()
{
  target.store(get_target_velocity(trajectory, micros() - start));

  //   // Joystick example code
  //   receiver.loop();

  //   if (receiver.isConnected())
  //   {
  //     // if (!enabled.load())
  //     // {
  //     //   enable_flag.store(true);
  //     // }
  //     JoyBridge::JoystickData data = receiver.getJoystickData();
  //     // Cube data left_x
  //     target.store(data.left_x * data.left_x * data.left_x * MAX_SPEED);
  //   }
  //   else
  //   {
  //     Serial.println("Joystick not connected.");
  //     // disable_flag.store(true);
  //   }

  //   delay(50);
}
