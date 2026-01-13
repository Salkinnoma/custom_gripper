#pragma once
#include "esphome.h"

/*
 * Robotiq-compatible Gripper for ESPHome
 *
 * Wiring (ESP32 -> RS-485 -> Gripper):
 * 
 * UART / Modbus:
 *   TX  -> GPIO17 -> RS-485 DI
 *   RX  <- GPIO16 <- RS-485 RO
 *   DE  -> GPIO4  -> RS-485 DE/RE (high = transmit, low = receive)
 *   GND -> GND    -> RS-485 GND
 *   5V  -> 5V     -> RS-485 VCC
 * 
 * Servo:
 *   PWM signal -> GPIO18
 *   GND        -> Servo GND
 *   5V         -> Servo VCC (check servo voltage)
 * 
 * Notes:
 * - This is a minimal hardcoded setup for testing and internal use.
 * - All Modbus holding and input registers are exposed on this ESPHome component.
 */

class RobotiqGripper : public Component {
 public:
  void setup() override;
  void loop() override;

 private:
  void servo_init();
  void servo_move(uint8_t pos);

  uint8_t current_position = 0;

  // You can include your holding/input register structs here
  struct {
    uint16_t r_control;
    uint16_t r_position;
    uint16_t r_speed_force;
  } holding_regs;

  struct {
    uint16_t g_status;
    uint16_t g_fault_echo;
    uint16_t g_pos_current;
  } input_regs;
};
