#include "robotiq_gripper.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "mbcontroller.h"

#define SERVO_GPIO      18
#define SERVO_MIN_US    500
#define SERVO_MAX_US    2500

#define MB_PORT_NUM     2   // UART2
#define MB_SLAVE_ADDR   9
#define MB_BAUDRATE     115200

#define R_ACT  (1 << 0)
#define R_GTO  (1 << 3)
#define G_ACT  (1 << 0)
#define G_GTO  (1 << 3)

void CustomGripper::setup() {
  ESP_LOGI("ROB_GRIPPER", "Initializing gripper");

  // Servo init
  servo_init();

  // Modbus slave init
  void *mbc_slave = nullptr;
  mb_communication_info_t comm;
  ESP_ERROR_CHECK(mbc_slave_init(MB_PORT_SERIAL_SLAVE, &mbc_slave));

  comm.mode = MB_MODE_RTU;
  comm.slave_addr = MB_SLAVE_ADDR;
  comm.port = MB_PORT_NUM;
  comm.baudrate = MB_BAUDRATE;
  comm.parity = MB_PARITY_NONE;
  ESP_ERROR_CHECK(mbc_slave_setup(&comm));

  // Set up register areas (same as your code)
  mb_register_area_descriptor_t reg_area;

  reg_area.type = MB_PARAM_HOLDING;
  reg_area.start_offset = 1000;
  reg_area.address = &holding_regs;
  reg_area.size = sizeof(holding_regs);
  ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

  reg_area.type = MB_PARAM_INPUT;
  reg_area.start_offset = 2000;
  reg_area.address = &input_regs;
  reg_area.size = sizeof(input_regs);
  ESP_ERROR_CHECK(mbc_slave_set_descriptor(reg_area));

  ESP_ERROR_CHECK(mbc_slave_start());
  uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
}

void RobotiqGripper::loop() {
  // Read master commands
  uint8_t control = (holding_regs.r_control >> 8) & 0xFF;

  if (control & R_ACT) {
    input_regs.g_status |= (G_ACT << 8);
  }

  if (control & R_GTO) {
    uint8_t target = holding_regs.r_position & 0xFF;
    servo_move(target);
    input_regs.g_status |= (G_GTO << 8);
  }

  input_regs.g_pos_current = ((uint16_t)current_position << 8) | 10;
  input_regs.g_fault_echo = (0 << 8) | (holding_regs.r_position & 0xFF);

  delay(20);
}

void CustomGripper::servo_init() {
  ledc_timer_config_t timer = {};
  timer.speed_mode = LEDC_LOW_SPEED_MODE;
  timer.timer_num = LEDC_TIMER_0;
  timer.freq_hz = 50;
  timer.duty_resolution = LEDC_TIMER_16_BIT;
  ledc_timer_config(&timer);

  ledc_channel_config_t channel = {};
  channel.gpio_num = SERVO_GPIO;
  channel.speed_mode = LEDC_LOW_SPEED_MODE;
  channel.channel = LEDC_CHANNEL_0;
  channel.timer_sel = LEDC_TIMER_0;
  channel.duty = 0;
  ledc_channel_config(&channel);
}

void CustomGripper::servo_move(uint8_t pos) {
  uint32_t us = SERVO_MIN_US + (pos * (SERVO_MAX_US - SERVO_MIN_US)) / 255;
  uint32_t duty = (us * 65535) / 20000;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  current_position = pos;
}
