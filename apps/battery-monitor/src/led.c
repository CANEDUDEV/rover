#include "led.h"

#include "ports.h"
#include "stm32f3xx_hal.h"

void blink_leds_red(void) {
  static uint8_t blink = 0;
  if (blink > 0) {
    blink = 0;
    set_led_color(LED6, RED);
    set_led_color(LED7, RED);
  } else {
    blink = 1;
    set_led_color(LED6, NONE);
    set_led_color(LED7, NONE);
  }
}

void set_led_color(led_t led, led_color_t color) {
  GPIO_TypeDef *red_led_port = LED1_GPIO_PORT;
  GPIO_TypeDef *green_led_port = LED2_GPIO_PORT;
  uint16_t red_led = LED1_PIN;
  uint16_t green_led = LED2_PIN;

  if (led == LED7) {
    red_led_port = LED3_GPIO_PORT;
    green_led_port = LED4_GPIO_PORT;
    red_led = LED3_PIN;
    green_led = LED4_PIN;
  }

  switch (color) {
    case NONE:
      HAL_GPIO_WritePin(red_led_port, red_led, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(green_led_port, green_led, GPIO_PIN_RESET);
      break;
    case RED:
      HAL_GPIO_WritePin(red_led_port, red_led, GPIO_PIN_SET);
      HAL_GPIO_WritePin(green_led_port, green_led, GPIO_PIN_RESET);
      break;
    case GREEN:
      HAL_GPIO_WritePin(red_led_port, red_led, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(green_led_port, green_led, GPIO_PIN_SET);
      break;
    case ORANGE:
      HAL_GPIO_WritePin(red_led_port, red_led, GPIO_PIN_SET);
      HAL_GPIO_WritePin(green_led_port, green_led, GPIO_PIN_SET);
      break;
  }
}
