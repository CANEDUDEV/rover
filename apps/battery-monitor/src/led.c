#include "led.h"

#include "ports.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "timers.h"

typedef struct {
  GPIO_TypeDef *red_led_port;
  uint16_t red_led_pin;
  GPIO_TypeDef *green_led_port;
  uint16_t green_led_pin;
} led_cfg_t;

static const led_cfg_t led6_cfg = {
    .red_led_port = LED1_GPIO_PORT,
    .red_led_pin = LED1_PIN,
    .green_led_port = LED2_GPIO_PORT,
    .green_led_pin = LED2_PIN,
};

static const led_cfg_t led7_cfg = {
    .red_led_port = LED3_GPIO_PORT,
    .green_led_port = LED4_GPIO_PORT,
    .red_led_pin = LED3_PIN,
    .green_led_pin = LED4_PIN,
};

static StaticTimer_t led_timer_buf;
static TimerHandle_t led_timer;

void led_timer_callback(TimerHandle_t timer);
void blink_leds_red(void);

void led_init(void) {
  const uint16_t led_blink_period_ms = 100;
  led_timer =
      xTimerCreateStatic("LED timer", pdMS_TO_TICKS(led_blink_period_ms),
                         pdTRUE,  // Auto reload timer
                         NULL,    // Timer ID, unused
                         led_timer_callback, &led_timer_buf);
}

void led_signal_fault(void) {
  if (xTimerIsTimerActive(led_timer)) {
    return;
  }

  blink_leds_red();
  xTimerStart(led_timer, portMAX_DELAY);
}

void led_stop_signal_fault(void) {
  if (!xTimerIsTimerActive(led_timer)) {
    return;
  }

  xTimerStop(led_timer, portMAX_DELAY);
  set_led_color(LED6, NONE);
  set_led_color(LED7, NONE);
}

void led_timer_callback(TimerHandle_t timer) {
  (void)timer;
  blink_leds_red();
}

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
  const led_cfg_t *led_cfg = &led6_cfg;

  if (led == LED7) {
    led_cfg = &led7_cfg;
  }

  switch (color) {
    case NONE:
      HAL_GPIO_WritePin(led_cfg->red_led_port, led_cfg->red_led_pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_cfg->green_led_port, led_cfg->green_led_pin,
                        GPIO_PIN_RESET);
      break;

    case RED:
      HAL_GPIO_WritePin(led_cfg->red_led_port, led_cfg->red_led_pin,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(led_cfg->green_led_port, led_cfg->green_led_pin,
                        GPIO_PIN_RESET);
      break;

    case GREEN:
      HAL_GPIO_WritePin(led_cfg->red_led_port, led_cfg->red_led_pin,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(led_cfg->green_led_port, led_cfg->green_led_pin,
                        GPIO_PIN_SET);
      break;

    case ORANGE:
      HAL_GPIO_WritePin(led_cfg->red_led_port, led_cfg->red_led_pin,
                        GPIO_PIN_SET);
      HAL_GPIO_WritePin(led_cfg->green_led_port, led_cfg->green_led_pin,
                        GPIO_PIN_SET);
      break;
  }
}
