#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  NONE = 0,
  RED,
  GREEN,
  ORANGE,
} led_color_t;

// Naming based on power board schematic
typedef enum {
  LED6 = 0,
  LED7,
} led_t;

void led_init(void);
void led_signal_fault(void);
void led_stop_signal_fault(void);
void set_led_color(led_t led, led_color_t color);

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
