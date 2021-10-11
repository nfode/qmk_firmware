//#define RGB_MATRIX_ENABLE 1
//#define RGB_MATRIX_KEYPRESSES
//#define RGB_MATRIX_LED_PROCESS_LIMIT 0
//#define RGB_MATRIX_LED_FLUSH_LIMIT 0 // limits in milliseconds how frequently an animation will update the LEDs. 16 (16ms) is equivalent to limiting to 60fps (increases keyboard responsiveness)
//#define RGB_MATRIX_MAXIMUM_BRIGHTNESS 255 // limits maximum brightness of LEDs to 200 out of 255. If not defined maximum brightness is set to 255
#define RGB_MATRIX_STARTUP_MODE RGB_MATRIX_SOLID_COLOR
#define RGB_MATRIX_STARTUP_HUE 127 // Sets the default hue value, if none has been set
#define RGB_MATRIX_STARTUP_SAT 0 // Sets the default saturation value, if none has been set
#define RGB_MATRIX_STARTUP_VAL 255 // Sets the default brightness value, if none has been set
#define RGB_MATRIX_STARTUP_SPD 63
#define RGB_MATRIX_SPD_STEP 16

#define SLEEP_LED_MODE_ANIMATION RGB_MATRIX_BREATHING



#define TAPPING_TERM 200
#define ONESHOT_TAP_TOGGLE 5  /* Tapping this number of times holds the key until tapped once again. */
#define ONESHOT_TIMEOUT 5000  /* Time (in ms) before the one shot key is released */
