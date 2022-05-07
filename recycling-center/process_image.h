#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

/**
 * @brief return center position of an object
 * @return last calculated center position of an object
 */
uint16_t get_line_position(void);

/**
 * @brief init the camera thread
 */
void process_image_start(void);

/**
 * @param value: new value to set is_looking_for_base
 */
void set_looking_for_base(bool value);


void set_led_rgb(void);

#endif /* PROCESS_IMAGE_H */
