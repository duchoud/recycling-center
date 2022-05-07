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
 * @brief update the current_colour variable, indicating which colour the robot is searching
 * @param is_looking_for_base: to indicate wether the object search its base (in black) or an object
 */
void set_searched_colour(bool is_looking_for_base);

void set_led_rgb(void);

#endif /* PROCESS_IMAGE_H */
