#include "ch.h"
#include "hal.h"
#include <math.h>

#include "leds.h"
#include <camera/po8030.h>
#include <selector.h>

#include <main.h>
#include <process_image.h>

//what colour does the camera look for
enum COLOUR_LOOKED{
	RED,
	GREEN,
	BLACK
};

//static bool is_looking_for_base = 0;					//search base or search object
static uint16_t line_position = NOTFOUND;				//if the camera doesn't find an object
static enum COLOUR_LOOKED current_colour = BLACK;


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns NOTFOUND if line not found
 */
void extract_object_position(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;							//i:camera pixel number in x
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;		//stop: stops if it finds an object

	do{
		wrong_line = 0;

		//find the beginning of the left part of the object
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if((buffer[i] - buffer[i+WIDTH_SLOPE]) > HTHRESHOLD)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}

		//if a begin was found, find the end of the right part of the object
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if((buffer[i-WIDTH_SLOPE] - buffer[i]) < -HTHRESHOLD)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if no end was found
		    if (i > IMAGE_BUFFER_SIZE || !end )
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		if (!line_not_found) {
			//if a line too small has been detected, continues the search
			uint32_t meancolour = 0;			//the mean colour number of the object
			for(int i = begin; i < end; i++){
				meancolour += buffer[i];
			}
			meancolour /= (end-begin);

			//chprintf((BaseSequentialStream *)&SDU1, "mean=%d  ", meancolour);
			//compare the mean colour numbers of the object we see and the object we search
			bool wrong_colour = false;
			if (current_colour == RED && meancolour > RED_THRESHOLD) {
				wrong_colour = true;
			} else if (current_colour == GREEN && meancolour > GREEN_THRESHOLD){
				wrong_colour = true;
			} else if (current_colour == BLACK && meancolour > BLACK_THRESHOLD) {
				wrong_colour = true;
			}

			//if the object we see is too thin or not the right colour
			if(wrong_colour || (!line_not_found && (end-begin) < MIN_LINE_WIDTH)){
				i = end;
				begin = 0;
				end = 0;
				stop = 0;
				wrong_line = 1;
			}
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		line_position = NOTFOUND;
	}else{
		line_position = (begin + end)/2; //gives the line position of the center of the object.
	}
}

void set_leds(void){
	clear_leds();
	switch (current_colour){
		case RED:
			set_led(LED7, 1);
			break;

		case GREEN:
			set_led(LED3, 1);
			break;

		case BLACK:
			set_led(LED5, 1);
			break;
	}
}


static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 240, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		for(int i = 0; i < IMAGE_BUFFER_SIZE; i++) {
			//uses mask to extract the bits for red colour
			uint8_t red = (img_buff_ptr[2 * i] & 0b11111000);
			//uses mask to extract the bits for green colour
			uint8_t green =  ((((img_buff_ptr[2 * i] & 0b111)<<3) | ((img_buff_ptr[2 * i + 1] & 0b11100000)>>5))/2) << 3;
			//uses both red and green chanel for black colour
			uint8_t black = (red+green)/2;
			uint8_t dist = abs(i-IMAGE_BUFFER_SIZE/2);

			switch (current_colour) {
			case BLACK:
				black = black + dist * dist * COEFF_MOD_CAM * dist;  // formula to correct the error of the camera in dist^3
				image[i] = black;
				break;
			case GREEN:
				red = red + dist * dist * COEFF_MOD_CAM * dist ; // formula to correct the error of the camera in dist^3
				image[i] = red;
				break;
			case RED:
				green = green + dist * dist * COEFF_MOD_CAM * dist; // formula to correct the error of the camera in dist^3
				image[i] = green;
				break;
			}
		}
		//search for an object in the image and gets its position in pixels
		extract_object_position(image);
    }
}

uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), LOWPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), LOWPRIO, CaptureImage, NULL);
}

void set_searched_colour(bool is_looking_for_base) {
	// each time we calls this functions is when the state as changed, thus we update the leds value
	line_position = NOTFOUND;

	if (is_looking_for_base) {
		// configure the datas from the camera to see a black object
		current_colour = BLACK;
	} else {
		if((get_selector() >= 0 && get_selector() <= 4) || (get_selector() >= 13 && get_selector() <= 15)) {
			// configure the datas from the camera to see a green object
			current_colour = GREEN;
		} else {
			// configure the datas from the camera to see a red object
			current_colour = RED;
		}
	}

	set_leds();
}
