#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <selector.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

enum COLOUR_LOOKED{
	RED,
	GREEN,
	BLACK
};

static bool is_looking_for_base = 0;					//search base or search object
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static enum COLOUR_LOOKED current_colour = GREEN;


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 */
void extract_object_position(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;

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
		    //if an end was not found
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

			//use mean to clearly see the object on python
			for(int i = begin; i < end; i++){
				if(abs(meancolour - buffer[i]) < THRESHOLD_COLOUR){
					buffer[i] = meancolour;
				}
			}
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

			//if the object we see is too thin
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
	//chprintf((BaseSequentialStream *)&SDU1, "line position=%d \r\n", line_position);
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

	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();
		//init_selector();
		//Extracts only the red pixels
		for(int i = 0; i < IMAGE_BUFFER_SIZE; i++) {
			//uint16_t blue = (img_buff_ptr[2 * i + 1] & 0b11111);
			uint16_t red = (img_buff_ptr[2 * i] & 0b11111000);
			uint16_t green =  ((((img_buff_ptr[2 * i] & 0b111)<<3) | ((img_buff_ptr[2 * i + 1] & 0b11100000)>>5))/2) << 3;
			uint16_t black = (red+green)/2;

			if (is_looking_for_base) {
				current_colour = BLACK;
				uint16_t dist = abs(i-IMAGE_BUFFER_SIZE/2);
				black = black + dist * dist * COEFF_MOD_CAM * dist;
				image[i] = black;
			} else {
				if((get_selector() % 2) == 0) {
					current_colour = GREEN;
					uint16_t dist = abs(i-IMAGE_BUFFER_SIZE/2);
					red = red + dist * dist * COEFF_MOD_CAM * dist ;
					image[i] = red;
				}
				if((get_selector() % 2) == 1){
					current_colour = RED;
					uint16_t dist = abs(i-IMAGE_BUFFER_SIZE/2);
					green = green + dist * dist * COEFF_MOD_CAM * dist;
					image[i] = green;
				}
			}

		}

		//search for an object in the image and gets its position in pixels
		extract_object_position(image);

		if(send_to_computer){
			//sends to the computer the image
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		//invert the bool
		send_to_computer = !send_to_computer;
    }
}



uint16_t get_line_position(void){
	return line_position;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void set_looking_for_base(bool value) {
	is_looking_for_base = value;
}
