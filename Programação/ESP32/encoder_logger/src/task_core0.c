#include "task_core0.h"
#include "initializers.h"
#include "sycronization.h"
#include "global_variables.h"
#include "pwm.h"
#include "pid.h"


void core0fuctions(void *params){

    pwm_motors_init();

    //selecting pcnt units
    uint32_t pcnt_unit_0 = 0;
    uint32_t pcnt_unit_1 = 1;

    // Create rotary encoder instances
    rotary_encoder_config_t config_encoder_left = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_0, PCNT_CHA_LEFT, PCNT_CHB_LEFT);
    rotary_encoder_t *encoder_left = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_left, &encoder_left));

    rotary_encoder_config_t config_encoder_right = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit_1, PCNT_CHA_RIGHT, PCNT_CHB_RIGHT);
    rotary_encoder_t *encoder_right = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config_encoder_right, &encoder_right));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder_left->set_glitch_filter(encoder_left, 1));
    ESP_ERROR_CHECK(encoder_right->set_glitch_filter(encoder_right, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder_left->start(encoder_left));
    ESP_ERROR_CHECK(encoder_right->start(encoder_right));

    int count_get_real = 0;

    //float local_motor_angular_speed_left = 0; 
    //float local_motor_angular_speed_right = 0;

    xEventGroupSetBits(initialization_groupEvent, task0_init_done);
    //xEventGroupWaitBits(initialization_groupEvent, task1_init_done, true, true, portMAX_DELAY);

    while(true){

        if(count_get_real == ENCODER_COUNTER_WAIT_PID_OP){
            xSemaphoreTake(xSemaphore_getSpeed,portMAX_DELAY);
            //global_motor_angular_speed_left = ((float) encoder_left->get_counter_value(encoder_left))*(ENCODER_RESOLUTION/((float)count));
            //global_motor_angular_speed_right = ((float) encoder_right->get_counter_value(encoder_right))*(ENCODER_RESOLUTION/((float)count));
            global_motor_angular_speed_left = ((float) encoder_left->get_counter_value(encoder_left))*ENCODER_RESOLUTION;
            global_motor_angular_speed_right = ((float) encoder_right->get_counter_value(encoder_right))*ENCODER_RESOLUTION;
            //local_motor_angular_speed_left = global_motor_angular_speed_left;
            //local_motor_angular_speed_right = global_motor_angular_speed_right;

            encoder_left->reset_counter_value(encoder_left);
            encoder_right->reset_counter_value(encoder_right);
            xSemaphoreGive(xSemaphore_getSpeed);

            count_get_real = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(PID_DELAY));

        count_get_real++;
    }

}

