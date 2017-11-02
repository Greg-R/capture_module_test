#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "mcpwm_basic_config_example.h"

//  Motor control related code begins here.

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;


xQueueHandle cap_queue;

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};


//  @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP0 signal and according take action
static void IRAM_ATTR isr_handler()
{
    uint32_t mcpwm_intr_status;
    capture evt;
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status.  This gets all 32 bits.
    if (mcpwm_intr_status & CAP0_INT_EN) { //Check for interrupt on rising edge on CAP0 signal.  This is going to capture any of 32 interrupts!  Why not use bit selector?
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt, NULL);  //  This is how data is written into the queue!
    }
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;  // Clear interrupt(s).
}

/**
 * @brief When interrupt occurs, we receive the counter value and display the time between two rising edge
 */
static void disp_captured_signal(void *arg)
{
    uint32_t *current_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    uint32_t *previous_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    capture evt;
    while (1) {
        xQueueReceive(cap_queue, &evt, portMAX_DELAY);
//        if (evt.sel_cap_signal == MCPWM_SELECT_CAP0) {
            current_cap_value[0] = evt.capture_signal - previous_cap_value[0];
            previous_cap_value[0] = evt.capture_signal;
            current_cap_value[0] = current_cap_value[0] / 80;
    //        current_cap_value[0] = (current_cap_value[0] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            printf("CAP0 : %d us\n", current_cap_value[0]);
     //       vTaskDelay(1000);
  //      }
    }
}

//  This is the motor controller task.
static void mcpwm_example_bldc_control(void * arg)  //  For use in task, add parameter void * arg
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();
    //2. initial mcpwm configuration.  Initalize the Pulse Width Modulator.
    printf("Configuring Initial Parameters of mcpwm bldc control...\n");
    //  Use pwm_config in global area so RPC can access it.
//    mcpwm_config_t pwm_config;
//    pwm_config.frequency = 1000;    //frequency = 1000Hz
//    pwm_config.cmpr_a = 50.0;    //duty cycle of PWMxA = 50.0%
//    pwm_config.cmpr_b = 50.0;    //duty cycle of PWMxb = 50.0%
//    pwm_config.counter_mode = MCPWM_UP_COUNTER;
//    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    //3. Capture configuration
    //configure CAP0, CAP1 and CAP2 signal to start capture counter on rising edge
    //we generate a gpio_test_signal of 20ms on GPIO 12 and connect it to one of the capture signal, the disp_captured_function displays the time between rising edge
    //In general practice you can connect Capture  to external signal, measure time between rising edge or falling edge and take action accordingly
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = (CAP0_INT_EN);  //Enable interrupt on  CAP0.
    //Set ISR Handler
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    //  PID Controller implementation here.
 //   while (1) {


//        }
    vTaskDelete(NULL);
}


void app_main(void)
{
	//  xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
	//  mcpwm_example_gpio_initialize();
	//  mcpwm_example_bldc_control();
	//  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    cap_queue = xQueueCreate(10, sizeof(capture));
	  printf("Starting Tasks");
	  xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 2, NULL);
	  xTaskCreate(mcpwm_example_bldc_control, "mcpwm_example_bldc_control", 4096, NULL, 2, NULL);
}

