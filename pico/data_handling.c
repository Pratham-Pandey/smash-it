// Layout::
//
// Core 0:
//      * Receiving wheel velocity from ROS2(PC)  and publishing it to the wheels.     
//      * Sending psoition estimate(from core 1) to ROS2(PC).
//
// Core 1:
//      * Position Estimate 1: Collect encoder Data, estimate robot position using Kinematics. 
//      * Position Estimate 2: Collect IMU Data, integrate it ovevr time to estimate robot position.
//      * Position Estimate 3: Fuse Position Estimate 1 and 2 using Extended Kalman Filter to obtain the final estimate.
//      * Pass the adta to core 0 for sending it to ROS2(PC).



// To drive all 4 motors at the same time. The command will be received from pc via wifi using ESP8266.

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define ENSURE_LESS_THAN_255(x) (((x)<=255) ? (x) : 255 )
#define LED_PIN 25

#define TOTAL_WHEELS 4

typedef struct {
    int R_EN;
    int L_EN;
    int R_PWM;
    int L_PWM;
} Wheel;

Wheel wheels[TOTAL_WHEELS];

void init_wheels() {
    wheels[0] = (Wheel) { .R_PWM = 21, .L_PWM = 20, .R_EN = 19, .L_EN = 18};       // Front Left Wheel
    wheels[1] = (Wheel) { .R_PWM = 10, .L_PWM = 11, .R_EN = 12, .L_EN = 13};       // Front Right Wheel
    wheels[2] = (Wheel) { .R_PWM = 2, .L_PWM = 3, .R_EN = 4, .L_EN = 5};           // Rear Left Wheel
    wheels[3] = (Wheel) { .R_PWM = 6, .L_PWM = 7, .R_EN = 8, .L_EN = 9};           // Rear Right Wheel
}

void setup_GPIO() {

    // Initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize MOTOR pins
    for(int i=0; i<TOTAL_WHEELS; i++){    
        gpio_init(wheels[i].R_EN);    
        gpio_init(wheels[i].L_EN);    

        gpio_set_dir(wheels[i].R_EN, GPIO_OUT);
        gpio_set_dir(wheels[i].L_EN, GPIO_OUT);
    }

    // Initialize USB for standard input/output
    stdio_init_all();

    // Setup UART Communication(between ESP and PICO)
    uart_init(uart0, 115200);
    gpio_set_function(0, GPIO_FUNC_UART);  // RX pin
    gpio_set_function(1, GPIO_FUNC_UART);  // TX pin

    // Wait for USB connection to be established
    sleep_ms(2000);  // Optional delay to allow connection setup
}

void setup_PWM(){
    for(int i=0; i<TOTAL_WHEELS; i++){    
        // Set pin type
        gpio_set_function(wheels[i].R_PWM, GPIO_FUNC_PWM);
        gpio_set_function(wheels[i].L_PWM, GPIO_FUNC_PWM);

        // Retrive pin slice
        uint slice_R_PWM = pwm_gpio_to_slice_num(wheels[i].R_PWM);
        uint slice_L_PWM = pwm_gpio_to_slice_num(wheels[i].L_PWM);

        // Wrap frequency
        pwm_set_wrap(slice_R_PWM, 255);
        pwm_set_wrap(slice_L_PWM, 255);
        
        // Set PWM frequency (example: 1 kHz or 10 kHz)
        pwm_set_clkdiv(slice_R_PWM, 24.5f);  // Adjust based on required frequency
        pwm_set_clkdiv(slice_L_PWM, 24.5f); 

        // Enable Pin
        pwm_set_enabled(slice_R_PWM, true);
        pwm_set_enabled(slice_L_PWM, true);

    }
}


volatile int pulseCount = 0;

float wheel_radius = 0.0635f;
float sum_of_robot_center_projection_on_X_Y_axis = 0.508f;

float vx;
float vy;
float wz;


void forward_kinematics(float FLV, float RLV, float RRV, float FRV){

    // Converting RPM to angular velocity   RPS * Radians in one rotation = Radins per seccond = angular velociiity
    FLV = 6.283185307 * FLV/60;
    RLV = 6.283185307 * RLV/60;
    RRV = 6.283185307 * RRV/60;
    FRV = 6.283185307 * FRV/60;

    vx = 0.25 * wheel_radius * (FLV + RLV + RRV + FRV);
    vy = 0.25 * wheel_radius * (-FLV + RLV - RRV + FRV);
    wz = 0.25 * wheel_radius / sum_of_robot_center_projection_on_X_Y_axis * (-FLV - RLV + RRV + FRV);
}

void motorStep1() {
    pulseCount++;
}

void encoder_core1(){
    const uint pinE = 16;  // Change this to the pin you're using on the Pico
    int lastPulseCount = 0;

    absolute_time_t lastTime = get_absolute_time();  
    
    stdio_init_all();

    gpio_init(pinE);
    gpio_set_dir(pinE, GPIO_IN);   
    
    gpio_set_irq_enabled_with_callback(pinE, GPIO_IRQ_EDGE_RISE, true, motorStep1);

    while (true) {
        // Get the current time
        absolute_time_t currentTime = get_absolute_time();

        // Check if 1 second has passed
        if (absolute_time_diff_us(lastTime, currentTime) >= 1000000) {  // 1 second has passed
            // Calculate revolutions per second (RPS)
            int pulsesInLastSecond = pulseCount - lastPulseCount;
            float revolutionsPerSecond = pulsesInLastSecond / (float)20;

            // Convert RPS to RPM
            float rpm = revolutionsPerSecond * 60.0;

            // Print the calculated RPM
            //printf("RPM: %.2f\n", rpm);

            float rpm_f = (float)rpm;   // To simulate sendiing coordinates.

            // Calculating velocciiity
            //forward_kinematics(rpm_f, 600.0f, 600.0f, 600.0f);
            forward_kinematics(rpm_f, rpm_f, rpm_f, rpm_f);

            //printf("Linear X: %.2f\n", vx);
            //printf("Linear Y: %.2f\n", vy);
            //printf("Angular Z: %.2f\n", wz);

            //Send data to core 0 for further processsing.
            multicore_fifo_push_timeout_us(*(uint32_t*)&rpm_f, 0);

            // Update the lastPulseCount and lastTime for the next interval
            lastPulseCount = pulseCount;
            lastTime = currentTime;  // Update the time for the next 1-second interval
        }
    }

}



int main(){
    init_wheels();

    setup_GPIO();
    setup_PWM();

    uint32_t received_value;
    bool got;

    // Launch task on core 1(Sensor data collection, sensor fusion, state estimation )
    multicore_launch_core1(encoder_core1);

    while (true) {

        got = multicore_fifo_pop_timeout_us(0, &received_value);
        
        if (got){
            float received_float = *(float*)&received_value;
            //printf("Core 0 RPM: %f\n", received_float);
        }

        char read_buff[128];

        char temp_buff[20];

        int len = 0;

        while ((len<sizeof(read_buff)-1)){
            int c = getchar_timeout_us(0);

            if (c >= 0){            // To check whether valid data has been received from the UART
                if (c == '~'){
                    //printf("Data: %s\n", read_buff);
                    break;
                }            
                else{
                    read_buff[len++] = c;
                }
            }

        }
        
        // Null-terminate the string received
        // read_buff[len++] = '%';
        read_buff[len] = '\0';
        // printf("Received: %s\n", read_buff);

        strncpy(temp_buff, read_buff + strlen(read_buff) -20, 20);
        temp_buff[20] = '\0';

        printf("%s\n", temp_buff);


        // printf("Received: %s \n", read_buff);
        
        if (len > 0){
            //std::string received_data(read_buff);
            //double value = std::stod(received_data);

            char front_left_char[6];
            char front_right_char[6];                        
            char rear_left_char[6];
            char rear_right_char[6];            

            // NOTE: We use 'strncpy' for copying till a particular point and 'sstrcpy' to copy from any point till end. 'strcpy' automatically addts '\0' at end while 'strncpy' doesnot.            
            // Resource: 
            // * https://stackoverflow.com/questions/16807004/how-to-get-a-part-of-a-char-array-in-c
            // * https://cstdspace.quora.com/What-is-the-difference-between-strncpy-and-strcpy-for-strings-in-C-programming-language?top_ans=386912167
            

            strncpy(front_left_char, temp_buff, 5);
            strncpy(front_right_char, temp_buff + 5, 5);
            strncpy(rear_left_char, temp_buff + 10, 5);
            strncpy(rear_right_char, temp_buff + 15, 5);

            // strncpy(front_left_char, read_buff, 5);
            // strncpy(front_right_char, read_buff + 5, 5);
            // strncpy(rear_left_char, read_buff + 10, 5);
            // strncpy(rear_right_char, read_buff + 15, 5);
            
            front_left_char[5] = '\0';
            front_right_char[5] = '\0';
            rear_left_char[5] = '\0';
            rear_right_char[5] = '\0';
            
            //printf("Left: %s \n", rear_left_char);
            //printf("Right: %s \n", rear_right_char);                       
            
            double front_left = atof(front_left_char);
            double front_right = atof(front_right_char);
            double rear_left = atof(rear_left_char);
            double rear_right = atof(rear_right_char);

            //printf("Double Left: %d \n", (int)(fabs(rear_left/6.0) * 255));
            //printf("Double Right: %d \n", (int)(fabs(rear_right/6.0) * 255));          
            
            //// printf("Front LEFT: %f \n", front_left);
            //// printf("Front RIGHT: %f \n", front_right);
            //// printf("Rear LEFT: %f \n", rear_left);
            //// printf("Rear RIGHT: %f \n", rear_right);

            // Setting dutycycle;
            int duty_front_left = ENSURE_LESS_THAN_255((int)(fabs(front_left/6.0) * 255));
            int duty_front_right = ENSURE_LESS_THAN_255((int)(fabs(front_right/6.0) * 255));
            int duty_rear_left = ENSURE_LESS_THAN_255((int)(fabs(rear_left/6.0) * 255));
            int duty_rear_right = ENSURE_LESS_THAN_255((int)(fabs(rear_right/6.0) * 255));

            // Front Left
            if (front_left >= 0.5) {
                gpio_put(wheels[0].R_EN, 1);
                gpio_put(wheels[0].L_EN, 1);                 

                pwm_set_gpio_level(wheels[0].R_PWM, duty_front_left);            
                pwm_set_gpio_level(wheels[0].L_PWM, 0);            

                gpio_put(LED_PIN, 1);               

            }else if (front_left <= -0.5) {
                gpio_put(wheels[0].R_EN, 1);
                gpio_put(wheels[0].L_EN, 1);                 

                pwm_set_gpio_level(wheels[0].R_PWM, 0);            
                pwm_set_gpio_level(wheels[0].L_PWM, duty_front_left);            

                gpio_put(LED_PIN, 0);

            }else{
                gpio_put(wheels[0].R_EN, 1);
                gpio_put(wheels[0].L_EN, 1);                 

                pwm_set_gpio_level(wheels[0].R_PWM, 0);            
                pwm_set_gpio_level(wheels[0].L_PWM, 0);            

                gpio_put(LED_PIN, 0);
            }
            
            // Front Right
            if (front_right >= 0.5) {
                gpio_put(wheels[1].R_EN, 1);
                gpio_put(wheels[1].L_EN, 1);                 

                pwm_set_gpio_level(wheels[1].R_PWM, duty_front_right);            
                pwm_set_gpio_level(wheels[1].L_PWM, 0);            

            }else if (front_right <= -0.5) {
                gpio_put(wheels[1].R_EN, 1);
                gpio_put(wheels[1].L_EN, 1);                 

                pwm_set_gpio_level(wheels[1].R_PWM, 0);            
                pwm_set_gpio_level(wheels[1].L_PWM, duty_front_right);            

            }else{
                gpio_put(wheels[1].R_EN, 1);
                gpio_put(wheels[1].L_EN, 1);                 

                pwm_set_gpio_level(wheels[1].R_PWM, 0);            
                pwm_set_gpio_level(wheels[1].L_PWM, 0);            
            }


            // Rear Left
            if (rear_left >= 0.5) {
                gpio_put(wheels[2].R_EN, 1);
                gpio_put(wheels[2].L_EN, 1);                 

                pwm_set_gpio_level(wheels[2].R_PWM, duty_rear_left);            
                pwm_set_gpio_level(wheels[2].L_PWM, 0);            

            }else if (rear_left <= -0.5) {
                gpio_put(wheels[2].R_EN, 1);
                gpio_put(wheels[2].L_EN, 1);                 

                pwm_set_gpio_level(wheels[2].R_PWM, 0);            
                pwm_set_gpio_level(wheels[2].L_PWM, duty_rear_left);            

            }else{
                gpio_put(wheels[2].R_EN, 1);
                gpio_put(wheels[2].L_EN, 1);                 

                pwm_set_gpio_level(wheels[2].R_PWM, 0);            
                pwm_set_gpio_level(wheels[2].L_PWM, 0);            
            }

            // Rear Right
            if (rear_right >= 0.5) {
                gpio_put(wheels[3].R_EN, 1);
                gpio_put(wheels[3].L_EN, 1);                 

                pwm_set_gpio_level(wheels[3].R_PWM, duty_rear_right);            
                pwm_set_gpio_level(wheels[3].L_PWM, 0);            

            }else if (rear_right <= -0.5) {
                gpio_put(wheels[3].R_EN, 1);
                gpio_put(wheels[3].L_EN, 1);                 

                pwm_set_gpio_level(wheels[3].R_PWM, 0);            
                pwm_set_gpio_level(wheels[3].L_PWM, duty_rear_right);            

            }else{
                gpio_put(wheels[3].R_EN, 1);
                gpio_put(wheels[3].L_EN, 1);                 

                pwm_set_gpio_level(wheels[3].R_PWM, 0);            
                pwm_set_gpio_level(wheels[3].L_PWM, 0);            
            }


            // if (rear_right >= 0.5) {
            //     gpio_put(rear_right_motor_forward, 1);
            //     gpio_put(rear_right_motor_reverse, 0);                  
            // }else if (rear_right <= -0.5){
            //     gpio_put(rear_right_motor_forward, 0);
            //     gpio_put(rear_right_motor_reverse, 1);              
            // }else{
            //     gpio_put(rear_right_motor_forward, 0);
            //     gpio_put(rear_right_motor_reverse, 0);              
            // }   

            // int duty_rear_left = ((((int)(fabs(rear_left/6.0) * 255))<=100) ? ((int)(fabs(rear_left/6.0) * 255)) : 100 );    
            // int duty_rear_right = ((((int)(fabs(rear_right/6.0) * 255))<=100) ? ((int)(fabs(rear_right/6.0) * 255)) : 100 );

            ////printf("LEFT Duty: %d \n", duty_rear_left);                       
            // printf("RIGHT Duty: %d \n", duty_rear_right);                       

            //pwm_set_gpio_level(rear_left_PWM, duty_rear_left);            
//            pwm_set_gpio_level(rear_right_PWM, duty_rear_right);            
        }            
        
        sleep_ms(100); // To avoid CPU overload 
            
        }

}

