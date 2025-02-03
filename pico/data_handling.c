// Layout::
//
// Core 0:
//     * Receiving wheel velocities from ESP and forwarding it to motor controller.
//     * Generating interrupt requesting data from Core 1.
//     * Estimating robot position using Encoder and IMU readings.
//     * Forwarding th estimated robot position to ESP.
//
// Core 1:
//      
//     * Collecting data from 4 encoders and an IMU.
//     * Sending data to core 0 as soon as an interrupt is generated. Core 1 is kept reserved for sensor data collection.



// To move roboto a given point.

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <iostream>

#define ENSURE_LESS_THAN_255(x) (((x)<=255) ? (x) : 255 )
#define LED_PIN 25

#define TOTAL_WHEELS 4

// MPU-6050 registers
#define I2C_PORT i2c0   
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H 0x43
#define MPU6050_ADDR 0x68   // I2C address of the MPU-6050

#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define TEMP_OUT_H   0x41
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

#define UART_ID uart0

#define SIZE 6

typedef struct {
    int R_PWM;
    int L_PWM;
    int R_EN;
    int L_EN;
} Wheel;

Wheel wheels[TOTAL_WHEELS];

void init_wheels() {
    wheels[0] = (Wheel) { .R_PWM = 21, .L_PWM = 20, .R_EN = 19, .L_EN = 18};       // Front Left Wheel
    wheels[1] = (Wheel) { .R_PWM = 10, .L_PWM = 11, .R_EN = 12, .L_EN = 13};       // Front Right Wheel
    wheels[2] = (Wheel) { .R_PWM = 2, .L_PWM = 3, .R_EN = 4, .L_EN = 5};           // Rear Left Wheel
    wheels[3] = (Wheel) { .R_PWM = 6, .L_PWM = 7, .R_EN = 8, .L_EN = 9};           // Rear Right Wheel
}

// GPIO pins for sensors
const uint encoder_pins[4] = {28, 22, 14, 15};      // Sequence: Front Left, Front Right, Rear Left, Rear Right


// GPIO pins for IMU
const uint imu_pins[2] = {16, 17};  // Sequence: SDA, SCL

// Initialize the MPU-6050
void mpu6050_init() {
    uint8_t data[2];
    data[0] = PWR_MGMT_1;
    data[1] = 0x00;  // Wake up the MPU-6050 by writing 0x00 to the power management register
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, data, 2, false);
}                       

// Read IMU Data
int16_t read_mpu6050_data(uint8_t reg_high){
    uint8_t data[2];
    int16_t value;

    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg_high, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, data, 2, false);

    value = (data[0] << 8) | data[1];  // Combine high and low bytes
    return value;
}


void setup_GPIO() {

    // Initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize Encoder Pins
    for (int i = 0; i < 4; i++) {
        gpio_init(encoder_pins[i]);
        gpio_set_dir(encoder_pins[i], GPIO_IN);
    }


    // Initialize MOTOR pins
    for(int i=0; i<TOTAL_WHEELS; i++){    
        gpio_init(wheels[i].R_EN);    
        gpio_init(wheels[i].L_EN);    

        gpio_set_dir(wheels[i].R_EN, GPIO_OUT);
        gpio_set_dir(wheels[i].L_EN, GPIO_OUT);
    }

    //Initialize MPU_6050
    i2c_init(I2C_PORT, 400000);  // 400kHz I2C speed
    gpio_set_function(imu_pins[0], GPIO_FUNC_I2C);
    gpio_set_function(imu_pins[1], GPIO_FUNC_I2C);
    gpio_pull_up(imu_pins[0]);
    gpio_pull_up(imu_pins[1]);


    // Initialize USB for standard input/output
    stdio_init_all();

    // Setup UART Communication(between ESP and PICO)
    uart_init(UART_ID, 115200);
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

// float wheel_radius = 0.0635f;
// float sum_of_robot_center_projection_on_X_Y_axis = 0.508f;

typedef struct {
    int wheel1_enc;     // Encoder Pulse counter for Wheel 1
    int wheel2_enc;     // Encoder Pulse counter for Wheel 2
    int wheel3_enc;     // Encoder Pulse counter for Wheel 3
    int wheel4_enc;     // Encoder Pulse counter for Wheel 4
    int accel_x;        // IMU acceleration in X axis
    int accel_y;        // IMU acceleration in Y axis
    int gyro_z;         // IMU angular velocity in Z axis
} Core1_Counters;

Core1_Counters sensor_data = {0, 0, 0, 0, 0, 0, 0};
Core1_Counters sensor_data_final = {0, 0, 0, 0, 0, 0, 0};

// Data Request Interrupt Handler
void data_request_interrupt_handler() {
    bool got;
    uint32_t received_val;

    // Check if there is a valid request in the FIFO from Core 0
    if (multicore_fifo_rvalid()) {
          
        got = multicore_fifo_pop_timeout_us(0, &received_val); // Non-blocking pop (with 0 timeout)
        
        if (got && received_val==1) {
            // To send data
            // We can either send data 5 times(push data to FIFO 5 times)(4 data units for encoders and the last one for IMU[note IMU may require more than one data push for acceleration in x and y and angular in z]).
            // Or we can send a whole structure once containing all the encoder and IMU data. 
            // For now we will try send the whole structure. If pico does not alows it, we will send data one by one.


            //  Add IMU data to 'sensor_data' structure before sending.
            // Accelerometer Data
            int16_t accel_x = (read_mpu6050_data(ACCEL_XOUT_H)/16384)*9.81; 
            int16_t accel_y = (read_mpu6050_data(ACCEL_YOUT_H)/16384)*9.81;
            //int16_t accel_z = (read_mpu6050_data(ACCEL_ZOUT_H)/16384)*9.81;

            // Gyroscope Data
            // int16_t gyro_x = read_mpu6050_data(GYRO_XOUT_H)/131; 
            // int16_t gyro_y = read_mpu6050_data(GYRO_YOUT_H)/131;
            int16_t gyro_z = read_mpu6050_data(GYRO_ZOUT_H)/131;
            
            // IMU Temprature
            // int16_t temp_raw = read_mpu6050_data(TEMP_OUT_H);
            // float temp = (temp_raw / 340.0) + 36.53;


            // sensor_data = {0, 0, 0, 0, accel_x, accel_y, gyro_z}; // Add IMU data to 'sensor_data'

            sensor_data.accel_x = accel_x;
            sensor_data.accel_y = accel_y;
            sensor_data.gyro_z = gyro_z;
            

            sensor_data_final = sensor_data;      
            memset(&sensor_data, 0, sizeof(sensor_data));   // Reset the counter after sending

            // Sending data in one go
            // if (!multicore_fifo_push_timeout_us(sensor_data, 0)) {      // Here send 'sensor_data' instead of 'count'.
            //     printf("FIFO push failed, FIFO is full!\n");
            // } else {
            //     sensor_data = {0, 0, 0, 0, 0, 0, 0}; // Reset the counter after sending
            // }

            // Sending data one by one
            // multicore_fifo_push_blocking(*(uint32_t*)&data);

        }
    }
    // Clear the interrupt flag
    multicore_fifo_clear_irq();
}

// Interrupt Service Routine (ISR) for handling encoder pulse from different encoders
void enc1(uint gpio, uint32_t events) {
    sensor_data.wheel1_enc++;   // Increment the pulse count
}
void enc2(uint gpio, uint32_t events) {
    sensor_data.wheel2_enc++;   // Increment the pulse count
}
void enc3(uint gpio, uint32_t events) {
    sensor_data.wheel3_enc++;   // Increment the pulse count
}
void enc4(uint gpio, uint32_t events) {
    sensor_data.wheel4_enc++;   // Increment the pulse count
}


void core1_entry() {
    // Set up FIFO interrupt for Core 1
    irq_set_exclusive_handler(SIO_IRQ_PROC1, data_request_interrupt_handler);
    irq_set_enabled(SIO_IRQ_PROC1, true);
    irq_set_priority(SIO_IRQ_PROC1, 0); // Higher priority for Core 0 interrupt
    multicore_fifo_clear_irq(); // Clear any pending interrupts

    // Encoder 1 Interrupt
    gpio_set_irq_enabled_with_callback(encoder_pins[0], GPIO_IRQ_EDGE_RISE, true, enc1);
    gpio_set_irq_enabled_with_callback(encoder_pins[1], GPIO_IRQ_EDGE_RISE, true, enc2);
    gpio_set_irq_enabled_with_callback(encoder_pins[2], GPIO_IRQ_EDGE_RISE, true, enc3);
    gpio_set_irq_enabled_with_callback(encoder_pins[3], GPIO_IRQ_EDGE_RISE, true, enc4);
    irq_set_priority(IO_IRQ_BANK0, 1);    // Lower priority for sensor interrupt. Priority of all GPIO pins are set usign this.



    while (true) {
        __wfi();    // To keep the function active. The processor goes to low power consumption state when this function is called.
    }
}

// Sensor Fusion
class MatrixOperations{
    public:
        void matrix_add(float *A, float *B, float *result, int rows, int cols) {
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    *(result + i * cols + j) = *(A + i * cols + j) + *(B + i * cols + j);
                }
            }
        }

        void matrix_sub(float *A, float *B, float *result, int rows, int cols) {
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    *(result + i * cols + j) = *(A + i * cols + j) - *(B + i * cols + j);
                }
            }
        }

        void matrix_multiply(float* A, float* B, float* result, int rowsA, int colsA, int rowsB, int colsB) {
            if (colsA != rowsB) {
                std::cout << "Matrix dimensions are incompatible for multiplication!" << std::endl;
                return;
            }

            for (int i = 0; i < rowsA; i++) {
                for (int j = 0; j < colsB; j++) {
                    *(result + i * colsB + j) = 0; // Initialize result[i][j] to 0
                    for (int k = 0; k < colsA; k++) {
                        *(result + i * colsB + j) += (*(A + i * colsA + k)) * (*(B + k * colsB + j));
                    }
                }
            }
        }

        void transposeMatrix(float* A, float* result, int rows, int cols) {
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    *(result + j * rows + i) = *(A + i * cols + j);
                }
            }
        }

        
        // Function to perform Gaussian Elimination
        bool inverseMatrix(float A[SIZE][SIZE], float inv[SIZE][SIZE]) {
            // Augment A with the identity matrix I
            float augmented[SIZE][SIZE * 2];
            for (int i = 0; i < SIZE; i++) {
                for (int j = 0; j < SIZE; j++) {
                    augmented[i][j] = A[i][j];
                    augmented[i][j + SIZE] = (i == j) ? 1.0 : 0.0;  // Identity matrix
                }
            }

            // Perform Gaussian Elimination
            for (int i = 0; i < SIZE; i++) {
                // Find the pivot row and swap if necessary
                if (augmented[i][i] == 0) {
                    // Find a row below the current row with a non-zero element to swap
                    bool swapped = false;
                    for (int j = i + 1; j < SIZE; j++) {
                        if (augmented[j][i] != 0) {
                            std::swap(augmented[i], augmented[j]);
                            swapped = true;
                            break;
                        }
                    }
                    if (!swapped) {
                        std::cerr << "Matrix is singular, cannot compute inverse.\n";
                        return false;  // Singular matrix, no inverse
                    }
                }

                // Scale the pivot row
                float pivot = augmented[i][i];
                for (int j = 0; j < SIZE * 2; j++) {
                    augmented[i][j] /= pivot;
                }

                // Eliminate all other rows
                for (int j = 0; j < SIZE; j++) {
                    if (j != i) {
                        float factor = augmented[j][i];
                        for (int k = 0; k < SIZE * 2; k++) {
                            augmented[j][k] -= factor * augmented[i][k];
                        }
                    }
                }
            }

            // Extract the inverse matrix from the augmented matrix
            for (int i = 0; i < SIZE; i++) {
                for (int j = 0; j < SIZE; j++) {
                    inv[i][j] = augmented[i][j + SIZE];
                }
            }

            return true;
        }
 
};


class KalmanFilter{
    
    public:
        KalmanFilter(){
            r = 0.0635f;
            proj = 0.6f; 
        }

        void printMatrix(){
            for (int i = 0; i < 6; i++) {
                for (int j = 0; j < 1; j++) {
                    cout << X[i][j] << " ";
                }
                cout << "\n";
            }
            cout << "\n\n\n\n";
        }
        

        void predict(float U[4][1]){

            // Mean Predict
            matOp.matrix_multiply((float *)A, (float *)X, (float *)tempAX, 6, 6, 6, 1);           // A*X
            matOp.matrix_multiply((float *)B, (float *)U, (float *)tempBU, 6, 4, 4, 1);           // B*U
            matOp.matrix_add((float *)tempAX, (float *)tempBU, (float *)X, 6, 1);                 // A*X + B*U

            // Covariance Predict
            // Note: Here At is transpose of A but because A is an identity matrix(or has elements in digonaly only), its transpose is itself.
            matOp.matrix_multiply((float *)A, (float *)P, (float *)tempAP, 6, 6, 6, 6);           // A*P
            matOp.matrix_multiply((float *)tempAP, (float *)A, (float *)tempAP_At, 6, 6, 6, 6);   // A*P*At
            matOp.matrix_add((float *)tempAP_At, (float *)R, (float *)P, 6, 6);                   // A*P*At + R
        }
        
        void update(float Z[6][1]){     // Here Z is sensor input
            // C[3][3] = prev_state[0][0];
            // C[4][3] = prev_state[1][0];
            // C[5][3] = prev_state[2][0];

            
            //matOp.matrix_multiply((float *)C, (float *)sensor_input, (float *)Z, 6, 4 , 6, 1);  // Calculating Z

            // Kalman Gain
            matOp.transposeMatrix((float *)C, (float *)tempCt, 6, 6);                                   // C transpose (Ct)
            matOp.matrix_multiply((float *)C, (float *)P, (float *)tempCP, 6, 6, 6, 6);                 // C*P
            matOp.matrix_multiply((float *)tempCP, (float *)tempCt, (float *)tempCP_Ct, 6, 6, 6, 6);    // C*P*Ct    
            matOp.matrix_add((float *)tempCP_Ct, (float *)Q, (float *)tempCP_Ct_Q, 6, 6);               // C*P*Ct+Q    
    
            if (matOp.inverseMatrix(tempCP_Ct_Q, tempCP_Ct_Q_inv)) {                                   // (C*P*Ct+Q)^-1    
                std::cout << "Inverse Sucessful!\n";
                // printMatrix(A_inv);
            } else {
                std::cerr << "Matrix is singular, no inverse.\n";
            }

            matOp.matrix_multiply((float *)P, (float *)tempCt, (float *)tempPCt, 6, 6, 6, 6);               // P*Ct    

            matOp.matrix_multiply((float *)tempPCt, (float *)tempCP_Ct_Q_inv, (float *)K, 6, 6, 6, 6);    // P*Ct*(C*P*Ct+Q)^-1    


            // Mean Update
            matOp.matrix_multiply((float *)C, (float *)X, (float *)tempCX, 6, 6, 6, 1);                 // C*X
            matOp.matrix_sub((float *)Z, (float*)tempCX, (float *)tempZ_CX, 6, 1);                      // Z-C*X
            matOp.matrix_multiply((float *)K, (float *)tempZ_CX, (float *) tempK_Z_CX, 6, 6, 6, 1);      // K(Z-C*X)
            matOp.matrix_add((float *)X, (float *)tempK_Z_CX, (float *)X, 6, 1);                        // U+K(Z-C*X)

            // Covariance Update
            matOp.matrix_multiply((float *)K, (float *)C, (float *)tempKC, 6, 6, 6, 6);                 // K*C
            matOp.matrix_sub((float *)I, (float *)tempKC, (float *)tempI_KC, 6, 6);                       // I-K*C
            matOp.matrix_multiply((float *)tempI_KC, (float *)P, (float *)P, 6, 6, 6, 6);               // (I-K*C)*P
        }

    private:
        //Robot Parameters
        float r;                                                        // Wheel Radius
        float proj;                                                     // Sum of distance of wheel from robot center

        // Kalman Filter Parameters 
        float dt = 1.0f;                                                // Change in time

        float X[6][1] = {{0.0f},                                        // State Vector(x, y orientation)   
                         {0.0f}, 
                         {0.0f},
                         {0.0f},
                         {0.0f},
                         {0.0f}};                             

        float X_prev[6][1] = {{0.0f},                                   // Previos State Vector(x, y orientation)   
                              {0.0f}, 
                              {0.0f},
                              {0.0f},
                              {0.0f},
                              {0.0f}};                             

        // float U[4][1] = {{0.0f},                                        // Control Input(FLV, RLV, RRV, FRV)
        //                  {0.0f}, 
        //                  {0.0f}, 
        //                  {0.0f}};      

        float A[6][6] = {{1, 0, 0, 0, 0, 0},                              // Keeping position as it is
                       {0, 1, 0, 0, 0, 0},                              // Making velocity 0 as it would be updated by B*U
                       {0, 0, 1, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0},
                       {0, 0, 0, 0, 0, 0}};

        float B[6][4] = {{dt*(0.25f*r),           dt*(0.25f*r),           dt*(0.25f*r),          dt*(0.25f*r)         }, 
                         {dt*(-0.25f*r),          dt*(0.25f*r),           dt*(-0.25f*r),         dt*(0.25f*r)         }, 
                         {dt*(-(0.25f * r)/proj), dt*(-(0.25f * r)/proj), dt*((0.25f * r)/proj), dt*((0.25f * r)/proj)},
                         {(0.25f*r),              (0.25f*r),              (0.25f*r),             (0.25f*r)            }, 
                         {(-0.25f*r),             (0.25f*r),              (-0.25f*r),            (0.25f*r)            }, 
                         {(-(0.25f * r)/proj),    (-(0.25f * r)/proj),    ((0.25f * r)/proj),    ((0.25f * r)/proj)   }};


        // NOTE: This represents the belief about initial state. If the initial state is known correctly, keep diagonal elements near to 0. Kepp them high otherwise.
        float P[6][6] = {{0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                            // Covariance matrix
                         {0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f},                        
                         {0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f}};

        float R[6][6] = {{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},                            // Process noise    
                         {0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, 
                         {0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f}};

        // float C[6][4] = {{1.0f,  0.0f,  0.0f,  0.0f},       // [[1,  0,  0,  0     ],
        //                  {0.0f,  1.0f,  0.0f,  0.0f},       //  [0,  1,  0,  0     ],
        //                  {0.0f,  0.0f,  1.0f,  0.0f},       //  [0,  0,  1,  0     ],
        //                  {dt,    0.0f,  0.0f,  0.0f},       //  [dt, 0,  0,  vPrevX],
        //                  {0.0f,  dt,    0.0f,  0.0f},       //  [0,  dt, 0,  vPrevY],    
        //                  {0.0f,  0.0f,  dt,    0.0f}};      //  [0,  0,  dt, vPrevZ]]

        // float C[6][3] = {{1/dt,      0,         0        },
        //                  {0,         1/dt,      0        },
        //                  {0,         0,         1/dt     },
        //                  {1/(dt*dt), 0,         0        },
        //                  {0,         1/(dt*dt), 0        },
        //                  {0,         0,         1/(dt*dt)}};

        float C[6][6] = {{0,    0,    0,     1,      0,      0   },
                         {0,    0,    0,     0,      1,      0   },
                         {0,    0,    0,     0,      0,      1   },
                         {0,    0,    0,     1/dt,   0,      0   },
                         {0,    0,    0,     0,      1/dt,   0   },
                         {0,    0,    0,     0,      0,      1   }};


        float Q[6][6] = {{0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},          // Sensor Error Covariance Matrix(Q)
                         {0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 0.2f, 0.0f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f},
                         {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.2f}};

        float I[6][6] = {{1, 0, 0, 0, 0, 0},                              // Identity Martix
                       {0, 1, 0, 0, 0, 0}, 
                       {0, 0, 1, 0, 0, 0},
                       {0, 0, 0, 1, 0, 0},
                       {0, 0, 0, 0, 1, 0},
                       {0, 0, 0, 0, 0, 1}};

        // float Z[6][1];                                               // Sensor Input       

        MatrixOperations matOp = MatrixOperations();

        // Temprary matrice: The following matrices are used to store intrermediate results.
        
        // Mean Predict
        float tempAX[6][1];                                             // Size = rows of A and columns of X
        float tempBU[6][1];                                             // Size = rows of B and columns of U
        
        // Covariance Predict
        float tempAP[6][6];
        float tempAP_At[6][6];
        
        // Kalman Gain
        float tempCt[6][6];
        float tempCP[6][6];
        float tempCP_Ct[6][6];
        float tempCP_Ct_Q[6][6];
        float tempCP_Ct_Q_inv[6][6];

        float tempPCt[6][6];

        float K[6][6];

        // Mean Update
        float tempCX[6][1];
        float tempZ_CX[6][1];
        float tempK_Z_CX[6][1];

        // Covariance Update
        float tempKC[6][6];
        float tempI_KC[6][6];
};


int main(){
    init_wheels();

    setup_GPIO();
    setup_PWM();

    mpu6050_init();

    uint32_t received_value;
    bool got;

    // Launch task on core 1(Sensor data collection, sensor fusion, state estimation )
    multicore_launch_core1(core1_entry);

    while (true) {

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
        read_buff[len] = '\0';
        // printf("Received: %s\n", read_buff);

        strncpy(temp_buff, read_buff + strlen(read_buff) -20, 20);  // To get the latest last 20 elements.
        temp_buff[20] = '\0';

        // printf("%s\n", temp_buff);
        
        if (len > 0){
            char front_left_char[6];
            char front_right_char[6];                        
            char rear_left_char[6];
            char rear_right_char[6];            

            // NOTE: We use 'strncpy' for copying till a particular point and 'sstrcpy' to copy from any point till end. 'strcpy' automatically adds '\0' at end while 'strncpy' does not.            
            // Resource: 
            // * https://stackoverflow.com/questions/16807004/how-to-get-a-part-of-a-char-array-in-c
            // * https://cstdspace.quora.com/What-is-the-difference-between-strncpy-and-strcpy-for-strings-in-C-programming-language?top_ans=386912167
            

            strncpy(front_left_char, temp_buff, 5);
            strncpy(front_right_char, temp_buff + 5, 5);
            strncpy(rear_left_char, temp_buff + 10, 5);
            strncpy(rear_right_char, temp_buff + 15, 5);
            
            front_left_char[5] = '\0';
            front_right_char[5] = '\0';
            rear_left_char[5] = '\0';
            rear_right_char[5] = '\0';
            
            double front_left = atof(front_left_char);
            double front_right = atof(front_right_char);
            double rear_left = atof(rear_left_char);
            double rear_right = atof(rear_right_char);

            //printf("Double Left: %d \n", (int)(fabs(rear_left/6.0) * 255));
            //printf("Double Right: %d \n", (int)(fabs(rear_right/6.0) * 255));          

            // Setting dutycycle;

            int duty_front_left = ENSURE_LESS_THAN_255((int)(fabs(front_left/255.0) * 255));
            int duty_front_right = ENSURE_LESS_THAN_255((int)(fabs(front_right/255.0) * 255));
            int duty_rear_left = ENSURE_LESS_THAN_255((int)(fabs(rear_left/255.0) * 255));
            int duty_rear_right = ENSURE_LESS_THAN_255((int)(fabs(rear_right/255.0) * 255));


            //printf("Duty Front LEFT: %f \n", front_left);
            //printf("Duty Front RIGHT: %f \n", front_right);
            //printf("Duty Rear LEFT: %f \n", rear_left);
            //printf("Duty Rear RIGHT: %f \n\n\n", rear_right);


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
 
        }            
        

        // Push dummy data to core 1. Core 1 will send sensor data once it receives interrupt.
        // if (!multicore_fifo_push_timeout_us(1, 0)) {        // Here sending 1 as dummy data, 0 is the timeout mean it wont wait.
        //     printf("FIFO push failed, FIFO is full!\n");
        // } else {
        //     printf("Request sent to Core 1\n");
        // }

        // multicore_fifo_push_timeout_us(1, 0); // Sending 1 as dummy data
        
        sensor_data_final = {1000, 2000, 1500, 1700, 5, -2, 10};

        // To receive data
        // We can either receive data 5 times(pop data from FIFO 5 times)(4 data units for encoders and the last one for IMU[note IMU may require more than one data pop for acceleration in x and y and angular in z]).
        // Or we can receive a whole structure once containing all the encoder and IMU data. 
        // For now we will try receiving the whole structure. If pico does nto alows it, we will receive data one by one.

        // Send sensor data to ESP
        std::string result = std::to_string(sensor_data_final.wheel1_enc) + "@" + std::to_string(sensor_data_final.wheel2_enc) + "@" +
                            std::to_string(sensor_data_final.wheel3_enc) + "@" +std::to_string(sensor_data_final.wheel4_enc) + "@" +
                            std::to_string(sensor_data_final.accel_x) + "@" +std::to_string(sensor_data_final.accel_y) + "@" + 
                            std::to_string(sensor_data_final.gyro_z) + "~";
                            
        uart_puts(UART_ID, result.c_str());

        // Receiving in one go
        // Core1_Counters received_sensor_data;
        // memcpy(&received_data, (void*)multicore_fifo_pop_blocking(), sizeof(Core1_Counters));
        // printf("Encoder 1: %d, Encoder 2: %d, Encoder 3: %d, Encoder 4: %d\n", received_sensor_data.wheel1_enc, received_sensor_data.wheel2_enc, received_sensor_data.wheel3_enc, received_sensor_data.wheel4_enc);

        // Receiving one by one

        // uint32_t value;
        // bool got;
        //
        // if (multicore_fifo_rvalid()) {
        //     got = multicore_fifo_pop_timeout_us(0, &value); // Non-blocking pop with timeout
        //     if (got && value != -1) {
        //         printf("Received from Core 1: %d\n", value);
        //     } else {
        //         printf("No data received from Core 1\n");
        //     }
        // } else {
        //     printf("FIFO is empty\n");
        // }

        sleep_ms(100); // To avoid CPU overload 
    }

}   






// Max size of FIFO Butter is 16 bytes 128(16*8)bits

// This approach only works if the structure sensor_data_t is 32 bits or smaller. If it exceeds 32 bits, additional steps are required to send the full data.

// For encoder data we can use "uint8_t" which can store 255 at max.
// For IMU data we can use "float" 


// Able to push 13 elements of int. int is 4 bytes. 13*4 = 52 bytes of space.
// Pushed 16 elements of type double.
