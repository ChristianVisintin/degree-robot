/**
 *   degree-robot
 *   Developed by Christian Visintin
 * 
 * MIT License
 * Copyright (c) 2016-2020 Christian Visintin
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <NewPing.h>
#include <Servo.h>

// @! Defines
#define DISTANCE_MATRIX_ROWS 10
#define DISTANCE_MATRIX_COLUMNS 3
#define DISTANCE_MATRIX_DEFAULT_DISTANCE 50
#define DISTANCE_MATRIX_COL_RIGHT 2
#define DISTANCE_MATRIX_COL_CENTER 1
#define DISTANCE_MATRIX_COL_LEFT 0

// @! Types
typedef unsigned long millis_t;
typedef unsigned int cm_t; // Centimeters

typedef enum DeviceState {
    StateRunning,
    StateStopped
} DeviceState;

typedef enum MotorDirection {
    MotorForward,
    MotorBackward
} MotorDirection;

typedef enum ServoDirection {
    ServoLeft,
    ServoCenter,
    ServoRight
} ServoDirection;

typedef enum PowerState {
    PowerOn,
    PowerOff
} PowerState;

typedef enum MovementDirection {
    DirectionForward,
    DirectionRight,
    DirectionLeft,
    DirectionBackward
} MovementDirection;

// @! Pinout definition

// Servo
const byte servo_right_pin = 9; // Right servo pin
const byte servo_left_pin = 10; // Left servo pin
// Distance sensors
const byte distsensor_trigger_left_pin = 13;
const byte distsensor_echo_left_pin = 12;
const byte distsensor_trigger_central_pin = 11;
const byte distsensor_echo_central_pin = 8;
const byte distsensor_trigger_right_pin = 7;
const byte distsensor_echo_right_pin = 6;
// DcMotors
const byte dcmotor_pwr_pin = 5; // Gives power to dcmotors
const byte dcmotor_fwd_pin = 3; // If HIGH, dcmotor moves "forward"
const byte dcmotor_bck_pin = 4; // If HIGH, dcmotor moves "backward"
// Misc
const byte pwr_button_pin = 2; // Service / Stop button

// @! Globals
// Matrix
cm_t distance_matrix[DISTANCE_MATRIX_ROWS][DISTANCE_MATRIX_COLUMNS] = { DISTANCE_MATRIX_DEFAULT_DISTANCE }; // Stores last 10 records of distance in centimeters NOTE: newer values at first indexes
// Dist sensors
NewPing ping_left = NewPing(distsensor_trigger_left_pin, distsensor_echo_left_pin, 150);
NewPing ping_central = NewPing(distsensor_trigger_central_pin, distsensor_echo_central_pin, 150);
NewPing ping_right = NewPing(distsensor_trigger_right_pin, distsensor_echo_right_pin, 150);
// Servos
Servo servo_right;
Servo servo_left;
// Current device state NOTE: start as stopped
DeviceState dev_state = StateStopped;
millis_t last_distance_fetch = 0;
bool state_changed = false;
MovementDirection last_direction = DirectionForward;

// @! Constants
const millis_t max_millis = 4294967295;
const millis_t fetch_interval = 500; // Distance Fetch interval

// @! Functions

// Hardware functions

/**
 * @brief set motor direction
 * @param direction
 */

void set_motor_direction(const MotorDirection direction) {
    switch (direction) {
        case MotorForward:
        digitalWrite(dcmotor_fwd_pin, HIGH);
        digitalWrite(dcmotor_bck_pin, LOW);
        break;
        case MotorBackward:
        digitalWrite(dcmotor_fwd_pin, LOW);
        digitalWrite(dcmotor_bck_pin, HIGH);
        break;
    }
}

/**
 * @brief change servo direction to steer
 * @param direction
 */

void steer(const ServoDirection direction) {
    switch (direction) {
        case ServoRight:
        servo_right.write(85);
        servo_left.write(52);
        break;
        case ServoCenter:
        servo_right.write(70);
        servo_left.write(45);
        break;
        case ServoLeft:
        servo_right.write(60);
        servo_left.write(40);
        break;
    }
}

/**
 * @brief set logical power state
 * @param power
 */

void set_power_state(const PowerState power) {
    switch (power) {
        case PowerOn:
        digitalWrite(dcmotor_pwr_pin, HIGH);
        break;
        case PowerOff:
        digitalWrite(dcmotor_pwr_pin, LOW);
        break;
    }
}

/**
 * @brief read distance from distance sensor
 * @param sensor
 * @return centimeters
 */

cm_t read_distance_from_sensor(NewPing* sensor) {
    const unsigned int distance = sensor->ping();
    return sensor->convert_cm(distance);
}

// Distance matrix functions

/**
 * @brief reset direction matrix
 */

void reset_distance_matrix() {
    for (unsigned int i = 0; i < DISTANCE_MATRIX_ROWS; i++) {
        for (unsigned int j = 0; j < DISTANCE_MATRIX_COLUMNS; j++) {
            distance_matrix[i][j] = DISTANCE_MATRIX_DEFAULT_DISTANCE;
        }
    }
}

/**
 * @brief write a new row in the distance matrix. Previous rows are shifted back
 * @param distance_left
 * @param distance_center
 * @param distance_right
 */

void write_distance_matrix_row(cm_t distance_left, cm_t distance_center, cm_t distance_right) {
    // Shift all matrix elements right
    cm_t tmp = 0;
    for (unsigned int i = DISTANCE_MATRIX_ROWS - 1; i > 0; i--) {
        for (unsigned int j = 0; j < DISTANCE_MATRIX_COLUMNS; j++) {
            distance_matrix[i][j] = distance_matrix[i - 1][j];
        }
    }
    // Write new values at the beginning of matrix
    distance_matrix[0][DISTANCE_MATRIX_COL_LEFT] = distance_left;
    distance_matrix[0][DISTANCE_MATRIX_COL_CENTER] = distance_center;
    distance_matrix[0][DISTANCE_MATRIX_COL_RIGHT] = distance_right;
}

// @! Path calc functions

/**
 * @brief calculate the "average" direction on a distance; newer values are more important in calculating the distance
 * @param distances
 * @return avg
 */

cm_t calc_avg_direction_distance(cm_t distances[]) {
    cm_t avg = 0;
    float factor = 2.0;
    for (unsigned int i = 0; i < DISTANCE_MATRIX_ROWS; i++) {
        avg += (distances[i] * factor);
        factor *= 0.8;
    }
    // Finally divide avg by rows
    return avg / DISTANCE_MATRIX_ROWS;
}

/**
 * @brief calculate the direction the device has to take
 * @return direction
 */

MovementDirection calculate_direction() {
    // Check if it is possible to keep going forward
    if (distance_matrix[0][DISTANCE_MATRIX_COL_CENTER] >= 50) { // 50 centimeters
        return DirectionForward;
    }
    // Check anti-collision (backward)
    if (distance_matrix[0][DISTANCE_MATRIX_COL_CENTER] <= 15) { // 15 centimeters
        return DirectionBackward;
    }
    // The device must steer right/left; let's get the direction it must turn
    // Copy distances to array
    cm_t left_side[DISTANCE_MATRIX_ROWS];
    cm_t right_side[DISTANCE_MATRIX_ROWS];
    for (unsigned int i = 0; i < DISTANCE_MATRIX_ROWS; i++) {
        left_side[i] = distance_matrix[i][DISTANCE_MATRIX_COL_LEFT];
        right_side[i] = distance_matrix[i][DISTANCE_MATRIX_COL_RIGHT];
    }
    // The bigger average, is the one where there's more space
    const cm_t avg_distance_left = calc_avg_direction_distance(left_side);
    const cm_t avg_distance_right = calc_avg_direction_distance(right_side);
    if (avg_distance_left >= avg_distance_right) {
        return DirectionLeft;
    } else {
        return DirectionRight;
    }
}

// @! Misc

/**
 * @brief checks whether the fetch interval between `now` and `last_distance_fetch` has elapsed
 * @return bool
 */

bool has_fetch_interval_elapsed() {
    const millis_t t_now = millis();
    // Calculate elapsed_time. NOTE: Time overflows after reaching max_millis; so check if time overflowed
    const millis_t elapsed_time = (t_now >= last_distance_fetch) ? (t_now - last_distance_fetch) : ((max_millis - last_distance_fetch) + t_now);
    // Return true if elapsed_timr >= fetch_interval
    return elapsed_time >= fetch_interval;
}

// @! Machine state functions

/**
 * @brief main function for running state
 */

void run() {
    // Check if fetch_interval has elapsed
    if (has_fetch_interval_elapsed()) {
        // Read distances from sensors
        const cm_t left_sensor_distance = read_distance_from_sensor(&ping_left);
        const cm_t center_sensor_distance = read_distance_from_sensor(&ping_central);
        const cm_t right_sensor_distance = read_distance_from_sensor(&ping_right);
        // Update matrix
        write_distance_matrix_row(left_sensor_distance, center_sensor_distance, left_sensor_distance);
        // Update last fetch time
        last_distance_fetch = millis();
        // Calculate direction
        const MovementDirection direction = calculate_direction();
        if (last_direction != direction) {
            // Switch on direction and move device
            switch (direction) {
                case DirectionBackward:
                set_motor_direction(MotorBackward);
                steer(ServoCenter);
                break;
                case DirectionForward:
                set_motor_direction(MotorForward);
                steer(ServoCenter);
                break;
                case DirectionLeft:
                set_motor_direction(MotorForward);
                steer(ServoLeft);
                break;
                case DirectionRight:
                set_motor_direction(MotorForward);
                steer(ServoRight);
                break;
            }
        }
        last_direction = direction;
    }
    delay(50);
}

// @! Arduino loop / setup

void setup() {
    // Setup PINs
    pinMode(dcmotor_pwr_pin, OUTPUT);
    pinMode(dcmotor_fwd_pin, OUTPUT);
    pinMode(dcmotor_bck_pin, OUTPUT);
    pinMode(pwr_button_pin, INPUT);
    // Configure servo
    servo_right.attach(servo_right_pin);
    servo_left.attach(servo_left_pin);
}

void loop() {
    // Read power button
    if (digitalRead(pwr_button_pin) == HIGH) {
        dev_state = (dev_state == StateRunning) ? StateStopped : StateRunning;
        state_changed = true;
    }
    // Machine state
    if (dev_state == StateRunning) {
        // If state changed, turn power on
        if (state_changed) {
            set_power_state(PowerOn);
        }
        // Run
        run();
    } else if (dev_state == StateStopped) {
        // If state changed, turn power OFF
        if (state_changed) {
            set_power_state(PowerOff);
        }
    }
}
