#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

// Function declarations
boolean emergency_routine(boolean toggle_emergency);

void bt_routine(void);
boolean bt_check_for_connection_state(void);
char bt_get_input(boolean state);
void bt_execute_cmd(char cmd);

void make_connect_sound(void);
void make_disconnect_sound(void);
void set_output(uint8_t output, boolean output_state);

void lighting_routine(void);
void movement_routine(void);

int increase_speed(int speed);
int decrease_speed(int speed);
int increase_angle_speed(int speed);
int decrease_angle_speed(int speed);

typedef enum
{
    FORWARD_LEFT,
    FORWARD,
    FORWARD_RIGHT,
    LEFT,
    RIGHT,
    BACK_LEFT,
    BACK,
    BACK_RIGHT,
    IDLE
} MovementDirection;

typedef enum
{
    LEFT_TURN,
    RIGHT_TURN,
    NO_TURN
} TurnDirection;

typedef struct
{
    boolean emergency;
    boolean remote_connected;

    boolean head_lights;
    boolean indicator_left;
    boolean indicator_right;

    boolean tail_lights;
    boolean brake_lights;

    MovementDirection movement_direction;
    MovementDirection next_movement_direction;

    TurnDirection turn_direction;
    TurnDirection next_turn_direction;

    int speed;
    int next_speed;

    int angular_speed;
    int next_angular_speed;
} State;

#endif