#include <Arduino.h>
#include "BluetoothSerial.h"
#include "main.h"

// Values
BluetoothSerial serialBT;

const uint8_t DEBUG = 2;

const uint8_t TAIL_LIGHTS = 16;
const uint8_t BRAKE_LIGHTS = 22;

const uint8_t HEAD_LIGHTS = 21;
const uint8_t INDIKATOR_LIGHTS_R = 4;
const uint8_t INDIKATOR_LIGHTS_L = 0;

const uint8_t EMERGENCY_BTN_IN = 36;
const uint8_t BUZZER = 13;
const int BUZZER_CH = 0;

const uint8_t MOTOR_F_A1 = 25;
const uint8_t MOTOR_F_A2 = 26;
const uint8_t MOTOR_F_B3 = 27;
const uint8_t MOTOR_F_B4 = 14;
const uint8_t MOTOR_F_en_A = 32;
const uint8_t MOTOR_F_en_B = 33;
const int MOTOR_F_A_CH = 2;
const int MOTOR_F_B_CH = 3;

const uint8_t MOTOR_B_A1 = 19;
const uint8_t MOTOR_B_A2 = 18;
const uint8_t MOTOR_B_B3 = 5;
const uint8_t MOTOR_B_B4 = 17;
const uint8_t MOTOR_B_en_A = 12;
const uint8_t MOTOR_B_en_B = 23;
const int MOTOR_B_A_CH = 4;
const int MOTOR_B_B_CH = 5;

const int START_SPEED = 190;

const float SPEED_MULTIPLIER = 1;
const int MIN_SPEED = 0;
const int MAX_SPEED = 255 - START_SPEED;

const float ANGULAR_SPEED_MULTIPLIER = 2;
const int MIN_ANGULAR_SPEED = 0;
const int MAX_ANGULAR_SPEED = 255 - START_SPEED;

const int BLINK_DURATION = 250;

unsigned long prev_time = -1;
unsigned long cur_time = -1;
unsigned long blink_l_start = -1;
unsigned long blink_r_start = -1;
unsigned long blink_e_start = -1;

State state = {false, false, false, false, false, false, false, IDLE, IDLE, NO_TURN, NO_TURN, 0, 0, 0, 0};

void setup()
{
  Serial.begin(115200);
  serialBT.begin("OmniBot");

  pinMode(DEBUG, OUTPUT);

  pinMode(TAIL_LIGHTS, OUTPUT);
  pinMode(BRAKE_LIGHTS, OUTPUT);

  pinMode(HEAD_LIGHTS, OUTPUT);
  pinMode(INDIKATOR_LIGHTS_R, OUTPUT);
  pinMode(INDIKATOR_LIGHTS_L, OUTPUT);

  pinMode(EMERGENCY_BTN_IN, INPUT);
  pinMode(BUZZER, OUTPUT);

  pinMode(MOTOR_B_A1, OUTPUT);
  pinMode(MOTOR_B_A2, OUTPUT);
  pinMode(MOTOR_B_B3, OUTPUT);
  pinMode(MOTOR_B_B4, OUTPUT);
  pinMode(MOTOR_B_en_A, OUTPUT);
  pinMode(MOTOR_B_en_B, OUTPUT);

  pinMode(MOTOR_F_A1, OUTPUT);
  pinMode(MOTOR_F_A2, OUTPUT);
  pinMode(MOTOR_F_B3, OUTPUT);
  pinMode(MOTOR_F_B4, OUTPUT);
  pinMode(MOTOR_F_en_A, OUTPUT);
  pinMode(MOTOR_F_en_B, OUTPUT);

  digitalWrite(BUZZER, LOW);

  ledcSetup(BUZZER_CH, 1000, 8);
  ledcAttachPin(BUZZER, 0); // pin, channel

  ledcSetup(MOTOR_B_A_CH, 1000, 8);
  ledcAttachPin(MOTOR_B_en_A, MOTOR_B_A_CH);

  ledcSetup(MOTOR_B_B_CH, 1000, 8);
  ledcAttachPin(MOTOR_B_en_B, MOTOR_B_B_CH);

  ledcSetup(MOTOR_F_A_CH, 1000, 8);
  ledcAttachPin(MOTOR_F_en_A, MOTOR_F_A_CH);

  ledcSetup(MOTOR_F_B_CH, 1000, 8);
  ledcAttachPin(MOTOR_F_en_B, MOTOR_F_B_CH);

  digitalWrite(MOTOR_F_A1, LOW);
  digitalWrite(MOTOR_F_A2, LOW);
  digitalWrite(MOTOR_F_B3, LOW);
  digitalWrite(MOTOR_F_B4, LOW);

  digitalWrite(MOTOR_B_A1, LOW);
  digitalWrite(MOTOR_B_A2, LOW);
  digitalWrite(MOTOR_B_B3, LOW);
  digitalWrite(MOTOR_B_B4, LOW);

  ledcWrite(MOTOR_F_A_CH, START_SPEED);
  ledcWrite(MOTOR_F_A_CH, START_SPEED);
  ledcWrite(MOTOR_B_A_CH, START_SPEED);
  ledcWrite(MOTOR_B_B_CH, START_SPEED);

  set_output(HEAD_LIGHTS, true);
  set_output(TAIL_LIGHTS, true);
  prev_time = millis();
  cur_time = millis();
}

void loop()
{
  prev_time = cur_time;
  delay(20);
  cur_time = millis();
  if (emergency_routine(false))
  {
    return;
  }
  lighting_routine();
  movement_routine();
  bt_routine();
}

// Function definitions
boolean emergency_routine(boolean toggle_emergency)
{
  if (digitalRead(EMERGENCY_BTN_IN) == HIGH || toggle_emergency)
  {
    state.emergency = !state.emergency;
    if (state.emergency)
    {
      digitalWrite(DEBUG, HIGH);
      digitalWrite(MOTOR_F_A1, LOW);
      digitalWrite(MOTOR_F_A2, LOW);
      digitalWrite(MOTOR_F_B3, LOW);
      digitalWrite(MOTOR_F_B4, LOW);

      digitalWrite(MOTOR_B_A1, LOW);
      digitalWrite(MOTOR_B_A2, LOW);
      digitalWrite(MOTOR_B_B3, LOW);
      digitalWrite(MOTOR_B_B4, LOW);

      ledcWrite(MOTOR_F_A_CH, START_SPEED);
      ledcWrite(MOTOR_F_B_CH, START_SPEED);
      ledcWrite(MOTOR_B_A_CH, START_SPEED);
      ledcWrite(MOTOR_B_B_CH, START_SPEED);
      state.next_speed = 0;
      state.speed = 0;
      state.next_speed = 0;
      state.speed = 0;
      state.next_movement_direction = IDLE;
      state.movement_direction = IDLE;
      state.next_turn_direction = NO_TURN;
      state.turn_direction = NO_TURN;
    }
    else
    {
      digitalWrite(DEBUG, LOW);
      blink_e_start = -1;
      set_output(HEAD_LIGHTS, true);
      set_output(TAIL_LIGHTS, true);
    }
    delay(500);
  }
  if (state.emergency)
  {
    if (blink_e_start == -1 || cur_time - blink_e_start >= BLINK_DURATION)
    {
      Serial.println("[WARN]: Currently in emergency mode!");

      set_output(HEAD_LIGHTS, !state.head_lights);
      set_output(TAIL_LIGHTS, !state.tail_lights);
      blink_e_start = cur_time;
    }
    return true;
  }

  return false;
}

void set_output(uint8_t output, boolean output_state)
{
  uint8_t output_state_value;
  if (output_state)
  {
    output_state_value = HIGH;
  }
  else
  {
    output_state_value = LOW;
  }

  switch (output)
  {
  case HEAD_LIGHTS:
    state.head_lights = output_state;
    break;
  case INDIKATOR_LIGHTS_L:
    state.indicator_left = output_state;
    break;
  case INDIKATOR_LIGHTS_R:
    state.indicator_right = output_state;
    break;
  case TAIL_LIGHTS:
    state.tail_lights = output_state;
    break;
  case BRAKE_LIGHTS:
    state.brake_lights = output_state;
    break;

  default:
    break;
  }

  digitalWrite(output, output_state_value);
}

void bt_routine(void)
{
  const boolean bt_state = bt_check_for_connection_state();
  const int cmd = bt_get_input(bt_state);
  bt_execute_cmd(cmd);
}

boolean bt_check_for_connection_state(void)
{
  if (!state.remote_connected && serialBT.connected())
  {
    state.remote_connected = true;
    Serial.println("BT remote connected");
    make_connect_sound();
  }
  else if (state.remote_connected && !serialBT.connected())
  {
    state.remote_connected = false;
    Serial.println("BT remote disconnected");
    make_disconnect_sound();
  }

  return state.remote_connected;
}

void make_connect_sound(void)
{
  digitalWrite(DEBUG, HIGH);
  ledcWriteNote(BUZZER_CH, NOTE_F, 4);
  delay(250);
  digitalWrite(DEBUG, LOW);
  ledcWriteNote(BUZZER_CH, NOTE_G, 4);
  delay(250);
  digitalWrite(DEBUG, HIGH);
  ledcWriteNote(BUZZER_CH, NOTE_A, 4);
  delay(250);
  ledcWrite(BUZZER_CH, 0);
  digitalWrite(DEBUG, LOW);
}

void make_disconnect_sound(void)
{
  digitalWrite(DEBUG, HIGH);
  ledcWriteNote(BUZZER_CH, NOTE_A, 4);
  delay(250);
  digitalWrite(DEBUG, LOW);
  ledcWriteNote(BUZZER_CH, NOTE_G, 4);
  delay(250);
  digitalWrite(DEBUG, HIGH);
  ledcWriteNote(BUZZER_CH, NOTE_F, 4);
  delay(250);
  ledcWrite(BUZZER_CH, 0);
  digitalWrite(DEBUG, LOW);
}

char bt_get_input(boolean state)
{
  if (state && serialBT.available())
  {
    const char cmd = serialBT.read();
    return cmd;
  }

  return ' ';
}

void bt_execute_cmd(char cmd)
{
  switch (cmd)
  {
  case 'z':
    Serial.println("[INFO]: received emergency");
    emergency_routine(true);
    break;
  case '1':
    Serial.println("[INFO]: received left_forwards");
    state.next_movement_direction = FORWARD_LEFT;
    break;
  case '2':
    Serial.println("[INFO]: received forwards");
    state.next_movement_direction = FORWARD;
    break;
  case '3':
    Serial.println("[INFO]: received right_forwards");
    state.next_movement_direction = FORWARD_RIGHT;
    break;
  case '4':
    Serial.println("[INFO]: received left");
    state.next_movement_direction = LEFT;
    break;
  case '5':
    Serial.println("[INFO]: received right");
    state.next_movement_direction = RIGHT;
    break;
  case '6':
    Serial.println("[INFO]: received left_backwards");
    state.next_movement_direction = BACK_LEFT;
    break;
  case '7':
    Serial.println("[INFO]: received backwards");
    state.next_movement_direction = BACK;
    break;
  case '8':
    Serial.println("[INFO]: received right_backwards");
    state.next_movement_direction = BACK_RIGHT;
    break;
  case '9':
    Serial.println("[INFO]: received turn_left");
    state.next_turn_direction = LEFT_TURN;
    break;
  case 'a':
    Serial.println("[INFO]: received turn_right");
    state.next_turn_direction = RIGHT_TURN;
    break;
  case 'b':
    Serial.println("[INFO]: received stop movement");
    state.next_movement_direction = IDLE;
    break;
  case 'c':
    Serial.println("[INFO]: received stop turn");
    state.next_turn_direction = NO_TURN;
    break;
  default:
    break;
  }
}

void lighting_routine(void)
{
  if ((
          state.turn_direction == LEFT_TURN ||
          state.movement_direction == LEFT ||
          state.movement_direction == FORWARD_LEFT ||
          state.movement_direction == BACK_LEFT) &&
      (blink_l_start == -1 || cur_time - blink_l_start >= BLINK_DURATION))
  {
    set_output(INDIKATOR_LIGHTS_L, !state.indicator_left);
    blink_l_start = cur_time;
  }
  else if ((state.turn_direction == RIGHT_TURN ||
            state.movement_direction == RIGHT ||
            state.movement_direction == FORWARD_RIGHT ||
            state.movement_direction == BACK_RIGHT) &&
           (blink_r_start == -1 || cur_time - blink_r_start >= BLINK_DURATION))
  {
    set_output(INDIKATOR_LIGHTS_R, !state.indicator_right);
    blink_r_start = cur_time;
  }

  if (state.turn_direction != LEFT_TURN && state.next_turn_direction != LEFT_TURN)
  {
    set_output(INDIKATOR_LIGHTS_L, false);
    blink_l_start = -1;
  }

  if (state.turn_direction != RIGHT_TURN && state.next_turn_direction != RIGHT_TURN)
  {
    set_output(INDIKATOR_LIGHTS_R, false);
    blink_r_start = -1;
  }

  if (state.next_movement_direction != state.movement_direction && !state.brake_lights)
  {
    set_output(BRAKE_LIGHTS, true);
  }
  else if (state.next_movement_direction == state.movement_direction && state.brake_lights)
  {
    set_output(BRAKE_LIGHTS, false);
  }
}

void movement_routine(void)
{
  if (state.next_turn_direction != state.turn_direction)
  {
    state.next_angular_speed = decrease_angle_speed(state.angular_speed);
  }
  else if (state.next_turn_direction == state.turn_direction && state.turn_direction != NO_TURN)
  {
    state.next_angular_speed = increase_angle_speed(state.angular_speed);
  }

  switch (state.next_turn_direction)
  {
  case NO_TURN:
    if (state.next_movement_direction != state.movement_direction)
    {
      state.next_speed = decrease_speed(state.speed);
    }
    else if (state.next_movement_direction == state.movement_direction && state.movement_direction != IDLE)
    {
      state.next_speed = increase_speed(state.speed);
    }
    switch (state.next_movement_direction)
    {
    case IDLE:
      digitalWrite(MOTOR_F_A1, LOW);
      digitalWrite(MOTOR_F_A2, LOW);
      digitalWrite(MOTOR_F_B3, LOW);
      digitalWrite(MOTOR_F_B4, LOW);

      digitalWrite(MOTOR_B_A1, LOW);
      digitalWrite(MOTOR_B_A2, LOW);
      digitalWrite(MOTOR_B_B3, LOW);
      digitalWrite(MOTOR_B_B4, LOW);

      ledcWrite(MOTOR_F_A_CH, START_SPEED);
      ledcWrite(MOTOR_F_B_CH, START_SPEED);
      ledcWrite(MOTOR_B_A_CH, START_SPEED);
      ledcWrite(MOTOR_B_B_CH, START_SPEED);
      break;
    case FORWARD_LEFT:
      digitalWrite(MOTOR_F_A1, LOW);
      digitalWrite(MOTOR_F_A2, LOW);
      digitalWrite(MOTOR_F_B3, LOW);
      digitalWrite(MOTOR_F_B4, LOW);

      digitalWrite(MOTOR_B_A1, HIGH);
      digitalWrite(MOTOR_B_A2, LOW);
      digitalWrite(MOTOR_B_B3, HIGH);
      digitalWrite(MOTOR_B_B4, LOW);

      ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_speed);
      break;
    case FORWARD:
      digitalWrite(MOTOR_F_A1, HIGH);
      digitalWrite(MOTOR_F_A2, LOW);
      digitalWrite(MOTOR_F_B3, HIGH);
      digitalWrite(MOTOR_F_B4, LOW);

      digitalWrite(MOTOR_B_A1, HIGH);
      digitalWrite(MOTOR_B_A2, LOW);
      digitalWrite(MOTOR_B_B3, HIGH);
      digitalWrite(MOTOR_B_B4, LOW);

      ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_speed);
      break;
    case FORWARD_RIGHT:
      digitalWrite(MOTOR_F_A1, HIGH);
      digitalWrite(MOTOR_F_A2, LOW);
      digitalWrite(MOTOR_F_B3, HIGH);
      digitalWrite(MOTOR_F_B4, LOW);

      digitalWrite(MOTOR_B_A1, LOW);
      digitalWrite(MOTOR_B_A2, LOW);
      digitalWrite(MOTOR_B_B3, LOW);
      digitalWrite(MOTOR_B_B4, LOW);

      ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_speed);
      break;
    case LEFT:
      digitalWrite(MOTOR_B_A1, HIGH);
      digitalWrite(MOTOR_B_A2, LOW);
      digitalWrite(MOTOR_B_B3, HIGH);
      digitalWrite(MOTOR_B_B4, LOW);

      digitalWrite(MOTOR_F_A1, LOW);
      digitalWrite(MOTOR_F_A2, HIGH);
      digitalWrite(MOTOR_F_B3, LOW);
      digitalWrite(MOTOR_F_B4, HIGH);

      ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_speed);
      break;
    case RIGHT:
      digitalWrite(MOTOR_F_A1, HIGH);
      digitalWrite(MOTOR_F_A2, LOW);
      digitalWrite(MOTOR_F_B3, HIGH);
      digitalWrite(MOTOR_F_B4, LOW);

      digitalWrite(MOTOR_B_A1, LOW);
      digitalWrite(MOTOR_B_A2, HIGH);
      digitalWrite(MOTOR_B_B3, LOW);
      digitalWrite(MOTOR_B_B4, HIGH);

      ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_speed);
      break;
    case BACK_LEFT:
      digitalWrite(MOTOR_F_A1, LOW);
      digitalWrite(MOTOR_F_A2, HIGH);
      digitalWrite(MOTOR_F_B3, LOW);
      digitalWrite(MOTOR_F_B4, HIGH);

      digitalWrite(MOTOR_B_A1, LOW);
      digitalWrite(MOTOR_B_A2, LOW);
      digitalWrite(MOTOR_B_B3, LOW);
      digitalWrite(MOTOR_B_B4, LOW);

      ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_speed);
      break;
    case BACK:
      digitalWrite(MOTOR_F_A1, LOW);
      digitalWrite(MOTOR_F_A2, HIGH);
      digitalWrite(MOTOR_F_B3, LOW);
      digitalWrite(MOTOR_F_B4, HIGH);

      digitalWrite(MOTOR_B_A1, LOW);
      digitalWrite(MOTOR_B_A2, HIGH);
      digitalWrite(MOTOR_B_B3, LOW);
      digitalWrite(MOTOR_B_B4, HIGH);

      ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_speed);
      break;
    case BACK_RIGHT:
      digitalWrite(MOTOR_F_A1, LOW);
      digitalWrite(MOTOR_F_A2, LOW);
      digitalWrite(MOTOR_F_B3, LOW);
      digitalWrite(MOTOR_F_B4, LOW);

      digitalWrite(MOTOR_B_A1, LOW);
      digitalWrite(MOTOR_B_A2, HIGH);
      digitalWrite(MOTOR_B_B3, LOW);
      digitalWrite(MOTOR_B_B4, HIGH);

      ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_speed);
      ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_speed);
      break;
    default:
      break;
    }

    Serial.println("[INFO]: SPEED");
    Serial.println(state.next_speed);

    if (state.next_speed == MIN_SPEED)
    {
      state.movement_direction = state.next_movement_direction;
    }

    Serial.println("[INFO]: dir");
    Serial.println(state.movement_direction);

    Serial.println("[INFO]: next dir");
    Serial.println(state.next_movement_direction);

    break;
  case RIGHT_TURN:
    digitalWrite(MOTOR_F_A1, HIGH);
    digitalWrite(MOTOR_F_A2, LOW);

    digitalWrite(MOTOR_B_A1, HIGH);
    digitalWrite(MOTOR_B_A2, LOW);

    digitalWrite(MOTOR_F_B3, LOW);
    digitalWrite(MOTOR_F_B4, HIGH);

    digitalWrite(MOTOR_B_B3, LOW);
    digitalWrite(MOTOR_B_B4, HIGH);

    ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_angular_speed);
    ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_angular_speed);
    ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_angular_speed);
    ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_angular_speed);
    break;
  case LEFT_TURN:
    digitalWrite(MOTOR_F_B3, HIGH);
    digitalWrite(MOTOR_F_B4, LOW);

    digitalWrite(MOTOR_B_B3, HIGH);
    digitalWrite(MOTOR_B_B4, LOW);

    digitalWrite(MOTOR_F_A1, LOW);
    digitalWrite(MOTOR_F_A2, HIGH);

    digitalWrite(MOTOR_B_A1, LOW);
    digitalWrite(MOTOR_B_A2, HIGH);

    ledcWrite(MOTOR_F_A_CH, START_SPEED + state.next_angular_speed);
    ledcWrite(MOTOR_F_B_CH, START_SPEED + state.next_angular_speed);
    ledcWrite(MOTOR_B_A_CH, START_SPEED + state.next_angular_speed);
    ledcWrite(MOTOR_B_B_CH, START_SPEED + state.next_angular_speed);
    break;
  default:
    break;
  }

  Serial.println("[INFO]: ANGULAR SPEED");
  Serial.println(state.next_angular_speed);

  state.speed = state.next_speed;
  state.angular_speed = state.next_angular_speed;

  if (state.next_angular_speed == MIN_ANGULAR_SPEED)
  {
    state.turn_direction = state.next_turn_direction;
  }
}

int increase_speed(int speed)
{
  if (speed == MIN_SPEED)
  {
    return MIN_SPEED + SPEED_MULTIPLIER;
  }

  const int new_speed = speed + SPEED_MULTIPLIER;
  if (new_speed >= MAX_SPEED)
  {
    return MAX_SPEED;
  }

  return new_speed;
}

int decrease_speed(int speed)
{
  const int new_speed = speed - SPEED_MULTIPLIER;
  if (new_speed <= MIN_SPEED)
  {
    return MIN_SPEED;
  }

  return new_speed;
}

int increase_angle_speed(int speed)
{
  if (speed == MIN_ANGULAR_SPEED)
  {
    return MIN_ANGULAR_SPEED + ANGULAR_SPEED_MULTIPLIER;
  }
  const int new_speed = speed + ANGULAR_SPEED_MULTIPLIER;
  if (new_speed >= MAX_ANGULAR_SPEED)
  {
    return MAX_ANGULAR_SPEED;
  }

  return new_speed;
}

int decrease_angle_speed(int speed)
{
  const int new_speed = speed - ANGULAR_SPEED_MULTIPLIER;
  if (new_speed <= MIN_ANGULAR_SPEED)
  {
    return MIN_ANGULAR_SPEED;
  }

  return new_speed;
}
