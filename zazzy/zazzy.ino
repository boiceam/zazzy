#include <Adafruit_NeoPixel.h>

#define BIKE_MODE 1
#define SIGN_MODE 0

#define RF_D3_PIN    8     // PB4
#define PWM3_PIN     9     // PB5
#define PWM2_PIN     10    // PB6
#define PWM4_PIN     11    // PB7

#define PWM1_PIN     5     // PC6
#define LOAD_PIN     13    // PC7

#define RF_D0_PIN    3     // PD0
#define RF_D1_PIN    2     // PD1
#define UART_RX_PIN  0     // PD2
#define UART_TX_PIN  1     // PD3
#define RF_PWR_PIN   4     // PD4
#define STATUS_LED_PIN  12 // PD6
#define RF_D2_PIN    6     // PD7

#define CHAN2_PIN    A5     // PF0
#define CHAN1_PIN    A4     // PF1
#define CHAN3_PIN    A3     // PF4
#define CHAN4_PIN    A1     // PF6

#define CHANNEL_COUNT  4
#define PATTERN_COUNT  1
#define STEP_PARTS     4

#define LED_COUNT 568

#define ROW_COUNT  17
#define COL_COUNT  40

const uint16_t mapping[17][6] = {
  {24, 6, 0, 0, 562, 1}, // 0
  {20, 13, 0, 0, 549, 0}, // 1
  {2, 7, 13, 17, 525, 1}, // 2
  {1, 10, 8, 19, 496, 0}, // 3
  {0, 39, 0, 0, 457, 1}, // 4
  {0, 40, 0, 0, 417, 0}, // 5
  {0, 40, 0, 0, 377, 1}, // 6
  {0, 40, 0, 0, 337, 0}, // 7
  {1, 39, 0, 0, 298, 1}, // 8
  {1, 39, 0, 0, 259, 0}, // 9
  {1, 39, 0, 0, 220, 1}, // 10
  {1, 39, 0, 0, 181, 0}, // 11
  {2, 37, 0, 0, 144, 1}, // 12
  {2, 37, 0, 0, 107, 0}, // 13
  {2, 37, 0, 0, 70, 1}, // 14
  {3, 35, 0, 0, 35, 0}, // 15
  {3, 35, 0, 0, 0, 1}, // 16
};

uint8_t mode = 0;
unsigned long last_led_update_time = 0;
uint16_t led_blink_period_ms = 0;

unsigned long next_load_update_time = 0;
unsigned long next_frame_time = 0;
unsigned long next_status_time = 0;
unsigned long last_frame_time = 0;

uint8_t channel_step[CHANNEL_COUNT];
uint8_t channel_cycle[CHANNEL_COUNT];
int16_t channel_duty[CHANNEL_COUNT];

// rise_rate, hold, fall_rate, delay
const uint8_t pattern[4][STEP_PARTS] = {
  {10, 20, 10, 20},
  {10, 10, 10, 10},
  {5, 5, 5, 5},
  {5, 10, 5, 10},
};

uint16_t cycle = 0;
uint16_t subcycle = 0;

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, CHAN1_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pinMode(RF_D0_PIN, INPUT);
  pinMode(RF_D1_PIN, INPUT);
  pinMode(RF_D2_PIN, INPUT);
  pinMode(RF_D3_PIN, INPUT);

  digitalWrite(RF_PWR_PIN, HIGH);
  pinMode(RF_PWR_PIN, OUTPUT);

  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT);

  digitalWrite(LOAD_PIN, HIGH);
  pinMode(LOAD_PIN, OUTPUT);

  digitalWrite(CHAN1_PIN, LOW);
  pinMode(CHAN1_PIN, OUTPUT);

  digitalWrite(CHAN2_PIN, LOW);
  pinMode(CHAN2_PIN, OUTPUT);

  digitalWrite(CHAN2_PIN, LOW);
  pinMode(CHAN2_PIN, OUTPUT);

  digitalWrite(CHAN2_PIN, LOW);
  pinMode(CHAN2_PIN, OUTPUT);

  digitalWrite(PWM1_PIN, HIGH);
  pinMode(PWM1_PIN, OUTPUT);
  
  digitalWrite(PWM2_PIN, HIGH);
  pinMode(PWM2_PIN, OUTPUT);

  digitalWrite(PWM3_PIN, HIGH);
  pinMode(PWM3_PIN, OUTPUT);

  digitalWrite(PWM4_PIN, HIGH);
  pinMode(PWM4_PIN, OUTPUT);

#if SIGN_MODE
  delay(2000);
  digitalWrite(PWM1_PIN, LOW);
#endif


  strip.begin();
  strip.setBrightness(255);
  strip.show();

  Serial.begin(115200);
  Serial.write("Starting Zazzy");
}

void loop() {
  unsigned long current_time = millis();

  // Process remote button input
  if (digitalRead(RF_D3_PIN) == HIGH) {
    mode = 1;
    cycle = 0;
  } else if (digitalRead(RF_D2_PIN) == HIGH) {
    mode = 2;
    cycle = 0;
  } else if (digitalRead(RF_D1_PIN) == HIGH) {
    mode = 3;
    cycle = 0;
  } else if (digitalRead(RF_D0_PIN) == HIGH) {
    mode = 0;
    cycle = 0;
  }

  // Update status led period based on mode
  if (mode == 1) {
    led_blink_period_ms = 50;
  } else if (mode == 2) {
    led_blink_period_ms = 250;
  } else if (mode == 3) {
    led_blink_period_ms = 500;
  } else {
    led_blink_period_ms = 0;
  }

  // Blink the status led
  if (led_blink_period_ms > 0) {
    if (last_led_update_time == 0 || current_time - last_led_update_time > led_blink_period_ms) {
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
      last_led_update_time = current_time;
    }
  } else {
    digitalWrite(STATUS_LED_PIN, LOW);
  }

#if BIKE_MODE
  // Apply periodic load to keep USB power bank alive
  if (next_load_update_time == 0 || current_time > next_load_update_time) {
    if (digitalRead(LOAD_PIN)) {
      digitalWrite(LOAD_PIN, LOW);
      next_load_update_time = current_time + 50;
    } else {
      digitalWrite(LOAD_PIN, HIGH);
      next_load_update_time = current_time + 5000;
    }
  }
  
  // Call frame task every 40ms (25Hz)
  if (next_frame_time == 0 || current_time > next_frame_time) {
    next_frame_time = current_time + 40;
    FrameTask();
    last_frame_time = millis() - current_time;
  }
#endif // BIKE_MODE

#if SIGN_MODE
  if (next_frame_time == 0 || current_time > next_frame_time) {
    next_frame_time = current_time + 40;
    PixelTask();
    last_frame_time = millis() - current_time;
  }
#endif // SIGN_MODE

  if (next_status_time == 0 || current_time > next_status_time) {
    next_status_time = current_time + 1000;
    Serial.print("T=");
    Serial.print(last_frame_time);
    Serial.println("ms");
  }
}

void FrameTask(void) {
  // Called every 40ms (25Hz)
  if (mode == 0) {
    analogWrite(PWM1_PIN, 255);
    analogWrite(PWM2_PIN, 255);
    analogWrite(PWM3_PIN, 255);
    analogWrite(PWM4_PIN, 255);
  } else if (mode == 1) {
    if (cycle >= 8) {
      cycle = 0;
      switch (subcycle) {
        case 0:
          analogWrite(PWM1_PIN, 0);
          analogWrite(PWM2_PIN, 255);
          analogWrite(PWM3_PIN, 255);
          analogWrite(PWM4_PIN, 255);
          subcycle++;
          break;
        case 1:
          analogWrite(PWM1_PIN, 255);
          analogWrite(PWM2_PIN, 0);
          analogWrite(PWM3_PIN, 255);
          analogWrite(PWM4_PIN, 255);
          subcycle++;
          break;
        case 2:
          analogWrite(PWM1_PIN, 255);
          analogWrite(PWM2_PIN, 255);
          analogWrite(PWM3_PIN, 0);
          analogWrite(PWM4_PIN, 255);
          subcycle++;
          break;
        default:
          analogWrite(PWM1_PIN, 255);
          analogWrite(PWM2_PIN, 255);
          analogWrite(PWM3_PIN, 255);
          analogWrite(PWM4_PIN, 0);
          subcycle = 0;
          break;
      }
    }
  } else if (mode == 2) {
    UpdateChannelState(0, 0);
    UpdateChannelState(1, 1);
    UpdateChannelState(2, 2);
    UpdateChannelState(3, 3);
  } else if (mode == 3) {
    analogWrite(PWM1_PIN, 0);
    analogWrite(PWM2_PIN, 0);
    analogWrite(PWM3_PIN, 0);
    analogWrite(PWM4_PIN, 0);
  }
  cycle++;
}

void UpdateChannelState(uint8_t channel, uint8_t pattern_id) {
  switch (channel_step[channel]) {
    case 0:
      channel_duty[channel] = channel_duty[channel] - pattern[pattern_id][0];
      channel_cycle[channel] = 0;
      if (channel_duty[channel] <= 0) {
        channel_duty[channel] = 0;
        channel_step[channel]++;
      }
      break;
    case 1:
      channel_cycle[channel]++;
      if (channel_cycle[channel] >= pattern[pattern_id][1]) {
        channel_cycle[channel] = 0;
        channel_step[channel]++;
      }
      break;
    case 2:
      channel_duty[channel] = channel_duty[channel] + pattern[pattern_id][2];
      channel_cycle[channel] = 0;
      if (channel_duty[channel] >= 255) {
        channel_duty[channel] = 255;
        channel_step[channel]++;
      }
      break;
    case 3:
      channel_cycle[channel]++;
      if (channel_cycle[channel] >= pattern[pattern_id][3]) {
        channel_cycle[channel] = 0;
        channel_step[channel] = 0;
      }
      break;
  }

  switch (channel) {
    case 0:
      analogWrite(PWM1_PIN, channel_duty[channel]);
      break;
    case 1:
      analogWrite(PWM2_PIN, channel_duty[channel]);
      break;
    case 2:
      analogWrite(PWM3_PIN, channel_duty[channel]);
      break;
    case 3:
      analogWrite(PWM4_PIN, channel_duty[channel]);
      break;
  }
}

void PixelTask() {
  // Called every 40ms (25Hz)
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;

  for (size_t i = 0; i < strip.numPixels(); i++) {
    if (mode == 1) {
      strip.setPixelColor(i, GetWheelColor(((i * 256 / strip.numPixels()) + cycle) & 255));
    } else if (mode == 3) {
      strip.setPixelColor(i, GetWheelColor(cycle));
    } else {
      strip.setPixelColor(i, 0);
    }
  }

  if (mode == 2) {
    ShaderA();
  }

  cycle++;
  if (cycle == 255) {
    cycle = 0;
  }

  strip.show();
}

void ShaderA(void) {
  uint8_t y = 0;
  uint8_t x = 0;
  int16_t location = -1;
  for (y = 0; y < ROW_COUNT; y++) {
    for (x = 0; x < COL_COUNT; x++) {
      location = GetPixelLocation(x, y);
      if (location != -1) {
        strip.setPixelColor(location, strip.Color(y * 10, x * 5, 0));
      }
    }
  }
}

int16_t GetPixelLocation(uint8_t x, uint8_t y) {
  int16_t location = 0;
  uint8_t row_offset = 0;

  uint8_t offset_1 = mapping[y][0];
  uint8_t size_1 = mapping[y][1];
  uint8_t gap = mapping[y][2];
  uint8_t size_2 = mapping[y][3];
  uint16_t strip_offset = mapping[y][4];
  bool reversed = mapping[y][5];

  uint8_t row_size = size_1 + size_2;
  uint8_t max_1 = offset_1 + size_1;
  uint8_t offset_2 = max_1 + gap;
  uint8_t max_2 = offset_2 + size_2;

  if (x >= offset_1 && (x < max_1) ) {
    row_offset = x - offset_1;
  } else if (x >= offset_2 && (x < max_2) ) {
    row_offset = x - offset_2 + size_1;
  } else {
    // Not a valid location
    return -1;
  }

  if (reversed) {
    location = (strip_offset + row_size - 1 - row_offset);
  } else {
    location = (strip_offset + row_offset);
  }

  return location;
}

// Input a value 0 to 255 to get a color value.
// The colors transition r -> g -> b -> r
uint32_t GetWheelColor(uint8_t position) {
  position = 255 - position;
  if (position < 85) {
    return strip.Color(255 - position * 3, 0, position * 3);
  } else if (position < 170) {
    position -= 85;
    return strip.Color(0, position * 3, 255 - position * 3);
  } else {
    position -= 170;
    return strip.Color(position * 3, 255 - position * 3, 0);
  }
}
