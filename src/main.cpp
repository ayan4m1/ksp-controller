#define ASSIGNABLE_I2C_PINS

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <Button2.h>
#include <SPI.h>

#include <gfx_cpp14.hpp>
#include <tft_io.hpp>

#include "ssd1306.hpp"

#define PIN_BUTTON 3
#define PIN_ADC_ALRT 0
#define PIN_ADC_SCL 21
#define PIN_ADC_SDA 20
#define PIN_OLED_SCL 19
#define PIN_OLED_SDA 18
#define ADC_CHANNELS 4
#define ADC_SAMPLE_RATE 860
#define ADC_CHANNEL_SAMPLE_RATE ADC_SAMPLE_RATE / (ADC_CHANNELS * 1.0f)
// #define
#define ADC_I2C_BUS_SPEED_HZ 800000
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ROTATION 0
#define OLED_BIT_DEPTH 4
#define OLED_I2C_ADDRESS 0xBC
#define TOTAL_CALIBRATION_SAMPLES 128
#define TOTAL_POSITIONS 5
#define TOTAL_STICKS 2

using namespace arduino;
using namespace gfx;

using bus_type = tft_i2c_ex<1, PIN_OLED_SDA, PIN_OLED_SCL>;
using lcd_type = ssd1306<OLED_WIDTH, OLED_HEIGHT, bus_type, OLED_ROTATION,
                         OLED_BIT_DEPTH, OLED_I2C_ADDRESS>;
using lcd_color = color<typename lcd_type::pixel_type>;

lcd_type lcd;
const srect16 positionRects[5] = {
    srect16(spoint16(lcd.bounds().center(lcd.bounds()).offset(-8, -8).x1,
                     lcd.bounds().center(lcd.bounds()).offset(-8, -8).y1),
            ssize16(16, 16)),
    srect16(spoint16(lcd.bounds().top_left()),
            ssize16(lcd.bounds().top_left().offset(16, 16).x,
                    lcd.bounds().top_left().offset(16, 16).y)),
    srect16(spoint16(lcd.bounds().top_right()),
            ssize16(lcd.bounds().top_right().offset(-16, 16).x,
                    lcd.bounds().top_right().offset(-16, 16).y)),
    srect16(spoint16(lcd.bounds().bottom_right()),
            ssize16(lcd.bounds().bottom_right().offset(-16, -16).x,
                    lcd.bounds().bottom_right().offset(-16, -16).y)),
    srect16(spoint16(lcd.bounds().top_right()),
            ssize16(lcd.bounds().bottom_left().offset(16, -16).x,
                    lcd.bounds().bottom_left().offset(16, -16).y))};

uint32_t muxChannels[ADC_CHANNELS] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0, ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2, ADS1X15_REG_CONFIG_MUX_SINGLE_3};
int16_t sample[ADC_CHANNELS];
volatile uint8_t channel = 0;

volatile bool calibrating = false;
volatile bool calibrateNextChannel = false;
volatile bool calibrateNextPosition = false;

srect16 bounds[TOTAL_STICKS];
spoint16 centers[TOTAL_STICKS];
int16_t calibrationValues[4];
int16_t calibrationSamples[TOTAL_CALIBRATION_SAMPLES];
// 0 = center, 1-4 = corners
uint8_t calibrationPositionIndex = 0;
uint8_t calibrationChannelIndex = 0;
uint16_t calibrationSampleIndex = 0;

Adafruit_ADS1115 adc;
uint32_t startTime;
Button2 calibrateBtn;

void pressCalibration(Button2 &btn) {
  if (!calibrating) {
    calibrating = true;
    return;
  }

  if (calibrationSampleIndex == TOTAL_CALIBRATION_SAMPLES &&
      calibrationChannelIndex == ADC_CHANNELS &&
      calibrationPositionIndex < TOTAL_POSITIONS) {
    calibrateNextPosition = true;
  }
}

uint32_t getCalibrationAverage() {
  uint32_t total = 0;

  for (uint16_t i = 0; i < TOTAL_CALIBRATION_SAMPLES; i++) {
    total += calibrationSamples[i];
  }

  return total / TOTAL_CALIBRATION_SAMPLES;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }
  Serial.println(F("Joystick Demo"));

  Serial.printf("INIT %d", lcd.initialize());

  lcd.clear(lcd.bounds());
  draw::filled_rectangle(lcd, positionRects[calibrationPositionIndex],
                         lcd_color::white);

  calibrateBtn.setPressedHandler(pressCalibration);
  calibrateBtn.begin(PIN_BUTTON);

  Wire.setClock(ADC_I2C_BUS_SPEED_HZ);
  Wire.setSCL(PIN_ADC_SCL);
  Wire.setSDA(PIN_ADC_SDA);
  Wire.begin();

  if (!adc.begin()) {
    Serial.println(F("Fatal Error Init ADC"));
    while (1) {
    }
  }

  adc.setGain(GAIN_TWOTHIRDS);
  adc.setDataRate(RATE_ADS1115_860SPS);

  startTime = micros();

  adc.startADCReading(muxChannels[channel], false);
}

void loop() {
  calibrateBtn.loop();

  if (!adc.conversionComplete()) {
    return;
  } else if (calibrating) {
    calibrationSamples[calibrationSampleIndex++] =
        adc.getLastConversionResults();

    if (calibrationSampleIndex == TOTAL_CALIBRATION_SAMPLES) {
      uint32_t average = getCalibrationAverage();

      calibrationSampleIndex = 0;

      if (calibrationChannelIndex < ADC_CHANNELS) {
        calibrationValues[calibrationChannelIndex++] = average;
      } else {
        if (calibrationPositionIndex == 0) {
          centers[calibrationPositionIndex] =
              spoint16(calibrationValues[0], calibrationValues[1]);
        } else {
          bounds[calibrationPositionIndex] = srect16(0, 0, 1, 1);
        }

        draw::filled_rectangle(lcd, lcd.bounds().center(rect16(0, 0, 32, 12)),
                               lcd_color::black);
        // draw::text();
      }

      if (calibrateNextPosition) {
        Serial.println(F("Reset cNP"));
        calibrateNextPosition = false;
        if (calibrationPositionIndex < TOTAL_POSITIONS) {
          Serial.println(F("Increment cPI"));
          calibrationPositionIndex++;
          calibrationChannelIndex = 0;

          lcd.clear(lcd.bounds());
          draw::filled_rectangle(lcd, positionRects[calibrationPositionIndex],
                                 lcd_color::white);
        } else {
          // print final results

          Serial.printf("Center S1:%dx%d S2:%dx%d\n", centers[0], centers[1],
                        centers[2], centers[3]);
          for (uint8_t i = 0; i < TOTAL_POSITIONS - 1; i++) {
            Serial.printf("Corner %d S1:%dx%d S2:%dx%d\n", i, bounds[i][0],
                          bounds[i][1], bounds[i][2], bounds[i][3]);
          }
        }
      }
    }

    adc.startADCReading(muxChannels[calibrationChannelIndex], false);
    return;
  }

  const auto result = adc.getLastConversionResults();

  sample[channel++] = result;
  if (channel == ADC_CHANNELS) {
    // Serial.printf("Took %d microseconds\n", micros() - startTime);
    // Serial.printf("X1:%d Y1:%d  X1:%d Y2:%d\n", sample[0], sample[1],
    // sample[2],
    //               sample[3]);

    startTime = micros();
    channel = 0;
  }

  adc.startADCReading(muxChannels[channel], false);
}
