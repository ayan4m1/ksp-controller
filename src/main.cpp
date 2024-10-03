#define ASSIGNABLE_I2C_PINS

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <Button2.h>
#include <SPI.h>

#include <gfx_cpp14.hpp>
#include <tft_io.hpp>

#define MICROGROTESK_IMPLEMENTATION

#include "MicroGrotesk.hpp"
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
#define OLED_ROTATION 2
#define OLED_BIT_DEPTH 4
#define OLED_I2C_ADDRESS 0xBC
#define TOTAL_CALIBRATION_SAMPLES 128
#define TOTAL_POSITIONS 5
#define TOTAL_STICKS 2

#define CHANNEL_X1 1
#define CHANNEL_X2 3
#define CHANNEL_Y1 0
#define CHANNEL_Y2 2

#define POSITION_TOP 0
#define POSITION_RIGHT 1
#define POSITION_BOTTOM 2
#define POSITION_LEFT 3

using namespace arduino;
using namespace gfx;

using bus_type = tft_i2c_ex<1, PIN_OLED_SDA, PIN_OLED_SCL>;
using lcd_type = ssd1306<OLED_WIDTH, OLED_HEIGHT, bus_type, OLED_ROTATION,
                         OLED_BIT_DEPTH, OLED_I2C_ADDRESS>;
using lcd_color = color<typename lcd_type::pixel_type>;

lcd_type lcd;

/*const srect16 positionRects[TOTAL_POSITIONS] = {
    srect16(spoint16((lcd.bounds().width() / 2) - 4,
                     (lcd.bounds().height() / 2) - 4),
            ssize16(16, 16)),
    srect16(spoint16(lcd.bounds().top_left()), ssize16(16, 16)),
    srect16(spoint16(lcd.bounds().top_right().offset(-16, 0)), ssize16(16, 16)),
    srect16(spoint16(lcd.bounds().bottom_right().offset(-16, -16)),
            ssize16(16, 16)),
    srect16(spoint16(lcd.bounds().bottom_left().offset(0, -16)),
            ssize16(16, 16))};*/
const uint8_t rectHeight = 8;
const uint8_t rectWidth = 16;
const srect16 positionRects[TOTAL_POSITIONS] = {
    srect16(spoint16((lcd.bounds().width() / 2) - 4,
                     (lcd.bounds().height() / 2) - 4),
            ssize16(16, 16)),
    srect16(
        spoint16((lcd.bounds().width() - rectWidth) / 2, lcd.bounds().top()),
        ssize16(rectWidth, rectHeight)),
    srect16(spoint16(lcd.bounds().right() - rectHeight,
                     (lcd.bounds().height() - rectWidth) / 2),
            ssize16(rectHeight, rectWidth)),
    srect16(spoint16((lcd.bounds().width() - rectWidth) / 2,
                     lcd.bounds().bottom() - rectHeight),
            ssize16(rectWidth, rectHeight)),
    srect16(spoint16(rectHeight, (lcd.bounds().height() - rectWidth) / 2),
            ssize16(rectHeight, rectWidth))};

uint32_t muxChannels[ADC_CHANNELS] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0, ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2, ADS1X15_REG_CONFIG_MUX_SINGLE_3};
int16_t channelValues[ADC_CHANNELS];
volatile uint8_t channel = 0;

bool calibrated = false;
volatile bool calibrating = false;
volatile bool waitingForButtonPress = true;

srect16 bounds[TOTAL_STICKS] = {srect16(0, 0, 0, 0), srect16(0, 0, 0, 0)};
spoint16 centers[TOTAL_STICKS];
int16_t calibrationValues[ADC_CHANNELS];
int16_t calibrationSamples[TOTAL_CALIBRATION_SAMPLES];
// 0 = center, 1-4 = corners
int8_t calibrationPositionIndex = -1;
uint8_t calibrationChannelIndex = 0;
uint16_t calibrationSampleIndex = 0;

Adafruit_ADS1115 adc;
uint32_t startTime;
Button2 calibrateBtn;

void pressCalibration(Button2 &btn) {
  if (!calibrating) {
    calibrating = true;
  }

  if (waitingForButtonPress && calibrationPositionIndex < TOTAL_POSITIONS) {
    waitingForButtonPress = false;
    calibrationPositionIndex++;
    calibrationChannelIndex = 0;
    calibrationSampleIndex = 0;

    lcd.clear(lcd.bounds());
    draw::filled_rectangle(lcd, positionRects[calibrationPositionIndex],
                           lcd_color::white);
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

  lcd.initialize();
  lcd.clear(lcd.bounds());

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

  // adc.startADCReading(muxChannels[channel], false);
}

void loop() {
  calibrateBtn.loop();

  if (calibrating) {
    if (!adc.conversionComplete()) {
      return;
    } else if (!waitingForButtonPress) {
      calibrationSamples[calibrationSampleIndex++] =
          adc.getLastConversionResults();

      if (calibrationSampleIndex == TOTAL_CALIBRATION_SAMPLES) {
        uint32_t average = getCalibrationAverage();

        calibrationSampleIndex = 0;

        if (calibrationChannelIndex < ADC_CHANNELS) {
          calibrationValues[calibrationChannelIndex++] = average;
        } else {
          if (calibrationPositionIndex == 0) {
            centers[0] = spoint16(calibrationValues[CHANNEL_X1],
                                  calibrationValues[CHANNEL_Y1]);
            centers[1] = spoint16(calibrationValues[CHANNEL_X2],
                                  calibrationValues[CHANNEL_Y2]);
          } else {
            switch (calibrationPositionIndex - 1) {
              case POSITION_TOP:
                bounds[0].y1 = calibrationValues[CHANNEL_Y1];
                bounds[1].y1 = calibrationValues[CHANNEL_Y2];
                break;
              case POSITION_RIGHT:
                bounds[0].x2 = calibrationValues[CHANNEL_X1];
                bounds[1].x2 = calibrationValues[CHANNEL_X2];
                break;
              case POSITION_BOTTOM:
                bounds[0].y2 = calibrationValues[CHANNEL_Y1];
                bounds[1].y2 = calibrationValues[CHANNEL_Y2];
                break;
              case POSITION_LEFT:
                bounds[0].x1 = calibrationValues[CHANNEL_X1];
                bounds[1].x1 = calibrationValues[CHANNEL_X2];
                break;
            }
          }

          if (calibrationPositionIndex == TOTAL_POSITIONS) {
            calibrating = false;
            calibrated = true;

            Serial.println(F("!!! Calibration Complete !!!\n"));
            for (uint8_t i = 0; i < TOTAL_STICKS; i++) {
              Serial.printf("Stick %d\n", i);
              Serial.printf("    Center X:%d Y:%d\n", centers[i].x,
                            centers[i].y);
              Serial.printf("    Bounds T:%d R:%d B:%d L:%d\n", bounds[i].y1,
                            bounds[i].x2, bounds[i].y2, bounds[i].x1);
            }
          } else {
            waitingForButtonPress = true;
          }
        }
      }

      if (!waitingForButtonPress) {
        adc.startADCReading(muxChannels[calibrationChannelIndex], false);
      }
      return;
    }
  } else if (calibrated) {
    if (!adc.conversionComplete()) {
      return;
    }

    const auto result = adc.getLastConversionResults();

    channelValues[channel++] = result;
    if (channel == ADC_CHANNELS) {
      const open_font &f = MicroGrotesk;
      char buffer[32];
      const float scale = f.scale(14);

      sprintf(buffer, "%d %d", channelValues[0], channelValues[1]);

      lcd.suspend();

      lcd.clear(lcd.bounds());

      srect16 textRect = srect16(
          spoint16::zero(),
          f.measure_text(ssize16::max(), spoint16::zero(), buffer, scale));
      rect16 textPos =
          (rect16)textRect.center((srect16)lcd.bounds()).offset(0, -7);

      draw::text(lcd, textPos, spoint16::zero(), buffer, f, scale,
                 lcd_color::white);

      sprintf(buffer, "%d %d", channelValues[2], channelValues[3]);

      textRect = srect16(
          spoint16::zero(),
          f.measure_text(ssize16::max(), spoint16::zero(), buffer, scale));
      textPos = (rect16)textRect.center((srect16)lcd.bounds()).offset(0, 7);
      draw::text(lcd, textPos, spoint16::zero(), buffer, f, scale,
                 lcd_color::white);

      lcd.resume();

      delay(25);
      channel = 0;
    }

    adc.startADCReading(muxChannels[channel], false);
  }
}
