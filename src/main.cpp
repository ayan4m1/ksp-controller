#define ASSIGNABLE_I2C_PINS

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <Button2.h>
#include <FastLED.h>
#include <KerbalSimpit.h>
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
#define ADC_I2C_BUS_SPEED_HZ 400000
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ROTATION 2
#define OLED_BIT_DEPTH 4
#define OLED_I2C_ADDRESS 0xBC
#define TOTAL_CALIBRATION_SAMPLES 128
#define TOTAL_POSITIONS 5
#define TOTAL_STICKS 2

#define CHANNEL_X1 0
#define CHANNEL_X2 2
#define CHANNEL_Y1 1
#define CHANNEL_Y2 3

#define INVERT_X1 false
#define INVERT_X2 false
#define INVERT_Y1 false
#define INVERT_Y2 true

#define POSITION_TOP 0
#define POSITION_RIGHT 1
#define POSITION_BOTTOM 2
#define POSITION_LEFT 3

#define STICK_DEADBAND 100
#define STICK_MAX_VALUE 18000

using namespace arduino;
using namespace gfx;

using bus_type = tft_i2c_ex<1, PIN_OLED_SDA, PIN_OLED_SCL>;
using lcd_type = ssd1306<OLED_WIDTH, OLED_HEIGHT, bus_type, OLED_ROTATION,
                         OLED_BIT_DEPTH, OLED_I2C_ADDRESS>;
using lcd_color = color<typename lcd_type::pixel_type>;

lcd_type lcd;

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
int16_t channelValues[ADC_CHANNELS] = {0, 0, 0, 0};
int16_t previousValues[ADC_CHANNELS] = {0, 0, 0, 0};
uint8_t channel = 0;

float altitude = 0;
float surfaceVelocity = 0;

bool calibrated = false;
bool calibrating = true;
// bool waitingForButtonPress = true;

// srect16 bounds[TOTAL_STICKS] = {srect16(0, 0, 0, 0), srect16(0, 0, 0, 0)};
spoint16 centers[TOTAL_STICKS];
int16_t calibrationValues[ADC_CHANNELS];
int16_t calibrationSamples[TOTAL_CALIBRATION_SAMPLES];
// 0 = center, 1-4 = corners
// int8_t calibrationPositionIndex = -1;
uint8_t calibrationChannelIndex = 0;
uint16_t calibrationSampleIndex = 0;

KerbalSimpit kerbal(Serial);
Adafruit_ADS1115 adc;
Button2 calibrateBtn;

int scale(int value) {
  return map(value, 0, STICK_MAX_VALUE, INT16_MIN, INT16_MAX);
}

void pressCalibration(Button2 &btn) {
  /*if (!calibrated && !calibrating) {
    calibrating = true;
    adc.startADCReading(muxChannels[calibrationChannelIndex], false);
  } else if (waitingForButtonPress &&
             calibrationPositionIndex < TOTAL_POSITIONS) {
    waitingForButtonPress = false;
    calibrationPositionIndex++;
    calibrationChannelIndex = 0;
    calibrationSampleIndex = 0;

    adc.startADCReading(muxChannels[calibrationChannelIndex], false);
  }*/
  if (!calibrated && !calibrating) {
    calibrating = true;
    adc.startADCReading(muxChannels[calibrationChannelIndex], false);
  }
}

void messageHandler(byte messageType, byte message[], byte messageSize) {
  switch (messageType) {
    case ALTITUDE_MESSAGE: {
      if (messageSize != sizeof(altitudeMessage)) {
        return;
      }

      altitudeMessage altMsg = parseMessage<altitudeMessage>(message);

      altitude = altMsg.sealevel;
      break;
    }
    case VELOCITY_MESSAGE: {
      if (messageSize != sizeof(velocityMessage)) {
        return;
      }

      velocityMessage velMsg = parseMessage<velocityMessage>(message);

      surfaceVelocity = velMsg.surface;
      break;
    }
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
  pinMode(LED_BUILTIN, OUTPUT);
  bool state = false;
  while (!kerbal.init()) {
    digitalWrite(LED_BUILTIN, state ? HIGH : LOW);
    state = !state;
    delay(500);
  }

  kerbal.inboundHandler(messageHandler);
  kerbal.registerChannel(ALTITUDE_MESSAGE);
  kerbal.registerChannel(VELOCITY_MESSAGE);
  kerbal.printToKSP(F("Controller connected!"), PRINT_TO_SCREEN);

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

  adc.startADCReading(muxChannels[channel], false);
}

void loop() {
  kerbal.update();
  calibrateBtn.loop();

  if (calibrating) {
    if (!adc.conversionComplete()) {
      return;
    }

    calibrationSamples[calibrationSampleIndex++] =
        adc.getLastConversionResults();

    if (calibrationSampleIndex == TOTAL_CALIBRATION_SAMPLES) {
      calibrationChannelIndex++;
      calibrationSampleIndex = 0;

      switch (calibrationChannelIndex) {
        case CHANNEL_X1:
          centers[0].x = getCalibrationAverage();
          break;
        case CHANNEL_Y1:
          centers[0].y = getCalibrationAverage();
          break;
        case CHANNEL_X2:
          centers[1].x = getCalibrationAverage();
          break;
        case CHANNEL_Y2:
          centers[1].y = getCalibrationAverage();
          break;
      }

      if (calibrationChannelIndex == ADC_CHANNELS) {
        calibrating = false;
        calibrated = true;
        channel = 0;
        adc.startADCReading(muxChannels[channel], false);
      } else {
        adc.startADCReading(muxChannels[calibrationChannelIndex], false);
      }
    }
  } else if (calibrated) {
    if (!adc.conversionComplete()) {
      return;
    }

    const auto result = adc.getLastConversionResults();

    channelValues[channel++] = result;
    if (channel == ADC_CHANNELS) {
      channel = 0;

      rotationMessage rotation;
      translationMessage translation;

      if (channelValues[CHANNEL_Y2] > centers[1].y - STICK_DEADBAND ||
          channelValues[CHANNEL_Y2] < centers[1].y + STICK_DEADBAND) {
        rotation.setPitch(scale(channelValues[CHANNEL_Y2]));
      } else {
        rotation.setPitch(0);
      }

      if (channelValues[CHANNEL_X2] > centers[1].x - STICK_DEADBAND ||
          channelValues[CHANNEL_X2] < centers[1].x + STICK_DEADBAND) {
        rotation.setYaw(scale(channelValues[CHANNEL_X2]));
      } else {
        rotation.setYaw(0);
      }

      if (channelValues[CHANNEL_Y1] > centers[0].y - STICK_DEADBAND ||
          channelValues[CHANNEL_Y1] < centers[0].y + STICK_DEADBAND) {
        translation.setY(scale(channelValues[CHANNEL_Y1]));
      } else {
        translation.setY(0);
      }

      if (channelValues[CHANNEL_X1] > centers[0].x - STICK_DEADBAND ||
          channelValues[CHANNEL_X1] < centers[0].x + STICK_DEADBAND) {
        translation.setX(scale(channelValues[CHANNEL_X1]));
      } else {
        translation.setX(0);
      }

      // EVERY_N_SECONDS(5) {
      //   char buffer[128];
      //   sprintf(buffer, "%d %d %d %d", rotation.pitch, rotation.yaw,
      //           translation.X, translation.Y);
      //   kerbal.printToKSP(buffer, PRINT_TO_SCREEN);
      // }

      kerbal.send(ROTATION_MESSAGE, rotation);
      kerbal.send(TRANSLATION_MESSAGE, translation);

      // memcpy(&previousValues, channelValues, sizeof(previousValues));

      const open_font &f = MicroGrotesk;
      char buffer[32];
      const float scale = f.scale(14);

      sprintf(buffer, "Alt- %.0f", altitude);

      lcd.suspend();

      lcd.clear(lcd.bounds());

      srect16 textRect = srect16(
          spoint16::zero(),
          f.measure_text(ssize16::max(), spoint16::zero(), buffer, scale));
      rect16 textPos =
          (rect16)textRect.center((srect16)lcd.bounds()).offset(0, -7);

      draw::text(lcd, textPos, spoint16::zero(), buffer, f, scale,
                 lcd_color::white);

      sprintf(buffer, "Vel- %.0f", surfaceVelocity);

      textRect = srect16(
          spoint16::zero(),
          f.measure_text(ssize16::max(), spoint16::zero(), buffer, scale));
      textPos = (rect16)textRect.center((srect16)lcd.bounds()).offset(0, 7);
      draw::text(lcd, textPos, spoint16::zero(), buffer, f, scale,
                 lcd_color::white);

      lcd.resume();
    }

    adc.startADCReading(muxChannels[channel], false);
  }
}
