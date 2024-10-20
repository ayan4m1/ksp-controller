// #define ASSIGNABLE_I2C_PINS
#define ASSIGNABLE_SPI_PINS

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
#include "ili9341.hpp"

#define PIN_BUTTON 3
#define PIN_ADC_ALRT 0
#define PIN_ADC_SCL 21
#define PIN_ADC_SDA 20

// #define PIN_OLED_SCL 19
// #define PIN_OLED_SDA 18

#define PIN_LCD_CLK 2
#define PIN_LCD_MOSI 3
#define PIN_LCD_MISO 4
#define PIN_LCD_CS 5
#define PIN_LCD_DC 10
#define PIN_LCD_RST 11
#define PIN_LCD_BL 12

#define ADC_CHANNELS 4
#define ADC_SAMPLE_RATE 860
#define ADC_CHANNEL_SAMPLE_RATE ADC_SAMPLE_RATE / (ADC_CHANNELS * 1.0f)
// #define
#define ADC_I2C_BUS_SPEED_HZ 400000

// #define OLED_WIDTH 128
// #define OLED_HEIGHT 64
// #define OLED_ROTATION 2
// #define OLED_BIT_DEPTH 4
// #define OLED_I2C_ADDRESS 0xBC

#define LCD_WIDTH 320
#define LCD_HEIGHT 240
#define LCD_ROTATION 1

#define TOTAL_CALIBRATION_SAMPLES 128
#define TOTAL_POSITIONS 5
#define TOTAL_STICKS 2

#define CHANNEL_X1 0
#define CHANNEL_X2 2
#define CHANNEL_Y1 1
#define CHANNEL_Y2 3

#define INVERT_X1 true
#define INVERT_X2 true
#define INVERT_Y1 false
#define INVERT_Y2 false

#define POSITION_TOP 0
#define POSITION_RIGHT 1
#define POSITION_BOTTOM 2
#define POSITION_LEFT 3

#define STICK_DEADBAND 100
#define STICK_MAX_VALUE 18000

#define ROTATION_PITCH 0
#define ROTATION_YAW 1
#define TRANSLATION_X 0
#define TRANSLATION_Y 1

using namespace arduino;
using namespace gfx;

// using bus_type = tft_i2c_ex<1, PIN_OLED_SDA, PIN_OLED_SCL>;
// using lcd_type = ssd1306<OLED_WIDTH, OLED_HEIGHT, bus_type, OLED_ROTATION,
//                          OLED_BIT_DEPTH, OLED_I2C_ADDRESS>;

using bus_type = tft_spi_ex<0, PIN_LCD_CS, PIN_LCD_MOSI, PIN_LCD_MISO,
                            PIN_LCD_CLK, SPI_MODE0>;
using lcd_type = ili9341<PIN_LCD_DC, PIN_LCD_RST, PIN_LCD_BL, bus_type,
                         LCD_ROTATION, true, 400, 200>;

// using bus_type = tft_spi_ex<0, PIN_LCD_CS, PIN_LCD_MOSI, PIN_LCD_MISO,
//                             PIN_LCD_CLK, SPI_MODE0>;
// using lcd_type = st7789<LCD_WIDTH, LCD_HEIGHT, PIN_LCD_DC, PIN_LCD_RST,
//                         PIN_LCD_BL, bus_type, LCD_ROTATION, false>;

lcd_type lcd;

constexpr static const size16 bmp_size(LCD_WIDTH, LCD_HEIGHT);
using bmp_type = bitmap<decltype(lcd)::pixel_type>;
using bmp_color = color<typename bmp_type::pixel_type>;
uint8_t bmp_buf[bmp_type::sizeof_buffer(bmp_size)];
bmp_type bmp(bmp_size, bmp_buf);

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

const srect16 stickRects[2] = {
    (srect16)rect16(0, 0, 96, 96).center(lcd.bounds()).offset(-64, 0),
    (srect16)rect16(0, 0, 96, 96).center(lcd.bounds()).offset(64, 0)};

uint32_t muxChannels[ADC_CHANNELS] = {
    ADS1X15_REG_CONFIG_MUX_SINGLE_0, ADS1X15_REG_CONFIG_MUX_SINGLE_1,
    ADS1X15_REG_CONFIG_MUX_SINGLE_2, ADS1X15_REG_CONFIG_MUX_SINGLE_3};
int16_t channelValues[ADC_CHANNELS] = {0, 0, 0, 0};
uint8_t channel = 0;

uint16_t currentTranlation[2] = {0, 0};
uint16_t currentRotation[2] = {0, 0};
float currentAltitude = 0;
float currentSurfaceVelocity = 0;

const open_font &fnt = MicroGrotesk;
const float fontScale = fnt.scale(32);

const char *calibratingMsg = "Calibrating...";
const char *adcErrorMsg = "ADC Init Error!";
const char *connectingMsg = "Waiting for KSP...";

bool initialized = false;
bool calibrating = true;
// bool waitingForButtonPress = true;

// srect16 bounds[TOTAL_STICKS] = {srect16(0, 0, 0, 0), srect16(0, 0, 0, 0)};
spoint16 centers[TOTAL_STICKS];
// int16_t calibrationChannels[ADC_CHANNELS];
int16_t calibrationSamples[TOTAL_CALIBRATION_SAMPLES];
// 0 = center, 1-4 = corners
// int8_t calibrationPositionIndex = -1;
uint8_t calibrationChannelIndex = 0;
uint16_t calibrationSampleIndex = 0;

KerbalSimpit kerbal(Serial);
Adafruit_ADS1115 adc;
Button2 calibrateBtn;

void drawScreen() { draw::bitmap(lcd, lcd.bounds(), bmp, bmp.bounds()); }

int scale(int value, bool invert) {
  return map(value, 0, STICK_MAX_VALUE, invert ? INT16_MAX : INT16_MIN,
             invert ? INT16_MIN : INT16_MAX);
}

// void pressCalibration(Button2 &btn) {
//   /*if (!calibrated && !calibrating) {
//     calibrating = true;
//     adc.startADCReading(muxChannels[calibrationChannelIndex], false);
//   } else if (waitingForButtonPress &&
//              calibrationPositionIndex < TOTAL_POSITIONS) {
//     waitingForButtonPress = false;
//     calibrationPositionIndex++;
//     calibrationChannelIndex = 0;
//     calibrationSampleIndex = 0;

//     adc.startADCReading(muxChannels[calibrationChannelIndex], false);
//   }*/
//   if (!calibrated && !calibrating) {
//     calibrating = true;
//     adc.startADCReading(muxChannels[calibrationChannelIndex], false);
//   }
// }

void messageHandler(byte messageType, byte message[], byte messageSize) {
  switch (messageType) {
    case ROTATION_DATA_MESSAGE: {
      if (messageSize != sizeof(vesselPointingMessage)) {
        return;
      }

      vesselPointingMessage msg = parseMessage<vesselPointingMessage>(message);

      break;
    }
    case ORBIT_MESSAGE: {
      if (messageSize != sizeof(orbitInfoMessage)) {
        return;
      }

      orbitInfoMessage msg = parseMessage<orbitInfoMessage>(message);

      break;
    }
    case AIRSPEED_MESSAGE: {
      if (messageSize != sizeof(airspeedMessage)) {
        return;
      }

      airspeedMessage msg = parseMessage<airspeedMessage>(message);

      break;
    }
    case FLIGHT_STATUS_MESSAGE: {
      if (messageSize != sizeof(flightStatusMessage)) {
        return;
      }

      flightStatusMessage msg = parseMessage<flightStatusMessage>(message);

      break;
    }
    case ALTITUDE_MESSAGE: {
      if (messageSize != sizeof(altitudeMessage)) {
        return;
      }

      altitudeMessage msg = parseMessage<altitudeMessage>(message);

      currentAltitude = msg.sealevel;
      break;
    }
    case VELOCITY_MESSAGE: {
      if (messageSize != sizeof(velocityMessage)) {
        return;
      }

      velocityMessage msg = parseMessage<velocityMessage>(message);

      currentSurfaceVelocity = msg.surface;
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

void connectToSimpit() {
  srect16 textRect = srect16(spoint16::zero(),
                             fnt.measure_text(ssize16::max(), spoint16::zero(),
                                              connectingMsg, fontScale));
  rect16 textPos = (rect16)textRect.center((srect16)lcd.bounds());

  bool state = false;
  while (!kerbal.init()) {
    bmp.fill(bmp.bounds(),
             state ? bmp_color::dark_gray : bmp_color::slate_blue);

    draw::text(bmp, textPos, spoint16::zero(), connectingMsg, fnt, fontScale,
               bmp_color::white);

    drawScreen();

    state = !state;
    delay(500);
  }

  bmp.fill(bmp.bounds(), bmp_color::purple);

  kerbal.inboundHandler(messageHandler);
  kerbal.registerChannel(ORBIT_MESSAGE);
  kerbal.registerChannel(AIRSPEED_MESSAGE);
  kerbal.registerChannel(ALTITUDE_MESSAGE);
  kerbal.registerChannel(VELOCITY_MESSAGE);
  kerbal.registerChannel(FLIGHT_STATUS_MESSAGE);
  kerbal.registerChannel(ROTATION_DATA_MESSAGE);
  kerbal.printToKSP(F("Controller connected!"), PRINT_TO_SCREEN);
}

void setup() {
  Serial.begin(115200);

  lcd.initialize();
  lcd.fill(lcd.bounds(), color<decltype(lcd)::pixel_type>::purple);

  // calibrateBtn.setPressedHandler(pressCalibration);
  // calibrateBtn.begin(PIN_BUTTON);

  Wire.setClock(ADC_I2C_BUS_SPEED_HZ);
  Wire.setSCL(PIN_ADC_SCL);
  Wire.setSDA(PIN_ADC_SDA);
  Wire.begin();

  if (!adc.begin()) {
    srect16 textRect = srect16(
        spoint16::zero(), fnt.measure_text(ssize16::max(), spoint16::zero(),
                                           adcErrorMsg, fontScale));
    rect16 textPos = (rect16)textRect.center((srect16)lcd.bounds());

    draw::text(bmp, textPos, spoint16::zero(), adcErrorMsg, fnt, fontScale,
               bmp_color::white);
    return;
  } else {
    srect16 textRect = srect16(
        spoint16::zero(), fnt.measure_text(ssize16::max(), spoint16::zero(),
                                           calibratingMsg, fontScale));
    rect16 textPos = (rect16)textRect.center((srect16)lcd.bounds());

    draw::text(bmp, textPos, spoint16::zero(), calibratingMsg, fnt, fontScale,
               bmp_color::white);
  }

  drawScreen();
  initialized = true;

  adc.setGain(GAIN_TWOTHIRDS);
  adc.setDataRate(RATE_ADS1115_860SPS);
  adc.startADCReading(muxChannels[channel], false);
}

void loop() {
  if (!initialized) {
    return;
  }

  // kerbal.update();
  // calibrateBtn.loop();

  if (!adc.conversionComplete()) {
    return;
  }

  if (calibrating) {
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
        channel = 0;
        // connectToSimpit();
        adc.startADCReading(muxChannels[channel], false);
      } else {
        adc.startADCReading(muxChannels[calibrationChannelIndex], false);
      }
    }
  } else {
    const auto result = adc.getLastConversionResults();

    channelValues[channel++] = result;
    if (channel == ADC_CHANNELS) {
      channel = 0;

      /*bool needRotation = false, needTranslation = false;
      rotationMessage rotation;
      translationMessage translation;

      if (channelValues[CHANNEL_Y2] > centers[1].y + STICK_DEADBAND ||
          channelValues[CHANNEL_Y2] < centers[1].y - STICK_DEADBAND) {
        auto newPitch = scale(channelValues[CHANNEL_Y2]);

        if (abs(newPitch - currentRotation[ROTATION_PITCH]) > STICK_DEADBAND) {
          needRotation = true;
          currentRotation[ROTATION_PITCH] = newPitch;
          rotation.setPitch(newPitch);
        }
      }

      if (channelValues[CHANNEL_X2] > centers[1].x + STICK_DEADBAND ||
          channelValues[CHANNEL_X2] < centers[1].x - STICK_DEADBAND) {
        auto newYaw = scale(channelValues[CHANNEL_X2]);

        if (abs(newYaw - currentRotation[ROTATION_YAW]) > STICK_DEADBAND) {
          needRotation = true;
          currentRotation[ROTATION_YAW] = newYaw;
          rotation.setYaw(newYaw);
        }
      }

      if (channelValues[CHANNEL_Y1] > centers[0].y + STICK_DEADBAND ||
          channelValues[CHANNEL_Y1] < centers[0].y - STICK_DEADBAND) {
        auto newY = scale(channelValues[CHANNEL_Y1]);

        if (abs(newY - currentTranlation[TRANSLATION_Y]) > STICK_DEADBAND) {
          needTranslation = true;
          currentTranlation[TRANSLATION_Y] = newY;
          translation.setY(newY);
        }
      }

      if (channelValues[CHANNEL_X1] > centers[0].x + STICK_DEADBAND ||
          channelValues[CHANNEL_X1] < centers[0].x - STICK_DEADBAND) {
        auto newX = scale(channelValues[CHANNEL_X1]);

        if (abs(newX - currentTranlation[TRANSLATION_X]) > STICK_DEADBAND) {
          needTranslation = true;
          currentTranlation[TRANSLATION_X] = newX;
          translation.setX(newX);
        }
      }

      if (needRotation) {
        kerbal.send(ROTATION_MESSAGE, rotation);
      }
      if (needTranslation) {
        kerbal.send(TRANSLATION_MESSAGE, translation);
      }*/

      rotationMessage rotationMsg;
      throttleMessage throttleMsg;

      if (channelValues[CHANNEL_Y2] > centers[1].y + STICK_DEADBAND ||
          channelValues[CHANNEL_Y2] < centers[1].y - STICK_DEADBAND) {
        rotationMsg.setPitch(scale(channelValues[CHANNEL_Y2], INVERT_Y2));
      }

      if (channelValues[CHANNEL_X2] > centers[1].x + STICK_DEADBAND ||
          channelValues[CHANNEL_X2] < centers[1].x - STICK_DEADBAND) {
        rotationMsg.setRoll(scale(channelValues[CHANNEL_X2], INVERT_X2));
      }

      if (channelValues[CHANNEL_Y1] > centers[0].y + STICK_DEADBAND ||
          channelValues[CHANNEL_Y1] < centers[0].y - STICK_DEADBAND) {
        throttleMsg.throttle = scale(channelValues[CHANNEL_Y1], INVERT_Y1);
      }

      if (channelValues[CHANNEL_X1] > centers[0].x + STICK_DEADBAND ||
          channelValues[CHANNEL_X1] < centers[0].x - STICK_DEADBAND) {
        rotationMsg.setYaw(scale(channelValues[CHANNEL_X1], INVERT_X1));
      }

      // kerbal.send(ROTATION_MESSAGE, rotationMsg);
      // kerbal.send(THROTTLE_MESSAGE, throttleMsg);

      spoint16 positionOffsets[TOTAL_STICKS] = {
          spoint16(
              stickRects[0].left() +
                  (channelValues[CHANNEL_X1] / (double)STICK_MAX_VALUE) * 96,
              stickRects[0].top() +
                  (channelValues[CHANNEL_Y1] / (double)STICK_MAX_VALUE) * 96),
          spoint16(
              stickRects[1].left() +
                  (channelValues[CHANNEL_X2] / (double)STICK_MAX_VALUE) * 96,
              stickRects[1].top() +
                  (channelValues[CHANNEL_Y2] / (double)STICK_MAX_VALUE) * 96)};

      for (auto stickIndex = 0; stickIndex < TOTAL_STICKS; stickIndex++) {
        draw::filled_ellipse(bmp, stickRects[stickIndex].inflate(2, 2),
                             bmp_color::white);
        draw::filled_ellipse(bmp, stickRects[stickIndex], bmp_color::black);

        draw::point(bmp, positionOffsets[stickIndex], bmp_color::white);
      }
      // draw::line(bmp, stickRects[0])

      // const open_font &font = MicroGrotesk;
      // char buffer[32];
      // const float scale = font.scale(32);

      // sprintf(buffer, "Alt %.0f", currentAltitude);

      // bmp.fill(bmp.bounds(), bmp_color::black);

      // srect16 textRect = srect16(
      //     spoint16::zero(),
      //     font.measure_text(ssize16::max(), spoint16::zero(), buffer,
      //     scale));
      // rect16 textPos =
      // (rect16)textRect.center_vertical((srect16)lcd.bounds())
      //                      .offset(4, -16);

      // draw::text(bmp, textPos, spoint16::zero(), buffer, font, scale,
      //            bmp_color::green);

      // sprintf(buffer, "Vel %.0f", currentSurfaceVelocity);

      // textRect = srect16(
      //     spoint16::zero(),
      //     font.measure_text(ssize16::max(), spoint16::zero(), buffer,
      //     scale));
      // textPos =
      //     (rect16)textRect.center_vertical((srect16)bmp.bounds()).offset(4,
      //     16);
      // draw::text(bmp, textPos, spoint16::zero(), buffer, font, scale,
      //            bmp_color::green);
    }

    EVERY_N_MILLIS(100) { drawScreen(); }

    adc.startADCReading(muxChannels[channel], false);
  }
}
