#include "Arduino.h"
#include "SD.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include <driver/i2s.h>
#include "SPIFFS.h"
#include <EEPROM.h>
#include <NeoPixelBus.h>

uint8_t VOLUME = 7; // DEFAULT = 7 // 0..255

const uint8_t SONGS_PER_BUTTON = 7;
const uint8_t SONGS_COUNT = SONGS_PER_BUTTON * 5;
#define DISCHARGED_STATE 2000 // #define FULLYCHARGED_STATE 2400

uint8_t _startSongId; // 0..SONGS_COUNT-1
uint8_t _currentInput;
uint8_t _rootFolder, _buttonFolder, _songIndex;
uint8_t _previousButtonFolder = 255;

#define EEPROM_SIZE 1

#define RedButton GPIO_NUM_0
#define YellowButton GPIO_NUM_4
#define GreenButton GPIO_NUM_18
#define WhiteButton GPIO_NUM_19
#define BlueButton GPIO_NUM_32

#define I2SR (i2s_port_t)0
#define PW GPIO_NUM_21   // Amp power ON
#define GAIN GPIO_NUM_23 //
#define BLOCK_SIZE 128
#define SD_CS 13
#define I2S_DOUT 26
#define I2S_BCLK 5
#define I2S_LRC 25
#define I2S_DIN 35
#define SPI_MOSI 15
#define SPI_MISO 2
#define SPI_SCK 14

typedef bool (*vExitPredicate)();

const i2s_config_t i2s_configR = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX), // Receive, transfer
    .sample_rate = 44100,                                            //
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                    //
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,                    // although the SEL config should be left, it seems to transmit on right
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
    .dma_buf_count = 64,                      // number of buffers
    .dma_buf_len = BLOCK_SIZE                 // samples per buffer
};

i2s_pin_config_t pin_configR =
    {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_DIN};

// NeoPixel led control
#define PixelCount 1
#define PixelPin 22
RgbColor BATTERYLOW(255, 0, 0);
RgbColor BATTERYCHARGED(0, 3, 0);
RgbColor BATTERYCHARGING(0, 128, 0);
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

static void playAudio(char *fileName, vExitPredicate exitPredicate)
{
  File file;
  static int16_t buffer[1024];
  int bufferSize;
  int16_t buffer64[64];

  Serial.printf("Opening file ");
  Serial.println(fileName);

  file = SD.open(fileName, FILE_READ);
  if (file == NULL)
  {
    Serial.println("Error opening file");
    return;
  }

  i2s_start(I2SR);
  i2s_set_clk(I2SR, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

  file.read((uint8_t *)buffer, 44);

  do
  {
    bufferSize = (int)file.read((uint8_t *)buffer, 1024 * 2);
    if (bufferSize < 0)
    {
      Serial.println("Error reading file");
      goto stopPlaying;
    }

    for (int i = 0; i < bufferSize / 2; i++)
    {
      buffer64[i % 64] = buffer[i] * VOLUME >> 8;
      if (i % 64 == 63)
      {
        i2s_write_bytes(I2SR, (const char *)buffer64, 128, portMAX_DELAY);
      }
      if (exitPredicate != NULL && exitPredicate())
      {
        goto stopPlaying;
      }
    }

  } while (bufferSize > 0);

stopPlaying:
  for (int i = 0; i < 64; i++)
    buffer64[i] = 0;
  int bytesWritten = 0;
  while (bytesWritten == 0)
    bytesWritten = i2s_write_bytes(I2SR, (const char *)buffer64, 128, portMAX_DELAY);
  i2s_zero_dma_buffer(I2SR);

  file.close();
  i2s_stop(I2SR);

  Serial.println("Exit playAudio");
}

void Unmount_SD()
{
  // Tested with Kingston 32GB
  unsigned long time1;
  time1 = micros();
  Serial.println("Ejecting the SD");
  SD.end();
  SPI.end();
  // Set MOSI High
  pinMode(15, INPUT_PULLUP);
  // Disable the SD by an HIGH CS/SS
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  // Provide dummy clocks
  pinMode(14, OUTPUT);
  for (int i = 1; i <= 8; i++)
  {
    digitalWrite(14, LOW);
    digitalWrite(14, HIGH);
  }
  // Set CLK High
  pinMode(14, INPUT_PULLUP);
  // Check if MISO became "LOW"
  pinMode(2, INPUT_PULLDOWN);
  delay(1);
  if (digitalRead(2))
  {
    Serial.println(F("Failed to unmount the SD"));
  }
  else
  {
    Serial.printf("Unmounting of the SD-micro card takes : %ld [uSec]\n", (micros() - time1));
    Serial.println(F("SD has been unmounted safely"));
  }
}

void setup()
{
  Serial.begin(115200);

  if (!SPIFFS.begin(true))
    Serial.println("SPIFFS failed!");

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  if (!SD.begin(SD_CS))
    Serial.println("SD initialization failed!");

  //Init butons
  gpio_reset_pin(RedButton);
  gpio_set_direction(RedButton, GPIO_MODE_INPUT);
  gpio_set_pull_mode(RedButton, GPIO_PULLUP_ONLY);
  gpio_reset_pin(YellowButton);
  gpio_set_direction(YellowButton, GPIO_MODE_INPUT);
  gpio_set_pull_mode(YellowButton, GPIO_PULLUP_ONLY);
  gpio_reset_pin(GreenButton);
  gpio_set_direction(GreenButton, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GreenButton, GPIO_PULLUP_ONLY);
  gpio_reset_pin(WhiteButton);
  gpio_set_direction(WhiteButton, GPIO_MODE_INPUT);
  gpio_set_pull_mode(WhiteButton, GPIO_PULLUP_ONLY);
  gpio_reset_pin(BlueButton);
  gpio_set_direction(BlueButton, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BlueButton, GPIO_PULLUP_ONLY);

  // enable amp power
  gpio_reset_pin(PW);
  gpio_set_direction(PW, GPIO_MODE_OUTPUT);
  gpio_set_level(PW, 1);

  gpio_reset_pin(GAIN);
  gpio_set_direction(GAIN, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GAIN, GPIO_PULLDOWN_ONLY); // 15dB
  i2s_driver_install(I2SR, &i2s_configR, 0, NULL);
  i2s_set_pin(I2SR, &pin_configR);
  i2s_stop(I2SR);

  // debug:
  //playAudio("/debug/16bit.wav", (vExitPredicate)AnyButtonPressed);

  // play start song:
  EEPROM.begin(EEPROM_SIZE);
  int startSongDataAddress = 0;
  _startSongId = EEPROM.read(startSongDataAddress);
  _startSongId = _startSongId == 255 ? 0 : _startSongId;
  _startSongId++;
  if (_startSongId >= SONGS_COUNT)
    _startSongId = 0;
  EEPROM.write(startSongDataAddress, _startSongId);
  EEPROM.commit();

  // init LED
  strip.Begin();

  // init ADC interface for battery survey
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_11);

  playSoundIfBatteryUncharged();
}

bool AnyButtonPressed()
{
  return gpio_get_level(RedButton) == 0 || gpio_get_level(YellowButton) == 0 || gpio_get_level(GreenButton) == 0 || gpio_get_level(WhiteButton) == 0 || gpio_get_level(BlueButton) == 0;
}

bool AllButtonsUnpressed()
{
  return gpio_get_level(RedButton) == 1 && gpio_get_level(YellowButton) == 1 && gpio_get_level(GreenButton) == 1 && gpio_get_level(WhiteButton) == 1 && gpio_get_level(BlueButton) == 1;
}

uint8_t waitForInput()
{
  uint16_t iteration = 0;
  uint8_t result = 0;
  for (;;)
  {
    if (AnyButtonPressed())
    {
      break;
    }
    iteration++;
    if (iteration > 500)
    {
      return 0;
    }
    delay(10);
  }
  for (;;)
  {
    result = result | !gpio_get_level(RedButton) | !gpio_get_level(YellowButton) << 1 | !gpio_get_level(GreenButton) << 2 | !gpio_get_level(WhiteButton) << 3 | !gpio_get_level(BlueButton) << 4;
    if (AllButtonsUnpressed())
    {
      break;
    }
    delay(10);
  }
  return result;
}

void playSoundIfBatteryUncharged()
{
  int batteryVoltage = adc1_get_raw(ADC1_GPIO33_CHANNEL);
  if (batteryVoltage < DISCHARGED_STATE)
  {
    playAudio("/battery-low.wav", NULL);
    strip.SetPixelColor(0, BATTERYLOW);
    strip.Show();
  }
  else
  {
    strip.SetPixelColor(0, BATTERYCHARGED);
    strip.Show();
  }
}

bool _initialized = false;

void loop()
{
  if (!_initialized)
  {
    _rootFolder = _startSongId % 12 == 0 ? 1 : 0;
    _buttonFolder = _startSongId / SONGS_PER_BUTTON;
    _songIndex = _startSongId % SONGS_PER_BUTTON;
    _initialized = true;
  }

  while (true)
  {
    String songFileName = "/" + String(_rootFolder) + "/" + String(_buttonFolder) + "/" + String(_songIndex) + ".wav";
    char *fileName = &songFileName[0];
    playSoundIfBatteryUncharged();
    playAudio(fileName, (vExitPredicate)AnyButtonPressed);
    if (AnyButtonPressed())
      break;

    _songIndex++;
    if (_songIndex > SONGS_PER_BUTTON)
      _songIndex = 0;
  }

  _currentInput = waitForInput();
  if (_currentInput == 0)
  {
    return;
  }
  bool red = (_currentInput >> 0) & 1 == 1;
  bool yellow = (_currentInput >> 1) & 1 == 1;
  bool green = (_currentInput >> 2) & 1 == 1;
  bool white = (_currentInput >> 3) & 1 == 1;
  bool blue = (_currentInput >> 4) & 1 == 1;

  if (red && blue)
  {
    Serial.println("Restarting...");
    ESP.restart();
  }

  if (red && yellow)
  {
    Unmount_SD();
    return;
  }

  if (red && green)
  {
    VOLUME = 15;
  }

  if (yellow && blue)
  {
    _rootFolder++;
    if (_rootFolder == 2)
      _rootFolder = 0;
  }

  int pressedButtonIndex =
      red      ? 0
      : yellow ? 1
      : green  ? 2
      : white  ? 3
      : blue   ? 4
               : -1;

  _buttonFolder = pressedButtonIndex;
  if (_buttonFolder != _previousButtonFolder)
  {
    _songIndex = 0;
  }
  else
  {
    _songIndex++;
    if (_songIndex > SONGS_PER_BUTTON)
      _songIndex = 0;
  }
  _previousButtonFolder = _buttonFolder;
}