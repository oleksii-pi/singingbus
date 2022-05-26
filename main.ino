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

const uint8_t VOLUME = 7; // OK = 7 // 0..63

const uint8_t SONGS_PER_BUTTON = 7;
const uint8_t SONGS_COUNT = SONGS_PER_BUTTON * 5;

uint8_t _startSongId; // 0..SONGS_COUNT-1

#define EEPROM_SIZE 1

#define B1 GPIO_NUM_0
#define B2 GPIO_NUM_4
#define B3 GPIO_NUM_18
#define B4 GPIO_NUM_19
#define B5 GPIO_NUM_32

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
RgbColor RED(255, 0, 0);
RgbColor GREEN(0, 255, 0);
RgbColor BLUE(0, 0, 255);
RgbColor YELLOW(255, 128, 0);
RgbColor WHITE(255, 255, 255);
RgbColor BLACK(0, 0, 0);

RgbColor REDL(64, 0, 0);
RgbColor GREENL(0, 64, 0);
RgbColor BLUEL(0, 0, 64);
RgbColor WHITEL(64, 64, 64);
RgbColor BLACKL(0, 0, 0);
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

static void playAudio(char *fileName, vExitPredicate exitPredicate)
{
  File file;
  int16_t signal;
  static int8_t buffer[44100];
  int bufferSize;
  uint16_t buffer64[64];

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
    bufferSize = (int)file.read((uint8_t *)buffer, 44100);
    if (bufferSize < 0)
    {
      Serial.println("Error reading file");
      goto stopPlaying;
    }

    for (int i = 0; i < bufferSize; i++)
    {
      signal = (((int16_t)(buffer[i] & 0xFF)) - 128) << 8;
      signal = signal * VOLUME >> 6;
      buffer64[i % 64] = (uint16_t)signal;
      if (i % 64 == 63)
      {
        int bytesWritten = 0;
        while (bytesWritten == 0)
        {
          bytesWritten = i2s_write_bytes(I2SR, (const char *)buffer64, 128, portMAX_DELAY);
        }
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
  gpio_reset_pin(B1);
  gpio_set_direction(B1, GPIO_MODE_INPUT);
  gpio_set_pull_mode(B1, GPIO_PULLUP_ONLY);
  gpio_reset_pin(B2);
  gpio_set_direction(B2, GPIO_MODE_INPUT);
  gpio_set_pull_mode(B2, GPIO_PULLUP_ONLY);
  gpio_reset_pin(B3);
  gpio_set_direction(B3, GPIO_MODE_INPUT);
  gpio_set_pull_mode(B3, GPIO_PULLUP_ONLY);
  gpio_reset_pin(B4);
  gpio_set_direction(B4, GPIO_MODE_INPUT);
  gpio_set_pull_mode(B4, GPIO_PULLUP_ONLY);
  gpio_reset_pin(B5);
  gpio_set_direction(B5, GPIO_MODE_INPUT);
  gpio_set_pull_mode(B5, GPIO_PULLUP_ONLY);

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

  String songFileName = "/0/" + String(_startSongId / SONGS_PER_BUTTON) + "/" + String(_startSongId % SONGS_PER_BUTTON) + ".wav";
  char *fileName = &songFileName[0];
  playAudio(fileName, (vExitPredicate)AnyButtonPressed);

  // init ADC interface for battery survey
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_11);

  // init LED
  strip.Begin();
}

bool AnyButtonPressed()
{
  return gpio_get_level(B1) == 0 || gpio_get_level(B2) == 0 || gpio_get_level(B3) == 0 || gpio_get_level(B4) == 0 || gpio_get_level(B5) == 0;
}

bool AllButtonsUnpressed()
{
  return gpio_get_level(B1) == 1 && gpio_get_level(B2) == 1 && gpio_get_level(B3) == 1 && gpio_get_level(B4) == 1 && gpio_get_level(B5) == 1;
}

uint8_t waitForInput()
{
  uint8_t result = 0;
  for (;;)
  {
    if (AnyButtonPressed())
    {
      break;
    }
    delay(10);
  }
  for (;;)
  {
    result = result | !gpio_get_level(B1) | !gpio_get_level(B2) << 1 | !gpio_get_level(B3) << 2 | !gpio_get_level(B4) << 3 | !gpio_get_level(B5) << 4;
    if (AllButtonsUnpressed())
    {
      break;
    }
    delay(10);
  }
  return result;
}

#define DISCHARGED_STATE 1800
#define FULLYCHARGED_STATE 2400
void checkBatteryStatus()
{
  int batteryVoltage = adc1_get_raw(ADC1_GPIO33_CHANNEL);
  if (batteryVoltage < DISCHARGED_STATE)
  {
    playAudio("/debug/debug.wav", NULL);
    strip.SetPixelColor(0, RED);
    strip.Show();
  }
  if (batteryVoltage > FULLYCHARGED_STATE)
  {
    strip.SetPixelColor(0, GREEN);
    strip.Show();
  }

  Serial.printf("Battery charging status: ");
  Serial.println(batteryVoltage);
}

uint8_t _currentInput;
uint8_t _rootFolder, _buttonFolder, _previousButtonFolder, _songIndex;

void loop()
{
  _currentInput = waitForInput();
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

  while (true)
  {
    String songFileName = "/" + String(_rootFolder) + "/" + String(_buttonFolder) + "/" + String(_songIndex) + ".wav";
    char *fileName = &songFileName[0];
    checkBatteryStatus();
    playAudio(fileName, (vExitPredicate)AnyButtonPressed);
    if (AnyButtonPressed())
      return;

    _songIndex++;
    if (_songIndex > SONGS_PER_BUTTON)
      _songIndex = 0;
  }
}