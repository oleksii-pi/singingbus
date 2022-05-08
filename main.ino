#include "Arduino.h"
#include "SD.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include <driver/i2s.h>
#include "SPIFFS.h"

const uint8_t VOLUME = 2; // OK = 7 // 0..63

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

typedef bool (*vExitPlayingPredicate)();

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

static void playAudio(char *fileName, vExitPlayingPredicate exitPlayingPredicate)
{
  i2s_start(I2SR);

  Serial.printf("Playing file ");
  Serial.printf(fileName);
  Serial.printf("\n");

  int16_t s0, s1;
  static int8_t c[44100];
  int l;
  size_t t;
  uint16_t s16[64];
  int a = 0;

  i2s_set_clk(I2SR, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

  File f = SD.open(fileName, FILE_READ);
  if (f == NULL)
    Serial.printf("Error opening file\n");

  f.read((uint8_t *)c, 44);

  do
  {
    l = (int)f.read((uint8_t *)c, 44100);
    if (l < 0)
      Serial.printf("Error reading WAV file \n");
    for (int i = 0; i < l; i++)
    {
      s0 = (((int16_t)(c[i] & 0xFF)) - 128) << 8;
      s0 = s0 >> a;
      s0 = s0 * VOLUME >> 6;
      s16[i % 64] = (uint16_t)s0;
      if (((i + 1) % 64) == 0)
      {
        int n = 0;
        while (n == 0)
          n = i2s_write_bytes(I2SR, (const char *)s16, 128, portMAX_DELAY);
        if (exitPlayingPredicate != NULL && exitPlayingPredicate())
          goto cleanup;
      }
    }

  } while (l > 0);

cleanup:
  // muting after playing
  for (int i = 0; i < 64; i++)
    s16[i] = 0;
  int n = 0;
  while (n == 0)
    n = i2s_write_bytes(I2SR, (const char *)s16, 128, portMAX_DELAY);
  i2s_zero_dma_buffer(I2SR);

  f.close();
  i2s_stop(I2SR);

  Serial.printf("Stop\n");
}

void setup()
{
  Serial.begin(115200);

  //if (!SPIFFS.begin(true))
  //  Serial.println("SPIFFS failed \n");

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (!SD.begin(SD_CS))
  {
    Serial.printf("SD initialization failed!\n");
  }

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

  // Amp power enable
  gpio_reset_pin(PW);
  gpio_set_direction(PW, GPIO_MODE_OUTPUT);
  gpio_set_level(PW, 1);

  gpio_reset_pin(GAIN);
  gpio_set_direction(GAIN, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GAIN, GPIO_PULLDOWN_ONLY); // 15dB
  i2s_driver_install(I2SR, &i2s_configR, 0, NULL);
  i2s_set_pin(I2SR, &pin_configR);
  i2s_stop(I2SR);
}

bool ExitPlayingPredicate()
{
  return gpio_get_level(B1) == 0 || gpio_get_level(B2) == 0 || gpio_get_level(B3) == 0 || gpio_get_level(B4) == 0 || gpio_get_level(B5) == 0;
}

void PrintButtonsState()
{
  Serial.print(gpio_get_level(B1));
  Serial.print(gpio_get_level(B2));
  Serial.print(gpio_get_level(B3));
  Serial.print(gpio_get_level(B4));
  Serial.println(gpio_get_level(B5));
}

uint8_t waitForInput()
{
  uint8_t result = 0;
  for (;;)
  {
    if (gpio_get_level(B1) == 0 || gpio_get_level(B2) == 0 || gpio_get_level(B3) == 0 || gpio_get_level(B4) == 0 || gpio_get_level(B5) == 0)
    {
      break;
    }
    delay(10);
  }
  for (;;)
  {
    result = result | !gpio_get_level(B1) | !gpio_get_level(B2) << 1 | !gpio_get_level(B3) << 2 | !gpio_get_level(B4) << 3 | !gpio_get_level(B4) << 4;
    if (gpio_get_level(B1) == 1 && gpio_get_level(B2) == 1 && gpio_get_level(B3) == 1 && gpio_get_level(B4) == 1 && gpio_get_level(B5) == 1)
    {
      break;
    }
    delay(10);
  }
  return result;
}

uint8_t _currentInput;

void loop()
{
  //playAudio("/debug.wav", NULL);
  _currentInput = waitForInput();

  if ((_currentInput >> 0) & 1 == 1)
  {
    playAudio("/1/cherepaha-aha-aha-8bit-mono.wav", (vExitPlayingPredicate)ExitPlayingPredicate);
  }
}