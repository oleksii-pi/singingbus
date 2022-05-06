#include "Arduino.h"
#include "SD.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include <driver/i2s.h>
#include "SPIFFS.h"

const uint8_t VOLUME = 7; // 0..63

#define I2SR (i2s_port_t)0
#define VM GPIO_NUM_0    // button
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

static void playAudio(char *fileName)
{
  printf("Playing file ");
  printf(fileName);
  printf("\n");

  int16_t s0, s1;
  static int8_t c[44100];
  int l;
  size_t t;
  uint16_t s16[64];
  int a = 0;

  i2s_set_clk(I2SR, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

  File f = SD.open(fileName, FILE_READ);
  if (f == NULL)
    printf("Error opening file\n");

  f.read((uint8_t *)c, 44);

  do
  {
    l = (int)f.read((uint8_t *)c, 44100);
    if (l < 0)
      printf("Error \n");
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
      }
    }
  } while (l > 0);

  // muting after playing
  for (int i = 0; i < 64; i++)
    s16[i] = 0;
  int n = 0;
  while (n == 0)
    n = i2s_write_bytes(I2SR, (const char *)s16, 128, portMAX_DELAY);
  i2s_zero_dma_buffer(I2SR);

  f.close();
  printf("Stop\n");
}

void setup()
{
  Serial.begin(115200);

  if (!SPIFFS.begin(true))
    Serial.println("SPIFFS failed \n");

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (!SD.begin(SD_CS))
  {
    Serial.printf("SD initialization failed!\n");
  }
  else
  {
    Serial.printf("SD initialization successfully\n");
  }

  //Init buton
  gpio_reset_pin(VM);
  gpio_set_direction(VM, GPIO_MODE_INPUT);
  gpio_set_pull_mode(VM, GPIO_PULLDOWN_ONLY);

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

void loop()
{
  if (gpio_get_level(VM) == 0)
  {
    i2s_start(I2SR);
    playAudio("/1/cherepaha-aha-aha-8bit-mono.wav");
    delay(100);
    i2s_stop(I2SR);
  }
}