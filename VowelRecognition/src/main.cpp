/* Edge Impulse Arduino examples
 * CopyVOWEL_I (c) 2021 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the VOWEL_Is
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyVOWEL_I notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYVOWEL_I HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 3

// If your target is limited in memory remove this macro to save 10K RAM
// #define EIDSP_QUANTIZE_FILTERBANK 0

/* Includes ---------------------------------------------------------------- */
#include <vowel_inferencing.h>

// Other includes
#include "Seeed_Arduino_FreeRTOS.h" //Including Arduino FreeRTOS library
#include "TFT_eSPI.h" // This should come with Wio Terminal package install

#include "bitmaps.h"

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif

// Settings
#define LCD_WIDTH    (320)
#define LCD_HEIGHT   (240)
#define IMAGE_WIDTH  (176)
#define IMAGE_HEIGHT (200)
#define IMAGE_SIZE   (IMAGE_WIDTH * IMAGE_HEIGHT)

#define DEBUG 1 // Enable pin pulse during ISR
enum
{
  ADC_BUF_LEN = 4096
};                              // Size of one of the DMA double buffers
static const int debug_pin = 1; // Toggles each DAC ISR (if DEBUG is set to 1)
static const float maf_threshold = 0.6;

// Labels
typedef enum
{
  LABEL_NONE,
  _SILENCE,
  VOWEL_A,
  VOWEL_E,
  VOWEL_I,
  VOWEL_O,
  VOWEL_U
} label_t;

// DMAC descriptor structure
typedef struct
{
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor;

/** Audio buffers, pointers and selectors */
typedef struct
{
  signed short *buffers[2];
  unsigned char buf_select;
  unsigned char buf_ready;
  unsigned int buf_count;
  unsigned int n_samples;
} inference_t;

static inference_t inference;
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

// Globals - DMA and ADC
volatile uint8_t recording = 0;
volatile boolean results0Ready = false;
volatile boolean results1Ready = false;
uint16_t adc_buf_0[ADC_BUF_LEN];                                             // ADC results array 0
uint16_t adc_buf_1[ADC_BUF_LEN];                                             // ADC results array 1
volatile dmacdescriptor wrb[DMAC_CH_NUM] __attribute__((aligned(16)));       // Write-back DMAC descriptors
dmacdescriptor descriptor_section[DMAC_CH_NUM] __attribute__((aligned(16))); // DMAC channel descriptors
dmacdescriptor descriptor __attribute__((aligned(16)));                      // Place holder descriptor

// Globals - LCD
TFT_eSPI tft;
TaskHandle_t Handle_inferTask;
TaskHandle_t Handle_lcdTask;

// High pass butterworth filter order=1 alpha1=0.0125
class FilterBuHp1
{
public:
  FilterBuHp1()
  {
    v[0] = 0.0;
  }

private:
  float v[2];

public:
  float step(float x) // class II
  {
    v[0] = v[1];
    v[1] = (9.621952458291035404e-1f * x) + (0.92439049165820696974f * v[0]);
    return (v[1] - v[0]);
  }
};

FilterBuHp1 filter;

/*******************************************************************************
 * Interrupt Service Routines (ISRs)
 */

/**
 * @brief      Copy sample data in selected buf and signal ready when buffer is full
 *
 * @param[in]  *buf  Pointer to source buffer
 * @param[in]  buf_len  Number of samples to copy from buffer
 */
static void audio_rec_callback(uint16_t *buf, uint32_t buf_len)
{
  // Copy samples from DMA buffer to inference buffer
  if (recording)
  {
    for (uint32_t i = 0; i < buf_len; i++)
    {

      // Convert 12-bit unsigned ADC value to 16-bit PCM (signed) audio value
      inference.buffers[inference.buf_select][inference.buf_count++] = filter.step(((int16_t)buf[i] - 1024) * 16);
      // Swap double buffer if necessary
      if (inference.buf_count >= inference.n_samples)
      {
        inference.buf_select ^= 1;
        inference.buf_count = 0;
        inference.buf_ready = 1;
      }
    }
  }
}

/**
 * Interrupt Service Routine (ISR) for DMAC 1
 */
void DMAC_1_Handler()
{

  static uint8_t count = 0;

  // Check if DMAC channel 1 has been suspended (SUSP)
  if (DMAC->Channel[1].CHINTFLAG.bit.SUSP)
  {

    // Debug: make pin high before copying buffer
#if DEBUG
    digitalWrite(debug_pin, HIGH);
#endif

    // Restart DMAC on channel 1 and clear SUSP interrupt flag
    DMAC->Channel[1].CHCTRLB.reg = DMAC_CHCTRLB_CMD_RESUME;
    DMAC->Channel[1].CHINTFLAG.bit.SUSP = 1;

    // See which buffer has filled up, and dump results into large buffer
    if (count)
    {
      audio_rec_callback(adc_buf_0, ADC_BUF_LEN);
    }
    else
    {
      audio_rec_callback(adc_buf_1, ADC_BUF_LEN);
    }

    // Flip to next buffer
    count = (count + 1) % 2;
    // Debug: make pin low after copying buffer
#if DEBUG
    digitalWrite(debug_pin, LOW);
#endif
  }
}

// Configure DMA to sample from ADC at regular interval
void config_dma_adc()
{
  // Configure DMA to sample from ADC at a regular interval (triggered by timer/counter)
  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;                     // Specify the location of the descriptors
  DMAC->WRBADDR.reg = (uint32_t)wrb;                                     // Specify the location of the write back descriptors
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);           // Enable the DMAC peripheral
  DMAC->Channel[1].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(TC5_DMAC_ID_OVF) | // Set DMAC to trigger on TC5 timer overflow
                                 DMAC_CHCTRLA_TRIGACT_BURST;             // DMAC burst transfer

  descriptor.descaddr = (uint32_t)&descriptor_section[1];                    // Set up a circular descriptor
  descriptor.srcaddr = (uint32_t)&ADC1->RESULT.reg;                          // Take the result from the ADC0 RESULT register
  descriptor.dstaddr = (uint32_t)adc_buf_0 + sizeof(uint16_t) * ADC_BUF_LEN; // Place it in the adc_buf_0 array
  descriptor.btcnt = ADC_BUF_LEN;                                            // Beat count
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                           // Beat size is HWORD (16-bits)
                      DMAC_BTCTRL_DSTINC |                                   // Increment the destination address
                      DMAC_BTCTRL_VALID |                                    // Descriptor is valid
                      DMAC_BTCTRL_BLOCKACT_SUSPEND;                          // Suspend DMAC channel 0 after block transfer
  memcpy(&descriptor_section[0], &descriptor, sizeof(descriptor));           // Copy the descriptor to the descriptor section

  descriptor.descaddr = (uint32_t)&descriptor_section[0];                    // Set up a circular descriptor
  descriptor.srcaddr = (uint32_t)&ADC1->RESULT.reg;                          // Take the result from the ADC0 RESULT register
  descriptor.dstaddr = (uint32_t)adc_buf_1 + sizeof(uint16_t) * ADC_BUF_LEN; // Place it in the adc_buf_1 array
  descriptor.btcnt = ADC_BUF_LEN;                                            // Beat count
  descriptor.btctrl = DMAC_BTCTRL_BEATSIZE_HWORD |                           // Beat size is HWORD (16-bits)
                      DMAC_BTCTRL_DSTINC |                                   // Increment the destination address
                      DMAC_BTCTRL_VALID |                                    // Descriptor is valid
                      DMAC_BTCTRL_BLOCKACT_SUSPEND;                          // Suspend DMAC channel 0 after block transfer
  memcpy(&descriptor_section[1], &descriptor, sizeof(descriptor));           // Copy the descriptor to the descriptor section

  // Configure NVIC
  NVIC_SetPriority(DMAC_1_IRQn, 0); // Set the Nested Vector Interrupt Controller (NVIC) priority for DMAC1 to 0 (highest)
  NVIC_EnableIRQ(DMAC_1_IRQn);      // Connect DMAC1 to Nested Vector Interrupt Controller (NVIC)

  // Activate the suspend (SUSP) interrupt on DMAC channel 1
  DMAC->Channel[1].CHINTENSET.reg = DMAC_CHINTENSET_SUSP;

  // Configure ADC
  ADC1->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_AIN12_Val; // Set the analog input to ADC0/AIN2 (PB08 - A4 on Metro M4)
  while (ADC1->SYNCBUSY.bit.INPUTCTRL)
    ;                                // Wait for synchronization
  ADC1->SAMPCTRL.bit.SAMPLEN = 0x00; // Set max Sampling Time Length to half divided ADC clock pulse (2.66us)
  while (ADC1->SYNCBUSY.bit.SAMPCTRL)
    ;                                           // Wait for synchronization
  ADC1->CTRLA.reg = ADC_CTRLA_PRESCALER_DIV128; // Divide Clock ADC GCLK by 128 (48MHz/128 = 375kHz)
  ADC1->CTRLB.reg = ADC_CTRLB_RESSEL_12BIT |    // Set ADC resolution to 12 bits
                    ADC_CTRLB_FREERUN;          // Set ADC to free run mode
  while (ADC1->SYNCBUSY.bit.CTRLB)
    ;                         // Wait for synchronization
  ADC1->CTRLA.bit.ENABLE = 1; // Enable the ADC
  while (ADC1->SYNCBUSY.bit.ENABLE)
    ;                         // Wait for synchronization
  ADC1->SWTRIG.bit.START = 1; // Initiate a software trigger to start an ADC conversion
  while (ADC1->SYNCBUSY.bit.SWTRIG)
    ; // Wait for synchronization

  // Enable DMA channel 1
  DMAC->Channel[1].CHCTRLA.bit.ENABLE = 1;

  // Configure Timer/Counter 5
  GCLK->PCHCTRL[TC5_GCLK_ID].reg = GCLK_PCHCTRL_CHEN |     // Enable perhipheral channel for TC5
                                   GCLK_PCHCTRL_GEN_GCLK1; // Connect generic clock 0 at 48MHz

  TC5->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ; // Set TC5 to Match Frequency (MFRQ) mode
  TC5->COUNT16.CC[0].reg = 3000 - 1;            // Set the trigger to 16 kHz: (4Mhz / 16000) - 1
  while (TC5->COUNT16.SYNCBUSY.bit.CC0)
    ; // Wait for synchronization

  // Start Timer/Counter 5
  TC5->COUNT16.CTRLA.bit.ENABLE = 1; // Enable the TC5 timer
  while (TC5->COUNT16.SYNCBUSY.bit.ENABLE)
    ; // Wait for synchronization
}

/**
 * @brief      Printf function uses vsnprintf and output using Arduino Serial
 *
 * @param[in]  format     Variable argument list
 */
void ei_printf(const char *format, ...)
{
  static char print_buf[1024] = {0};

  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);

  if (r > 0)
  {
    Serial.write(print_buf);
  }
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
  bool ret = true;

  if (inference.buf_ready == 1)
  {
    ei_printf(
        "Error sample buffer overrun. Decrease the number of slices per model window "
        "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
    ret = false;
  }

  while (inference.buf_ready == 0)
  {
    delay(10);
  }

  inference.buf_ready = 0;

  return ret;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

  return 0;
}

void lcd_print(const uint8_t* bitmap, const char* label)
{
  // Disable recording for 1-second hold-off
  recording = 0;

  tft.fillScreen(TFT_WHITE);
  tft.drawBitmap(((LCD_WIDTH - IMAGE_WIDTH) / 2), 0, bitmap, IMAGE_WIDTH, IMAGE_HEIGHT, TFT_PINK);
  tft.setTextColor(TFT_PINK, TFT_WHITE);

  uint32_t poX = ((LCD_WIDTH - IMAGE_WIDTH) / 2) - 12;
  uint32_t poY = IMAGE_HEIGHT;

  tft.drawString(label, poX, poY);

  delay(10);

  recording = 1;
}

static void inference_init(void)
{
  run_classifier_init();

  // Create double buffer for inference
  inference.buffers[0] = (int16_t *)malloc(EI_CLASSIFIER_SLICE_SIZE * sizeof(int16_t));

  if (inference.buffers[0] == NULL)
  {
    ei_printf("ERROR: Failed to create inference buffer 0");
    return;
  }
  inference.buffers[1] = (int16_t *)malloc(EI_CLASSIFIER_SLICE_SIZE *
                                           sizeof(int16_t));
  if (inference.buffers[1] == NULL)
  {
    ei_printf("ERROR: Failed to create inference buffer 1");
    free(inference.buffers[0]);
    return;
  }

  // Set inference parameters
  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = EI_CLASSIFIER_SLICE_SIZE;
  inference.buf_ready = 0;

  // Configure DMA to sample from ADC at 16kHz (start sampling immediately)
  config_dma_adc();

  // Start recording to inference buffers
  recording = 1;
}

static void infer_task(void* pvParameters)
{
  static float vowel_a_prev = 0.0;
  static float vowel_e_prev = 0.0;
  static float vowel_i_prev = 0.0;
  static float vowel_o_prev = 0.0;
  static float vowel_u_prev = 0.0;
  static label_t idx_prev = LABEL_NONE;
  label_t idx = LABEL_NONE;

  inference_init();

  while (1)
  {
    bool m = microphone_inference_record();
    if (!m)
    {
      ei_printf("ERR: Failed to record audio...\n");
      return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &microphone_audio_signal_get_data;
    ei_impulse_result_t result = {0};

    EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK)
    {
      ei_printf("ERR: Failed to run classifier (%d)\n", r);
      return;
    }

    // Calculate 2-point moving average filter (MAF) for a label
    float vowel_a_val = result.classification[1].value;
    float vowel_a_maf = (vowel_a_prev + vowel_a_val) / 2;
    vowel_a_prev = vowel_a_val;

    // Calculate 2-point moving average filter (MAF) for e label
    float vowel_e_val = result.classification[2].value;
    float vowel_e_maf = (vowel_e_prev + vowel_e_val) / 2;
    vowel_e_prev = vowel_e_val;

    // Calculate 2-point moving average filter (MAF) for i label
    float vowel_i_val = result.classification[3].value;
    float vowel_i_maf = (vowel_i_prev + vowel_i_val) / 2;
    vowel_i_prev = vowel_i_val;

    // Calculate 2-point moving average filter (MAF) for o label
    float vowel_o_val = result.classification[4].value;
    float vowel_o_maf = (vowel_o_prev + vowel_o_val) / 2;
    vowel_o_prev = vowel_o_val;

    // Calculate 2-point moving average filter (MAF) for u label
    float vowel_u_val = result.classification[5].value;
    float vowel_u_maf = (vowel_u_prev + vowel_u_val) / 2;
    vowel_u_prev = vowel_u_val;

    // Figure out if any MAF values surpass threshold
    if (vowel_a_maf > maf_threshold)
    {
      idx = VOWEL_A;
    }
    else if (vowel_e_maf > maf_threshold)
    {
      idx = VOWEL_E;
    }
    else if (vowel_i_maf > maf_threshold)
    {
      idx = VOWEL_I;
    }
    else if (vowel_o_maf > maf_threshold)
    {
      idx = VOWEL_O;
    }
    else if (vowel_u_maf > maf_threshold)
    {
      idx = VOWEL_U;
    }
    else
    {
      idx = _SILENCE;
    }

    // Print label to LCD if predicted class is different from last iteration
    if (idx != idx_prev)
    {
      switch (idx)
      {
      case VOWEL_A:
        lcd_print(g_image_a, "VOWEL A");
        break;
      case VOWEL_E:
        lcd_print(g_image_e, "VOWEL E");
        break;
      case VOWEL_I:
        lcd_print(g_image_i, "VOWEL I");
        break;
      case VOWEL_O:
        lcd_print(g_image_o, "VOWEL O");
        break;
      case VOWEL_U:
        lcd_print(g_image_u, "VOWEL U");
        break;
      case _SILENCE:
        lcd_print(g_image_silence, "SILENCE");
        break;
      case LABEL_NONE:
        break;
      }
    }

    idx_prev = idx;
  }
}

/**
 * @brief      Arduino setup function
 */
void setup()
{
  // Configure pin to toggle on DMA interrupt
#if DEBUG
  pinMode(debug_pin, OUTPUT);
#endif

  // put your setup code here, to run once:
  Serial.begin(115200);

  vNopDelayMS(1000); // prevents usb driver crash on startup, do not omit this

  // Configure LCD
  tft.begin();
  tft.setRotation(3);
  tft.setFreeFont(&FreeSansBoldOblique24pt7b);

  Serial.println("Edge Impulse Inferencing Demo");

  xTaskCreate(infer_task, "infer_task",1024, NULL, tskIDLE_PRIORITY + 1, &Handle_inferTask);

  // Start the RTOS, this function will never return and will schedule the tasks.
  vTaskStartScheduler();
}

void loop()
{
  // NOTHING
}