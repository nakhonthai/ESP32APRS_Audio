
#include <Arduino.h>
#include "AFSK.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include <driver/timer.h>
#ifdef CONFIG_IDF_TARGET_ESP32
#include "hal/adc_hal.h"
#include "hal/adc_hal_common.h"
#endif
//#include "cppQueue.h"
#include "fir_filter.h"

#include "driver/sdm.h"
#include "driver/gptimer.h"

#include <hal/misc.h>
#include <soc/syscon_struct.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/sigmadelta.h"

#include "driver/gptimer.h"

#include "esp_dsp.h"
#include <dsps_fir.h>
#ifdef CONFIG_IDF_TARGET_ESP32C3
#include <esp32c3/rom/crc.h>
#else
#include "esp32/rom/crc.h"
#endif

#include "modem.h"

#include "fx25.h"

extern "C"
{
#include "soc/syscon_reg.h"
#include "soc/syscon_struct.h"
}

#define DEBUG_TNC

// uint16_t bitrate = 1200;
// uint16_t sampleperbit = (SAMPLERATE / bitrate);
// uint8_t bit_sutuff_len = 5;
// uint8_t phase_bits = 32;
// uint8_t phase_inc = 4;
// uint16_t phase_max = (sampleperbit * phase_bits);
// uint16_t phase_threshold = (phase_max / 2);
// uint16_t mark_freq = 1200;
// uint16_t space_freq = 2200;
// uint32_t pll_inc = (1LLU << 32) / sampleperbit;

extern unsigned long custom_preamble;
extern unsigned long custom_tail;
int adcVal;

// bool input_HPF = false;
// bool input_BPF = false;

int8_t _sql_pin, _ptt_pin, _pwr_pin, _dac_pin, _adc_pin;
bool _sql_active, _ptt_active, _pwr_active;

uint8_t adc_atten;

static const adc_unit_t unit = ADC_UNIT_1;

void sample_dac_isr();
bool hw_afsk_dac_isr = false;

// static filter_t bpf;
// static filter_t lpf;
// static filter_t hpf;

// Afsk *AFSK_modem;
// extern Afsk modem;

static int Vref = 950;

#define FILTER_TAPS 8 // Number of taps in the FIR filter
uint16_t SAMPLERATE = 38400;
// Resampling configuration
#define INPUT_RATE 38400
#define OUTPUT_RATE 9600
uint16_t RESAMPLE_RATIO = (SAMPLERATE / OUTPUT_RATE); // 38400/9600 = 3
uint16_t BLOCK_SIZE = (SAMPLERATE / 50);              // Must be multiple of resample ratio
float *audio_buffer = NULL;

// AGC configuration
#define AGC_TARGET_RMS 0.2f // Target RMS level (-10dBFS)
#define AGC_ATTACK 0.02f    // Fast attack rate
#define AGC_RELEASE 0.001f  // Slow release rate
#define AGC_MAX_GAIN 10.0f
#define AGC_MIN_GAIN 0.1f

int8_t _led_rx_pin = 2;
int8_t _led_tx_pin = 4;
int8_t _led_strip_pin = -1;
uint8_t r_old = 0, g_old = 0, b_old = 0;
unsigned long rgbTimeout = 0;

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel *strip;

extern float markFreq;  // mark frequency
extern float spaceFreq; // space freque
extern float baudRate;  // baudrate


/****************** Ring Buffer gen from DeepSeek *********************/
#define BUFFER_SIZE 1500
typedef struct {
    int16_t buffer[BUFFER_SIZE]; // Buffer to store int16_t data
    int head;                    // Index for the next write
    int tail;                    // Index for the next read
    int count;                   // Number of elements in the buffer
    bool lock;
} RingBuffer;

// Initialize the ring buffer
void RingBuffer_Init(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->lock = false;
}

// Check if the buffer is full
bool RingBuffer_IsFull(const RingBuffer *rb) {
    return rb->count == BUFFER_SIZE;
}

// Check if the buffer is empty
bool RingBuffer_IsEmpty(const RingBuffer *rb) {
    return rb->count == 0;
}

// Add an element to the buffer (push)
bool RingBuffer_Push(RingBuffer *rb, int16_t data) {
    if (RingBuffer_IsFull(rb)) {
        return false; // Buffer is full
    }
    rb->lock = true;
    if(rb->head >= BUFFER_SIZE || rb->head < 0) rb->head = 0; // Check if head exceeds buffer size
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % BUFFER_SIZE; // Wrap around using modulo
    rb->count++;
    rb->lock = false;
    return true;
}

// Remove an element from the buffer (pop)
bool RingBuffer_Pop(RingBuffer *rb, int16_t *data) {
    if (RingBuffer_IsEmpty(rb)) {
        return false; // Buffer is empty
    }
    int to=0;
    while(rb->lock) // Wait until the buffer is not locked
    {
      delay(1); // Avoid busy-waiting, adjust as needed
      if(to++ > 100) return false; // Timeout after 1 second
    }
    if(rb->tail >= BUFFER_SIZE || rb->tail < 0) rb->tail = 0; // Check if tail exceeds buffer size
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % BUFFER_SIZE; // Wrap around using modulo
    rb->count--;
    return true;
}

// Get the number of elements in the buffer
int RingBuffer_Size(const RingBuffer *rb) {
    return rb->count;
}

RingBuffer fifo; // Declare a ring buffer


/******************************************************************** */

// #define MARK_INC (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)1200, CONFIG_AFSK_DAC_SAMPLERATE))
// #define SPACE_INC (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)2200, CONFIG_AFSK_DAC_SAMPLERATE))

// #if defined(CONFIG_IDF_TARGET_ESP32S3)
// // #define USE_DSP
// #define BAUD_RATE 1200
// #define FILTER_ORDER 64
// // Filter contexts and buffers
// fir_f32_t firMark, firSpace;
// float markBuffer[BLOCK_SIZE];
// float spaceBuffer[BLOCK_SIZE];
// float delayMark[FILTER_ORDER + BLOCK_SIZE] = {0};
// float delaySpace[FILTER_ORDER + BLOCK_SIZE] = {0};

// // Cosine table indexed by unsigned byte.
// static float fcos256_table[256];

// #define fcos256(x) (fcos256_table[((x) >> 24) & 0xff])
// #define fsin256(x) (fcos256_table[(((x) >> 24) - 64) & 0xff])

// const float fir1200[] = {
//     0.00316367f, 0.00398861f, 0.00437723f, 0.00413208f, 0.00298872f, 0.00074047f, -0.00260284f, -0.00668306f, -0.01075057f, -0.01375085f, -0.01454411f, -0.01221729f, -0.00641016f, 0.00244106f, 0.01303073f, 0.02333366f, 0.03097455f, 0.03375193f, 0.03020101f, 0.02005505f, 0.00447537f, -0.01403413f, -0.03204008f, -0.04585163f, -0.05231281f, -0.04953725f, -0.03740889f, -0.01771846f, 0.00611527f, 0.02969454f, 0.04852908f, 0.05896765f, 0.05896765f, 0.04852908f, 0.02969454f, 0.00611527f, -0.01771846f, -0.03740889f, -0.04953725f, -0.05231281f, -0.04585163f, -0.03204008f, -0.01403413f, 0.00447537f, 0.02005505f, 0.03020101f, 0.03375193f, 0.03097455f, 0.02333366f, 0.01303073f, 0.00244106f, -0.00641016f, -0.01221729f, -0.01454411f, -0.01375085f, -0.01075057f, -0.00668306f, -0.00260284f, 0.00074047f, 0.00298872f, 0.00413208f, 0.00437723f, 0.00398861f, 0.00316367f};
// const float fir2200[] = {
//     0.00231442f, -0.00057143f, -0.00355072f, -0.00511464f, -0.00386876f, 0.00049756f, 0.00630273f, 0.01005055f, 0.00823119f, -0.00000000f, -0.01113572f, -0.01837338f, -0.01552207f, -0.00164028f, 0.01686771f, 0.02888214f, 0.02512596f, 0.00483548f, -0.02209390f, -0.03977362f, -0.03574035f, -0.00950762f, 0.02554950f, 0.04903640f, 0.04561719f, 0.01504976f, -0.02642793f, -0.05490854f, -0.05294572f, -0.02045258f, 0.02456674f, 0.05628144f, 0.05628144f, 0.02456674f, -0.02045258f, -0.05294572f, -0.05490854f, -0.02642793f, 0.01504976f, 0.04561719f, 0.04903640f, 0.02554950f, -0.00950762f, -0.03574035f, -0.03977362f, -0.02209390f, 0.00483548f, 0.02512596f, 0.02888214f, 0.01686771f, -0.00164028f, -0.01552207f, -0.01837338f, -0.01113572f, -0.00000000f, 0.00823119f, 0.01005055f, 0.00630273f, 0.00049756f, -0.00386876f, -0.00511464f, -0.00355072f, -0.00057143f, 0.00231442f};
// // Sample 9600Hz
// //  const float fir1200[] = {
// //  0.00207372f,0.00094242f,-0.00108431f,-0.00312228f,-0.00380735f,-0.00194212f,0.00239364f,0.00708630f,0.00861607f,0.00429478f,-0.00511124f,-0.01452088f,-0.01689924f,-0.00805945f,0.00918445f,0.02502154f,0.02797307f,0.01283878f,-0.01410570f,-0.03711208f,-0.04013115f,-0.01784123f,0.01901132f,0.04856762f,0.05104653f,0.02207739f,-0.02290384f,-0.05700372f,-0.05840175f,-0.02463269f,0.02493093f,0.06055125f,0.06055125f,0.02493093f,-0.02463269f,-0.05840175f,-0.05700372f,-0.02290384f,0.02207739f,0.05104653f,0.04856762f,0.01901132f,-0.01784123f,-0.04013115f,-0.03711208f,-0.01410570f,0.01283878f,0.02797307f,0.02502154f,0.00918445f,-0.00805945f,-0.01689924f,-0.01452088f,-0.00511124f,0.00429478f,0.00861607f,0.00708630f,0.00239364f,-0.00194212f,-0.00380735f,-0.00312228f,-0.00108431f,0.00094242f,0.00207372f
// //  };
// //  const float fir2200[] = {
// //  0.00043830f,0.00245964f,0.00018549f,-0.00331765f,-0.00132588f,0.00455582f,0.00347822f,-0.00577204f,-0.00701809f,0.00624079f,0.01198993f,-0.00505680f,-0.01795665f,0.00137868f,0.02397076f,0.00528852f,-0.02869738f,-0.01485215f,0.03067614f,0.02651018f,-0.02866679f,-0.03879992f,0.02199267f,0.04982518f,-0.01078912f,-0.05762045f,-0.00391802f,0.06057051f,0.02033804f,-0.05778330f,-0.03622740f,0.04932107f,0.04932107f,-0.03622740f,-0.05778330f,0.02033804f,0.06057051f,-0.00391802f,-0.05762045f,-0.01078912f,0.04982518f,0.02199267f,-0.03879992f,-0.02866679f,0.02651018f,0.03067614f,-0.01485215f,-0.02869738f,0.00528852f,0.02397076f,0.00137868f,-0.01795665f,-0.00505680f,0.01198993f,0.00624079f,-0.00701809f,-0.00577204f,0.00347822f,0.00455582f,-0.00132588f,-0.00331765f,0.00018549f,0.00245964f,0.00043830f
// //  };

// float m_osc_delta, s_osc_delta;
// static inline void push_sample(float val, float *buff, int size)
// {
//   // memccpy(buff + 1, buff, (size - 1) * sizeof(float));
//   memmove(buff + 1, buff, (size - 1));
//   buff[0] = val;
// }
// /* FIR filter kernel. */

// static inline float convolve(const float *data, const float *filter, int filter_taps)
// {
//   float sum = 0.0f;
//   int j;

//   // #pragma GCC ivdep				// ignored until gcc 4.9
//   for (j = 0; j < filter_taps; j++)
//   {
//     sum += filter[j] * data[j];
//   }
//   return (sum);
// }
// #endif

void LED_Status(uint8_t r, uint8_t g, uint8_t b)
{
  // portENTER_CRITICAL_ISR(&ledMux);          // ISR start
  if (r == r_old && g == g_old && b == b_old)
  {
    rgbTimeout = millis() + 100;
  }
  else
  {
    if (millis() > rgbTimeout)
    {
      rgbTimeout = millis() + 100;
      r_old = r;
      g_old = g;
      b_old = b;
      if (_led_strip_pin > -1 && strip != NULL)
      {
        strip->setPixelColor(0, strip->Color(r, g, b));
        strip->show();
      }
      else
      {
        if (_led_tx_pin > -1)
        {
          if (r > 0)
            digitalWrite(_led_tx_pin, HIGH);
          else
            digitalWrite(_led_tx_pin, LOW);
        }
        if (_led_rx_pin > -1)
        {
          if (g > 0)
            digitalWrite(_led_rx_pin, HIGH);
          else
            digitalWrite(_led_rx_pin, LOW);
        }
      }
    }
  }
  // portEXIT_CRITICAL_ISR(&ledMux);
}

// const float resample_coeffs[FILTER_TAPS] = {
//   -0.001296,  -0.005406,  -0.012387,  -0.010745,  0.020421,  0.090370,  0.178421,  0.240624,  0.240624,  0.178421,  0.090370,  0.020421,  -0.010745,  -0.012387,  -0.005406,  -0.001296
// };
// Filter coefficients (designed for 38400→9600 resampling)
// cutoff = 4800  # Nyquist for 9600Hz
const float resample_coeffs[FILTER_TAPS] = {
    0.003560, 0.038084, 0.161032, 0.297324, 0.297324, 0.161032, 0.038084, 0.003560};
// Filter instance and buffers
fir_f32_t resample_fir;
// float input_buffer[BLOCK_SIZE] = {0};
// float output_buffer[BLOCK_SIZE / RESAMPLE_RATIO] = {0};
// float filter_state[FILTER_TAPS + BLOCK_SIZE - 1] = {0};
void resample_audio(float *input_buffer)
{
  // Apply anti-aliasing filter and decimate
  for (int i = 0; i < BLOCK_SIZE / RESAMPLE_RATIO; i++)
  {
    float sum = 0;
    for (int j = 0; j < FILTER_TAPS; j++)
    {
      int index = i * RESAMPLE_RATIO + j;
      if (index < BLOCK_SIZE)
      {
        sum += input_buffer[index] * resample_coeffs[j];
      }
    }
    input_buffer[i] = sum;
  }
}

// AGC state
float agc_gain = 1.0f;

tcb_t tcb;

// Audio processing
volatile bool new_samples = false;

float update_agc(float *input_buffer, size_t len)
{
  // Calculate RMS of current block
  float sum_sq = 0;
  for (int i = 0; i < len; i++)
  {
    sum_sq += input_buffer[i] * input_buffer[i];
  }
  float rms = sqrtf(sum_sq / len);

  // Adjust gain based on RMS level
  float error = AGC_TARGET_RMS / (rms + 1e-6f);
  float rate = (error < 1.0f) ? AGC_RELEASE : AGC_ATTACK;
  agc_gain = agc_gain * (1.0f - rate) + (agc_gain * error) * rate;
  agc_gain = fmaxf(fminf(agc_gain, AGC_MAX_GAIN), AGC_MIN_GAIN);
  return agc_gain;
}

#define AX25_FLAG 0x7e
#define AX25_MASK 0xfc      // bit mask of MSb six bits
#define AX25_EOP 0xfc       // end of packet, 7e << 1
#define AX25_STUFF_BIT 0x7c // bit stuffing bit, five of continuous one bits
// #define AX25_MIN_PKT_SIZE (7 * 2 + 1 + 1 + 2) // call sign * 2 + control + PID + FCS
#define AX25_FLAG_BITS 6

#define AX25_ADDR_LEN 7
#define AX25_MIN_PKT_SIZE (AX25_ADDR_LEN * 2 + 1 + 2) // src addr + dst addr + Control + FCS
#define AX25_SSID_MASK 0x0f

// static int ax25_check_fcs(uint8_t data[], int len)
// {
//   // uint16_t fcs, crc;

//   if (len < AX25_MIN_PKT_SIZE)
//     return false;

// #if 0
//     fcs = data[len - 1] << 8 | data[len - 2];
//     crc = crc16_le(0, data, len - 2);

//     static int count = 0;
//     if (fcs == crc && ++count > 10) {
// 	ESP_LOGI(TAG, "GOOD_CRC = %04x", crc16_le(0, data, len));
// 	count = 0;
//     }

//     return fcs == crc;
// #else

// #define GOOD_CRC 0x0f47

//   return crc16_le(0, data, len) == GOOD_CRC;
// #endif
// }

// enum STATE
// {
//   FLAG = 0,
//   DATA
// };

extern AX25Ctx AX25;

#ifdef CONFIG_IDF_TARGET_ESP32
//#define I2S_INTERNAL 1
//#include "driver/i2s.h"
uint8_t adc_pins[] = {36, 39, 34, 35}; // some of ADC1 pins for ESP32
#else
uint8_t adc_pins[] = {0, 2, 3, 4}; // ADC1 common pins for ESP32S2/S3 + ESP32C3/C6 + ESP32H2
#endif

// adc_continuous_handle_t adc_handle[SOC_ADC_PERIPH_NUM];
// extern adc_handle_t adc_handle[SOC_ADC_PERIPH_NUM];
adc_continuous_handle_t AdcHandle = NULL;
adc_cali_handle_t AdcCaliHandle = NULL;

// Calculate how many pins are declared in the array - needed as input for the setup function of ADC Continuous
uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);

// Flag which will be set in ISR when conversion is done
volatile bool adc_coversion_done = false;

// Result structure for ADC Continuous reading
adc_continuous_data_t *result = NULL;

// adc_attenuation_t cfg_adc_atten = ADC_0db;
adc_atten_t cfg_adc_atten = ADC_ATTEN_DB_0;
void afskSetADCAtten(uint8_t val)
{
  adc_atten = val;
  if (adc_atten == 0)
  {
    cfg_adc_atten = ADC_ATTEN_DB_0;
    Vref = 950;
  }
  else if (adc_atten == 1)
  {
    cfg_adc_atten = ADC_ATTEN_DB_2_5;
    Vref = 1250;
  }
  else if (adc_atten == 2)
  {
    cfg_adc_atten = ADC_ATTEN_DB_6;
    Vref = 1750;
  }
  else if (adc_atten == 3)
  {
    cfg_adc_atten = ADC_ATTEN_DB_11;
    Vref = 2450;
  }
  else if (adc_atten == 4)
  {
    cfg_adc_atten = ADC_ATTEN_DB_12;
    Vref = 3300;
  }
}

uint8_t CountOnesFromInteger(uint8_t value)
{
  uint8_t count;
  for (count = 0; value != 0; count++, value &= value - 1)
    ;
  return count;
}

uint16_t CountOnesFromInteger(uint16_t value)
{
  uint16_t count = 0;
  for (int i = 0; i < 16; i++)
  {
    if (value & 0x0001)
      count++;
    value >>= 1;
  }
  return count;
}

#define IMPLEMENTATION FIFO
#define ADC_SAMPLES_COUNT (BLOCK_SIZE / 2)

// #if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
// cppQueue adcq(sizeof(int16_t), 768, IMPLEMENTATION, true); // Instantiate queue
// #else
// cppQueue adcq(sizeof(int16_t), 768 * 2, IMPLEMENTATION, true); // Instantiate queue
// #endif

int8_t adcEn = 0;
int8_t dacEn = 0;

#ifdef I2S_INTERNAL
#define ADC_PATT_LEN_MAX (16)
#define ADC_CHECK_UNIT(unit) RTC_MODULE_CHECK(adc_unit < ADC_UNIT_2, "ADC unit error, only support ADC1 for now", ESP_ERR_INVALID_ARG)
#define RTC_MODULE_CHECK(a, str, ret_val)                          \
  if (!(a))                                                        \
  {                                                                \
    log_d("%s:%d (%s):%s", __FILE__, __LINE__, __FUNCTION__, str); \
    return (ret_val);                                              \
  }

i2s_event_t i2s_evt;
static QueueHandle_t i2s_event_queue;

static esp_err_t adc_set_i2s_data_len(adc_unit_t adc_unit, int patt_len)
{
  ADC_CHECK_UNIT(adc_unit);
  RTC_MODULE_CHECK((patt_len < ADC_PATT_LEN_MAX) && (patt_len > 0), "ADC pattern length error", ESP_ERR_INVALID_ARG);
  // portENTER_CRITICAL(&rtc_spinlock);
  if (adc_unit & ADC_UNIT_1)
  {
    SYSCON.saradc_ctrl.sar1_patt_len = patt_len - 1;
  }
  if (adc_unit & ADC_UNIT_2)
  {
    SYSCON.saradc_ctrl.sar2_patt_len = patt_len - 1;
  }
  // portEXIT_CRITICAL(&rtc_spinlock);
  return ESP_OK;
}

static esp_err_t adc_set_i2s_data_pattern(adc_unit_t adc_unit, int seq_num, adc_channel_t channel, adc_bits_width_t bits, adc_atten_t atten)
{
  ADC_CHECK_UNIT(adc_unit);
  if (adc_unit & ADC_UNIT_1)
  {
    RTC_MODULE_CHECK((adc1_channel_t)channel < ADC1_CHANNEL_MAX, "ADC1 channel error", ESP_ERR_INVALID_ARG);
  }
  RTC_MODULE_CHECK(bits < ADC_WIDTH_MAX, "ADC bit width error", ESP_ERR_INVALID_ARG);
  RTC_MODULE_CHECK(atten < ADC_ATTENDB_MAX, "ADC Atten Err", ESP_ERR_INVALID_ARG);

  // portENTER_CRITICAL(&rtc_spinlock);
  // Configure pattern table, each 8 bit defines one channel
  //[7:4]-channel [3:2]-bit width [1:0]- attenuation
  // BIT WIDTH: 3: 12BIT  2: 11BIT  1: 10BIT  0: 9BIT
  // ATTEN: 3: ATTEN = 11dB 2: 6dB 1: 2.5dB 0: 0dB
  uint8_t val = (channel << 4) | (bits << 2) | (atten << 0);
  if (adc_unit & ADC_UNIT_1)
  {
    SYSCON.saradc_sar1_patt_tab[seq_num / 4] &= (~(0xff << ((3 - (seq_num % 4)) * 8)));
    SYSCON.saradc_sar1_patt_tab[seq_num / 4] |= (val << ((3 - (seq_num % 4)) * 8));
  }
  if (adc_unit & ADC_UNIT_2)
  {
    SYSCON.saradc_sar2_patt_tab[seq_num / 4] &= (~(0xff << ((3 - (seq_num % 4)) * 8)));
    SYSCON.saradc_sar2_patt_tab[seq_num / 4] |= (val << ((3 - (seq_num % 4)) * 8));
  }
  // portEXIT_CRITICAL(&rtc_spinlock);
  return ESP_OK;
}

void I2S_Init(i2s_mode_t MODE, i2s_bits_per_sample_t BPS)
{
  i2s_config_t i2s_config = {
      //.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN | I2S_MODE_ADC_BUILT_IN),
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
      .sample_rate = SAMPLERATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
      .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = 0, // ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 5,
      .dma_buf_len = 768 * 2,
      //.tx_desc_auto_clear   = true,
      .use_apll = false // no Audio PLL ( I dont need the adc to be accurate )
  };

  // if (MODE == I2S_MODE_RX || MODE == I2S_MODE_TX)
  // {
  //   log_d("using I2S_MODE");
  //   i2s_pin_config_t pin_config;
  //   pin_config.bck_io_num = PIN_I2S_BCLK;
  //   pin_config.ws_io_num = PIN_I2S_LRC;
  //   if (MODE == I2S_MODE_RX)
  //   {
  //     pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  //     pin_config.data_in_num = PIN_I2S_DIN;
  //   }
  //   else if (MODE == I2S_MODE_TX)
  //   {
  //     pin_config.data_out_num = PIN_I2S_DOUT;
  //     pin_config.data_in_num = I2S_PIN_NO_CHANGE;
  //   }

  //   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  //   i2s_set_pin(I2S_NUM_0, &pin_config);
  //   i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, BPS, I2S_CHANNEL_MONO);
  // }
  // else if (MODE == I2S_MODE_DAC_BUILT_IN || MODE == I2S_MODE_ADC_BUILT_IN)
  // {
  log_d("Using I2S DAC/ADC_builtin");
  // install and start i2s driver
  if (i2s_driver_install(I2S_NUM_0, &i2s_config, 5, &i2s_event_queue) != ESP_OK)
  {
    log_d("ERROR: Unable to install I2S drives");
  }
  // GPIO36, VP
  // init ADC pad
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
  // i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, BPS, I2S_CHANNEL_MONO);
  i2s_adc_enable(I2S_NUM_0);
  delay(500); // required for stability of ADC

  // ***IMPORTANT*** enable continuous adc sampling
  SYSCON.saradc_ctrl2.meas_num_limit = 0;
  // sample time SAR setting
  SYSCON.saradc_ctrl.sar_clk_div = 2;
  SYSCON.saradc_fsm.sample_cycle = 2;
  adc_set_i2s_data_pattern(ADC_UNIT_1, 0, ADC_CHANNEL_0, ADC_WIDTH_BIT_12, cfg_adc_atten); // Input Vref 1.36V=4095,Offset 0.6324V=1744
  adc_set_i2s_data_len(ADC_UNIT_1, 1);

  i2s_set_pin(I2S_NUM_0, NULL);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN); // IO26
  i2s_zero_dma_buffer(I2S_NUM_0);
  // i2s_start(I2S_NUM_0);
  //  dac_output_enable(DAC_CHANNEL_1);
  // dac_output_enable(DAC_CHANNEL_2);
  // dac_i2s_enable();
  //}
}

void AFSK_TimerEnable(bool sts)
{
  if (sts == true)
  {
    i2s_adc_enable(I2S_NUM_0);
    adcEn = 0;
  }
  else
  {
    i2s_adc_disable(I2S_NUM_0);    
    adcEn = 0;
  }
}

#else

// static bool check_valid_data(const adc_digi_output_data_t *data);

// #define GET_UNIT(x) ((x >> 3) & 0x1)
// static uint16_t adc1_chan_mask = BIT(0);
// static uint16_t adc2_chan_mask = 0;
// static adc_channel_t channel[1] = {(adc_channel_t)ADC_CHANNEL_0};
static const char *TAG = "--(TAG ADC DMA)--";
#if defined(CONFIG_IDF_TARGET_ESP32)
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
void IRAM_ATTR sample_adc_isr()
{
  if (!hw_afsk_dac_isr)
  {
    //portENTER_CRITICAL_ISR(&timerMux); // ISR start
    int16_t adc=analogReadMilliVolts(adc_pins[0]);
    RingBuffer_Push(&fifo, adc);
    //portEXIT_CRITICAL_ISR(&timerMux); // ISR end
  }
}
#else
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#endif

// Forward declerations
// int afsk_getchar(void);

// hw_timer_t *timer = NULL;



bool adcStopFlag = false;
void AFSK_TimerEnable(bool sts)
{
  // if (xSemaphoreTake(xI2CSemaphore, (TickType_t)DEFAULT_SEMAPHORE_TIMEOUT) == pdTRUE)
  // {
  // vTaskSuspendAll ();
  //adcq.flush();
  if (sts == true)
  {
    //   timerAlarmEnable(timer);
    if (AdcHandle != NULL)
    {
      adc_continuous_start(AdcHandle);
      //HAL_FORCE_MODIFY_U32_REG_FIELD(SYSCON.saradc_ctrl2, meas_num_limit, 1);
    }
    adcEn = 0;
  }
  else
  {

    //   timerAlarmDisable(timer);
    if (AdcHandle != NULL)
    {      
      //log_d("SAR DIV=%d LEN=%d", SYSCON.saradc_ctrl.sar_clk_div, SYSCON.saradc_ctrl.sar1_patt_len);
      //HAL_FORCE_MODIFY_U32_REG_FIELD(SYSCON.saradc_ctrl2, meas_num_limit, 0);
      //SYSCON.saradc_ctrl.sar1_patt_p_clear = 1;
      adc_continuous_stop(AdcHandle);
      // adcStopFlag = true;
    }
    adcEn = 0;
  }
  // xTaskResumeAll ();
  // }
}
#endif

hw_timer_t *timer_dac = NULL;
hw_timer_t *timer_adc = NULL;

SemaphoreHandle_t xI2CSemaphore;

#define DEFAULT_SEMAPHORE_TIMEOUT 10

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void DAC_TimerEnable(bool sts)
{
  if (timer_dac == NULL)
    return;
  portENTER_CRITICAL_ISR(&timerMux);
  if (sts == true)
  {
    timerStart(timer_dac);
  }
  else
  {
    timerStop(timer_dac);
  }
  portEXIT_CRITICAL_ISR(&timerMux);
  dacEn = 0;
}

bool getTransmit()
{
  bool ret = false;
  if ((digitalRead(_ptt_pin) ^ _ptt_active) == 0) // signal active with ptt_active
    ret = true;
  else
    ret = false;
  if (hw_afsk_dac_isr)
    ret = true;
  return ret;
}

void setTransmit(bool val)
{
  hw_afsk_dac_isr = val;
}

bool getReceive()
{
  bool ret = false;
  if ((digitalRead(_ptt_pin) ^ _ptt_active) == 0) // signal active with ptt_active
    return false;                                 // PTT Protection receive
  if (digitalRead(LED_RX_PIN))                    // Check RX LED receiving.
    ret = true;
  return ret;
}

/**
 * @brief Controls PTT output
 * @param state False - PTT off, true - PTT on
 */
void setPtt(bool state)
{
  if (state)
  {
    setTransmit(true);
    if (_ptt_pin > 34)
      _ptt_pin = 32;
    if (_ptt_active)
    {
      pinMode(_ptt_pin, OUTPUT);
      digitalWrite(_ptt_pin, HIGH);
    }
    else
    { // Open Collector to LOW
      pinMode(_ptt_pin, OUTPUT_OPEN_DRAIN);
      digitalWrite(_ptt_pin, LOW);
    }
    LED_Status(255, 0, 0);
  }
  else
  {
    setTransmit(false);
    //adcq.flush();
    if (_ptt_active)
    {
      pinMode(_ptt_pin, OUTPUT);
      digitalWrite(_ptt_pin, LOW);
    }
    else
    { // Open Collector to HIGH
      pinMode(_ptt_pin, OUTPUT_OPEN_DRAIN);
      digitalWrite(_ptt_pin, HIGH);
    }
    LED_Status(0, 0, 0);
  }
}

uint8_t modem_config = 0;
void afskSetModem(uint8_t val, bool bpf,uint16_t timeSlot,uint16_t preamble,uint8_t fx25Mode)
{
  if (bpf)
    ModemConfig.flatAudioIn = 1;
  else
    ModemConfig.flatAudioIn = 0;

  if (val == 0)
  {
    ModemConfig.modem = MODEM_300;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    SAMPLERATE = 9600;
#elif defined(CONFIG_IDF_TARGET_ESP32)
    SAMPLERATE = 19200;
#else
    SAMPLERATE = 38400;
#endif
    BLOCK_SIZE = (SAMPLERATE / 50); // Must be multiple of resample ratio
    RESAMPLE_RATIO = (SAMPLERATE / 9600);
  }
  else if (val == 1)
  {
    ModemConfig.modem = MODEM_1200;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    SAMPLERATE = 9600;
#elif defined(CONFIG_IDF_TARGET_ESP32)
    SAMPLERATE = 28800;
#else
    SAMPLERATE = 38400;
#endif
    BLOCK_SIZE = (SAMPLERATE / 50); // Must be multiple of resample ratio
    RESAMPLE_RATIO = (SAMPLERATE / 9600);
  }
  else if (val == 2)
  {
    ModemConfig.modem = MODEM_1200_V23;
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    SAMPLERATE = 9600;
#elif defined(CONFIG_IDF_TARGET_ESP32)
    SAMPLERATE = 28800;
#else
    SAMPLERATE = 38400;
#endif
    BLOCK_SIZE = (SAMPLERATE / 50); // Must be multiple of resample ratio
    RESAMPLE_RATIO = (SAMPLERATE / 9600);
  }
  else if (val == 3)
  {
    ModemConfig.modem = MODEM_9600;
    SAMPLERATE = 38400;
    BLOCK_SIZE = (SAMPLERATE / 100); // Must be multiple of resample ratio
    RESAMPLE_RATIO = (SAMPLERATE / 38400);
  }
  if (audio_buffer != NULL)
  {
    free(audio_buffer);
    audio_buffer = NULL;
  }
  audio_buffer = (float *)calloc(BLOCK_SIZE, sizeof(float));
  if (audio_buffer == NULL)
  {
    log_d("Error allocating memory for audio buffer");
    return;
  }
  log_d("Modem: %d, SampleRate: %d, BlockSize: %d", ModemConfig.modem, SAMPLERATE, BLOCK_SIZE);
  ModemConfig.usePWM = 1;
  ModemInit();
  Ax25Init(fx25Mode);
  if(fx25Mode>0) Fx25Init();
  Ax25TimeSlot(timeSlot);
  Ax25TxDelay(preamble);
}

void afskSetHPF(bool val)
{
  // input_HPF = val;
}
void afskSetBPF(bool val)
{
  // input_BPF = val;
}
void afskSetSQL(int8_t val, bool act)
{
  _sql_pin = val;
  _sql_active = act;
}
void afskSetPTT(int8_t val, bool act)
{
  _ptt_pin = val;
  _ptt_active = act;
}
void afskSetPWR(int8_t val, bool act)
{
  _pwr_pin = val;
  _pwr_active = act;
}

//uint32_t ret_num;
//uint8_t *resultADC;

typedef struct
{
  voidFuncPtr fn;
  void *arg;
} interrupt_config_t;

// portMUX_TYPE DRAM_ATTR timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile SemaphoreHandle_t timerSemaphore;

#ifndef I2S_INTERNAL
int16_t adcPush;
//static TaskHandle_t s_task_handle;
// adc_oneshot_unit_handle_t adc1_handle;
static bool s_conv_done_cb(adc_continuous_handle_t stAdcHandle, const adc_continuous_evt_data_t *edata, void *user_data)
{
  //BaseType_t mustYield = pdFALSE;
  //Notify that ADC continuous driver has done enough number of conversions
  //vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
  //portENTER_CRITICAL_ISR(&timerMux);
  fifo.lock = true;
  for (uint32_t k = 0; k < edata->size; k += SOC_ADC_DIGI_RESULT_BYTES)
  {
    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&edata->conv_frame_buffer[k];

#if defined(CONFIG_IDF_TARGET_ESP32)
    if (p->type1.channel > 0)
      continue;
    adcPush = (int16_t)p->type1.data;
#else
    if ((p->type2.channel > 0) || (p->type2.unit > 0))
      continue;
    adcPush = (int)p->type2.data;
#endif

    if(fifo.head >= BUFFER_SIZE || fifo.head < 0) fifo.head = 0; // Check if head exceeds buffer size
    fifo.buffer[fifo.head] = adcPush;
    fifo.head = (fifo.head + 1) % BUFFER_SIZE; // Wrap around using modulo
    fifo.count++;    
//RingBuffer_Push(&fifo, adcPush);
  }  
  fifo.lock = false;
  //portEXIT_CRITICAL_ISR(&timerMux);
  // vTaskDelay(TMP102_UPDATE_CICLE_MS / portTICK_PERIOD_MS);
  //return (mustYield == pdTRUE);
  return true;
}
// static TaskHandle_t s_task_handle;
// static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
// {
//     BaseType_t mustYield = pdFALSE;
//     //Notify that ADC continuous driver has done enough number of conversions
//     vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

//     return (mustYield == pdTRUE);
// }

void adc_continue_init(void)
{
  adc_channel_t channel;
  adc_unit_t adc_unit = ADC_UNIT_1;
  esp_err_t Err = adc_continuous_io_to_channel(adc_pins[0], &adc_unit, &channel);
  if (Err != ESP_OK)
  {
    log_e("Pin %u is not ADC pin!", adc_pins[0]);
  }
  if (adc_unit != 0)
  {
    log_e("Only ADC1 pins are supported in continuous mode!");
  }

  uint32_t conv_frame_size = (uint32_t)(BLOCK_SIZE * SOC_ADC_DIGI_RESULT_BYTES);
  // uint32_t conv_frame_size = ADC_SAMPLES_COUNT;
  /* On initialise l'ADC : */
  adc_continuous_handle_cfg_t AdcHandleConfig = {
      .max_store_buf_size = (uint32_t)conv_frame_size * 2,
      .conv_frame_size = (uint32_t)conv_frame_size,
  };
  #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
  AdcHandleConfig.flags.flush_pool  = true;
  #endif  
  Err = adc_continuous_new_handle(&AdcHandleConfig, &AdcHandle);
  if (Err != ESP_OK)
    log_d("AdMeasure : Adc Continuous Init Failed.");
  else
  {
    /* On configure l'ADC : */
    adc_continuous_config_t AdcConfig = {
#if defined(CONFIG_IDF_TARGET_ESP32)
        .sample_freq_hz = (uint32_t)(SAMPLERATE * 11 / 9),
#else
        .sample_freq_hz = (uint32_t)(SAMPLERATE),
#endif
        .conv_mode = ADC_CONV_SINGLE_UNIT_1, // On utilise uniquement l'ADC1.
        .format = ADC_OUTPUT_TYPE,           // On utilise le type 2. Pris dans l'exemple.
    };
    log_d("ADC Continuous Configuration Done. SAMPLERATE: %d Hz", AdcConfig.sample_freq_hz);

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    AdcConfig.pattern_num = 1;
    // for (int ii = 0; ii < NBR_CHANNELS; ii++)
    //{
    // uint8_t unit = ADC_UNIT_1;
    // uint8_t ch = channel & 0x7;
    adc_pattern[0].atten = cfg_adc_atten;
    adc_pattern[0].channel = channel;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    //}
    AdcConfig.adc_pattern = adc_pattern;
    Err = adc_continuous_config(AdcHandle, &AdcConfig);
    if (Err != ESP_OK)
      ESP_LOGI(TAG, "=====> AdMeasure : Adc Continuous Configuration Failed.");
    else
    {
      // Setup callbacks for complete event
      adc_continuous_evt_cbs_t cbs = {
          .on_conv_done = s_conv_done_cb,
          //.on_pool_ovf can be used in future
      };
      // adc_handle[adc_unit].adc_interrupt_handle.fn = (voidFuncPtr)userFunc;
      Err = adc_continuous_register_event_callbacks(AdcHandle, &cbs, NULL);
      if (Err != ESP_OK)
      {
        log_e("adc_continuous_register_event_callbacks failed!");
      }

      // Attach the pins to the ADC unit
#ifdef ARDUINO_TEST
      for (int i = 0; i < cfg.channels; i++)
      {
        adc_channel = cfg.adc_channels[i];
        adc_continuous_channel_to_io(ADC_UNIT, adc_channel, &io_pin);
        // perimanSetPinBus: uint8_t pin, peripheral_bus_type_t type, void * bus, int8_t bus_num, int8_t bus_channel
        if (!perimanSetPinBus(io_pin, ESP32_BUS_TYPE_ADC_CONT, (void *)(ADC_UNIT + 1), ADC_UNIT, adc_channel))
        {
          LOGE("perimanSetPinBus to Continuous an ADC Unit %u failed!", ADC_UNIT);
          return false;
        }
      }
#endif

// adc_cali_line_fitting_config_t cali_config = {
//       .unit_id = ADC_UNIT_1,
//       .bitwidth = ADC_BITWIDTH_12,
//       .atten = ADC_ATTEN_DB_0,
//     };

// adc_continuous_start(AdcHandle);

// adc_cali_line_fitting_config_t cali_config = {
//     .unit_id = ADC_UNIT_1,
//     .atten = ADC_ATTEN_DB_0,
//     .bitwidth = ADC_BITWIDTH_12,
// };
// log_d("Creating ADC_UNIT_%d line cali handle", 1);
// Err = adc_cali_create_scheme_line_fitting(&cali_config, &AdcCaliHandle);
// // Err = adc_cali_create_scheme_curve_fitting(&Curve_Fitting, &AdcCaliHandle);
// if (Err != ESP_OK)
//   ESP_LOGI(TAG, "=====> AdMeasure : Fail to create the Curve Fitting Scheme.");
// else
// {
//   /* On démarre l'ADC : */
//   //SYSCON.saradc_ctrl2.meas_num_limit = 0;
//   adc_continuous_start(AdcHandle);
//   //adc_continuous_stop(AdcHandle);
//   //SYSCON.saradc_ctrl2.meas_num_limit = 0;
//   //HAL_FORCE_MODIFY_U32_REG_FIELD(SYSCON.saradc_ctrl2, meas_num_limit, 0);
//   log_d("ADC Continuous has been start");
// }
/* Mise en place de la calibration permettant d'avoir les résultats en mV directement :
 * On utilise le schéma de calibration de type Courbe (Le seul proposé par la puce). */
#ifdef CONFIG_IDF_TARGET_ESP32
      adc_cali_line_fitting_config_t cali_config = {
#else
      adc_cali_curve_fitting_config_t cali_config = {
#endif
        .unit_id = ADC_UNIT_1,
        .atten = cfg_adc_atten,
        .bitwidth = ADC_BITWIDTH_12,
      };

#ifdef CONFIG_IDF_TARGET_ESP32
      Err = adc_cali_create_scheme_line_fitting(&cali_config, &AdcCaliHandle);
      log_d("Creating ADC_UNIT_%d line cali handle", 1);
//
#else
      Err = adc_cali_create_scheme_curve_fitting(&cali_config, &AdcCaliHandle);
      log_d("Creating ADC_UNIT_%d cerve cali handle", 1);
//
#endif
 
      //s_task_handle = xTaskGetCurrentTaskHandle();

      if (Err != ESP_OK)
        ESP_LOGI(TAG, "=====> AdMeasure : Fail to create the Curve Fitting Scheme.");
      else
      {
        //if(_sql_pin < 0){
          /* On démarre l'ADC : */
          // SYSCON.saradc_ctrl2.meas_num_limit = 0;
          adc_continuous_start(AdcHandle);
          // adc_continuous_stop(AdcHandle);
          // SYSCON.saradc_ctrl2.meas_num_limit = 0;
          // HAL_FORCE_MODIFY_U32_REG_FIELD(SYSCON.saradc_ctrl2, meas_num_limit, 0);
          log_d("ADC Continuous has been start");
          adcStopFlag = false;

                 // }else{
        //   adc_continuous_stop(AdcHandle);
        //   adcStopFlag = true; // Stop ADC if SQL pin is set
        // }
      }
    }
  }
}
#endif

/*
 * Configure and initialize the sigma delta modulation
 * on channel 0 to output signal on GPIO4
 */
static void sigmadelta_init(void)
{
  sigmadelta_config_t sigmadelta_cfg = {
      .channel = SIGMADELTA_CHANNEL_0,
      .sigmadelta_duty = 127,
      .sigmadelta_prescale = 96,
      #ifdef CONFIG_IDF_TARGET_ESP32C3
      .sigmadelta_gpio = GPIO_NUM_1, // GPIO1 is used for ESP32C3
      #else 
      .sigmadelta_gpio = GPIO_NUM_26,
      #endif
  };
  sigmadelta_config(&sigmadelta_cfg);
}

void AFSK_hw_init(void)
{
  // Set up ADC
  if (_sql_pin > -1)
    pinMode(_sql_pin, INPUT_PULLUP);
  if (_pwr_pin > -1)
    pinMode(_pwr_pin, OUTPUT);
  if (_led_tx_pin > -1)
  {
    pinMode(_led_tx_pin, OUTPUT);
  }
  if (_led_rx_pin > -1)
  {
    pinMode(_led_rx_pin, OUTPUT);
  }
  if(_led_strip_pin > -1)
  {
    strip = new Adafruit_NeoPixel(1, _led_strip_pin, NEO_GRB + NEO_KHZ800);
  }

  if (_pwr_pin > -1)
    digitalWrite(_pwr_pin, !_pwr_active);

    RingBuffer_Init(&fifo);

#ifdef I2S_INTERNAL
  //  Initialize the I2S peripheral
  I2S_Init(I2S_MODE_DAC_BUILT_IN, I2S_BITS_PER_SAMPLE_16BIT);
#else

adc_continue_init();

// Initialize FIR filter
// resample_fir.coeffs = (float *)resample_coeffs;
// resample_fir.delay = filter_state;
// resample_fir.N = FILTER_TAPS;
// resample_fir.pos = 0;

// pinMode(MIC_PIN, INPUT);
//  Init ADC and Characteristics
//  esp_adc_cal_characteristics_t characteristics;
//  adc1_config_width(ADC_WIDTH_BIT_12);
//  adc1_config_channel_atten(SPK_PIN, cfg_adc_atten); // Input 1.24Vp-p,Use R 33K-(10K//10K) divider input power 1.2Vref
//  esp_adc_cal_characterize(ADC_UNIT_1, cfg_adc_atten, ADC_WIDTH_BIT_12, 0, &adc1_chars);

// adc1_config_channel_atten(SPK_PIN, ADC_ATTEN_DB_11); //Input 3.3Vp-p,Use R 10K divider input power 3.3V
// esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, &characteristics);
// log_d("v_ref routed to 3.3V\n");
#endif

  sigmadelta_init();

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();
  // Set timer frequency to 20Mhz
  timer_dac = timerBegin(20000000);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer_dac, &sample_dac_isr); // Attaches the handler function to the timer
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer_dac, (uint64_t)20000000 / CONFIG_AFSK_DAC_SAMPLERATE, true, 0);
  timerStop(timer_dac);


  //   ESP_LOGI(TAG, "Create timer handle");
  //   gptimer_handle_t gptimer = NULL;
  //   gptimer_config_t timer_config = {
  //       .clk_src = GPTIMER_CLK_SRC_DEFAULT,
  //       .direction = GPTIMER_COUNT_UP,
  //       .resolution_hz = 8000000, // 1MHz, 1 tick=1us
  //   };
  //   ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

  //   gptimer_event_callbacks_t cbs = {
  //       .on_alarm = example_timer_on_alarm_cb,
  //   };
  //   ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));

  //   ESP_LOGI(TAG, "Enable timer");
  //   ESP_ERROR_CHECK(gptimer_enable(gptimer));

  //   ESP_LOGI(TAG, "Start timer, stop it at alarm event");
  //   gptimer_alarm_config_t alarm_config1 = {
  //       .alarm_count = (uint64_t)8000000/ SAMPLERATE, // period = 1s
  //   };
  //   alarm_config1.flags.auto_reload_on_alarm = true;
  //   alarm_config1.reload_count = 0;
  //   ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
  //   ESP_ERROR_CHECK(gptimer_start(gptimer));
  // if (xQueueReceive(queue, &ele, pdMS_TO_TICKS(2000))) {
  //     ESP_LOGI(TAG, "Timer stopped, count=%llu", ele.event_count);
  // } else {
  //     ESP_LOGW(TAG, "Missed one count event");
  // }

  setTransmit(false);
}

void AFSK_init(int8_t adc_pin, int8_t dac_pin, int8_t ptt_pin, int8_t sql_pin, int8_t pwr_pin, int8_t led_tx_pin, int8_t led_rx_pin, int8_t led_strip_pin,bool ptt_act,bool sql_act,bool pwr_act)
{
  _adc_pin = adc_pin;
  _dac_pin = dac_pin;
  _ptt_pin = ptt_pin;
  _sql_pin = sql_pin;
  _pwr_pin = pwr_pin;
  _led_tx_pin = led_tx_pin;
  _led_rx_pin = led_rx_pin;
  _led_strip_pin = led_strip_pin;

  _ptt_active = ptt_act;
  _sql_active = sql_act;
  _pwr_active = pwr_act;

  if (xI2CSemaphore == NULL)
  {
    xI2CSemaphore = xSemaphoreCreateMutex();
    if ((xI2CSemaphore) != NULL)
      xSemaphoreGive((xI2CSemaphore));
  }

  tcb_t *tp = &tcb;
  int i = 0;
  tp->port = i;
  tp->kiss_type = i << 4;
  tp->avg = 2048;
  tp->cdt = false;
  tp->cdt_lvl = 0;
  tp->cdt_led_pin = 2;
  tp->cdt_led_on = 2;
  log_d("cdt_led_pin = %d, cdt_led_on = %d, port = %d", tp->cdt_led_pin, tp->cdt_led_on, tp->port);
  // tp->ptt_pin = 12;
  #ifdef FX25TNCR2
  tp->sta_led_pin = STA_LED_PIN[i];
#endif

#ifdef FX25_ENABLE
  tp->fx25_parity = FX25_PARITY[i]; // FX.25 parity
#endif
  tp->cdt_sem = xSemaphoreCreateBinary();
  assert(tp->cdt_sem);
  assert(xSemaphoreGive(tp->cdt_sem) == pdTRUE); // initialize

  // KISS default parameter
  // tp->fullDuplex = kiss_fullduplex[i]; // half duplex
  tp->fullDuplex = true;  // full duplex
  tp->SlotTime = 10;      // 100ms
  tp->TXDELAY = 50;       // 500ms
  tp->persistence_P = 63; // P = 0.25
  AFSK_hw_init();
}

int offset = 0;
int dc_offset = 620;
// void afskSetDCOffset(int val)
// {
//   dc_offset = offset = val;
// }

// bool dac_push(uint8_t data)
// {
//   if (dacq.isFull())
//   {
//     delay(5);
//   }
//   dacq.push(&data);
//   return true;
// }

bool sqlActive = false;

// #ifndef I2S_INTERNAL
//  int x=0;

uint8_t sinwave = 127;

bool adcq_lock = false;

uint16_t phaseAcc = 0;

void IRAM_ATTR sample_dac_isr()
{
  if (hw_afsk_dac_isr)
  {
    portENTER_CRITICAL_ISR(&timerMux); // ISR start
    sinwave = MODEM_BAUDRATE_TIMER_HANDLER();
    // Sigma-delta duty of one channel, the value ranges from -128 to 127, recommended range is -90 ~ 90.The waveform is more like a random one in this range.
    int8_t sine = (int8_t)(((sinwave - 127) * 12) >> 4); // Redue sine = -85 ~ 85
    sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, sine);
    portEXIT_CRITICAL_ISR(&timerMux); // ISR end
    // Give a semaphore that we can check in the loop
    // xSemaphoreGiveFromISR(timerSemaphore, NULL);
  }
}

extern int mVrms;
extern float dBV;

long mVsum = 0;
int mVsumCount = 0;
uint8_t dcd_cnt = 0;
bool sqlActiveOld = false;
#define READ_LEN 256
//uint32_t ret_num = 0;
//uint8_t resultADC[READ_LEN] = {0};

void AFSK_Poll(bool SA818, bool RFPower)
{

  int mV;
  int x = 0;
  int16_t adc = 0;
  // uint8_t sintable[8] = {127, 217, 254, 217, 127, 36, 0, 36};
#ifdef I2S_INTERNAL
  size_t bytesRead;
  uint16_t pcm_in[ADC_SAMPLES_COUNT];
  uint16_t pcm_out[ADC_SAMPLES_COUNT];
#endif

  if (_sql_pin > -1)
  {                                                 // Set SQL pin active
    if ((digitalRead(_sql_pin) ^ _sql_active) == 0){ // signal active with sql_active
      sqlActive = true;
      // if(sqlActiveOld == false){
      //   AFSK_TimerEnable(true); // Start ADC if SQL pin is set
      // }    
    } else {
      sqlActive = false;
      // if(sqlActiveOld == true){
      //     AFSK_TimerEnable(false); // Stop ADC if SQL pin is not set
      // }
    }
  }
  else
  {
    sqlActive = true;
  }
  sqlActiveOld = sqlActive;

  if (hw_afsk_dac_isr)
  {

    // #ifdef I2S_INTERNAL
    //     memset(pcm_out, 0, sizeof(pcm_out));
    //     for (x = 0; x < ADC_SAMPLES_COUNT; x++)
    //     {
    //       // LED_RX_ON();
    //       adcVal = (int)AFSK_dac_isr(AFSK_modem);
    //       //  log_d(adcVal,HEX);
    //       //  log_d(",");
    //       if (AFSK_modem->sending == false && adcVal == 0)
    //         break;

    //       // float adcF = hp_filter.Update((float)adcVal);
    //       // adcVal = (int)adcF;
    //       // ไม่สามารถใช้งานในโหมด MONO ได้ จะต้องส่งข้อมูลตามลำดับซ้ายและขวา เอาต์พุต DAC บน I2S เป็นสเตอริโอเสมอ
    //       //  Ref: https://lang-ship.com/blog/work/esp32-i2s-dac/#toc6
    //       //  Left Channel GPIO 26
    //       pcm_out[x] = (uint16_t)adcVal; // MSB
    //       if (SA818)
    //       {
    //         pcm_out[x] <<= 7;
    //         pcm_out[x] += 10000;
    //       }
    //       else
    //       {
    //         pcm_out[x] <<= 8;
    //       }
    //       x++;
    //       // Right Channel GPIO 25
    //       pcm_out[x] = 0;
    //     }

    //     // size_t writeByte;
    //     if (x > 0)
    //     {

    //       // if (i2s_write_bytes(I2S_NUM_0, (char *)&pcm_out, (x * sizeof(uint16_t)), portMAX_DELAY) == ESP_OK)
    //       size_t writeByte;
    //       if (i2s_write(I2S_NUM_0, (char *)&pcm_out, (x * sizeof(uint16_t)), &writeByte, portMAX_DELAY) != ESP_OK)
    //       {
    //         log_d("I2S Write Error");
    //       }
    //     }
    //     // size_t writeByte;
    //     // int availableBytes = x * sizeof(uint16_t);
    //     // int buffer_position = 0;
    //     // size_t bytesWritten = 0;
    //     // if (x > 0)
    //     // {
    //     //   do
    //     //   {

    //     //     // do we have something to write?
    //     //     if (availableBytes > 0)
    //     //     {
    //     //       // write data to the i2s peripheral
    //     //       i2s_write(I2S_NUM_0, buffer_position + (char *)&pcm_out,availableBytes, &bytesWritten, portMAX_DELAY);
    //     //       availableBytes -= bytesWritten;
    //     //       buffer_position += bytesWritten;
    //     //     }
    //     //     delay(bytesWritten);
    //     //   } while (bytesWritten > 0);
    //     // }

    //     // รอให้ I2S DAC ส่งให้หมดบัพเฟอร์ก่อนสั่งปิด DAC/PTT
    //     if (AFSK_modem->sending == false)
    //     {
    //       int txEvents = 0;
    //       memset(pcm_out, 0, sizeof(pcm_out));
    //       // log_d("TX TAIL");
    //       //  Clear Delay DMA Buffer
    //       size_t writeByte;
    //       for (int i = 0; i < 5; i++)
    //         i2s_write(I2S_NUM_0, (char *)&pcm_out, (ADC_SAMPLES_COUNT * sizeof(uint16_t)), &writeByte, portMAX_DELAY);
    //       // i2s_write_bytes(I2S_NUM_0, (char *)&pcm_out, (ADC_SAMPLES_COUNT * sizeof(uint16_t)), portMAX_DELAY);
    //       // wait on I2S event queue until a TX_DONE is found
    //       while (xQueueReceive(i2s_event_queue, &i2s_evt, portMAX_DELAY) == pdPASS)
    //       {
    //         if (i2s_evt.type == I2S_EVENT_TX_DONE) // I2S DMA finish sent 1 buffer
    //         {
    //           if (++txEvents > 6)
    //           {
    //             // log_d("TX DONE");
    //             break;
    //           }
    //           delay(10);
    //         }
    //       }
    //       dac_i2s_disable();
    //       i2s_zero_dma_buffer(I2S_NUM_0);
    //       // i2s_adc_enable(I2S_NUM_0);
    //       digitalWrite(_pwr_pin, !_pwr_active);
    //       digitalWrite(_ptt_pin, !_ptt_active);
    //     }
    // #endif
  }
  else
  {
    
    if (audio_buffer != NULL)
    {
      tcb_t *tp = &tcb;
#ifdef I2S_INTERNAL
      // log_d("RX Signal");
      sqlActive = false;

      if (i2s_read(I2S_NUM_0, (char *)&pcm_in, (BLOCK_SIZE * sizeof(uint16_t)), &bytesRead, portMAX_DELAY) == ESP_OK)
      {
        x = 0;
        digitalWrite(4, HIGH);
        mVsum = 0;
        mVsumCount = 0;
        // log_d("%i,%i,%i,%i", pcm_in[0], pcm_in[1], pcm_in[2], pcm_in[3]);
        for (int i = 0; i < (bytesRead / sizeof(uint16_t)); i += 2)
        {
          /* Converts Raw ADC Reading To Calibrated Value & Return the results in mV */
          // adc_cali_raw_to_voltage()
          // adcVal = (int)esp_adc_cal_raw_to_voltage((int)pcm_in[i], &adc1_chars); //Read ADC
          adc = (int)pcm_in[i];
          x++;
#else
// ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
// size_t resNum = BLOCK_SIZE * SOC_ADC_DIGI_RESULT_BYTES;
//       uint8_t *resultADC = (uint8_t *)malloc(resNum);
//       if(resultADC){
//        esp_err_t ret = adc_continuous_read(AdcHandle, resultADC, resNum, &ret_num, 0);
//             if (ret == ESP_OK) {
//                 for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
//                   adc_digi_output_data_t *p = (adc_digi_output_data_t*)&resultADC[i];
//                   #if defined(CONFIG_IDF_TARGET_ESP32)
//                       if (p->type1.channel > 0)
//                         continue;
//                       adcPush = (int16_t)p->type1.data;
//                   #else
//                       if ((p->type2.channel > 0) || (p->type2.unit > 0))
//                         continue;
//                       adcPush = (int)p->type2.data;
//                   #endif
//                   RingBuffer_Push(&fifo, adcPush);
//                 }
//               }
//               free(resultADC);
//       }
      //while (adcq.getCount() >= BLOCK_SIZE)
      while(RingBuffer_Size(&fifo) >= BLOCK_SIZE)
      {
        //digitalWrite(4, HIGH);
        
        mVsum = 0;
        mVsumCount = 0;
        //portENTER_CRITICAL_ISR(&timerMux);
        for (x = 0; x < BLOCK_SIZE; x++)
        {
          // while(adcq_lock) delay(1);
          //if (!adcq.pop(&adc)) // Pull queue buffer
          
          bool ret = RingBuffer_Pop(&fifo, &adc);          
          if (!ret)
            break;
          // if (!RingBuffer_Pop(&fifo, &adc))
          //   break;          

#endif       
          tp->avg_sum += adc - tp->avg_buf[tp->avg_idx];
          tp->avg_buf[tp->avg_idx++] = adc;
          if (tp->avg_idx >= TCB_AVG_N)
            tp->avg_idx -= TCB_AVG_N;
          tp->avg = tp->avg_sum / TCB_AVG_N;

          // carrier detect
          adcVal = (int)adc - tp->avg;
          int m = 1;
          if ((RESAMPLE_RATIO > 1) || (ModemConfig.modem == MODEM_9600))
          {
            m = 4;
          }

          if (x % m == 0)
          {
            adc_cali_raw_to_voltage(AdcCaliHandle, adc, &mV);
            mV -= offset;
            // mVsum += powl(mV, 2); // VRMS = √(1/n)(V1^2 +V2^2 + … + Vn^2)
            // mV = (adcVal * Vref) >> 12;
            mVsum += mV * mV; // Accumulate squared voltage values
            mVsumCount++;
          }

          float sample = adcVal / 2048.0f * agc_gain;
          audio_buffer[x] = sample;
        }
        //portEXIT_CRITICAL_ISR(&timerMux);
        // Update AGC gain
        update_agc(audio_buffer, BLOCK_SIZE);
        adc_cali_raw_to_voltage(AdcCaliHandle, tp->avg, &offset);

        if (mVsumCount > 0)
        {
          tp->cdt_lvl = mVrms = sqrtl(mVsum / mVsumCount); // RMS voltage  VRMS = √(1/mVsumCount)(mVsum)
          mVsum = 0;
          mVsumCount = 0;
          if (mVrms > 10) // >-40dBm
          {
            if (dcd_cnt < 100)
              dcd_cnt++;
          }
          else if (mVrms < 5) // <-46dBm
          {
            if (dcd_cnt > 0)
              dcd_cnt--;
          }
          // Tool conversion dBv <--> Vrms at http://sengpielaudio.com/calculator-db-volt.htm
          // dBV = 20.0F * log10(Vrms);
          // log_d("Audio dc_offset=%d mVrms=%d", offset, mVrms);
        }

        if ((dcd_cnt > 3)||(ModemConfig.modem == MODEM_9600))
        {
          tp->cdt = true;
          //  Process audio block
          if (RESAMPLE_RATIO > 1)
            resample_audio(audio_buffer);

          // Process audio block
          int16_t sample;

          for (int i = 0; i < BLOCK_SIZE / RESAMPLE_RATIO; i++)
          {
            // Convert back to 12-bit audio (0-4095)
            sample = audio_buffer[i] * 2048;
            MODEM_DECODE(sample, mVrms);
          }
        }
        else
        {
          tp->cdt = false;
        }
        //digitalWrite(4, LOW);
      }
    }
    // #ifdef SQL
    //     }
    //     else
    //     {
    //       hdlc_flag_count = 0;
    //       hdlc_flage_end = false;
    //       LED_RX_OFF();
    //       sqlActive = false;
    //     }
    // #endif
  }
}
