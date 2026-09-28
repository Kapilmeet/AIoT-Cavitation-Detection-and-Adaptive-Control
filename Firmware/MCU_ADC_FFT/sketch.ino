#include "Arduino_RouterBridge.h"
#include <vector>
#include <math.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>

#define BUFFER_SIZE      16384     // Ring buffer sized for high-throughput bursts
#define SAMPLE_RATE      50000

#define FFT_SIZE         4096
#define FFT_BINS         (FFT_SIZE / 2 + 1)
#define DC_OFFSET        8192      // 14-bit ADC midpoint

// --- Zephyr ADC Definitions ---
#define ADC_DEVICE_NODE  DT_NODELABEL(adc1)
#define CHANNEL_A0       9
#define CHANNEL_A1       10
#define CHANNEL_A2       11

static const struct device *adc_dev;
static int16_t sample_a0;
static int16_t sample_a1;
static int16_t sample_a2;

static struct adc_sequence seq_a0 = {
  .channels    = BIT(CHANNEL_A0),
  .buffer      = &sample_a0,
  .buffer_size = sizeof(sample_a0),
  .resolution  = 14,
};

static struct adc_sequence seq_a1 = {
  .channels    = BIT(CHANNEL_A1),
  .buffer      = &sample_a1,
  .buffer_size = sizeof(sample_a1),
  .resolution  = 14,
};

static struct adc_sequence seq_a2 = {
  .channels    = BIT(CHANNEL_A2),
  .buffer      = &sample_a2,
  .buffer_size = sizeof(sample_a2),
  .resolution  = 14,
};

// 5 ADC clock cycles acquisition time (<0.5 us per channel)
static const struct adc_channel_cfg ch_cfg_a0 = {
  .gain             = ADC_GAIN_1,
  .reference        = ADC_REF_INTERNAL,
  .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 5),
  .channel_id       = CHANNEL_A0,
};

static const struct adc_channel_cfg ch_cfg_a1 = {
  .gain             = ADC_GAIN_1,
  .reference        = ADC_REF_INTERNAL,
  .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 5),
  .channel_id       = CHANNEL_A1,
};

static const struct adc_channel_cfg ch_cfg_a2 = {
  .gain             = ADC_GAIN_1,
  .reference        = ADC_REF_INTERNAL,
  .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 5),
  .channel_id       = CHANNEL_A2,
};

// --- Three Parallel Ring Buffers ---
uint16_t bufferA0[BUFFER_SIZE];
uint16_t bufferA1[BUFFER_SIZE];
uint16_t bufferA2[BUFFER_SIZE];

uint32_t totalSamples = 0;
uint32_t readCursor = 0;

// --- Timers ---
unsigned long previousMicros = 0;
const unsigned long sampleInterval = 1000000UL / SAMPLE_RATE; // 20 us

// --- Onboard FFT state (A0 only) ---
static const float FFT_PI = 3.14159265358979323846f;

float fftReal[FFT_SIZE];
float fftImag[FFT_SIZE];
float fftMagnitudeDb[FFT_BINS];
float hannWindow[FFT_SIZE];
float twiddleReal[FFT_SIZE / 2];
float twiddleImag[FFT_SIZE / 2];

uint16_t fftInputIndex = 0;
uint32_t fftUpdateCount = 0;   // increments each time a fresh FFT result is ready

void initFftTables() {
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    hannWindow[i] = 0.5f * (1.0f - cosf(2.0f * FFT_PI * i / (FFT_SIZE - 1)));
  }
  for (uint16_t i = 0; i < FFT_SIZE / 2; i++) {
    float angle = -2.0f * FFT_PI * i / FFT_SIZE;
    twiddleReal[i] = cosf(angle);
    twiddleImag[i] = sinf(angle);
  }
}

void computeFFT(float* real, float* imag, uint16_t n) {
  // Bit-reversal permutation
  uint16_t j = 0;
  for (uint16_t i = 0; i < n - 1; i++) {
    if (i < j) {
      float tr = real[i]; real[i] = real[j]; real[j] = tr;
      float ti = imag[i]; imag[i] = imag[j]; imag[j] = ti;
    }
    uint16_t k = n >> 1;
    while (k <= j) {
      j -= k;
      k >>= 1;
    }
    j += k;
  }

  // Butterfly stages, reusing the precomputed N/2-entry twiddle table at every stage
  for (uint16_t step = 1; step < n; step <<= 1) {
    uint16_t twiddleStride = FFT_SIZE / (step << 1);
    for (uint16_t group = 0; group < n; group += (step << 1)) {
      for (uint16_t pair = 0; pair < step; pair++) {
        uint16_t tIdx = pair * twiddleStride;
        float wr = twiddleReal[tIdx];
        float wi = twiddleImag[tIdx];
        uint16_t idxA = group + pair;
        uint16_t idxB = idxA + step;
        float tr = wr * real[idxB] - wi * imag[idxB];
        float ti = wr * imag[idxB] + wi * real[idxB];
        real[idxB] = real[idxA] - tr;
        imag[idxB] = imag[idxA] - ti;
        real[idxA] += tr;
        imag[idxA] += ti;
      }
    }
  }
}

// Returns an interleaved array: [a0, a1, a2, a0, a1, a2, ...]
// Length is always a multiple of 3 - no trailing status byte, this is
// pure sensor stream for the Python side to reshape and feed the dashboard.
std::vector<uint16_t> get_new_samples_synced(int max_count) {
  std::vector<uint16_t> out;
  uint32_t available = totalSamples - readCursor;

  if (available > BUFFER_SIZE) {
    readCursor = totalSamples - BUFFER_SIZE;
    available = BUFFER_SIZE;
  }

  uint32_t n = available;
  if ((int)n > (uint32_t)max_count) n = (uint32_t)max_count;

  out.reserve(n * 3);
  for (uint32_t i = 0; i < n; i++) {
    uint32_t pos = (readCursor + i) % BUFFER_SIZE;
    out.push_back(bufferA0[pos]);
    out.push_back(bufferA1[pos]);
    out.push_back(bufferA2[pos]);
  }

  readCursor += n;
  return out;
}

uint32_t get_sample_count() {
  return totalSamples;
}

// Latest onboard FFT magnitude spectrum of A0, in dB. Length is always FFT_BINS.
std::vector<float> get_fft_result() {
  std::vector<float> out(fftMagnitudeDb, fftMagnitudeDb + FFT_BINS);
  return out;
}

// Increments every time a new FFT result is ready. Python polls this cheaply
// and only fetches/plots get_fft_result() when the count has changed.
uint32_t get_fft_update_count() {
  return fftUpdateCount;
}

void setup() {
  Bridge.begin();
  Monitor.begin();

  // 1. Prime ADC hardware clock and voltage regulator
  analogReadResolution(14);
  analogRead(A0);
  analogRead(A1);
  analogRead(A2);

  // 2. Configure hardware channels directly via Zephyr
  adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
  if (device_is_ready(adc_dev)) {
    adc_channel_setup(adc_dev, &ch_cfg_a0);
    adc_channel_setup(adc_dev, &ch_cfg_a1);
    adc_channel_setup(adc_dev, &ch_cfg_a2);
  }

  Bridge.provide_safe("get_new_samples_synced", get_new_samples_synced);
  Bridge.provide_safe("get_sample_count", get_sample_count);
  Bridge.provide_safe("get_fft_result", get_fft_result);
  Bridge.provide_safe("get_fft_update_count", get_fft_update_count);

  initFftTables();

  Monitor.println("3-Channel High Speed ADC bridge ready - streaming + onboard FFT.");
}

void loop() {
  unsigned long currentMicros = micros();

  // FAST 3-CHANNEL CONVERSION (Every 20 us = 50 kHz), always running
  if (currentMicros - previousMicros >= sampleInterval) {
    previousMicros = currentMicros;

    if (adc_read(adc_dev, &seq_a0) == 0 &&
        adc_read(adc_dev, &seq_a1) == 0 &&
        adc_read(adc_dev, &seq_a2) == 0) {
      uint32_t pos = totalSamples % BUFFER_SIZE;
      bufferA0[pos] = (uint16_t)sample_a0;
      bufferA1[pos] = (uint16_t)sample_a1;
      bufferA2[pos] = (uint16_t)sample_a2;
      totalSamples++;

      // Feed A0 into the FFT accumulator (DC-removed, Hann-windowed)
      if (fftInputIndex < FFT_SIZE) {
        float centered = (float)sample_a0 - DC_OFFSET;
        fftReal[fftInputIndex] = centered * hannWindow[fftInputIndex];
        fftImag[fftInputIndex] = 0.0f;
        fftInputIndex++;
      }
    }
  }

  if (fftInputIndex >= FFT_SIZE) {
    computeFFT(fftReal, fftImag, FFT_SIZE);

    for (uint16_t k = 0; k < FFT_BINS; k++) {
      float magnitude = sqrtf(fftReal[k] * fftReal[k] + fftImag[k] * fftImag[k]) * 2.0f / FFT_SIZE;
      if (magnitude < 1e-10f) magnitude = 1e-10f;
      fftMagnitudeDb[k] = 20.0f * log10f(magnitude);
    }

    fftUpdateCount++;
    fftInputIndex = 0;
  }

  yield();
}
