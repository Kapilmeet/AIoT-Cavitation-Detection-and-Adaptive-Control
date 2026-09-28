// UNO Q - Fast 3-Channel ADC read using Zephyr's native adc_read() sequence
// instead of 3x sequential analogRead() calls.
//
// WHY: analogRead() on this Zephyr-based core re-runs channel setup + a full
// blocking conversion on every single call. Calling it 3x per tick (A0,A1,A2)
// was costing ~1.7ms per tick -- this is what capped real throughput at
// ~580 samples/sec instead of the intended 50kHz.
//
// FIX: configure all 3 channels ONCE in setup(), then pull all 3 values in
// ONE adc_read() call per tick using a multi-channel sequence buffer.
//
// Channel numbers below are confirmed from the board's actual devicetree:
//   ArduinoCore-zephyr/variants/arduino_uno_q_stm32u585xx/arduino_uno_q_stm32u585xx.overlay
//     io-channels = <&adc1 9>,   A0 - PA4
//                   <&adc1 10>,  A1 - PA5
//                   <&adc1 11>,  A2 - PA6
//                   <&adc1 12>,  A3 - PA7
//                   <&adc1 2>,   A4 - PC1
//                   <&adc1 1>;   A5 - PC0
//   Channel resolution confirmed as 14-bit (zephyr,resolution = 0xe).

#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>

// Confirmed from the board's actual devicetree — see header comment above.
// NOTE: no trailing "//" comments on these #define lines — Zephyr's DT
// macros do deep recursive token-pasting and can choke if anything else
// shares the logical line with the definition.
#define ADC_DEVICE_NODE   DT_NODELABEL(adc1)
#define CHANNEL_A0        9
#define CHANNEL_A1        10
#define CHANNEL_A2        11

static const struct device *adc_dev;
static int16_t adc_sample_buffer[3];   // one slot per channel, filled by adc_read()

// resolution matches ADC_RESOLUTION used elsewhere in the sketch
static struct adc_sequence sequence = {
  .channels    = 0,
  .buffer      = adc_sample_buffer,
  .buffer_size = sizeof(adc_sample_buffer),
  .resolution  = 14,
};

// matches devicetree: zephyr,acquisition-time = <0x3fff> (ADC_ACQ_TIME_MAX)
static const struct adc_channel_cfg ch_cfg_a0 = {
  .gain             = ADC_GAIN_1,
  .reference        = ADC_REF_INTERNAL,
  .acquisition_time = ADC_ACQ_TIME_MAX,
  .channel_id       = CHANNEL_A0,
};
static const struct adc_channel_cfg ch_cfg_a1 = {
  .gain             = ADC_GAIN_1,
  .reference        = ADC_REF_INTERNAL,
  .acquisition_time = ADC_ACQ_TIME_MAX,
  .channel_id       = CHANNEL_A1,
};
static const struct adc_channel_cfg ch_cfg_a2 = {
  .gain             = ADC_GAIN_1,
  .reference        = ADC_REF_INTERNAL,
  .acquisition_time = ADC_ACQ_TIME_MAX,
  .channel_id       = CHANNEL_A2,
};

// Call once from setup(). Returns true on success.
bool fastAdcInit() {
  adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
  if (!device_is_ready(adc_dev)) {
    Monitor.println("ERROR: ADC device not ready — check ADC_DEVICE_NODE");
    return false;
  }

  if (adc_channel_setup(adc_dev, &ch_cfg_a0) != 0 ||
      adc_channel_setup(adc_dev, &ch_cfg_a1) != 0 ||
      adc_channel_setup(adc_dev, &ch_cfg_a2) != 0) {
    Monitor.println("ERROR: adc_channel_setup failed");
    return false;
  }

  sequence.channels = BIT(CHANNEL_A0) | BIT(CHANNEL_A1) | BIT(CHANNEL_A2);
  return true;
}

// Call every tick instead of 3x analogRead(). Fills out[0..2] = A0,A1,A2.
// Returns true on success; false leaves out[] unchanged (caller should skip
// this tick's sample rather than log garbage).
bool fastAdcReadAll(uint16_t out[3]) {
  int err = adc_read(adc_dev, &sequence);
  if (err != 0) return false;

  // adc_sample_buffer is filled in the order the channel bits were set —
  // with BIT(9)|BIT(10)|BIT(11) that's channel 9,10,11 in ascending order.
  out[0] = (uint16_t)adc_sample_buffer[0];  // A0
  out[1] = (uint16_t)adc_sample_buffer[1];  // A1
  out[2] = (uint16_t)adc_sample_buffer[2];  // A2
  return true;
}