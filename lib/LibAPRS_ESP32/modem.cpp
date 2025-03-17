/*
Copyright 2020-2023 Piotr Wilkon
This file is part of VP-Digi.

VP-Digi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.

VP-Digi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with VP-Digi.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <string.h>
#include <math.h>
#include "modem.h"
#include <stdlib.h>
#include <stdbool.h>
#include "ax25.h"
#include "common.h"
#include "esp_dsp.h"
#include <dsps_fir.h>

#include "AFSK.h"

/*
 * Configuration for PLL-based data carrier detection
 * 1. MAXPULSE - the maximum value of the DCD pulse counter. Higher values allow for more stability when a correct signal is detected,
 * but introduce a delay when releasing DCD
 * 2. THRES - the threshold value of the DCD pulse counter. When reached the input signal is assumed to be valid. Higher values mean more immunity to noise,
 * but introduce delay when setting DCD
 * 3. MAXPULSE and THRES difference sets the DCD "inertia" so that the DCD state won't change rapidly when a valid signal is present
 * 4. INC is the DCD pulse counter incrementation value when symbol changes near the PLL counter zero
 * 5. DEC is the DCD pulse counter decrementation value when symbol changes too far from PLL counter zero
 * 6. TUNE is the PLL counter tuning coefficient. It is applied when there is a symbol change (as the symbol change should occur when the PLL counter crosses zero)
 *
 * [       DCD OFF    *      |    DCD ON   ]
 * 0               COUNTER THRES        MAXPULSE
 *        <-DEC INC->
 *
 * The DCD mechanism is described in demodulate().
 * All values were selected by trial and error
 */
#define DCD1200_MAXPULSE 60
#define DCD1200_THRES 20
#define DCD1200_INC 2
#define DCD1200_DEC 1
#define DCD1200_TUNE 0.74f

#define DCD9600_MAXPULSE 60
#define DCD9600_THRES 40
#define DCD9600_INC 1
#define DCD9600_DEC 1
#define DCD9600_TUNE 0.74f

#define DCD300_MAXPULSE 80
#define DCD300_THRES 20
#define DCD300_INC 4
#define DCD300_DEC 1
#define DCD300_TUNE 0.74f

#define N1200 8 // samples per symbol @ fs=9600, oversampling = 38400 Hz
#define N9600 1 // fs=38400, oversampling = 38400 Hz
#define N300 32 // fs=9600, oversampling = 38400 Hz
#define NMAX 32 // keep this value equal to the biggest Nx

#define PLL1200_STEP (((uint64_t)1 << 32) / N1200) // PLL tick increment value
#define PLL9600_STEP (((uint64_t)1 << 32) / N9600)
#define PLL300_STEP (((uint64_t)1 << 32) / N300)

#define PLL1200_LOCKED_TUNE 0.74f
#define PLL1200_NOT_LOCKED_TUNE 0.50f
#define PLL9600_LOCKED_TUNE 0.89f
#define PLL9600_NOT_LOCKED_TUNE 0.50f // I have 0.67 noted somewhere as possibly better value
#define PLL300_LOCKED_TUNE 0.74f
#define PLL300_NOT_LOCKED_TUNE 0.50f

#define AMP_TRACKING_ATTACK 0.16f	// 0.16
#define AMP_TRACKING_DECAY 0.00004f // 0.00004

#define DAC_SINE_SIZE 128 // DAC sine table size

#define PLL_TUNE_BITS 8 // number of bits when tuning PLL to avoid floating point operations

// Oversampling factor
// This is a helper value, not a setting that can be changed without further code modification!
#define MODEM_LL_OVERSAMPLING_FACTOR 4

struct ModemDemodConfig ModemConfig;

static uint8_t N;						 // samples per symbol
static enum ModemTxTestMode txTestState; // current TX test mode
static uint8_t demodCount;				 // actual number of parallel demodulators
// static uint16_t dacSine[DAC_SINE_SIZE];										   // sine samples for DAC
static uint8_t dacSineIdx;													   // current sine sample index
static volatile uint16_t samples[MODEM_LL_OVERSAMPLING_FACTOR];				   // very raw received samples, filled directly by DMA
static uint8_t currentSymbol;												   // current symbol for NRZI encoding
static uint8_t scrambledSymbol;												   // current symbol after scrambling
float markFreq;																   // mark frequency
float spaceFreq;															   // space frequency
float baudRate;																   // baudrate
static uint16_t markStep;													   // mark timer step
static uint16_t spaceStep;													   // space timer step
static uint16_t baudRateStep;												   // baudrate timer step
static int16_t coeffHiI[NMAX], coeffLoI[NMAX], coeffHiQ[NMAX], coeffLoQ[NMAX]; // correlator IQ coefficients
static uint8_t dcd = 0;														   // multiplexed DCD state from both demodulators
static uint32_t lfsr = 0xFFFFF;												   // LFSR for 9600 Bd

/**
 * @brief BPF filter with 2200 Hz tone 6 dB preemphasis (it actually attenuates 1200 Hz tone by 6 dB)
 */
static const int16_t bpf1200[8] =
	{
		728,
		-13418,
		-554,
		19493,
		-554,
		-13418,
		728,
		2104};

/**
 * @brief BPF filter with 2200 Hz tone 6 dB deemphasis
 */
static const int16_t bpf1200Inv[8] =
	{
		-10513,
		-10854,
		9589,
		23884,
		9589,
		-10854,
		-10513,
		-879};

// fs=9600, rectangular, fc1=1500, fc2=1900, 0 dB @ 1600 Hz and 1800 Hz, N = 15, gain 65536
static const int16_t bpf300[15] =
	{
		186,
		8887,
		8184,
		-1662,
		-10171,
		-8509,
		386,
		5394,
		386,
		-8509,
		-10171,
		-1662,
		8184,
		8887,
		186,
};

#define BPF_MAX_TAPS 15

// fs=9600 Hz, raised cosine, fc=300 Hz (BR=600 Bd), beta=0.8, N=14, gain=65536
static const int16_t lpf300[14] =
	{
		4385,
		4515,
		4627,
		4720,
		4793,
		4846,
		4878,
		4878,
		4846,
		4793,
		4720,
		4627,
		4515,
		4385,
};

// I don't remember what are this filter parameters,
// but it seems to be the best among all I have tested
static const int16_t lpf1200[15] =
	{
		-6128,
		-5974,
		-2503,
		4125,
		12679,
		21152,
		27364,
		29643,
		27364,
		21152,
		12679,
		4125,
		-2503,
		-5974,
		-6128};

// fs=38400 Hz, Gaussian, fc=4800 Hz (9600 Bd), N=9, gain=65536
// seems like there is almost no difference between N=9 and any higher order
static const int16_t lpf9600[9] = {497, 2360, 7178, 13992, 17478, 13992, 7178, 2360, 497};

#define LPF_MAX_TAPS 15

#define FILTER_MAX_TAPS ((LPF_MAX_TAPS > BPF_MAX_TAPS) ? LPF_MAX_TAPS : BPF_MAX_TAPS)

struct Filter
{
	int16_t *coeffs;
	uint8_t taps;
	int32_t samples[FILTER_MAX_TAPS];
	uint8_t gainShift;
};

struct DemodState
{
	uint8_t rawSymbols;	 // raw, unsynchronized symbols
	uint8_t syncSymbols; // synchronized symbols

	enum ModemPrefilter prefilter;
	struct Filter bpf;
	int16_t correlatorSamples[NMAX];
	uint8_t correlatorSamplesIdx;
	struct Filter lpf;

	uint8_t dcd : 1; // DCD state

	int32_t pll; // bit recovery PLL counter
	int32_t pllStep;
	int32_t pllLockedTune;
	int32_t pllNotLockedTune;

	int32_t dcdPll;		   // DCD PLL main counter
	uint8_t dcdLastSymbol; // last symbol for DCD
	uint16_t dcdCounter;   // DCD "pulse" counter (incremented when RX signal is correct)
	uint16_t dcdMax;
	uint16_t dcdThres;
	uint16_t dcdInc;
	uint16_t dcdDec;
	int32_t dcdTune;

	int16_t peak;
	int16_t valley;
};

static struct DemodState demodState[MODEM_MAX_DEMODULATOR_COUNT];

static void decode(uint8_t symbol, uint8_t demod, uint16_t mV);
static int32_t demodulate(int16_t sample, struct DemodState *dem);

static int32_t filter(struct Filter *filter, int32_t input)
{
	int32_t out = 0;

	for (uint8_t i = filter->taps - 1; i > 0; i--)
		filter->samples[i] = filter->samples[i - 1]; // shift old samples

	filter->samples[0] = input; // store new sample
	for (uint8_t i = 0; i < filter->taps; i++)
	{
		out += (int32_t)filter->coeffs[i] * filter->samples[i];
	}
	// dsps_dotprod_s16_ae32(filter->coeffs,(int16_t *)filter->samples,)
	return out >> filter->gainShift;
}

float ModemGetBaudrate(void)
{
	return baudRate;
}

uint8_t ModemGetDemodulatorCount(void)
{
	return demodCount;
}

uint8_t ModemDcdState(void)
{
	return dcd;
}

uint8_t ModemIsTxTestOngoing(void)
{
	if (txTestState != TEST_DISABLED)
		return 1;

	return 0;
}

void ModemGetSignalLevel(uint8_t modem, int8_t *peak, int8_t *valley, uint8_t *level)
{
	*peak = (100 * (int32_t)demodState[modem].peak) >> 12;
	*valley = (100 * (int32_t)demodState[modem].valley) >> 12;
	*level = (100 * (int32_t)(demodState[modem].peak - demodState[modem].valley)) >> 13;
}

enum ModemPrefilter ModemGetFilterType(uint8_t modem)
{
	return demodState[modem].prefilter;
}

/**
 * @brief Set DCD LED
 * @param[in] state False - OFF, true - ON
 */
static void setDcd(bool state)
{
	if (state)
	{
		LED_Status(0, 255, 0);
	}
	else
	{
		LED_Status(0, 0, 0);
	}
}

static inline uint8_t descramble(uint8_t in)
{
	// G3RUH descrambling (x^17+x^12+1)
	uint8_t bit = ((lfsr & 0x10000) > 0) ^ ((lfsr & 0x800) > 0) ^ (in > 0);

	lfsr <<= 1;
	lfsr |= in;
	return bit;
}

static inline uint8_t scramble(uint8_t in)
{
	// G3RUH scrambling (x^17+x^12+1)
	uint8_t bit = ((lfsr & 0x10000) > 0) ^ ((lfsr & 0x800) > 0) ^ (in > 0);

	lfsr <<= 1;
	lfsr |= bit;
	return bit;
}

/**
 * @brief ISR for demodulator
 */

void MODEM_DECODE(int16_t sample, uint16_t mVrms)
{
	// each sample is 12 bits, output sample is 13 bits
	// int32_t sample = ((samples[0] + samples[1] + samples[2] + samples[3]) >> 1) - 4095; //calculate input sample (decimation)
	// int32_t sample = samples[0]-4095;

	bool partialDcd = false;

	for (uint8_t i = 0; i < demodCount; i++)
	{
		uint8_t symbol = (demodulate(sample, (struct DemodState *)&demodState[i]) > 0); // demodulate sample

		decode(symbol, i, mVrms); // recover bits, decode NRZI and call higher level function
		if (demodState[i].dcd)
			partialDcd = true;
	}

	if (partialDcd) // DCD on any of the demodulators
	{
		setDcd(true);
		dcd = 1;
	}
	else // no DCD on both demodulators
	{
		setDcd(false);
		dcd = 0;
	}
}

/**
 * @brief ISR for baudrate generator timer. NRZI encoding is done here.
 */
static uint16_t phaseAcc = 0;
static uint16_t sampleIndex = 0;
uint8_t MODEM_BAUDRATE_TIMER_HANDLER(void)
{
	uint8_t sinwave = 0;
	// MODEM_LL_BAUDRATE_TIMER_CLEAR_INTERRUPT_FLAG();
	if (sampleIndex == 0)
	{
		if (Ax25GetTxBit() == 0) // get next bit and check if it's 0
		{
			currentSymbol ^= 1; // change symbol - NRZI encoding
		}
		sampleIndex = baudRateStep;
	}

	if (ModemConfig.modem == MODEM_9600)
	{
		scrambledSymbol = scramble(currentSymbol);
		// if(ModemConfig.usePWM)
		sinwave = scrambledSymbol ? 240 : 20;
		// else
		//	sinwave = scrambledSymbol ? 15 : 0;

		// sinwave = filter(&demodState[0].lpf, sinwave);
	}
	else
	{
		if (currentSymbol)
		{
			phaseAcc += spaceStep;
		}
		else
		{
			phaseAcc += markStep;
		}
		phaseAcc %= SIN_LEN;
		sinwave = sinSample(phaseAcc);
	}
	if (sampleIndex > 0)
		sampleIndex--;
	return sinwave;
}

/**
 * @brief Demodulate received sample (4x oversampling)
 * @param[in] sample Received sample, no more than 13 bits
 * @param[in] *dem Demodulator state
 * @return Current tone (0 or 1)
 */
static int32_t demodulate(int16_t sample, struct DemodState *dem)
{
	// input signal amplitude tracking
	if (sample >= dem->peak)
	{
		dem->peak += (((int32_t)(AMP_TRACKING_ATTACK * (float)32768) * (int32_t)(sample - dem->peak)) >> 15);
	}
	else
	{
		dem->peak += (((int32_t)(AMP_TRACKING_DECAY * (float)32768) * (int32_t)(sample - dem->peak)) >> 15);
	}

	if (sample <= dem->valley)
	{
		dem->valley -= (((int32_t)(AMP_TRACKING_ATTACK * (float)32768) * (int32_t)(dem->valley - sample)) >> 15);
	}
	else
	{
		dem->valley -= (((int32_t)(AMP_TRACKING_DECAY * (float)32768) * (int32_t)(dem->valley - sample)) >> 15);
	}

	if (ModemConfig.modem != MODEM_9600)
	{
		if (dem->prefilter != PREFILTER_NONE) // filter is used
		{
			dem->correlatorSamples[dem->correlatorSamplesIdx++] = filter(&dem->bpf, sample);
		}
		else // no pre/deemphasis
		{
			dem->correlatorSamples[dem->correlatorSamplesIdx++] = sample;
		}

		dem->correlatorSamplesIdx %= N;

		int32_t outLoI = 0, outLoQ = 0, outHiI = 0, outHiQ = 0; // output values after correlating

		for (uint8_t i = 0; i < N; i++)
		{
			int16_t t = dem->correlatorSamples[(dem->correlatorSamplesIdx + i) % N]; // read sample
			outLoI += t * coeffLoI[i];												 // correlate sample
			outLoQ += t * coeffLoQ[i];
			outHiI += t * coeffHiI[i];
			outHiQ += t * coeffHiQ[i];
		}

		outHiI >>= 14;
		outHiQ >>= 14;
		outLoI >>= 14;
		outLoQ >>= 14;

		sample = (abs(outLoI) + abs(outLoQ)) - (abs(outHiI) + abs(outHiQ));
	}

	// DCD using "PLL"
	// PLL is running nominally at the frequency equal to the baudrate
	// PLL timer is counting up and eventually overflows to a minimal negative value
	// so it crosses zero in the middle
	// tone change should happen somewhere near this zero-crossing (in ideal case of exactly same TX and RX baudrates)
	// nothing is ideal, so we need to have some region around zero where tone change is expected
	// however in case of noise the demodulated output is completely random and has many symbol changes in different places
	// other than the PLL zero-crossing, thus making it easier to utilize this mechanism
	// if tone changed inside this region, then we add something to the DCD pulse counter and adjust counter phase for the counter to be closer to 0
	// if tone changes outside this region, then we subtract something from the DCD pulse counter
	// if some DCD pulse threshold is reached, then we claim that the incoming signal is correct and set DCD flag
	// when configured properly, it's generally immune to noise and sensitive to correct signal
	// it's also important to set some maximum value for DCD counter, otherwise the DCD is "sticky"

	dem->dcdPll = (int32_t)((uint32_t)(dem->dcdPll) + (uint32_t)(dem->pllStep)); // keep PLL ticking at the frequency equal to baudrate

	if ((sample > 0) != dem->dcdLastSymbol) // tone changed
	{
		if ((uint32_t)abs(dem->dcdPll) < (uint32_t)(dem->pllStep)) // tone change occurred near zero
		{
			dem->dcdCounter += dem->dcdInc;	   // increase DCD counter
			if (dem->dcdCounter > dem->dcdMax) // maximum DCD counter value reached
				dem->dcdCounter = dem->dcdMax; // avoid "sticky" DCD and counter overflow
		}
		else // tone change occurred far from zero
		{
			if (dem->dcdCounter >= dem->dcdDec) // avoid overflow
				dem->dcdCounter -= dem->dcdDec; // decrease DCD counter
			else
				dem->dcdCounter = 0;
		}

		// avoid floating point operations
		dem->dcdPll = ((int64_t)dem->dcdPll * (int64_t)dem->dcdTune) >> PLL_TUNE_BITS;
	}

	dem->dcdLastSymbol = sample > 0; // store last symbol for symbol change detection

	if (dem->dcdCounter > dem->dcdThres) // DCD threshold reached
		dem->dcd = 1;					 // DCD!
	else								 // below DCD threshold
		dem->dcd = 0;					 // no DCD

	return filter(&dem->lpf, sample) > 0;
}

/**
 * @brief Decode received symbol: bit recovery, NRZI decoding and pass the decoded bit to higher level protocol
 * @param[in] symbol Received symbol
 * @param demod Demodulator index
 */
static void decode(uint8_t symbol, uint8_t demod, uint16_t mV)
{
	struct DemodState *dem = (struct DemodState *)&demodState[demod];

	// This function provides bit/clock recovery and NRZI decoding
	// Bit recovery is based on PLL which is described in the function above (DCD PLL)
	// Current symbol is sampled at PLL counter overflow, so symbol transition should occur at PLL counter zero
	int32_t previous = dem->pll; // store last clock state

	dem->pll = (int32_t)((uint32_t)(dem->pll) + (uint32_t)(dem->pllStep)); // keep PLL running

	dem->rawSymbols <<= 1; // store received unsynchronized symbol
	dem->rawSymbols |= (symbol & 1);

	if ((dem->pll < 0) && (previous > 0)) // PLL counter overflow, sample symbol, decode NRZI and process in higher layer
	{
		dem->syncSymbols <<= 1; // shift recovered (received, synchronized) bit register

		uint8_t sym = dem->rawSymbols & 0x07;							  // take last three symbols for sampling. Seems that 1 symbol is not enough, but 3 symbols work well
		if (sym == 0b111 || sym == 0b110 || sym == 0b101 || sym == 0b011) // if there are 2 or 3 ones, then the received symbol is 1
			sym = 1;
		else
			sym = 0;

		if (ModemConfig.modem == MODEM_9600)
			sym = descramble(sym); // descramble

		dem->syncSymbols |= sym;

		// NRZI decoding
		if (((dem->syncSymbols & 0x03) == 0b11) || ((dem->syncSymbols & 0x03) == 0b00)) // two last symbols are the same - no symbol transition - decoded bit 1
		{

			Ax25BitParse(1, demod, mV);
		}
		else // symbol transition - decoded bit 0
		{
			Ax25BitParse(0, demod, mV);
		}
	}

	if (((dem->rawSymbols & 0x03) == 0b10) || ((dem->rawSymbols & 0x03) == 0b01)) // if there was a symbol transition, adjust PLL
	{
		// avoid floating point operations. Multiply by n-bit value and shift by n bits
		if (!dem->dcd) // PLL not locked
		{
			dem->pll = ((int64_t)dem->pll * (int64_t)dem->pllNotLockedTune) >> PLL_TUNE_BITS; // adjust PLL faster
		}
		else // PLL locked
		{
			dem->pll = ((int64_t)dem->pll * (int64_t)dem->pllLockedTune) >> PLL_TUNE_BITS; // adjust PLL slower
		}
	}
}

void ModemTxTestStart(enum ModemTxTestMode type)
{
	if (txTestState != TEST_DISABLED) // TX test is already running
		ModemTxTestStop();			  // stop this test

	setPtt(true); // PTT on
	txTestState = type;
}
extern int8_t adcEn;
extern int8_t dacEn;
extern bool hw_afsk_dac_isr;
void ModemTxTestStop(void)
{
	txTestState = TEST_DISABLED;

	hw_afsk_dac_isr = false;
	// DAC_TimerEnable(false);
	dacEn = -1;
	adcEn = 1;
	// timerStop(timer_dac);
	// AFSK_TimerEnable(true);
	setPtt(false); // PTT off
}

void ModemTransmitStart(void)
{
	txTestState = TEST_DISABLED;
	setPtt(true); // PTT on
	delay(50);
	hw_afsk_dac_isr = true;
	AFSK_TimerEnable(false);
	DAC_TimerEnable(true);
	// dacEn=1;
	// adcEn=-1;
	log_d("ModemTransmitStart");
}

/**
 * @brief Stop TX and go back to RX
 */
void ModemTransmitStop(void)
{
	setPtt(false);
	hw_afsk_dac_isr = false;
	DAC_TimerEnable(false);
	// dacEn=-1;
	adcEn = 1;
	// DAC_TimerEnable(false);
	// AFSK_TimerEnable(true);
	log_d("ModemTransmitStop");
}

/**
 * @brief Initialize AFSK module
 */
void ModemInit(void)
{
	memset(demodState, 0, sizeof(demodState));

	if (ModemConfig.modem > MODEM_9600)
		ModemConfig.modem = MODEM_1200;

	if ((ModemConfig.modem == MODEM_1200) || (ModemConfig.modem == MODEM_1200_V23))
	{
		// use one modem in FX.25 mode
		// FX.25 (RS) functions are not reentrant
		//		if (
		// #ifdef ENABLE_FX25
		// 			Ax25Config.fx25
		// #else
		// 			0
		// #endif
		// 		)
		// 			demodCount = 1;
		// 		else

		demodCount = 2;
		N = N1200;
		baudRate = 1200.f;

		demodState[0].pllStep = PLL1200_STEP;
		demodState[0].pllLockedTune = PLL1200_LOCKED_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);
		demodState[0].pllNotLockedTune = PLL1200_NOT_LOCKED_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);
		demodState[0].dcdMax = DCD1200_MAXPULSE;
		demodState[0].dcdThres = DCD1200_THRES;
		demodState[0].dcdInc = DCD1200_INC;
		demodState[0].dcdDec = DCD1200_DEC;
		demodState[0].dcdTune = DCD1200_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);

		demodState[1].pllStep = PLL1200_STEP;
		demodState[1].pllLockedTune = PLL1200_LOCKED_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);
		demodState[1].pllNotLockedTune = PLL1200_NOT_LOCKED_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);
		demodState[1].dcdMax = DCD1200_MAXPULSE;
		demodState[1].dcdThres = DCD1200_THRES;
		demodState[1].dcdInc = DCD1200_INC;
		demodState[1].dcdDec = DCD1200_DEC;
		demodState[1].dcdTune = DCD1200_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);

		demodState[1].prefilter = PREFILTER_NONE;
		demodState[1].lpf.coeffs = (int16_t *)lpf1200;
		demodState[1].lpf.taps = sizeof(lpf1200) / sizeof(*lpf1200);
		demodState[1].lpf.gainShift = 15;

		demodState[0].lpf.coeffs = (int16_t *)lpf1200;
		demodState[0].lpf.taps = sizeof(lpf1200) / sizeof(*lpf1200);
		demodState[0].lpf.gainShift = 15;
		demodState[0].prefilter = PREFILTER_NONE;

		if (ModemConfig.flatAudioIn) // when used with flat audio input, use deemphasis and flat modems
		{
#ifdef ENABLE_FX25
			if (Ax25Config.fx25)
				demodState[0].prefilter = PREFILTER_NONE;
			else
#endif
				demodState[0].prefilter = PREFILTER_DEEMPHASIS;
			demodState[0].bpf.coeffs = (int16_t *)bpf1200Inv;
			demodState[0].bpf.taps = sizeof(bpf1200Inv) / sizeof(*bpf1200Inv);
			demodState[0].bpf.gainShift = 15;
		}
		else // when used with normal (filtered) audio input, use flat and preemphasis modems
		{
			demodState[0].prefilter = PREFILTER_PREEMPHASIS;
			demodState[0].bpf.coeffs = (int16_t *)bpf1200;
			demodState[0].bpf.taps = sizeof(bpf1200) / sizeof(*bpf1200);
			demodState[0].bpf.gainShift = 15;
		}

		if (ModemConfig.modem == MODEM_1200) // Bell 202
		{
			markFreq = 1200.f;
			spaceFreq = 2200.f;
		}
		else // V.23
		{
			markFreq = 1300.f;
			spaceFreq = 2100.f;
		}
	}
	else if (ModemConfig.modem == MODEM_300)
	{
		demodCount = 1;
		N = N300;
		baudRate = 300.f;
		markFreq = 1600.f;
		spaceFreq = 1800.f;

		demodState[0].pllStep = PLL300_STEP;
		demodState[0].pllLockedTune = PLL300_LOCKED_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);
		demodState[0].pllNotLockedTune = PLL300_NOT_LOCKED_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);
		demodState[0].dcdMax = DCD300_MAXPULSE;
		demodState[0].dcdThres = DCD300_THRES;
		demodState[0].dcdInc = DCD300_INC;
		demodState[0].dcdDec = DCD300_DEC;
		demodState[0].dcdTune = DCD300_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);

		demodState[0].prefilter = PREFILTER_FLAT;
		demodState[0].bpf.coeffs = (int16_t *)bpf300;
		demodState[0].bpf.taps = sizeof(bpf300) / sizeof(*bpf300);
		demodState[0].bpf.gainShift = 16;
		demodState[0].lpf.coeffs = (int16_t *)lpf300;
		demodState[0].lpf.taps = sizeof(lpf300) / sizeof(*lpf300);
		demodState[0].lpf.gainShift = 15;
	}
	else if (ModemConfig.modem == MODEM_9600)
	{
		demodCount = 1;
		N = N9600;
		baudRate = 9600.f;
		markFreq = 38400.f / (float)DAC_SINE_SIZE; // use as DAC sample rate

		demodState[0].pllStep = PLL9600_STEP;
		demodState[0].pllLockedTune = PLL9600_LOCKED_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);
		demodState[0].pllNotLockedTune = PLL9600_NOT_LOCKED_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);
		demodState[0].dcdMax = DCD9600_MAXPULSE;
		demodState[0].dcdThres = DCD9600_THRES;
		demodState[0].dcdInc = DCD9600_INC;
		demodState[0].dcdDec = DCD9600_DEC;
		demodState[0].dcdTune = DCD9600_TUNE * (float)((uint32_t)1 << PLL_TUNE_BITS);

		demodState[0].prefilter = PREFILTER_NONE;
		// this filter will be used for RX and TX
		demodState[0].lpf.coeffs = (int16_t *)lpf9600;
		demodState[0].lpf.taps = sizeof(lpf9600) / sizeof(*lpf9600);
		demodState[0].lpf.gainShift = 16;
	}

	markStep = (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)markFreq, CONFIG_AFSK_DAC_SAMPLERATE));
	spaceStep = (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)spaceFreq, CONFIG_AFSK_DAC_SAMPLERATE));
	baudRateStep = CONFIG_AFSK_DAC_SAMPLERATE / (uint32_t)baudRate;

	log_d("markStep %d spaceStep %d baudRateStep %d", markStep, spaceStep, baudRateStep);

	for (uint8_t i = 0; i < N; i++) // calculate correlator coefficients
	{
		coeffLoI[i] = 4095.f * cosf(2.f * 3.1416f * (float)i / (float)N * markFreq / baudRate);
		coeffLoQ[i] = 4095.f * sinf(2.f * 3.1416f * (float)i / (float)N * markFreq / baudRate);
		coeffHiI[i] = 4095.f * cosf(2.f * 3.1416f * (float)i / (float)N * spaceFreq / baudRate);
		coeffHiQ[i] = 4095.f * sinf(2.f * 3.1416f * (float)i / (float)N * spaceFreq / baudRate);
	}

	if (ModemConfig.usePWM)
	{
		// MODEM_LL_PWM_INITIALIZE();
	}
}