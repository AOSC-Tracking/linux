#ifndef __LS_I2S_H__
#define __LS_I2S_H__

#define PSC_I2S_RATES SNDRV_PCM_RATE_8000_96000

#define PSC_I2S_FORMATS ( SNDRV_PCM_FMTBIT_S8 | \
		SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S20_3LE | \
		SNDRV_PCM_FMTBIT_S24_3LE | \
		SNDRV_PCM_FMTBIT_S32_LE)

bool ls_i2s_is_slave_mode(void);

#endif

