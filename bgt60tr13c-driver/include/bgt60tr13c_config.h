#ifndef BGT60TR13C_CONFIG_H
#define BGT60TR13C_CONFIG_H

/* DO NOT CHANGE */
#define XENSIV_BGT60TR13C_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS (3)
#define XENSIV_BGT60TR13C_CONF_NUM_TX_ANTENNAS (1)

/* Set to the same settings from the RADAR IDE */
#define XENSIV_BGT60TR13C_CONF_START_FREQ_HZ (61020100000) // TODO
#define XENSIV_BGT60TR13C_CONF_END_FREQ_HZ (61479904000) // TODO
#define XENSIV_BGT60TR13C_CONF_NUM_SAMPLES_PER_CHIRP (128) // TODO
#define XENSIV_BGT60TR13C_CONF_NUM_CHIRPS_PER_FRAME (64) // TODO
#define XENSIV_BGT60TR13C_CONF_SAMPLE_RATE (2000000) // TODO
#define XENSIV_BGT60TR13C_CONF_CHIRP_REPETITION_TIME_S (6.945e-05) // TODO
#define XENSIV_BGT60TR13C_CONF_FRAME_REPETITION_TIME_S (0.00500396) // TODO
#define XENSIV_BGT60TR13C_CONF_NUM_REGS (38)

const uint32_t register_list[] = { 
	0x001E8270,
	0x020A0210,
	0x08E967FD,
	0x0A0805B4,
	0x0C1027FF,
	0x0E010700,
	0x10000000,
	0x12000000,
	0x14000000,
	0x16000BE0,
	0x18000000,
	0x1A000000,
	0x1C000000,
	0x1E000B60,
	0x2013FC51,
	0x227FF41F,
	0x24701CE7,
	0x2C000490,
	0x3A000480,
	0x48000480,
	0x56000480,
	0x5811BE0E,
	0x5A5DF40A,
	0x5C03F000,
	0x5E787E1E,
	0x60A628CC,
	0x6200068E,
	0x640002B2,
	0x66000080,
	0x68000000,
	0x6A000000,
	0x6C000000,
	0x6E324B10,
	0x7E000100,
	0x8E000100,
	0x9E000100,
	0xAC000000,
	0xB6000000,
}

#endif /* BGT60TR13C_CONFIG_H */