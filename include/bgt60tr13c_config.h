#ifndef BGT60TR13C_CONFIG_H
#define BGT60TR13C_CONFIG_H

/* DO NOT CHANGE */
#define XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS (1)
#define XENSIV_BGT60TR13C_CONF_NUM_TX_ANTENNAS (1)

/* Set to my custom radar settings. You can find these in the JSON exported from RADAR IDE */
#define XENSIV_BGT60TR13C_CONF_START_FREQ_HZ (59479900000)
#define XENSIV_BGT60TR13C_CONF_END_FREQ_HZ (60436900000)
#define XENSIV_BGT60TR13C_CONF_NUM_SAMPLES_PER_CHIRP (128)
#define XENSIV_BGT60TR13C_CONF_NUM_CHIRPS_PER_FRAME (64)
#define XENSIV_BGT60TR13C_CONF_SAMPLE_RATE (2352941)
#define XENSIV_BGT60TR13C_CONF_CHIRP_REPETITION_TIME_S (0.000690)
#define XENSIV_BGT60TR13C_CONF_FRAME_REPETITION_TIME_S (0.002098)
#define XENSIV_BGT60TR13C_CONF_NUM_REGS (38)

/* Added this */
#define XENSIV_BGT60TR13C_IRQ_TRIGGER_FRAME_SIZE (1024)

const uint32_t radar_init_register_list[] = { 
	0x001E8270,
	0x02088210,
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
	0x2010C851,
	0x222FF41F,
	0x24005EF7,
	0x2C000490,
	0x3A000480,
	0x48000480,
	0x56000480,
	0x5811BE0E,
	0x5A58FC0A,
	0x5C03F000,
	0x5E787E1E,
	0x60CE378E,
	0x62000155,
	0x64000252,
	0x66000080,
	0x68000000,
	0x6A000000,
	0x6C000000,
	0x6E396B10,
	0x7E000100,
	0x8E000100,
	0x9E000100,
	0xAC000000,
	0xB6000000,
};

#endif /* BGT60TR13C_CONFIG_H */