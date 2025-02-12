This is where if gets tricky. If youa re going to use this script, place all of your registers exported from the RADAR IDE provided by Infineon. This will generate the register initialization array and set that up correctly, however it will not calcualte the top level definitions as seen below: 


#define XENSIV_BGT60TR13C_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define XENSIV_BGT60TR13C_CONF_START_FREQ_HZ (61020100000)
#define XENSIV_BGT60TR13C_CONF_END_FREQ_HZ (61479904000)
#define XENSIV_BGT60TR13C_CONF_NUM_SAMPLES_PER_CHIRP (128)
#define XENSIV_BGT60TR13C_CONF_NUM_CHIRPS_PER_FRAME (64)
#define XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS (3)
#define XENSIV_BGT60TR13C_CONF_NUM_TX_ANTENNAS (1)
#define XENSIV_BGT60TR13C_CONF_SAMPLE_RATE (2352941)
#define XENSIV_BGT60TR13C_CONF_CHIRP_REPETITION_TIME_S (6.945e-05)
#define XENSIV_BGT60TR13C_CONF_FRAME_REPETITION_TIME_S (0.00500396)
#define XENSIV_BGT60TR13C_CONF_NUM_REGS (38)

Although the following parameters are constant for this file: 

#define XENSIV_BGT60TR13C_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS (3)
#define XENSIV_BGT60TR13C_CONF_NUM_TX_ANTENNAS (1)

The rest of them will change based on the inputs into the RADAR designing IDE. You can extract those values and write over the existing ones above in bgt60tr13c_config.h