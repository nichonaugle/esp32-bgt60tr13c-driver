DEFAULT_REGISTER_TXT = "."

def format_to_32_bit_hex(address, data):
    address_array = [int(bit) for bit in bin(address)[2:].zfill(8)]
    address_array = address_array[1:]
    data_array = [int(bit) for bit in bin(data)[2:].zfill(32)]
    data_array = data_array[7:]
    binary_array = address_array + data_array
    binary_string = ''.join(str(bit) for bit in binary_array)
    return f"0x{int(binary_string,2):08X}"

if __name__ == "__main__":
    formatted_hex_array = []
    with open(DEFAULT_REGISTER_TXT + "/exported_registers.txt", "r") as f:
        for line in f.readlines():
            sections = line.strip().split()
            if len(sections) >= 3:
                formatted_hex_array.append(format_to_32_bit_hex(int(sections[1], 16), int(sections[2], 16)))

    with open(DEFAULT_REGISTER_TXT + "/GENERATED_register_array.txt", "w") as f:
        f.write("const uint32_t register_list[] = { \n")
        for hex_content in formatted_hex_array:
            f.write("\t" + hex_content + ",\n")
        f.write("};")
    
    print(f"Array successfully written to GENERATED_register_array.txt \nArray size was {len(formatted_hex_array)}")

"""
EXAMPLE INFINEON CONFIG FILE:


#define XENSIV_BGT60TR13C_CONF_DEVICE (XENSIV_DEVICE_BGT60TR13C)
#define XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS (3)
#define XENSIV_BGT60TR13C_CONF_NUM_TX_ANTENNAS (1)
#define XENSIV_BGT60TR13C_CONF_START_FREQ_HZ (61020100000)
#define XENSIV_BGT60TR13C_CONF_END_FREQ_HZ (61479904000)
#define XENSIV_BGT60TR13C_CONF_NUM_SAMPLES_PER_CHIRP (64)
#define XENSIV_BGT60TR13C_CONF_NUM_CHIRPS_PER_FRAME (1)
#define XENSIV_BGT60TR13C_CONF_SAMPLE_RATE (2000000)
#define XENSIV_BGT60TR13C_CONF_CHIRP_REPETITION_TIME_S (6.945e-05)
#define XENSIV_BGT60TR13C_CONF_FRAME_REPETITION_TIME_S (0.00500396)
#define XENSIV_BGT60TR13C_CONF_NUM_REGS (38)

#if defined(XENSIV_BGT60TR13C_CONF_IMPL)

const uint32_t register_list[] = { 
    0x11e8270UL, 
    0x3088210UL, 
    0x9e967fdUL, 
    0xb0805b4UL, 
    0xdf0227fUL, 
    0xf010700UL, 
    0x11000000UL, 
    0x13000000UL, 
    0x15000000UL, 
    0x17000be0UL, 
    0x19000000UL, 
    0x1b000000UL, 
    0x1d000000UL, 
    0x1f000b60UL, 
    0x2110c851UL, 
    0x232ff41fUL, 
    0x25006f7bUL, 
    0x2d000490UL, 
    0x3b000480UL, 
    0x49000480UL, 
    0x57000480UL, 
    0x5911be0eUL, 
    0x5b44c40aUL, 
    0x5d000000UL, 
    0x5f787e1eUL, 
    0x61f5208cUL, 
    0x630000a4UL, 
    0x65000252UL, 
    0x67000080UL, 
    0x69000000UL, 
    0x6b000000UL, 
    0x6d000000UL, 
    0x6f092910UL, 
    0x7f000100UL, 
    0x8f000100UL, 
    0x9f000100UL, 
    0xad000000UL, 
    0xb7000000UL
};


"""