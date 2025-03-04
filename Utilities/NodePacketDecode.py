import random
import struct

# Define the Packet Types
PKT_DAT = 0xDA
PKT_INF = 0xF0
PKT_CAL = 0xCA

def decode_data_packet(packet):
    """Decode the received data packet and print out a summary of the results."""
    pkt_type = packet[0]
    
    if pkt_type == PKT_DAT:
        # Unpack data packet
        data = struct.unpack('<BIBHHfff', packet)
        pkt_type, meta, curr_batt, prev_batt, temp, humidity, co2 = data
        
        # Decode metadata
        K33_VAL = (meta >> 28) & 0xF
        I2C_ERR = (meta >> 24) & 0xF
        K33_CHK = (meta >> 21) & 0x7
        Curr_BootReason = (meta >> 17) & 0xF
        Last_BootReason = (meta >> 13) & 0xF
        CRC_MISMATCH_FLAG = (meta >> 12) & 0x1
        Wake_Count = meta & 0xFFF
        
        print("Data Packet:")
        print(f"  K33_VAL: {K33_VAL}")
        print(f"  I2C_ERR: {I2C_ERR}")
        print(f"  K33_CHK: {K33_CHK}")
        print(f"  Curr_BootReason: {Curr_BootReason}")
        print(f"  Last_BootReason: {Last_BootReason}")
        print(f"  CRC_MISMATCH_FLAG: {CRC_MISMATCH_FLAG}")
        print(f"  Wake_Count: {Wake_Count}")
        print(f"  Curr_Batt_mVolts: {curr_batt} mV")
        print(f"  Prev_Batt_mVolts: {prev_batt} mV")
        print(f"  Filtered Temp: {temp:.2f} C")
        print(f"  Filtered Humidity: {humidity:.2f} %")
        print(f"  Filtered CO2: {co2:.2f} ppm")
    
    elif pkt_type == PKT_INF:
        # Unpack info packet
        data = struct.unpack('<BIBBBBBB', packet)
        pkt_type, meta, K33_VAL, I2C_ERR, I2C_REINITS, RST_SW, RST_PANIC, RST_ALL_WDT, RST_DEEPSLEEP, RST_BROWNOUT = data
        
        print("Info Packet:")
        print(f"  K33_VAL: {K33_VAL}")
        print(f"  I2C_ERR: {I2C_ERR}")
        print(f"  I2C_REINITS: {I2C_REINITS}")
        print(f"  RST_SW: {RST_SW}")
        print(f"  RST_PANIC: {RST_PANIC}")
        print(f"  RST_ALL_WDT: {RST_ALL_WDT}")
        print(f"  RST_DEEPSLEEP: {RST_DEEPSLEEP}")
        print(f"  RST_BROWNOUT: {RST_BROWNOUT}")
    
    elif pkt_type == PKT_CAL:
        # Unpack calibration packet
        data = struct.unpack('<BH4H5f', packet)
        pkt_type, cal_flags, curr_batt, cal_zero, cal_bcc, cal_trim, *co2_readings, temp, humidity = data
        
        print("Calibration Packet:")
        print(f"  CAL_ZERO_ARMED: {bool(cal_flags & 0x01)}")
        print(f"  CAL_ZERO_DONE: {bool(cal_flags & 0x02)}")
        print(f"  CAL_ZERO_OK: {bool(cal_flags & 0x04)}")
        print(f"  CAL_BACK_ARMED: {bool(cal_flags & 0x08)}")
        print(f"  CAL_BACK_DONE: {bool(cal_flags & 0x10)}")
        print(f"  CAL_BACK_OK: {bool(cal_flags & 0x20)}")
        print(f"  FACT_RESET: {bool(cal_flags & 0x40)}")
        print(f"  Curr_Batt_mVolts: {curr_batt} mV")
        print(f"  CAL_Zero: {cal_zero}")
        print(f"  CAL_BCC: {cal_bcc}")
        print(f"  CAL_Trim: {cal_trim}")
        print(f"  CO2 Readings: {co2_readings}")
        print(f"  Filtered Temp: {temp:.2f} C")
        print(f"  Filtered Humidity: {humidity:.2f} %")
    
    else:
        print("Unknown Packet Type!")

def generate_random_packet():
    """Generate a random packet for testing."""
    pkt_type = random.choice([PKT_DAT, PKT_INF, PKT_CAL])
    
    if pkt_type == PKT_DAT:
        meta = random.randint(0, 0xFFFFFFFF)
        curr_batt = random.randint(3000, 4200)
        prev_batt = random.randint(3000, 4200)
        temp = round(random.uniform(-10.0, 40.0), 2)
        humidity = round(random.uniform(0.0, 100.0), 2)
        co2 = round(random.uniform(300.0, 5000.0), 2)
        packet = struct.pack('<BIBHHfff', pkt_type, meta, curr_batt, prev_batt, temp, humidity, co2)
    
    elif pkt_type == PKT_INF:
        meta = random.randint(0, 0xFFFFFFFF)
        K33_VAL = random.randint(0, 0xFFFFFFFF)
        I2C_ERR = random.randint(0, 0xFFFFFFFF)
        I2C_REINITS = random.randint(0, 0xFFFFFFFF)
        RST_SW = random.randint(0, 0xFFFFFFFF)
        RST_PANIC = random.randint(0, 0xFFFFFFFF)
        RST_ALL_WDT = random.randint(0, 0xFFFFFFFF)
        RST_DEEPSLEEP = random.randint(0, 0xFFFFFFFF)
        RST_BROWNOUT = random.randint(0, 0xFFFFFFFF)
        packet = struct.pack('<BIBBBBBB', pkt_type, meta, K33_VAL, I2C_ERR, I2C_REINITS, RST_SW, RST_PANIC, RST_ALL_WDT, RST_DEEPSLEEP, RST_BROWNOUT)
    
    elif pkt_type == PKT_CAL:
        cal_flags = random.randint(0, 0xFF)
        curr_batt = random.randint(3000, 4200)
        cal_zero = random.randint(0, 500)
        cal_bcc = random.randint(0, 500)
        cal_trim = random.randint(0, 500)
        co2_readings = [round(random.uniform(300.0, 5000.0), 2) for _ in range(5)]
        temp = round(random.uniform(-10.0, 40.0), 2)
        humidity = round(random.uniform(0.0, 100.0), 2)
        packet = struct.pack('<BH4H5f', pkt_type, cal_flags, curr_batt, cal_zero, cal_bcc, cal_trim, *co2_readings, temp, humidity)
    
    return packet

def test_callback():
    """Test the callback function with random packets."""
    for _ in range(5):  # Generate and test 5 random packets
        packet = generate_random_packet()
        decode_data_packet(packet)

# Run the test
test_callback()
