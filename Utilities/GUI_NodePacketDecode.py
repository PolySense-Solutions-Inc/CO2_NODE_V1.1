import tkinter as tk
from tkinter import scrolledtext
import struct


# Function to decode CAL packet
def decode_cal_packet(hex_string):
     # Convert hex string to bytes
    data = bytes.fromhex(hex_string)

    # Ensure the data length is correct (32 bytes for this structure)
    if len(data) != 28:
        raise ValueError(f"Invalid packet length: {len(data)} bytes (expected 28 bytes).")

    # PKT_Type (1 byte at offset 0)
    pkt_type = data[0]

    # Bit-field byte (1 byte at offset 1)
    cal_status = data[1]
    cal_zero_armed = (cal_status >> 0) & 0x1
    cal_zero_done = (cal_status >> 1) & 0x1
    cal_zero_ok = (cal_status >> 2) & 0x1
    cal_back_armed = (cal_status >> 3) & 0x1
    cal_back_done = (cal_status >> 4) & 0x1
    cal_back_ok = (cal_status >> 5) & 0x1
    fact_reset = (cal_status >> 6) & 0x1
    unused_1 = (cal_status >> 7) & 0x1

    # Curr_Batt_mVolts (2 bytes at offset 2, little-endian)
    curr_batt_mvolts = int.from_bytes(data[2:4], byteorder='little')

    # CAL_Zero (2 bytes at offset 4, little-endian)
    cal_zero = int.from_bytes(data[4:6], byteorder='little')

    # CAL_BCC (2 bytes at offset 6, little-endian)
    cal_bcc = int.from_bytes(data[6:8], byteorder='little')

    # CAL_Trim (2 bytes at offset 8, little-endian)
    cal_trim = int.from_bytes(data[8:10], byteorder='little')

    # CO2_Readings (10 bytes total, 5 readings of 2 bytes each starting at offset 10, little-endian)
    co2_readings = [
        int.from_bytes(data[10:12], byteorder='little'),
        int.from_bytes(data[12:14], byteorder='little'),
        int.from_bytes(data[14:16], byteorder='little'),
        int.from_bytes(data[16:18], byteorder='little'),
        int.from_bytes(data[18:20], byteorder='little')
    ]

    # Filtered sensor data (4 bytes each, starting at offset 20)
    filtered_temp = struct.unpack('<f', data[20:24])[0]  # 4 bytes for float
    filtered_humidity = struct.unpack('<f', data[24:28])[0]  # 4 bytes for float


    return (
        f"  >  PKT_Type:            0x{pkt_type:02X}\n"
        f"  >  Curr_Batt_mVolts:    {curr_batt_mvolts} mV\n"
        f"  >  CAL_Zero:            {cal_zero}\n"
        f"  >  CAL_BCC:             {cal_bcc}\n"
        f"  >  CAL_Trim:            {cal_trim}\n"
        f"  >  CO2_Readings:        {co2_readings}\n"
        f"  >  Filtered Temp:       {filtered_temp} °C\n"
        f"  >  Filtered Humidity:   {filtered_humidity} %\n"
        f"  >  CAL_ZERO_ARMED:      {cal_zero_armed}\n"
        f"  >  CAL_ZERO_DONE:       {cal_zero_done}\n"
        f"  >  CAL_ZERO_OK:         {cal_zero_ok}\n"
        f"  >  CAL_BACK_ARMED:      {cal_back_armed}\n"
        f"  >  CAL_BACK_DONE:       {cal_back_done}\n"
        f"  >  CAL_BACK_OK:         {cal_back_ok}\n"
        f"  >  FACT_RESET:          {fact_reset}\n"
        f"  >  UNUSED_1:            {unused_1}\n"
    )


# Function to decode packet and display the output in the text box
def decode_data_packet():
    hex_string = entry.get().replace(' ', '')
    try:
        # Convert hex string to bytes
        data = bytes.fromhex(hex_string)

        # Ensure the data length is exactly 23 bytes
        if len(data) != 23:
            output_text.insert(tk.END, f"Invalid packet length: {len(data)} bytes (expected 23 bytes).\n")
            output_text.see(tk.END)  # Scroll to the bottom
            output_text.focus_set()  # Move focus to the output box
            return

        # PKT_Type is the first byte (data[0])
        pkt_type = data[0]

        if pkt_type == 0xDA:  # PKT_DAT
            output_text.insert(tk.END, f"\nDecoding Data Packet: {hex_string} ...\n")

            # Extract values from the packet
            k33_val = data[1]            # K33_VAL (1 byte)
            i2c_err = data[2]            # I2C_ERR (1 byte)
            k33_chk = data[3]            # K33_CHK (1 byte)

            # Extract the bit fields for Curr_BootReason (4 bits), Last_BootReason (4 bits), Wake_Count (15 bits), CRC_MISMATCH_FLAG (1 bit)
            metadata = int.from_bytes(data[4:6], byteorder='little')

            # Adjusted bit shifting to account for little-endian representation
            curr_boot_reason = (metadata & 0xF)               # Lowest 4 bits
            last_boot_reason = (metadata >> 4) & 0xF          # Next 4 bits
            wake_count = (metadata >> 8) & 0x7FFF             # Next 15 bits
            crc_mismatch_flag = (metadata >> 23) & 0x1        # Highest bit (23rd)

            # Extract battery measurements (uint16_t, little-endian)
            curr_batt_mvolts = int.from_bytes(data[7:9], byteorder='little')
            prev_batt_mvolts = int.from_bytes(data[9:11], byteorder='little')

            # Extract filtered sensor data (32-bit floats, little-endian)
            filtered_temp = struct.unpack('<f', data[11:15])[0]  # '<f' is for little-endian 32-bit float
            filtered_humidity = struct.unpack('<f', data[15:19])[0]
            filtered_co2 = struct.unpack('<f', data[19:23])[0]

            # Printing the decoded values to the output box
            output_text.insert(tk.END, f"  >  PKT_Type:            0x{format(pkt_type, 'X')}\n")
            output_text.insert(tk.END, f"  >  K33_VAL:             {k33_val}\n")
            output_text.insert(tk.END, f"  >  I2C_ERR:             {i2c_err}\n")
            output_text.insert(tk.END, f"  >  K33_CHK:             {k33_chk}\n")
            output_text.insert(tk.END, f"  >  Curr_BootReason:     {curr_boot_reason}\n")
            output_text.insert(tk.END, f"  >  Last_BootReason:     {last_boot_reason}\n")
            output_text.insert(tk.END, f"  >  Wake_Count:          {wake_count}\n")
            output_text.insert(tk.END, f"  >  CRC_MISMATCH_FLAG:   {crc_mismatch_flag}\n")
            output_text.insert(tk.END, f"  >  Curr_Batt_mVolts:    {curr_batt_mvolts} mV\n")
            output_text.insert(tk.END, f"  >  Prev_Batt_mVolts:    {prev_batt_mvolts} mV\n")
            output_text.insert(tk.END, f"  >  Filtered Temp:       {filtered_temp} °C\n")
            output_text.insert(tk.END, f"  >  Filtered Humidity:   {filtered_humidity} %\n")
            output_text.insert(tk.END, f"  >  Filtered CO2:        {filtered_co2} ppm\n(Done)\n")
        else:
            output_text.insert(tk.END, f"  >  Unknown packet type: {hex(pkt_type).upper()}\n")
    except (ValueError, struct.error, IndexError) as e:
        output_text.insert(tk.END, f"  >  Error decoding packet: {e}\n")
    finally:
        output_text.see(tk.END)  # Scroll to the bottom of the text box
        output_text.focus_set()  # Move focus to the output box


# Function to decode the packet based on the type
def decode_packet():
    hex_string = entry.get().replace(' ', '')
    try:
        data = bytes.fromhex(hex_string)

        # Get packet type from the first byte
        pkt_type = data[0]

        if pkt_type == 0xDA:  # PKT_DAT, 23 bytes
            if len(data) != 23:
                output_text.insert(tk.END, f"Invalid DAT packet length: {len(data)} bytes (expected 23 bytes).\n")
                return
            # Extract and display all the fields for PKT_DAT...
            decode_data_packet()

        elif pkt_type == 0xCA:  # PKT_CAL, 32 bytes
            result = decode_cal_packet(hex_string)
            output_text.insert(tk.END, f"\nDecoding Calibration Packet (PKT_CAL): {hex_string} ...\n")
            output_text.insert(tk.END, result)

        else:
            output_text.insert(tk.END, f"Unknown packet type: {hex(pkt_type).upper()}\n")

    except (ValueError, struct.error, IndexError) as e:
        output_text.insert(tk.END, f"Error decoding packet: {e}\n")
    finally:
        output_text.see(tk.END)  # Scroll to the bottom of the text box
        output_text.focus_set()  # Move focus to the output box


# Function to clear the input field when clicked
def clear_entry(event):
    entry.delete(0, tk.END)

# Create the main window
window = tk.Tk()
window.title("Packet Decoder")

# Create a label and entry widget for hex string input
entry_label = tk.Label(window, text="Packet Data (Hex):")
entry_label.grid(row=0, column=0, padx=5, pady=5)

# Create text entry field
entry = tk.Entry(window, width=50)
entry.grid(row=0, column=1, padx=5, pady=5)

# Bind the clear_entry function to the entry widget (clear on click)
entry.bind("<FocusIn>", clear_entry)

# Create a "Go" button
go_button = tk.Button(window, text="Go", command=decode_packet)
go_button.grid(row=0, column=2, padx=5, pady=5)

# Create a scrolled text widget for the output
output_text = scrolledtext.ScrolledText(window, width=80, height=20, wrap=tk.WORD)
output_text.grid(row=1, column=0, columnspan=3, padx=5, pady=5)

# Run the Tkinter event loop
window.mainloop()
