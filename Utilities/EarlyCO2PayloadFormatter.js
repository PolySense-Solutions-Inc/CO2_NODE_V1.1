function decodeUplink(input) {
    const bytes = input.bytes;

    // Check that bytes are available
    if (!bytes || bytes.length === 0) {
        return {
            data: {},
            warnings: ['No bytes received'],
            errors: [],
        };
    }

    // Example: Decode CO2 value from the last 4 bytes
    if (bytes.length >= 4) {
        const co2Bytes = bytes.slice(-4);  // Last 4 bytes
        const co2DataView = new DataView(new ArrayBuffer(4));
        co2Bytes.forEach((byte, index) => co2DataView.setUint8(index, byte));

        // Extract the CO2 value as a 32-bit float (little-endian)
        let co2Value = co2DataView.getFloat32(0, true);

        // Multiply the CO2 value by 10
        //co2Value *= 10;

        return {
            data: {
                co2: co2Value,
            },
            warnings: [],
            errors: [],
        };
    } else {
        return {
            data: {},
            warnings: ['Not enough bytes for CO2 value'],
            errors: [],
        };
    }
}
