import struct

packed = struct.pack('<f', 0.05)
print(packed.hex())

bytes.fromhex('003f0000')
value = struct.unpack('<f', bytes.fromhex('003f0000'))[0]
print(value) 