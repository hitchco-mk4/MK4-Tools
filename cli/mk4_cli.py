from serial import Serial
from serial.tools.list_ports import comports
import struct
from time import sleep
import struct
import sys


BLOCK_SIZE = 64


class DataBlock(object):

    def __init__(self, in_bytes):

        self.in_bytes = in_bytes

        self.f0 = struct.unpack('<f', bytearray(self.in_bytes[0:4]))[0]
        self.f1 = struct.unpack('<f', bytearray(self.in_bytes[4:8]))[0]
        self.f2 = struct.unpack('<f', bytearray(self.in_bytes[8:12]))[0]
        self.f3 = struct.unpack('<f', bytearray(self.in_bytes[12:16]))[0]
        self.f4 = struct.unpack('<f', bytearray(self.in_bytes[16:20]))[0]
        self.f5 = struct.unpack('<f', bytearray(self.in_bytes[20:24]))[0]
        self.f6 = struct.unpack('<f', bytearray(self.in_bytes[24:28]))[0]
        self.f7 = struct.unpack('<f', bytearray(self.in_bytes[28:32]))[0]
        self.f8 = struct.unpack('<f', bytearray(self.in_bytes[32:36]))[0]
        self.f9 = struct.unpack('<f', bytearray(self.in_bytes[36:40]))[0]
        self.f10 = struct.unpack('<f', bytearray(self.in_bytes[40:44]))[0]
        self.f11 = struct.unpack('<f', bytearray(self.in_bytes[44:48]))[0]

        self.i0 = struct.unpack('<h', bytearray(self.in_bytes[48:50]))[0]
        self.i1 = struct.unpack('<h', bytearray(self.in_bytes[50:52]))[0]
        self.i2 = struct.unpack('<h', bytearray(self.in_bytes[52:54]))[0]
        self.i3 = struct.unpack('<h', bytearray(self.in_bytes[54:56]))[0]

        self.b0 = struct.unpack('B', bytearray([self.in_bytes[56]]))[0]
        self.b1 = struct.unpack('B', bytearray([self.in_bytes[57]]))[0]
        self.b2 = struct.unpack('B', bytearray([self.in_bytes[58]]))[0]
        self.b3 = struct.unpack('B', bytearray([self.in_bytes[59]]))[0]
        self.b4 = struct.unpack('B', bytearray([self.in_bytes[60]]))[0]
        self.b5 = struct.unpack('B', bytearray([self.in_bytes[61]]))[0]
        self.b6 = struct.unpack('B', bytearray([self.in_bytes[62]]))[0]

        self.incoming_crc = struct.unpack('B', bytearray([self.in_bytes[63]]))[0]
        self.calculated_crc = self.calc_crc(in_bytes[0:BLOCK_SIZE-1])
        self.crc_pass = self.incoming_crc == self.calculated_crc

    def calc_crc(self, list_of_bytes):
        crc = 0
        for num in list_of_bytes:
            crc = (crc + num) % 256
        return crc

    def print(self):

        print("Raw Bytes: ")
        print_string = ""
        for b_index, b in enumerate(self.in_bytes):
            print_string += str(b) + ","
        print(print_string)

        floats = [self.f0, self.f1, self.f2, self.f3, self.f4, self.f5, self.f6, self.f7, self.f8, self.f9, self.f10, self.f11]
        print("Floats:")
        for f_index, f in enumerate(floats):
            print_string = ""
            print_string += "\t"
            print_string += str(f_index)
            print_string += " - ["
            print_string += str(f)
            print_string += "]"
            print(print_string)

        ints = [self.i0, self.i1, self.i2, self.i3]
        print("Ints:")
        for i_index, i in enumerate(ints):
            print_string = ""
            print_string += "\t"
            print_string += str(i_index)
            print_string += " - ["
            print_string += str(i)
            print_string += "]"
            print(print_string)

        bytez = [self.b0, self.b1, self.b2, self.b3, self.b4, self.b5, self.b6]
        print("Bytes:")
        for b_index, b in enumerate(bytez):
            print_string = ""
            print_string += "\t"
            print_string += str(b_index)
            print_string += " - ["
            print_string += str(b)
            print_string += "]"
            print(print_string)

        print("Meta:")
        print("\tIncoming CRC:", self.incoming_crc)
        print("\tCalculated CRC:", self.calculated_crc)
        print("\tCRC pass:", self.crc_pass)


if __name__ == "__main__":

	try:
		target_device = sys.argv[1]
	except IndexError:
		
		available_ports = comports()
		
		port_names = {}
		print("Available Serial Ports: ")
		for number, port in enumerate(available_ports):
			print("[" + str(number) + "] - " + str(port))
			port_names[number] = port.device

		raw_device = input("Select a device: ")

		target_device = None
		try:
			device_number = int(raw_device)
			target_device = port_names[device_number]
		except KeyError:
			print("Invalid device, exiting.")
			exit()
		except ValueError:
			print("Invalid input, exiting.")
			exit()

	print("Selected [" + str(target_device) + "] waiting for ready.")

	s = Serial(target_device, 9600, timeout=None)

	sleep(1)

	junk_count = 0

	while s.in_waiting > 0:
		junk_count += 1
		s.read()

	print("Cleared", junk_count, "bytes of junk on init")
	
	while True:

		command = input("Send Command To Arduino (enter q to exit):")
		
		if command.rstrip() is "q":
			break

		s.write(command.rstrip().encode())

		new_block = DataBlock(s.read(BLOCK_SIZE))

		new_block.print()

		while s.in_waiting > 0:
			s.read()
	
	print("bye")
	
	s.close()