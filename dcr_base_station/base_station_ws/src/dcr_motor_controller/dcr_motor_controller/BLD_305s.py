import minimalmodbus
import serial

class Motor:
    def __init__(self, port, address=1):
        self.instrument = minimalmodbus.Instrument(port, address, debug=False)
        self.instrument.serial.baudrate = 9600 # TODO: Increase Baudrate of motors during provisioning stage
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = serial.PARITY_NONE
        self.instrument.serial.stopbits = 1
        self.instrument.serial.timeout = 0.1

        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        self.instrument.close_port_after_each_call = True

        try:
            # Goes to internal Control Mode
            self.instrument.write_register(0x0136, 0x01, 0, 0x06)
            # Sets # of Pole Pairs to 2
            self.instrument.write_register(0x0116, 0x02, 0, 0x06)
        except Exception as e:
            print(f"Error initializing motor: {e}")

    def Provision(self, address): # This code is unlikely to be used directly for the rover TODO: Likely remove and put into different tool
        if(address == None):
            raise "error?"
        try:
            self.instrument.write_register(0x00A6, address, 0, 0x06)  # Change address of motor
            self.instrument.address = address
            self.instrument.write_register(0x80FF, 0x55AA, 0, 0x06)  # Save change
        except Exception as e:
            print(f"Error provisioning motor: {e}")

    def Start(self, address=1, direction=0x01):
        try:
            self.instrument.address = address
            self.instrument.write_register(0x0066, direction, 0, 0x06)  # start
        except Exception as e:
            print(f"Error starting motor: {e}")

    def SetSpeed(self, RPM=0):
        try:
            self.instrument.write_register(0x0056, RPM, 0, 0x06)  # speed RPM
        except Exception as e:
            print(f"Error setting speed: {e}")

    def BroadcastSTOP(self):
        try:
            self.instrument.address = 0 # 0 is the broadcast address
            self.instrument.write_register(0x0066, 0, 0, 0x06)
        except Exception as e:
            print(f"ERROR: software emergency stop failed: {e}")
