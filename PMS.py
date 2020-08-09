import serial

class PMS5003:
    def __init__(self):
        self.ser = serial.Serial(
               port='/dev/serial0',
               baudrate = 9600,
               parity=serial.PARITY_NONE,
               stopbits=serial.STOPBITS_ONE,
               bytesize=serial.EIGHTBITS,
               timeout=5
           )

    def read(self):
        while self.ser.in_waiting>0:
            self.ser.reset_input_buffer()
        self.ser.read_until(bytes([66,77,0,28]))
        s=self.ser.read(24)      
        values = [s[n]*256+s[n+1] for n in range(0,24,2)]
        varnames = ['PM1p0_CF1','PM2p5_CF1','PM10_CF1',
                    'PM1p0','PM2p5','PM10',
                    'DB0p3um','DB0p5um','DB1um',
                    'DB2p5um','DB5um','DB10um']
        return {k:v for (k,v) in zip(varnames,values)}
 
                        
            
