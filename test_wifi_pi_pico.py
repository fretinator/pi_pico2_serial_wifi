import time
import utime
import network
import socket
import uerrno
from machine import UART,Pin,I2C
import ssd1306

BAUDS = [300,1200,2400,4800,9600,19200, 38400,57600,115200]
DEFAULT_BAUD_SEL = 1
MAX_BAUD_SEL = 8
current_baud_sel = DEFAULT_BAUD_SEL
baud_change_pin = Pin(16, Pin.IN, Pin.PULL_UP)
FAILURE_RETRY_INTERVAL = 5 # seconds
SETTING_CHANGE_DELAY = 1 # second
IRQ_SLEEP = 400 # millis, prevent button bounce

do_reset = False
RESET_ERROR = "Reset" # IRQ fired because settings changed

def baud_change_handler(pin):
    global current_baud_sel,do_reset

    # Ignore if in the process of resetting
    if not do_reset:
        baud_change_pin.irq(handler = None)
        
        if current_baud_sel == MAX_BAUD_SEL:
            current_baud_sel = 0
        else:
            current_baud_sel += 1
        
        do_reset = True
        utime.sleep_ms(IRQ_SLEEP)
        baud_change_pin.irq(trigger=Pin.IRQ_RISING, handler= baud_change_handler)
    
    
    
    

def printScreen(line1: str, line2 = "", line3 = ""):
    display.fill(0)
    
    if not type(line1) is None and len(line1) > 0:
        display.text(line1, 0, 0, 1)
    
    if not type(line2) is None and len(line2) > 0:
        display.text(line2, 0, 12, 1)
        
    if not type(line3) is None and len(line3) > 0:
        display.text(line3, 0, 24, 1)
        
    display.show()

class SerialWifi:
    rs232: UART # Assumed already initialized
    sckt: Socket.Socket #
    host: str
    port: int
    buffer_size: int
    retries: int
    
    
    def __init__(
        self,
        rs232: UART,
        host: str,
        port: int,
        buffer_size: int,
        retries: int):
        
        self.rs232 = rs232
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.retries = retries
       
    def connect(self):
        addressInfo = socket.getaddrinfo(self.host, self.port)
        addr = addressInfo[0][-1]

        try:
            print("Obtaining socket connection");

            self.sckt = socket.socket()
            self.sckt.connect(addr)
            self.sckt.setblocking(False)
            
            return True
        except OSError as e:
            print(str(e))
            return False
        
    def run_service(self):
        global do_reset
        
        while(True):
            cur_tries: int
            
            if do_reset:
                try:
                    self.sckt.close()
                    self.rs232.deinit()
                except OSError as e:
                    print("Error closing bridge resources: " + str(e))
                
                do_reset = False
                
                return RESET_ERROR
            
            #data from serial port
            bytes_read = self.rs232.read(self.buffer_size)
            
            if not bytes_read is None and len(bytes_read) > 0:
                bytes_written = self.sckt.write(bytes_read)
            
                #print("SERIAL -> SOCKET: " + str(len(bytes_read)) + " read, " + str(bytes_written) + " written.") 

            #data from socket
            try:
                bytes_read = self.sckt.recv(self.buffer_size)
                if not bytes_read is None and len(bytes_read) > 0:
                    bytes_written = self.rs232.write(bytes_read)
                    
                    #print("SOCKET -> SERIAL: " + str(len(bytes_read)) + " read, " + str(bytes_written) + " written.") 
            except OSError as e:
                err = e.args[0]
                if not (uerrno.EAGAIN == err or uerrno.EWOULDBLOCK == err):
                    print("Error reading network socket: " + str(err) + " - " + str(e))
                    
                    cur_tries = 0
                    connected = False
                    
                    while not connected and cur_tries < self.retries:
                        connected = connect()
                        cur_tries += 1
                        
                        if not connected:
                            time.sleep(1)
                            
                    if not connected:
                        return "Comm error."
                            
                            
                              
#display
i2c = I2C(id=1,sda = Pin(2), scl = Pin(3))
display = ssd1306.SSD1306_I2C(128, 32, i2c)

display.poweron()

printScreen("Starting WiFi...")
        
baud_change_pin.irq(trigger=Pin.IRQ_RISING, handler= baud_change_handler)        
    
ssid = 'MyCharterWiFi49-2G'
password = 'LazarusRises971'


wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(ssid, password)

# Wait for connect or fail
print("Connecting to WiFi...")
strIP = "Not connected"

max_wait = 10
while max_wait > 0:
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    max_wait -= 1
    print('waiting for connection...')
    time.sleep(1)

# Handle connection error
if wlan.status() != 3:
    raise RuntimeError('network connection failed')
else:
    print('connected')
    
    status = wlan.ifconfig() 
    
    strIP = 'IP: ' + status[0]
    print( strIP )
    printScreen( strIP )
    time.sleep(1)

while(True):
    printScreen("Starting serial connection...")

    baud = BAUDS[current_baud_sel]

    #Serial connections
    serial1 = UART(0, baud)
    serial1.init(baudrate=baud, bits=8, parity=None, stop=1, tx=Pin(0),
                 rx=Pin(1), timeout=10, timeout_char=10, flow=0,rxbuf=512,txbuf=512)

    printScreen(strIP, "Baud: " + str(baud))


    #HTTP socket
    bridge = SerialWifi(rs232=serial1,host="192.168.1.233", port=8080, buffer_size=512, retries=3)
    try:
        if bridge.connect():
            err = bridge.run_service()
            
            if err == RESET_ERROR:
                printScreen("Setting change...")
                
        else:
            printScreen("Error connecting. ", "Retry in " + str(FAILURE_RETRY_INTERVAL) + "s.")
            time.sleep(FAILURE_RETRY_INTERVAL)
    except OSError as e:
        msg = str(e)
        printScreen("Error occurred: ", msg[0:20])
        time.sleep(FAILURE_RETRY_INTERVAL)