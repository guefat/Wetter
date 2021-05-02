# last changes 2.7.19  GF  - Faktor in Localdose von 1.3630152978 auf 1.2630152978 gesetzt
# last changes 11.9.19 GF - Versuche mit Schneehöhe und UVI
# last changes 21.10.20 GF - Neuer Garmind Lidar Lite v4 LED Sensor und Konsolidierung aller Libraries in GFSensorLib
import os,time
from time import sleep
import keyboard
import datetime
import math
import MySQLdb as mysql
import RPi.GPIO as GPIO
from GFSensorLib import bme280, bh1750, ads1015 , veml6075, pms5003, lidarv4
from statistics import mean
import json

f = open('config.json')
Qconfig = json.loads(f.read())
f.close()

GPIO.setwarnings(False)
pms = pms5003()                                     #Partikelsensor
sensor = bme280(0x76)                               #Sensor für Temp,Druck,Feuchte
lightsensor = bh1750(addr=0x23)                     #Helligkeitssensor
lightsensor.sensitivity(0.5)
uvsensor=veml6075()                                 #UV-Sensor
lidar = lidarv4()                                  #Garmin Lidar Lite v4 LED
GPIO.setmode(GPIO.BCM)                              #BCM bedeutet gültig sind GPIO Nummern
GPIO.setup(5, GPIO.IN, pull_up_down = GPIO.PUD_UP)  #Regenpulse (Pin29)
GPIO.setup(16, GPIO.OUT)                            #Heizungssteuerung für Regenmesser(Pin 36)
GPIO.setup(26, GPIO.OUT)                            #Raspi-Lüfter-Steuerung (Pin 37)
adc0 = ads1015(0x48)                                #Adresse von ADC 0
adc1 = ads1015(0x49)                                #Adresse von ADC 1

# Zeit zwischen Datenbank Eintraegen in Sekunden:
WARTEZEIT = 60


a = 7.5             #Konstante für Taupunktberechnung           
b = 273.3           #Konstante für Taupunktberechnung
gam = 0.0065        #Konstante für Druckumrecnung auf NN
e = 2.718281828     #Konstante für Druckumrecnung auf NN
Exp= -5.255         #Konstante für Druckumrecnung auf NN
h = 640             #Station über NN
volt = gray = Vmax = Vmaxx = Counter = rainamount = rh = 0
Tmin = Tminn = 50
Tmax = Tmaxx = -50
c1=-8.784695
c2=1.61139411
c3=2.338549
c4=-0.14611605
c5=-0.012308094     #Konstante zur Hitzeindexberechnung
c6=-0.016424826
c7=0.002211732
c8=0.00072546
c9=-0.000003582
lst_Wind=[]         #Def Liste Windaverage 

tmaxtime = (datetime.datetime.now().strftime("%H:%M"))
tmintime = (datetime.datetime.now().strftime("%H:%M"))
vmaxtime = (datetime.datetime.now().strftime("%H:%M"))



def readWindSpeed():                    #Funktion Windgeschwindigkeit (Adafruit Anemometer)
    UwindSpeed=0
    adc0.set_input(1)
    adc0.set_gain(1)    
    UwindSpeed = adc0.voltage()
    UwindSpeed = UwindSpeed - 0.4       #0.4 Volt entspricht 0 m/s  ???
    if UwindSpeed < 0:
        UwindSpeed = 0
    V=UwindSpeed*32.4/1.2             # (/1.2) Umrechnung Spannung auf Geschwindigkeit m/s 
    V = V *3.6                          #Umrechnung auf km/hv
    return V

def readwindDir():                      #Funktion für Windrichtung 
    voltage = 0
    adc0.set_input(2)
    adc0.set_gain(0)
    voltage = adc0.voltage()
    return voltage

def DecWindDir(U_WindDir, error_string=''):#Windrichtungswerte Kategorien zuweisen
    WindDirTable = [[3.74, 3.94, 'N'], [1.78, 2.08, 'NNO'], [2.15, 2.35, 'NO'],
                    [0.47, 0.5, 'OON'], [0.5, 0.56, 'O'], [0.405, 0.452, 'OOS'],
                    [0.8, 1.0, 'SO'], [0.59, 0.71, 'SSO'], [1.38, 1.58, 'S'],
                    [1.1, 1.35, 'SSW'], [2.98, 3.18, 'SW '], [2.83, 3.03, 'WWS'],
                    [4.5, 4.64, 'W'], [3.94, 4.14, 'WWN'], [4.21, 4.41, 'NW'],
                    [3.33, 3.53, 'NNW']]
    for WindDirEntry in WindDirTable:
        if (U_WindDir > WindDirEntry[0]) & (U_WindDir < WindDirEntry[1]):
            return WindDirEntry[2]
    return error_string
    


def readlocaldose():                    #Funktion Radioaktivität (SV500 - Feinsonde FHZ 72T)
    adc0.set_input(0)
    adc0.set_gain(0)
    volt = 0
    volt = adc0.voltage()
    sleep(0.7501)
    gray = 0.020692402*e**(1.2630152978*volt)
    return gray


def readSupply():                       #Funktion Systemspannungsdaten (ADS1015)
    adc1.set_input(0)
    adc1.set_gain(1)
    Ubat=adc1.voltage()                 #Batteriespannung
    ub=Ubat*182/33                      #Spannungsteilerberechnung U
    adc1.set_input(1)
    Uusv= adc1.voltage()                #USV-Spannung
    uusv=Uusv*183/33
    adc1.set_input(2)
    Ufan=adc1.voltage()                 #Spannung für Ventilator
    ufan=Ufan*120/38.3
    return ub,uusv,ufan


def Dec_uvi_read(uvi, error_string=''):#Funktion UVI-Daten Kategorien zuweisen
    uviTable = [[0.5, 2.2, 'Niedrig'],[2.3, 5.75, 'Moderat'], 
        [5.76, 7.75, 'Hoch'],[7.76, 10.3, 'Sehr hoch'],[10.31, 12.3, 'Extrem']]
        
    for uviEntry in uviTable:
        if (uvi >= uviEntry[0]) & (uvi < uviEntry[1]):
            return uviEntry[2]
    return error_string


def Interrupt(channel):                 #Funktion Interrupt Regenpulse
    global Counter
    Counter = Counter + 1    
GPIO.add_event_detect(5, GPIO.RISING, callback = Interrupt, bouncetime = 250)     


def calc_taupunkt(T,rh):                #Funktion Berechnung Taupunkt
    v = (a*T)/(b+T)+math.log10(rh/100)
    dewpoint = b*v/(a-v)
    return dewpoint


def calc_windchill(T,V):                #Funktion Berechnung Windchill
    chill = 13.12 + 0.6215*T + (0.3965*T-11.37)*V**0.16
    return chill


def Hitzeindex(T,rh):
    hi=c1+c2*T+c3*rh+c4*T*rh+c5*T**2+c6*rh**2+c7*T**2*rh+c8*T*rh**2+c9*T**2*rh**2
    if(T<27):
        return hi
    else:
        return T

snowlevel = 0
t0 = time.time()                        #Zeit jetzt
while True:
        
        T,p,rh = sensor.read()          #Auslesen von Temperatur, Druck, Luftfeuchte (BME280)    
        Tmax = max(T,Tmax)
        Tmin = min(T,Tmin)
        dewpoint = calc_taupunkt(T,rh)  #Auslesen Taupunkt
        Tk = 273.15 + T                 #Temperatur absolut
        pnn = p*((Tk/(Tk+gam*h))**Exp)  #Druckumrechnung auf NN
        particles = pms.read()          #Auslesen des Partikelsensors (PMS5003)
        old_snowlevel = snowlevel
        snowlevel = lidar.read()       #Auslesen der Schneehöhe
        snowlevel = 0
        if snowlevel>10000:
            snowlevel = old_snowlevel
        
        lightlevel = lightsensor.read() #Auslesen der Beleuchtungsstärke (BH1050)
        V = readWindSpeed()             #Auslesen der Windgeschwindigkeit (Adafruit Anempmeter)                      
        sleep(0.02)
        lst_Wind.append(V)              #Momentane Windgeschwindigkeit in Liste lst_Wind schreiben        
        Vmax = max(V,Vmax)              #Merken der Windspitze
        chill = calc_windchill(T,V)     #Gefühlte Temperatur (abhängig von T und Windgeschw.) - im Winter
        hi = Hitzeindex(T,rh)           #Gefühlte Temperatur (abhängig von T und Feuchte) - im Sommer
        U_WindDir = readwindDir()       #Auslesen Windrichtung
        WindDirEntry = DecWindDir(U_WindDir,error_string='')#Decodieren Windrichtung
        gray = readlocaldose()          #Radioaktivität Ortsdosis (SV500 - Feinsonde FHZ 72T)
        rainamount = Counter*0.3        #Regenmenge in mm         
        uvi=0
        uvi = uvsensor.read()
        uvi = uvi+1.9                     #Auslesen des UV-Sensors (veml6075)
        if (uvi <= 1.5): 
            uvi = 0
        uviEntry = Dec_uvi_read(uvi,error_string='') #Decodieren der UVI-Werte
        ub,uusv,ufan = readSupply()     #Auslesen der Systemspannungen        
        dev = os.popen('/opt/vc/bin/vcgencmd measure_temp')#Auslesen Prozessortemperatur
        cpu_temp_s = dev.read()[5:-3]
        cpu_temp = float(cpu_temp_s)

        if (ub>40):
            ub = 0
        if (cpu_temp > 50):                 #Temperaturbedingung für Raspilüfter
            GPIO.output(26, True)
        elif(cpu_temp < 45):
            GPIO.output(26,False)

##        if (T <= 1.5):                      #Temperaturbedingung für Heizung Regenmesser
##            GPIO.output(16, True)
##        elif(T > 2):
##            GPIO.output(16,False)
            

        t=(datetime.datetime.now().strftime("%H:%M")) #Zurückstellen von Tmax und Tmin des letzten Tages
        if(t >= ("00:00")) and (t <("00:02")):
            Tmax = -50
            Tmaxx = -50
            Tmin = 50
            Tminn = 50
            Vmax = 0
            Vmaxx = 0
            rainamount = 0
            Counter=0

        if (time.time()-t0 > WARTEZEIT):    #Zeit größer x Sekunden dann Ausgabe      
            t0 = time.time()
            V = (mean(lst_Wind))            #Mittelwert der Liste Wind
            
            #Ausgeben der Werte auf Schirm 
            print(time.strftime("\n%d.%m.%Y %H:%M:%S\n"))          
            print("CPU_Temp = %0.1f °C" % (cpu_temp))
            print("Solar = %0.1f V USV = %0.1f V Fan = %0.1f V\n" % (ub,uusv,ufan))            
            print("Lufttemperatur = %0.1f °C " % (T))
            if (Tmax != Tmaxx):
                tmaxtime = (datetime.datetime.now().strftime("%H:%M"))                
                Tmaxx = Tmax
            print("Temperaturmaximum war %0.1f °C um %s" % (Tmax,tmaxtime))
            if (Tmin != Tminn):
                tmintime = (datetime.datetime.now().strftime("%H:%M"))
                Tminn = Tmin
            print("Temperaturmimimum war %0.1f °C um %s" % (Tmin,tmintime))            
            if (T < 10 and V > 5):          #Gefühlte Temperatur wird nur unter diesen Bedingungen ausgegeben
                print("Gefühlte Temperatur = %2.1f °C" % (chill))
            if (T > 27 and rh > 40):
                print("Gefühlte Temperatur = %2.1f °C" % (hi))
            if (Vmax != Vmaxx):
                vmaxtime = (datetime.datetime.now().strftime("%H:%M"))                
                Vmaxx = Vmax
            print("Luftfeuchte = %0.0f %%" % (rh))
            print("Taupunkt = %2.1f °C" % (dewpoint))
            print("Spread = %2.1f °C (Lufttemperatur - Taupunkt)" % (T-dewpoint))
            print("LuftdruckStation = %0.1f hPa" % (p))      
            print("LuftdruckMeereshöhe = %7.1f hPa"%(pnn))
            print("Windgeschwindigkeit Durchschnitt = %3.1f km/h" % (V))
            print("Maximale Windböe %0.0f km/h war um %s" % (Vmax,vmaxtime))
            print("Windrichtung = %s" % (WindDirEntry))
            print("Regenmenge letzter Tag = %0.2f mm" % (rainamount))
            print("Schneehöhe = %0.0f cm" % (snowlevel))
            print("Beleuchtungsstärke = %0.0f lx" % (lightlevel))
            print("UV-Index = %0.1f (%s)" % (uvi,uviEntry))
            print("Feinstaub: PM1p0 = %d µg/m3 PM2p5 = %d µg/m3 PM10 = %d µg/m3" % (particles['PM1p0'],particles['PM2p5'],particles['PM10']))
            print("Radioaktivität = %0.3f µGy" % (gray))                       

            #Ausgabe in SQL-Datenbank
            db=mysql.connect(user="pi",passwd="raspberry",db="weatherdb") 
            cursor=db.cursor()

            columns = ', '.join([q['column'] for q in Qconfig])
            values = ', '.join([q['format'] % eval(q['variable']) for q in Qconfig])
            query = "insert into data (datetime, " + columns + ") " + "values(now(), " + values + ")"
                          
            lst_Wind=[] #Liste leeren
            cursor.execute(query)
            db.commit()
            cursor.close
            db.close
##            GPIO.cleanup() 





