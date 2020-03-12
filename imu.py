import os
import serial
import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from math import pi, sin, cos, radians, atan2, atan, sqrt, degrees

class Imu():
    '''
    Classe principal da IMU


    Modelo da IMU: MPU9250 (https://www.sparkfun.com/products/13762)
    Controlador: Beaglebone Black REV-C (https://www.sparkfun.com/products/12857)
    Biblioteca utilizada na IMU: https://github.com/jefmenegazzo/MPU9250

    '''

    def __init__(self):
        '''
        Inicializando a IMU, (MPU9250), na configuração padrão. 
        Bus usado: 2
        '''
        self.mpu = MPU9250(
        address_ak=AK8963_ADDRESS, 
        address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
        address_mpu_slave=None, 
        bus=2, 
        gfs=GFS_1000, 
        afs=AFS_8G, 
        mfs=AK8963_BIT_16, 
        mode=AK8963_MODE_C100HZ)

        self.mpu.configure()

        '''Iniciando variáveis'''

        self.accel = 0
        self.gyro = 0
        self.mag = 0
        self.temp = 0
        self.readAllRawData()

        # to-do: calibração do sensores....

    def readAllRawData(self):
        '''
        Função que atualiza os valores lidos pelos sensores nas variáveis.
        Cada função retorna 3 valores, (X, Y, Z) que podem ser acessados da seguinte forma:
        EX: self.accel[0] ---> valor da coordenada X do acelerômetro
            self.accel[1] ---> valor da coordanada Y do acelerômetro
            self.accel[2] ---< valor da coordenada Z do acelerôemtro


        OBS: o valor retornado pela temperatura é apenas um.
        '''
        self.accel = self.mpu.readAccelerometerMaster()
        self.gyro = self.mpu.readGyroscopeMaster()
        self.mag = self.mpu.readMagnetometerMaster()
        self.temp = self.mpu.readTemperatureMaster()

    def readAcellRaw(self):
        '''
        Atualiza individualmente os valores do acelerômetro. 
        UNIDADE: g (1g = 9.80665 m/s²)
        '''
        self.accel = self.mpu.readAccelerometerMaster()
        return self.accel

    def readGyroRaw(self):
        '''
        Atualiza individualmente os valores do giroscópio. 
        UNIDADE: graus por segundo (°/s)
        '''
        self.accel = self.mpu.readGyroscopeMaster()
        return self.gyro

    def readMag(self):
        '''
        Atualiza individualmente os valores do magnetômetro
        UNIDADE: microtesla (μT)
        '''
        self.accel = self.mpu.readMagnetometerMaster()
        return self.mag

    def readTempRaw(self):
        '''
        Atualiza individualmente os valores do termômetro
        UNIDADE: celsius degrees (°C)
        '''
        self.accel = self.mpu.readTemperatureMaster()
        return self.temp

    def readMagBearing(self):
        '''
        Retorna o valor do ângulo em relação ao norte magnético. 
        
        PROBLEMAS: 
        -Quando a IMU está nivelada, paralela ao chão, os valores retornados são corretos, porém quando
        a IMU é inclianda os valores começam a apresentar vários erros porque esse cálculo não leva em consideração o pitch e o roll.
        -Quando algum metal se aproxima da IMU, há uma distorção no campo magnético e o valores lidos pelo sensor são alterados.


        OBS: a biblioteca math do python calcula todos os valores de ângulo em radianos, e os valores da IMU são em graus,
        então deve-se converter antes de realizar os cálculos.
        '''
        mag_x = radians(self.mag[0])
        mag_y = radians(self.mag[1])

        bearing = degrees(atan2(mag_y, mag_x))
        if bearing < 0:
            return bearing + 360
        else:
            return bearing

    def readCompensatedBearing(self, pitch, roll):
        '''
        Retorna os valores do ângulo em relação ao norte magnético. 

        Esse cálculo leva em consideração a inclinação do IMU em relação ao chão, e compensa os valores para que ele se mantenha
        o mesmo não importa o quanto se incline a IMU.

        PROBLEMAS: 
        -Os valores retornados não são tão confiáveis assim. Se apontar a IMU para um ponto, o valor retornado será por exemplo
        120 graus. Se girarmos/balançarmos a IMU e retornar para o mesmo lugar apontado anteriormente, o valor não será o mesmo que o anterior. 
        '''
        cos_pitch = (cos(pitch))
        sin_pitch = (sin(pitch))
        
        cos_roll = (cos(roll))
        sin_roll = (sin(roll))

        Xh = (self.mag[0] * cos_pitch) + (self.mag[1] * sin_roll * sin_roll) + (self.mag[2] * cos_roll * sin_pitch)
        Yh = (self.mag[1] * cos_roll) - (self.mag[2] * sin_roll)
        
        bearing = atan2(Yh, Xh)
        if bearing < 0:
            return bearing + 360
        else:
            return bearing

    def distance(self, x, y):
        '''
        Retorna a distância entre dois pontos em um espaço 2d

        '''
        return sqrt((x * x) + (y * y))

    def read_x_rotation(self, x, y, z):
        '''
        Retorna a rotação entorno do eixo X em radianos
        '''
        return (atan2(y, self.distance(x, z)))
    
    def read_y_rotation(self, x, y, z):
        '''
        Retorna a rotação entorno do eixo Y em radianos
        '''
        return (-atan2(x, self.distance(y, z)))

    def read_pitch(self):
        '''
        Calcula o valor do pitch atual em radiano
        '''
        return self.read_x_rotation(self.accel[0], self.accel[1], self.accel[2])

    def read_roll(self):
        '''
        Calcula o valor do roll atual em radiano
        '''
        return self.read_y_rotation(self.accel[0], self.accel[1], self.accel[2])

if __name__ == "__main__":
    imu = Imu()

    print("iniciando processo de calibração......")
    print("certifique-se que a imu está alinhada com e paralela ao chão (apenas o z deve apresentar 1g)")
    print(" ")
    print(" ")
    time.sleep(3)
    imu.mpu.reset() # Reset sensors
    imu.mpu.configure() # After resetting you need to reconfigure the sensors
    accelbias = {"x": 0, "y": 0, "z": 0}
    gyrobias = {"x": 0, "y": 0, "z": 0}
    

    '''while True:
        accel = imu.mpu.readAccelerometerMaster()
        print("x: {:.2f} y: {:.2f} z: {:.2f}".format(accel[0], accel[1], accel[2]))'''

    for x in range(0, 100):
        imu.mpu.calibrateMPU6500()
        accelbias["x"] += imu.mpu.abias[0]
        accelbias["y"] += imu.mpu.abias[1]
        accelbias["z"] += imu.mpu.abias[2]

        gyrobias["x"] += imu.mpu.gbias[0]
        gyrobias["y"] += imu.mpu.gbias[1]
        gyrobias["z"] += imu.mpu.gbias[2]

        print(x)

    print(" ")
    print(" ")
    print("accel: ")
    print(accelbias)
    print("gyro: ")
    print(gyrobias)
    print(" ")
    print(" ")

    accel_x = accelbias["x"]  / 100
    accel_y = accelbias["y"]  / 100
    accel_z = accelbias["z"]  / 100

    gyro_x = gyrobias["x"]  / 100
    gyro_y = gyrobias["y"]  / 100
    gyro_z = gyrobias["z"]  / 100

    print("dividindo.....")
    print("accel: ")
    print(accel_x, accel_y, accel_z)
    print("gyro: ")
    print(gyro_x, gyro_y, gyro_z)



quit()