# skoliometer

Koneksi:
    SD Card:
        CS --> pin 10 
        SCK --> pin 13
        MISO --> pin 12
        MOSI --> pin 11
    IMU (MPU6050):
        SDA --> SDA (A4 di UNO)
        SCL --> SCL (A5 di UNO)
        INT --> interrupt pin (2 di UNO)
    Push Button:
        pin 3 UNO
    Rotary Encoder
        CLK --> output A (6)
        DT --> output B (7)
        SW --> nothing

#Branch:
master --> stable version (working: Rotary + IMU)
addSD -->still trouble when join SD Card + Rotary + IMU