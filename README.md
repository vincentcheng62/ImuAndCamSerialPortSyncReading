# ImuAndCamSerialPortSyncReading
An C++ implementation using QT SerialPort API to achieve imu and camera data reading in synchronization

Camera model: PointGrey Chameleon3 CM3-U3-13Y3M
IMU: ADIS16448
Baud rate: 115200
ttyUSB port: Any (Will be detected automatically by the program)

IMU data format:
[AA][Accel_X 4hex][Accel_Y 4hex][Accel_Z 4hex][Gyro_X 4hex][Gyro_Y 4hex][Gyro_Z 4hex][Synchronizarion hex][Counter 2hex][55]

Imu rate: 500Hz
Cam rate: 25Hz
Imu/cam ratio: 20
