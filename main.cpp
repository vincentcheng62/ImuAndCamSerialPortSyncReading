/****************************************************************************
**
** Copyright (C) 2013 Laszlo Papp <lpapp@kde.org>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtSerialPort/QSerialPort>
#include <QTextStream>
#include <QCoreApplication>
#include <QStringList>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include "stdafx.h"
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <vector>
#include "FlyCapture2.h"
#include <sstream>
#include <thread>
#include <mutex>
#include <iomanip>
#include <ctime>

using namespace FlyCapture2;
using namespace std;

#define GRAVITY (9.81007) // according to okvis (offical: 9.80665)
#define _USE_MATH_DEFINES
#define ACCELE_LSB_DIVIDER (1200.0/GRAVITY)
#define ACCELE_MAX_DECIMAL (21600)
#define GYRO_LSB_DIVIDER ((25.0*180)/M_PI)
#define GYRO_MAX_DECIMAL (25000)

#define CAM_IMAGE_WIDTH (640)
#define CAM_IMAGE_HEIGHT (512)

// Camera hardware setting: NA: 4.0, focusing range: infinity
#define ISCAMERA_SETTING_DEFAULT    (true)
#define CAMERA_SHUTTER_TIME_IN_MS (4.0) // tune the shutter time to avoid motion blur
#define CAMERA_SHUTTER_LOG_FOR_RUN_MULTIPLER    (1) // default 2.5 for looking at ceiling
#define CAMERA_EXPOSURE_IN_EV (1.0) // orig: 0.939
#define CAMERA_GAIN_IN_dB (35.0) // orig: 0.939
#define CAMERA_GAIN_LOG_FOR_RUN_MULTIPLER    (0.25) // default 0.25 for looking at ceiling

// [AA][acc_x][acc_y][acc_z][gyro_x][gyro_y][gyro_z][whether cam sync trigger][total count][55]


#define IsProgramDebug (false)
#define MAX_TTY_TRY_INDEX       (10)

#define LOG_FOR_CALIB_CAMERA_IMU        (0)
#define LOG_FOR_RUN                     (1)
#define LOG_FOR_CALIB_IMU_INTRINSIC     (2)
#define LOG_MODE                        (LOG_FOR_RUN)

#define SERIAL_READ_TIMEOUT_TIME (500) // in miliseconds
#define CAMERA_GRAB_TIMEOUT (500) // in miliseconds

// The following 3 para need to be tuned so that imu data can be captured at ~500Hz in software level
// It is becoz waitForReadyRead() takes 3.5~4ms randomly for each call
#define BYTE_TO_READ_PER_SERIALPORT  (32)
#define BYTE_TO_READ_PER_SERIALPORT2  (33)

// every nth use port2, so to speed up a little bit
// (30) for auto shutter in static
// (26) for 2ms shutter in static
// (35) for 2ms shutter in dynamic
#define BYTE_TO_READ_RATIO  (35)

#define IMU_FRAME_LENGTH (17)
#define IMU_Hz             (500)
#define IMU_TO_CAM_RATIO       (20)
#define CAM_Hz              (IMU_Hz/IMU_TO_CAM_RATIO)

//Keep it 10000 if want to achieve good repeatibility in okvis
#define IMU_FRAME_TO_CAPTURE (14000)

#define FIRST_FRAME_MATCH_MARGIN    (20000)
#define SYNC_MARGIN (5000) // margin in ms, after a new camera frame arrive, wait SYNC_MARGIN ms and get the latest imu frame with sync=0x01

//bool isFirstCamCapture=false;
//long long first_image_timestamp;

struct imudata
{
    long index;
    bool IsSync;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double acc_x;
    double acc_y;
    double acc_z;
};

bool BothStart=false;
std::vector<imudata> gIMUframes;
std::vector<TimeStamp> gImageframes;



QT_USE_NAMESPACE

static inline uint32_t __iter_div_u64_rem(uint64_t dividend, uint32_t divisor, uint64_t *remainder)
{
  uint32_t ret = 0;

  while (dividend >= divisor) {
    /* The following asm() prevents the compiler from
       optimising this loop into a modulo operation.  */
    asm("" : "+rm"(dividend));

    dividend -= divisor;
    ret++;
  }

  *remainder = dividend;

  return ret;
}

#define NSEC_PER_SEC  1000000000L
static inline void timespec_add_ns(struct timespec *a, uint64_t ns)
{
  a->tv_sec += __iter_div_u64_rem(a->tv_nsec + ns, NSEC_PER_SEC, &ns);
  a->tv_nsec = ns;
}

void PrintBuildInfo2()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion(&fc2Version);

    ostringstream version;
    version << "FlyCapture2 library version: " << fc2Version.major << "."
            << fc2Version.minor << "." << fc2Version.type << "."
            << fc2Version.build;
    cout << version.str() << endl;

    ostringstream timeStamp;
    timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
    cout << timeStamp.str() << endl << endl;
}

void PrintCameraInfo2(CameraInfo *pCamInfo)
{
    cout << endl;
    cout << "*** CAMERA INFORMATION ***" << endl;
    cout << "Serial number - " << pCamInfo->serialNumber << endl;
    cout << "Camera model - " << pCamInfo->modelName << endl;
    cout << "Camera vendor - " << pCamInfo->vendorName << endl;
    cout << "Sensor - " << pCamInfo->sensorInfo << endl;
    cout << "Resolution - " << pCamInfo->sensorResolution << endl;
    cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
    cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl
         << endl;
}

void PrintError2(Error error) { error.PrintErrorTrace(); }

bool PollForTriggerReady2(Camera *pCam)
{
    const unsigned int k_softwareTrigger = 0x62C;
    Error error;
    unsigned int regVal = 0;

    do
    {
        error = pCam->ReadRegister(k_softwareTrigger, &regVal);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            return false;
        }

    } while ((regVal >> 31) != 0);

    return true;
}

int SaveImage(Image image, char* dest)
{
    Error error;
    //timespec ts;
    //clock_gettime(CLOCK_REALTIME, &ts); // Works on Linux

    //std::cout << "time of capture: " << ts << std::endl;
    //printf("time of capture: %lld.%.9ld\n", (long long)ts.tv_sec, ts.tv_nsec);


    // Create a converted image
    Image convertedImage;

    // Convert the raw image
    error = image.Convert(PIXEL_FORMAT_MONO8, &convertedImage);
    if (error != PGRERROR_OK)
    {
        cout << "Error in image.Convert" << endl;
        PrintError2(error);
        return -1;
    }

    // Create a unique filename
    ostringstream filename;
    filename << dest << image.GetTimeStamp().seconds << std::setw(9) <<
             std::setfill('0') << image.GetTimeStamp().microSeconds*1000L <<  ".png";

    // Save the image. If a file format is not passed in, then the file
    // extension is parsed to attempt to determine the file format.
    error = convertedImage.Save(filename.str().c_str());
    if (error != PGRERROR_OK)
    {
        cout << "Error in convertedImage.Save()" << endl;
        PrintError2(error);
        return -1;
    }

    if(IsProgramDebug) cout << "Finish saving down image " << endl;
    return 0;
}

void CamRetrieveBuffer(Camera &cam, int frames, char* dest)
{
    Error error;
    Image image;
    int i=0;
    timespec tend;

    while(i<frames)
    {
        error = cam.RetrieveBuffer(&image);
        //clock_gettime(CLOCK_REALTIME, &tend);
        //cout << "End timestamp= " << (long long)tend.tv_sec << std::setw(9) <<
                //std::setfill('0') << (long)tend.tv_nsec<< endl;
        //double retrievetime = double((long long)tend.tv_sec - (long long)tstart.tv_sec) + double((long)tend.tv_nsec-(long)tstart.tv_nsec)*1e-9;
        //cout << "RetrieveBuffertime= " << retrievetime << endl;
        if (error != PGRERROR_OK)
        {
            cout << "Error in RetrieveBuffer, skip this image frame" << endl;
            cout << error.GetDescription() << endl;
            //PrintError2(error);
            //std::cin.get();
            //return -1;
        }
        else
        {
            //clock_gettime(CLOCK_REALTIME, &tend);
            //cout << "img Timestamp in micsecond: " << tend.tv_nsec << endl;
            //cout << "img Timestamp in API: " << image.GetTimeStamp().microSeconds*1000L << endl;

            //cout << "success" << endl;
            if(BothStart) gImageframes.push_back(image.GetTimeStamp());

            //if(!isFirstCamCapture)
            //{
                //isFirstCamCapture=true;
                //cout << "First image is captured!" << endl;
                //timespec t;
                //clock_gettime(CLOCK_REALTIME, &t);
                //first_image_timestamp = image.GetTimeStamp().seconds*1000000000L +image.GetTimeStamp().microSeconds*1000L;
            //}
            if(BothStart) std::thread (SaveImage, image, dest).detach();
            if(BothStart && i%5==0) cout << "i=" << i << ", An image frame is captured!" << endl;
        }

        if(BothStart) i++;
    }
    cout << "Finish CamRetrieveBuffer() thread" << endl;
}





void ReadIMUdata(fstream &fp, fstream &fpacc, QSerialPort &serialPort, int frames)
{
    unsigned long imu_period_in_nsec = 1000000000/IMU_Hz;
    unsigned long cam_period_in_nsec = 1000000000/CAM_Hz;
    int awayfromlastsynccounter = -1000;

    int currentoffset=-1; // -1 means not yet found 0xAA, >=0 means the position away from 0xAA
    std::vector<double> imuvalue;

    long i=0, k=0; // i: valid imu frame, k: complete frame received

    unsigned int b_prev=0;
    int sum;
    double value;
    timespec tstart, tend;

    //serialPort.readAll(); // clear the past data in the buffer
//    if(serialPort.clear(QSerialPort::Input)) // clear the past data in the buffer
//    {
//        cout << "serialPort.clear() fails!" << endl;
//        std::cin.get();
//    }

    while ((LOG_MODE==LOG_FOR_CALIB_IMU_INTRINSIC || i<frames))
    {
        serialPort.waitForReadyRead(SERIAL_READ_TIMEOUT_TIME);
//        if(serialPort.bytesAvailable()==0)
//        {
//            usleep(1e3);
//            continue;
//        }
        //clock_gettime(CLOCK_REALTIME, &tend);
        //double retrievetime = double((long long)tend.tv_sec - (long long)tstart.tv_sec) + double((long)tend.tv_nsec-(long)tstart.tv_nsec)*1e-9;
        //cout << "serialPort.waitForReadyRead time in ms= " << retrievetime*1e3 << endl;


        QByteArray tmp;
        tmp = serialPort.readAll(); // usually take less than 0.01 ms
//        if(i%BYTE_TO_READ_RATIO==0)
//        {
//            tmp = serialPort.read(BYTE_TO_READ_PER_SERIALPORT2);
//        }
//        else
//        {
//            tmp = serialPort.read(BYTE_TO_READ_PER_SERIALPORT);
//        }

//        QByteArray tmp = serialPort.read(IMU_FRAME_LENGTH);
//        if(tmp.length()==0)
//        {
//            cout << "No new data is fetched!" << endl;
//            usleep(1e3);
//            continue;
//        }

        //if(IsProgramDebug) readData.append(tmp);
        //cout << tmp.data() << endl;

        //standardOutput << QObject::tr("serialPort.bytesAvailable()=%1").arg(serialPort.bytesAvailable()) << endl;
        const char* c=tmp.data();

        //standardOutput << QObject::tr("tmp.size()=%1").arg(tmp.size()) << endl;
        //if(tmp.size()==0) cout << "No data is read, the frame is skip" << endl;
        //offsetofframeintmp=0;
        for (int j=0; j<tmp.length() && i<IMU_FRAME_TO_CAPTURE; j++)
        {
//            if(k_when_first_capture<0 && isFirstCamCapture)
//            {
//                k_when_first_capture=k;
//            }
            //Detect "0xAA", get a new frame of imu data
            unsigned int b1=*c;
            auto data_in_hex = b1 & 0xff;
            //if(IsProgramDebug) standardOutput << QObject::tr("data_in_hex=%1, j=%2, b1=%3").arg(data_in_hex).arg(j).arg(b1) << endl;
            if(currentoffset<0 && data_in_hex == 0xAA)
            {
//                if(offsetofframeintmp==0)
//                {
//                    clock_gettime(CLOCK_REALTIME, &imutime);
//                }
//                else
//                {
//                    timespec_add_ns(&imutime, 2000000); // add 2ms to trick
//                }
//                offsetofframeintmp++;
                currentoffset=0;
            }
            else
            {
                if(currentoffset == 16 && data_in_hex == 0x55) currentoffset=-1;
                else
                {
                    switch(currentoffset)
                    {
                        //waiting for the next byte to complete
                        case 1: case 3: case 5: case 7: case 9: case 11: case 14:
                            b_prev = data_in_hex;
                            break;

                        case 2: case 4: case 6:// accelerometer
                            sum = (b_prev<<8)+data_in_hex;
                            value = (sum>ACCELE_MAX_DECIMAL?sum-65536:sum)/ACCELE_LSB_DIVIDER;
                            //if(IsProgramDebug) standardOutput << QObject::tr("===Acc, sum=%1, value=%2").arg(sum).arg(value) << endl;
                            imuvalue.push_back(value);
                            break;

                        case 8: case 10: case 12: // gyroscope
                            sum = (b_prev<<8)+data_in_hex;
                            value = (sum>GYRO_MAX_DECIMAL?sum-65536:sum)/GYRO_LSB_DIVIDER;
                            //if(IsProgramDebug) standardOutput << QObject::tr("===Gyro, sum=%1, value=%2").arg(sum).arg(value) << endl;
                            imuvalue.push_back(value);
                            //cout << sum << endl;
                            break;

                        case 13: // whether a syn signal is sent to camera
                            //clock_gettime(CLOCK_REALTIME, &imutime);
//                               if(data_in_hex == 0x01)
//                               {
//                                   hasSync=1;
//                                   if(imuSyncIndex<0 && isFirstCamCapture)
//                                   {
//                                       imuSyncIndex = k;
//                                   }
//                               }
//                               else hasSync=0;
//                            if(data_in_hex == 0x01 && tmp.size() == 32 && LOG_MODE!=LOG_FOR_CALIB_IMU_INTRINSIC)
//                            {
//                                // Grab image
//                                clock_gettime(CLOCK_REALTIME, &tstart);
//                                error = cam.RetrieveBuffer(&image);
//                                clock_gettime(CLOCK_REALTIME, &tend);
//                                double retrievetime = double((long long)tend.tv_sec - (long long)tstart.tv_sec) + double((long)tend.tv_nsec-(long)tstart.tv_nsec)*1e-9;
////                                if(retrievetime > (double)cam_period_in_nsec*0.5*1e-9)
////                                {
////                                    cout << "Retrieve time > half camera period, matching fail!" << endl;
////                                    std::cin.get();
////                                    return 0;
////                                }

//                                //cout << "RetrieveBuffertime= " << retrievetime << endl;
//                                if (error != PGRERROR_OK)
//                                {
//                                    cout << "Error in RetrieveBuffer, skip this image frame" << endl;
//                                    //PrintError2(error);
//                                    //std::cin.get();
//                                    //return -1;
//                                }
//                                else
//                                {
//                                    std::thread (SaveImage, image, imagedestination).detach();
//                                    imutime = (image.GetTimeStamp().seconds-retrievetime)*1000000000L+image.GetTimeStamp().microSeconds*1000L;
//                                    //cout << "ImageTimestamp= " << imutime.seconds << std::setw(9) <<
//                                                 //std::setfill('0') << imutime.microSeconds*1000L << endl;
//                                    awayfromlastsynccounter=0;
//                                }
//                            }
//                            else awayfromlastsynccounter++;

                              if(data_in_hex == 0x01) awayfromlastsynccounter=0;
                              else awayfromlastsynccounter++;


                            break;

                        case 15: // a counter from 0 to 65535
                            sum = (b_prev<<8)+data_in_hex;
                            //if(IsProgramDebug) standardOutput << QObject::tr("Counter=%1").arg(sum) << endl;
                            break;

                        default:
                            //if(IsProgramDebug) standardOutput << QObject::tr("Parsing imu data error, j=%1, currentoffset=%2, data_in_hex=%3, re-search 0xAA !!").arg(j).arg(currentoffset).arg(data_in_hex) << endl;
                            currentoffset=-1;
                            awayfromlastsynccounter = -1000;
                            imuvalue.clear(); // remove some prev captured frames where start with 0xAA but not end in 0x55
                            break;
                    }
                }
            }
            c++;
            if(currentoffset>=0) currentoffset++;

            //Flush a new imu frame if imuvalue has more than 6 entry
            if(imuvalue.size()>=6)
            {
                if(LOG_MODE!=LOG_FOR_CALIB_IMU_INTRINSIC)
                {
                    //if(imuSyncIndex>0) // if the first image is captured
                    //{
                        //long long t = first_image_timestamp + (k-imuSyncIndex)*imu_period_in_nsec;
//                        long long t = imutime + (awayfromlastsynccounter)*imu_period_in_nsec;
//                        fp << t <<  ",";

//                        //fp << (long long)imutime.tv_sec << std::setw(9) <<
//                        //      std::setfill('0') << (long)imutime.tv_nsec<< "," ;
//                        //fp << awayfromlastsynccounter << ",";
//                        fp << imuvalue[3] << ",";
//                        fp << imuvalue[4] << ",";
//                        fp << imuvalue[5] << ",";
//                        fp << imuvalue[0] << ",";
//                        fp << imuvalue[1] << ",";
//                        fp << imuvalue[2] << endl;
                    imudata tmp;
                    tmp.index=i;
                    tmp.IsSync = (awayfromlastsynccounter==0);
                    tmp.gyro_x = imuvalue[3];
                    tmp.gyro_y = imuvalue[4];
                    tmp.gyro_z = imuvalue[5];
                    tmp.acc_x = imuvalue[0];
                    tmp.acc_y = imuvalue[1];
                    tmp.acc_z = imuvalue[2];

                    //clock_gettime(CLOCK_REALTIME, &tstart);
                    //cout << "imu frame timestamp: " << tstart.tv_nsec << endl;

                    if(BothStart)
                    {
                       gIMUframes.push_back(tmp);
                       i++;
                       if(i%100==0) cout << "i=" << i << ", An imu frame is captured!" << endl;
                    }
                    //}

                }
                else
                {
                    //clock_t frameend = clock();
                    //double timetillnow = double((long long)imutime.tv_sec - (long long)imucalib_starttime.tv_sec) + double((long)imutime.tv_nsec-(long)imucalib_starttime.tv_nsec)*1e-9;
                    double timetillnow = i*((double)imu_period_in_nsec/1e9);
                    fpacc << timetillnow << "\t";
                    fpacc << std::fixed << std::setprecision(16) << imuvalue[0] << "\t";
                    fpacc << std::fixed << std::setprecision(16) << imuvalue[1] << "\t";
                    fpacc << std::fixed << std::setprecision(16) << imuvalue[2] << endl;

                    fp << timetillnow << "\t";
                    fp << std::fixed << std::setprecision(16) << imuvalue[3] << "\t";
                    fp << std::fixed << std::setprecision(16) << imuvalue[4] << "\t";
                    fp << std::fixed << std::setprecision(16) <<  imuvalue[5] << endl;

                    if(i%100==0) cout << "i=" << i << ", time till now=" << timetillnow << "seconds" << endl;
                    i++;
                }
                imuvalue.erase (imuvalue.begin(),imuvalue.begin()+6);
                k++;
            }
        }

    }
    cout << "Finish ReadIMUdata() thread" << endl;
}

int main(int argc, char *argv[])
{
    std::thread t1,t2;
    QCoreApplication coreApplication(argc, argv);
    int argumentCount = QCoreApplication::arguments().size();
    QStringList argumentList = QCoreApplication::arguments();

    QTextStream standardOutput(stdout);

//    if (argumentCount == 1) {
//        standardOutput << QObject::tr("Usage: %1 <serialportname> [baudrate]").arg(argumentList.first()) << endl;
//        std::cin.get();
//        return 1;
//    }





    Camera cam;
    Error error;
    TriggerMode triggerMode;

    if(LOG_MODE!=LOG_FOR_CALIB_IMU_INTRINSIC)
    {
        //Setup the pointgrey camera
        PrintBuildInfo2();

        BusManager busMgr;
        unsigned int numCameras;
        error = busMgr.GetNumOfCameras(&numCameras);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        cout << "Number of cameras detected: " << numCameras << endl;

        if (numCameras < 1)
        {
            cout << "Insufficient number of cameras... exiting" << endl;
            std::cin.get();
            return -1;
        }

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex(0, &guid);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }



        // Connect to a camera
        error = cam.Connect(&guid);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        // Power on the camera
        const unsigned int k_cameraPower = 0x610;
        const unsigned int k_powerVal = 0x80000000;
        error = cam.WriteRegister(k_cameraPower, k_powerVal);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        const unsigned int millisecondsToSleep = 100;
        unsigned int regVal = 0;
        unsigned int retries = 10;

        // Wait for camera to complete power-up
        do
        {
            struct timespec nsDelay;
            nsDelay.tv_sec = 0;
            nsDelay.tv_nsec = (long)millisecondsToSleep * 1000000L;
            nanosleep(&nsDelay, NULL);

            error = cam.ReadRegister(k_cameraPower, &regVal);
            if (error == PGRERROR_TIMEOUT)
            {
                // ignore timeout errors, camera may not be responding to
                // register reads during power-up
            }
            else if (error != PGRERROR_OK)
            {
                PrintError2(error);
                std::cin.get();
                return -1;
            }

            retries--;
        } while ((regVal & k_powerVal) == 0 && retries > 0);

        // Check for timeout errors after retrying
        if (error == PGRERROR_TIMEOUT)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        // Get the camera information
        CameraInfo camInfo;
        error = cam.GetCameraInfo(&camInfo);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        PrintCameraInfo2(&camInfo);

        //char new_resl[512] = "640x512";
        //strcpy(camInfo.sensorResolution, new_resl);

        //error = cam.SetVideoModeAndFrameRate(VIDEOMODE_640x480Y8, FRAMERATE_15);

        auto imageSettings = new Format7ImageSettings();
        imageSettings->mode = MODE_1;
        imageSettings->width = CAM_IMAGE_WIDTH;
        imageSettings->height = CAM_IMAGE_HEIGHT;
        imageSettings->pixelFormat = PIXEL_FORMAT_MONO8;

        bool settingsValid = false;
        Format7PacketInfo packetInfo;
        error = cam.ValidateFormat7Settings(imageSettings, &settingsValid, &packetInfo);
        if (!settingsValid)
        {
            cout << "Settings are not valid" << endl;
            std::cin.get();
            return -1;
        }
        error = cam.SetFormat7Configuration(imageSettings, packetInfo.recommendedBytesPerPacket);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        // Set the shutter property of the camera
        Property prop;
        prop.type = SHUTTER;
        error = cam.GetProperty(&prop);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        cout << "default Shutter time is " << fixed << setprecision(2) << prop.absValue
             << "ms" << endl;

        prop.autoManualMode = false;
        prop.absControl = true;

        const float k_shutterVal = CAMERA_SHUTTER_TIME_IN_MS * (LOG_MODE==LOG_FOR_RUN?CAMERA_SHUTTER_LOG_FOR_RUN_MULTIPLER:1.0);
        prop.absValue = k_shutterVal;

        error = cam.SetProperty(&prop);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        cout << "Shutter time set to " << fixed << setprecision(2) << k_shutterVal
             << "ms" << endl;

        // Set the exposure property of the camera
        Property prop2;
        prop2.type = AUTO_EXPOSURE;
        error = cam.GetProperty(&prop2);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        cout << "default AUTO_EXPOSURE is " << fixed << setprecision(2) << prop2.absValue
             << "EV" << endl;

        prop2.autoManualMode = ISCAMERA_SETTING_DEFAULT;
        prop2.absControl = true;

        const float k_exposureVal = CAMERA_EXPOSURE_IN_EV;
        prop2.absValue = k_exposureVal;

        error = cam.SetProperty(&prop2);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        cout << "Exposure is set to " << fixed << setprecision(2) << k_exposureVal
             << "EV" << endl;

        // Set the gain property of the camera
        Property prop3;
        prop3.type = GAIN;
        error = cam.GetProperty(&prop3);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        cout << "default GAIN is " << fixed << setprecision(2) << prop3.absValue
             << "dB" << endl;

        prop3.autoManualMode = ISCAMERA_SETTING_DEFAULT;
        prop3.absControl = true;

        const float k_GainVal = CAMERA_GAIN_IN_dB* (LOG_MODE==LOG_FOR_RUN?CAMERA_GAIN_LOG_FOR_RUN_MULTIPLER:1.0);
        prop3.absValue = k_GainVal;

        error = cam.SetProperty(&prop3);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        cout << "Gain is set to " << fixed << setprecision(2) << k_GainVal
             << "dB" << endl;

        // Get current trigger settings
        error = cam.GetTriggerMode(&triggerMode);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        // Set camera to trigger mode 0
        triggerMode.onOff = true;
        triggerMode.mode = 0;
        triggerMode.parameter = 0;

        // Triggering the camera externally using source 0.
        triggerMode.source = 0;

        error = cam.SetTriggerMode(&triggerMode);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        // Poll to ensure camera is ready
        bool retVal = PollForTriggerReady2(&cam);
        if (!retVal)
        {
            cout << endl;
            cout << "Error polling for trigger ready!" << endl;
            std::cin.get();
            return -1;
        }

        // Get the camera configuration
        FC2Config config;
        error = cam.GetConfiguration(&config);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        // Set the grab timeout to 5 seconds
        config.grabTimeout = CAMERA_GRAB_TIMEOUT;

        // Set the camera configuration
        error = cam.SetConfiguration(&config);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        // Camera is ready, start capturing images
        error = cam.StartCapture();
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        cout << "Trigger the camera by sending a trigger pulse to GPIO"
             << triggerMode.source << endl;
    }





    //Open a folder whose name is current timestamp
    timespec starttime, tmp;
    clock_gettime(CLOCK_REALTIME, &starttime);
    char command[50];
    sprintf(command, "mkdir %lld", (long long)starttime.tv_sec);
    system(command);
    sprintf(command, "mkdir %lld/cam0", (long long)starttime.tv_sec);
    system(command);

    //Output imu data
    char filename[70];
    char imagedestination[100];
    fstream fp, fpacc;
    if(LOG_MODE==LOG_FOR_RUN)
    {
        sprintf(command, "mkdir %lld/imu0", (long long)starttime.tv_sec);
        system(command);
        sprintf(command, "mkdir %lld/cam0/data", (long long)starttime.tv_sec);
        system(command);
        sprintf(imagedestination, "%lld/cam0/data/", (long long)starttime.tv_sec);
        sprintf(filename, "%lld/imu0/data.csv", (long long)starttime.tv_sec);
    }
    else if(LOG_MODE==LOG_FOR_CALIB_CAMERA_IMU)
    {
        sprintf(imagedestination, "%lld/cam0/", (long long)starttime.tv_sec);
        sprintf(filename, "%lld/imu0.csv", (long long)starttime.tv_sec);
    }
    else
    {
        //according to the format of imu_tk library
        sprintf(filename, "%lld/xsens_acc.mat", (long long)starttime.tv_sec);

        fpacc.open(filename, ios::out);
        if(!fpacc){
            cout<<"Fail to open file: "<<filename<<endl;
        }

        sprintf(filename, "%lld/xsens_gyro.mat", (long long)starttime.tv_sec);

    }

    fp.open(filename, ios::out);
    if(!fp){
        cout<<"Fail to open file: "<<filename<<endl;
    }

    if(LOG_MODE!=LOG_FOR_CALIB_IMU_INTRINSIC)
    {
        //cout<<"File Descriptor: "<<fp<<endl;
        fp << "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1] \
            ,a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]" << endl;
    }


    Image image;
    long long imutime;
    //timespec imutime;
    //TimeStamp imutime;
    int currentoffset=-1; // -1 means not yet found 0xAA, >=0 means the position away from 0xAA
    std::vector<double> imuvalue;

    int i=0, k=0; // i: valid imu frame, k: complete frame received

    unsigned int b_prev=0;
    int sum;
    double value;
    int offsetofframeintmp=0;
    int hasSync=0;
    int imuSyncIndex=-1;
    int k_when_first_capture=-1;
    int last_i_index=0;
    bool IsMatched;
    bool IsFirstMatchedFinished=false;
    long totalwrongmatch=0;


    //waitForReadyRead(): Blocks until new data is available for reading and the readyRead()
    //signal has been emitted, or until msecs milliseconds have passed. If msecs is -1,
    //this function will not time out.

    //Returns true if new data is available for reading; otherwise returns false
    //(if the operation timed out or if an error occurred).
    //clock_gettime(CLOCK_REALTIME, &imutime);

    timespec imucalib_starttime, tstart, tend;
    clock_gettime(CLOCK_REALTIME, &imucalib_starttime);

    cout << "Now is time= " << (long long)imucalib_starttime.tv_sec << std::setw(9) <<
            std::setfill('0') << (long)imucalib_starttime.tv_nsec<< endl ;
    cout << "All data will be saved in directory /" << (long long)starttime.tv_sec << endl;
    //cout << "Main loop Start!! " << endl;
    if(LOG_MODE==LOG_FOR_CALIB_IMU_INTRINSIC)
    {
        cout << "In imu calibration mode" << endl;
        cout << "Manually forced to close the terminal when finish" << endl;
    }




    int ttyUSBtryindex=0;
    clock_t end, begin = clock();

    QSerialPort serialPort;
    QString serialPortName = "ttyUSB"+QString::number(ttyUSBtryindex);
    serialPort.setPortName(serialPortName);

    int serialPortBaudRate = QSerialPort::Baud115200;
    serialPort.setBaudRate(serialPortBaudRate);

    if (!serialPort.open(QIODevice::ReadOnly)) {
        standardOutput << QObject::tr("Failed to open port %1, error: %2, please check if the previous window is closed").arg(serialPortName).arg(serialPort.error()) << endl;
        cout << "Press any key to try other tty port" << endl;
        std::cin.get();

        ttyUSBtryindex++;
        while(ttyUSBtryindex<MAX_TTY_TRY_INDEX)
        {
            serialPortName = "ttyUSB"+QString::number(ttyUSBtryindex);
            serialPort.setPortName(serialPortName);
            if(!serialPort.open(QIODevice::ReadOnly))
            {
                standardOutput << QObject::tr("Failed to open port %1.").arg(serialPortName) << endl;
            }
            else break;

            ttyUSBtryindex++;
        }
        if(ttyUSBtryindex==MAX_TTY_TRY_INDEX)
        {
            cout << "All port fails!" << endl;
            std::cin.get();
            return 1;
        }
    }

    standardOutput << QObject::tr("Opened the serial port %1, at baudrate: %2").arg(serialPortName).arg(serialPortBaudRate) << endl;
    t2 = std::thread(ReadIMUdata, std::ref(fp), std::ref(fpacc), std::ref(serialPort), IMU_FRAME_TO_CAPTURE);
    t1 = std::thread(CamRetrieveBuffer, std::ref(cam), (IMU_FRAME_TO_CAPTURE/IMU_TO_CAM_RATIO), imagedestination);

    cout << "Sleep 5 seconds to let imu and camera read rate stable..." << endl << endl;
    usleep(5*1e6);

    if(LOG_MODE!=LOG_FOR_CALIB_IMU_INTRINSIC)
    {
        clock_gettime(CLOCK_REALTIME, &tmp);
        BothStart=true;
        cout << "BothStart=true" << endl;
        cout << "Now timestamp= " << tmp.tv_nsec<< endl;

        //Do matching on the global vector data
        std::vector<long> image_to_imu_index;
        long imagelistcurrentsize=0;
        while(gImageframes.size()<=(IMU_FRAME_TO_CAPTURE/IMU_TO_CAM_RATIO))
        {
            if(gImageframes.size()>imagelistcurrentsize) // if new image frame arrive
            {
                //cout << "Img timestamp in microsecond: " << gImageframes.back().microSeconds << endl;
                //Check if it is a jump in the index
                if(gImageframes.size()>imagelistcurrentsize+1)
                {
                    cout << "2 image frames come too close, please start again!" << endl;
                    std::cin.get();
                    return 1;
                }

                imagelistcurrentsize = gImageframes.size();

                //Since serialPort.waitForReadyRead() need to wait for 5ms
                usleep(SYNC_MARGIN);
                //if(IsFirstMatchedFinished) usleep(SYNC_MARGIN); // give some margin so the matching imu frame can arrive
                //else usleep(FIRST_FRAME_MATCH_MARGIN);

                IsMatched=false;

                //Find the latest imu data in gIMUframes whose
                for(long i=gIMUframes.size()-1; i>=0 ; i--)
                {
                    if(gIMUframes[i].IsSync)
                    {
                        cout << "latest imu data in gIMUframes whose IsSync is true: " << i << ", frame diff is " << i-last_i_index << ", gIMUframes.size() is " << gIMUframes.size() <<endl;
                        if(!IsFirstMatchedFinished ||
                            i-last_i_index==IMU_TO_CAM_RATIO)
                        {
//                            if(!IsFirstMatchedFinished && i>20)
//                            {
//                                cout << "First cam frame must match with imu frame <=20, otherwise has a chance of mismatch!" <<endl;
//                                std::cin.get();
//                                return -1;
//                            }
                            IsFirstMatchedFinished=true;
                            image_to_imu_index.push_back(i);
                            cout << "The " << imagelistcurrentsize-1 << "th image is match with imuframe " << i << ", frame diff is " << i-last_i_index << endl;
                            last_i_index=i;
                            IsMatched=true;
                        }
                        else    IsMatched=false;
                        break;

                    }
                }

                if(!IsMatched)
                {
                    if(IsFirstMatchedFinished)
                    {
                        image_to_imu_index.push_back(last_i_index+IMU_TO_CAM_RATIO);
                        totalwrongmatch++;
                        last_i_index+=IMU_TO_CAM_RATIO;
                    }
                    else
                    {
                        cout << "First cam frame cannot match, stop here, otherwise has a chance of mismatch!" <<endl;
                        std::cin.get();
                        return -1;
                        //image_to_imu_index.push_back(-1); // meaning the first match still not happen
                    }
                    cout << "The " << imagelistcurrentsize-1 << "th image is unmatch" << endl;

                }

                if(gImageframes.size()==(IMU_FRAME_TO_CAPTURE/IMU_TO_CAM_RATIO)) break;

            }
            usleep(100); // take a small snap in the empty loop

            //cout << "gImageframes.size(): " << gImageframes.size() << endl;
        }

        cout << "image_to_imu_index.size(): " << image_to_imu_index.size() << endl;
        cout << "Waiting for join()" << endl;
        t1.join();
        t2.join();
        end = clock();
        cout << "totalwrongmatch (2 is normal): " << totalwrongmatch << endl;

        cout << "Output matched data..." << endl;
        unsigned long imu_period_in_nsec = 1000000000/IMU_Hz;
        long imagecounter=0;

        //Get to the first imagecounter such that image_to_imu_index[imagecounter] != -1
        while(image_to_imu_index[imagecounter]==-1 && imagecounter+1<gImageframes.size())
        {
            imagecounter++;
        }

        bool IsHead=true;
        for(long i=0; i<gIMUframes.size() ; i++)
        {
            TimeStamp tnsp, tsp = gImageframes[imagecounter];
            long long tdiff, t2, t1 = tsp.seconds*1000000000L + tsp.microSeconds*1000L;
            long thissyncindex = image_to_imu_index[imagecounter];
            long nextsyncindex = -1;
            if(imagecounter+1<gImageframes.size()){
                nextsyncindex = image_to_imu_index[imagecounter+1];
                tnsp = gImageframes[imagecounter+1];
                t2 = tnsp.seconds*1000000000L + tnsp.microSeconds*1000L;
                tdiff = t2-t1;
            }

            if(i<thissyncindex) // head
            {
                if(IsHead)
                {
                    t1 -= (thissyncindex-i)*imu_period_in_nsec;
                }
            }
            else if(nextsyncindex>=0) // middle
            {
                t1 += (i-thissyncindex)*(tdiff/IMU_TO_CAM_RATIO);
                if(i+1==nextsyncindex)
                {
                    imagecounter++;
                    while(image_to_imu_index[imagecounter]==-1 && imagecounter+1<gImageframes.size())
                    {
                        imagecounter++;
                    }
                    IsHead=false;
                }
            }
            else //end
            {
                t1 += (i-thissyncindex)*imu_period_in_nsec;
            }

            fp << t1 <<  ",";
            //fp << gIMUframes[i].IsSync << ",";
            fp << std::fixed << std::setprecision(16) << gIMUframes[i].gyro_x << ",";
            fp << std::fixed << std::setprecision(16) << gIMUframes[i].gyro_y << ",";
            fp << std::fixed << std::setprecision(16) << gIMUframes[i].gyro_z << ",";
            fp << std::fixed << std::setprecision(16) << gIMUframes[i].acc_x << ",";
            fp << std::fixed << std::setprecision(16) << gIMUframes[i].acc_y << ",";
            fp << std::fixed << std::setprecision(16) << gIMUframes[i].acc_z << endl;
        }
    }
    else
    {
        t2.join();
        end = clock();
    }



    cout << "Main loop run in " << (double(end - begin)) / CLOCKS_PER_SEC << "seconds" << endl;
    //cout << "when_camera_first_capture, k is: " << k_when_first_capture << endl;
    //cout << "imuSyncIndex: " << imuSyncIndex << endl;



    //long long t = first_image_timestamp + (k-imuSyncIndex)*imu_period_in_nsec;
//                        long long t = imutime + (awayfromlastsynccounter)*imu_period_in_nsec;



    cout << "All data are saved in directory /" << (long long)starttime.tv_sec << endl;

//    if (serialPort.error() == QSerialPort::ReadError) {
//        standardOutput << QObject::tr("Failed to read from port %1, error: %2").arg(serialPortName).arg(serialPort.errorString()) << endl;
//        std::cin.get();
//        return 1;
//    } else if (serialPort.error() == QSerialPort::TimeoutError && (IsProgramDebug && readData.isEmpty())) {
//        standardOutput << QObject::tr("No data was currently available for reading from port %1").arg(serialPortName) << endl;
//        std::cin.get();
//        return 0;
//    }

    if(IsProgramDebug) standardOutput << QObject::tr("Data successfully received from port %1").arg(serialPortName) << endl;
    //standardOutput << readData.toHex() << endl;


    //Print the serial port output as well aligned hex format
//    if(IsProgramDebug)
//    {
//        const char* c=readData.data();
//        QString text;
//        unsigned int m_hexBytes;
//        char buf[16];
//        for (int i=0; i<readData.length(); i++)
//        {
//            if ((m_hexBytes % 16) == 0)
//            {
//                snprintf(buf, 16, "%08x: ", m_hexBytes);
//                text+=buf;
//            }
//            unsigned int b=*c;
//            snprintf(buf, 16, "%02x ", b & 0xff);
//            text+=buf;

//            m_hexBytes++;
//            if ((m_hexBytes % 16)==0)
//            {
//                text+="\n";
//            }
//            else if ((m_hexBytes % 8)==0)
//            {
//                text+="  ";
//            }
//            c++;
//        }

//        standardOutput << text << endl;
//    }


    if(LOG_MODE!=LOG_FOR_CALIB_IMU_INTRINSIC)
    {
        // Turn trigger mode off.
        triggerMode.onOff = false;
        error = cam.SetTriggerMode(&triggerMode);
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }
        cout << endl;
        cout << "Finished grabbing images" << endl;

        // Stop capturing images
        error = cam.StopCapture();
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }

        // Disconnect the camera
        error = cam.Disconnect();
        if (error != PGRERROR_OK)
        {
            PrintError2(error);
            std::cin.get();
            return -1;
        }
    }


    fp.close();
    if(LOG_MODE==LOG_FOR_CALIB_IMU_INTRINSIC)
    {
        fpacc.close();
    }
    std::cin.get();

    return 0;
}
