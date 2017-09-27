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
#include <math.h>
#include <vector>
using namespace std;

#define _USE_MATH_DEFINES
#define ACCELE_LSB_DIVIDER (1200.0)
#define ACCELE_MAX_DECIMAL (21600)
#define GYRO_LSB_DIVIDER ((25.0*180)/M_PI)
#define GYRO_MAX_DECIMAL (25000)

#define IsProgramDebug (false)

#define LOG_FOR_CALIB   (0)
#define LOG_FOR_RUN     (1)
#define LOG_MODE        (LOG_FOR_CALIB)

QT_USE_NAMESPACE

int main3(int argc, char *argv[])
{
    QCoreApplication coreApplication(argc, argv);
    int argumentCount = QCoreApplication::arguments().size();
    QStringList argumentList = QCoreApplication::arguments();

    QTextStream standardOutput(stdout);

    if (argumentCount == 1) {
        standardOutput << QObject::tr("Usage: %1 <serialportname> [baudrate]").arg(argumentList.first()) << endl;
        std::cin.get();
        return 1;
    }

    QSerialPort serialPort;
    QString serialPortName = argumentList.at(1);
    serialPort.setPortName(serialPortName);

    int serialPortBaudRate = (argumentCount > 2) ? argumentList.at(2).toInt() : QSerialPort::Baud115200;
    serialPort.setBaudRate(serialPortBaudRate);

    if (!serialPort.open(QIODevice::ReadOnly)) {
        standardOutput << QObject::tr("Failed to open port %1, error: %2").arg(serialPortName).arg(serialPort.error()) << endl;
        std::cin.get();
        return 1;
    }

    //Open a folder whose name is current timestamp
    timespec starttime;
    clock_gettime(CLOCK_REALTIME, &starttime);
    char command[50];
    sprintf(command, "mkdir %lld", (long long)starttime.tv_sec);
    system(command);
    sprintf(command, "mkdir %lld/cam0", (long long)starttime.tv_sec);
    system(command);

    //Output imu data
    char filename[70];
    fstream fp;
    if(LOG_MODE==LOG_FOR_RUN)
    {
        sprintf(command, "mkdir %lld/imu0", (long long)starttime.tv_sec);
        system(command);
        sprintf(command, "mkdir %lld/cam0/data", (long long)starttime.tv_sec);
        system(command);
        sprintf(filename, "%lld/imu0/data.csv", (long long)starttime.tv_sec);
    }
    else
    {
        sprintf(filename, "%lld/imu0.csv", (long long)starttime.tv_sec);
    }

    fp.open(filename, ios::out);
    if(!fp){
        cout<<"Fail to open file: "<<filename<<endl;
    }
    //cout<<"File Descriptor: "<<fp<<endl;
    fp<<"#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1] \
        ,a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"<<endl;

    timespec imutime;
    int currentoffset=-1; // -1 means not yet found 0xAA, >=0 means the position away from 0xAA
    std::vector<double> imuvalue;

    int i=0;
    QByteArray readData = serialPort.readAll();

    //waitForReadyRead(): Blocks until new data is available for reading and the readyRead()
    //signal has been emitted, or until msecs milliseconds have passed. If msecs is -1,
    //this function will not time out.

    //Returns true if new data is available for reading; otherwise returns false
    //(if the operation timed out or if an error occurred).
    while (serialPort.waitForReadyRead(5000) && i<50)
    {
        unsigned int b_prev=0;

        int sum;
        double value;
        QByteArray tmp = serialPort.readAll();
        clock_gettime(CLOCK_REALTIME, &imutime);

        if(IsProgramDebug) readData.append(tmp);
        if(IsProgramDebug) standardOutput << tmp.toHex() << endl;


        const char* c=tmp.data();

        if(IsProgramDebug) standardOutput << QObject::tr("tmp.length()=%1").arg(tmp.length()) << endl;
        for (int j=0; j<tmp.length(); j++)
        {
            //Detect "0xAA", get a new frame of imu data
            unsigned int b1=*c;
            auto data_in_hex = b1 & 0xff;
            if(IsProgramDebug) standardOutput << QObject::tr("data_in_hex=%1, j=%2, b1=%3").arg(data_in_hex).arg(j).arg(b1) << endl;
            if(currentoffset<0 && data_in_hex == 0xAA)
            {
                currentoffset=0;
            }
            else if(currentoffset>=0)
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
                            sum = b_prev*256+data_in_hex;
                            value = (sum>ACCELE_MAX_DECIMAL?sum-65536:sum)/ACCELE_LSB_DIVIDER;
                            if(IsProgramDebug) standardOutput << QObject::tr("===Acc, sum=%1, value=%2").arg(sum).arg(value) << endl;
                            imuvalue.push_back(value);
                            break;

                        case 8: case 10: case 12: // gyroscope
                            sum = b_prev*256+data_in_hex;
                            value = (sum>GYRO_MAX_DECIMAL?sum-65536:sum)/GYRO_LSB_DIVIDER;
                            if(IsProgramDebug) standardOutput << QObject::tr("===Gyro, sum=%1, value=%2").arg(sum).arg(value) << endl;
                            imuvalue.push_back(value);
                            break;

                        case 13: // whether a syn signal is sent to camera
                            break;

                        case 15: // a counter from 0 to 65535
                            sum = b_prev*256+data_in_hex;
                            break;

                        default:
                            if(IsProgramDebug) standardOutput << QObject::tr("Parsing imu data error, j=%1, currentoffset=%2, data_in_hex=%3, re-search 0xAA !!").arg(j).arg(currentoffset).arg(data_in_hex) << endl;
                            currentoffset=-1;
                    }
                }
            }
            c++;
            if(currentoffset>=0) currentoffset++;
        }


        //Flush a new imu frame if imuvalue has more than 6 entry
        if(imuvalue.size()>=6)
        {
            fp << (long long)imutime.tv_sec << (long)imutime.tv_nsec<< "," ;
            fp << imuvalue[3] << ",";
            fp << imuvalue[4] << ",";
            fp << imuvalue[5] << ",";
            fp << imuvalue[0] << ",";
            fp << imuvalue[1] << ",";
            fp << imuvalue[2] << endl;
            imuvalue.erase (imuvalue.begin(),imuvalue.begin()+6);
        }

        i++;

    }

    if (serialPort.error() == QSerialPort::ReadError) {
        standardOutput << QObject::tr("Failed to read from port %1, error: %2").arg(serialPortName).arg(serialPort.errorString()) << endl;
        std::cin.get();
        return 1;
    } else if (serialPort.error() == QSerialPort::TimeoutError && (IsProgramDebug && readData.isEmpty())) {
        standardOutput << QObject::tr("No data was currently available for reading from port %1").arg(serialPortName) << endl;
        std::cin.get();
        return 0;
    }

    if(IsProgramDebug) standardOutput << QObject::tr("Data successfully received from port %1").arg(serialPortName) << endl;
    //standardOutput << readData.toHex() << endl;



    const char* c=readData.data();
    QString text;
    unsigned int m_hexBytes;
    char buf[16];
    for (int i=0; i<readData.length(); i++)
    {
        if ((m_hexBytes % 16) == 0)
        {
            snprintf(buf, 16, "%08x: ", m_hexBytes);
            text+=buf;
        }
        unsigned int b=*c;
        snprintf(buf, 16, "%02x ", b & 0xff);
        text+=buf;

        m_hexBytes++;
        if ((m_hexBytes % 16)==0)
        {
            text+="\n";
        }
        else if ((m_hexBytes % 8)==0)
        {
            text+="  ";
        }
        c++;
    }

    standardOutput << text << endl;
    fp.close();
    std::cin.get();

    return 0;
}
