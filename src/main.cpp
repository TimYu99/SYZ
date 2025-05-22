//------------------------------------------ Includes ----------------------------------------------

// SDK includes
#include "sdk.h"                // The main SDK header. This include most of the SDK functionality
#include "platform/debug.h"
#include "utils/utils.h"
#include "utils/stringUtils.h"

// sdkExample includes
#include "platform.h"
#include "isa500App.h"
#include "isd4000App.h"
#include "ism3dApp.h"
#include "sonarApp.h"
#include "gpsApp.h"
#include <windows.h>
#include <thread>
#include <string>
#include <iostream>
#include "global.h"
#include "comms/ports/uartPort.h"
#include <regex>
#include <chrono>
#include "data_logger.h"

// ÿһ�����ŵĹ���״̬��1������������2�����������У�
extern std::map < std::string, int > sonar_status;
// ���Ŷ�Ӧ�Ķ˿�����
extern std::map < std::string, std::string > sonar_port_name;
// ÿһ�����ŵĶ�����������
extern std::map < std::string, int > sonar_reconnect_count;
// �Ƿ���һ���������ڹ���
extern bool sonar_working_flag;

using namespace IslSdk;

//------------------------------------------- Globals ----------------------------------------------

std::vector<App*> apps;
std::shared_ptr<GpsApp> gpsApp;
//��������ͺţ����Ҫ�Զ�����sonar1��2������Ҫ��app.cpp��callbackConnect�������������޸�
// ������ɵ��޸ĳ�����sonarApp.cpp���callbackPingData()�������recordPingData()
// ��Գ�ʼ������������Ҫ��������opencv�Ļ������ã������޷����г�����cmakelist.txt���޸�opencv��·��
//=============================���⴮��ͨ�Ų�������======================
SeriallPort serialPort;//����1
SeriallPort serialPort2;//����2
char sonar1[] = "sonar1 \r\n";
char sonar2[] = "sonar2 \r\n";
char readBuffer1[256];//����1����ʵ��վ�����Ĵ�����Ϣ���������ã�115200�����ʣ�COM1��Ĭ�϶��ڹ涨��ʵ��վЭ�飬��ʵ��վ���ݵĴ��ڴ�����ݵ��������ã�
int bytesRead1;//
int bytesWritten1;
char readBuffer2[256];//����2����432�����Ĵ�����Ϣ���������ã�115200�����ʣ�COM2��Ĭ��
int bytesRead2;
int bytesWritten2;//���ڹ涨��ʵ��վЭ�飬���ݵĴ���ָ�����ݵ���������

//���ڴ�����
void processSYZCommand(const std::string& command);//���ڴ���ʵ��վЭ��Ĵ���ָ��
void processDSPCommand(const std::string& command);//����432����ָ��
void sendReply(const std::string& portName, const std::string& message);//����1�ظ�ʵ��վЭ��Ĵ���ָ��
// std::string SeriallPort::calculateChecksum(const std::string& message);//����CKУ��ֵ
void sendToNextLevel(const std::string& portName, const std::string& message);//����2�ظ�


//�ص������Ĺ��캯������   
// These functions are the callbacks.
void newPort(const SysPort::SharedPtr& sysPort);
void newDevice(const Device::SharedPtr& device, const SysPort::SharedPtr& sysPort, const ConnectionMeta& meta);
void newNmeaDevice(const NmeaDevice::SharedPtr& device, const SysPort::SharedPtr& sysPort, const ConnectionMeta& meta);
void portOpen(SysPort& sysPort, bool_t failed);
void portClosed(SysPort& sysPort);
void portStats(SysPort& sysPort, uint_t txBytes, uint_t rxBytes, uint_t badPackets);
void portDiscoveryStarted(SysPort& sysPort, AutoDiscovery::Type type);
void portDiscoveryEvent(SysPort& sysPort, const ConnectionMeta&, AutoDiscovery::Type type, uint_t discoveryCount);
void portDiscoveryFinished(SysPort& sysPort, AutoDiscovery::Type type, uint_t discoveryCount, bool_t wasCancelled);
void portData(SysPort& sysPort, const uint8_t* data, uint_t size);
void portDeleted(SysPort& sysPort);

// These slots are used to connect to the SDK signals. Each slot is initialised with the function to call.
Slot<const SysPort::SharedPtr&> slotNewPort(&newPort);
Slot<const Device::SharedPtr&, const SysPort::SharedPtr&, const ConnectionMeta&> slotNewDevice(&newDevice);
Slot<const NmeaDevice::SharedPtr&, const SysPort::SharedPtr&, const ConnectionMeta&> slotNewNmeaDevice(&newNmeaDevice);
Slot<SysPort&, bool_t> slotPortOpen(&portOpen);
Slot<SysPort&> slotPortClosed(&portClosed);
Slot<SysPort&, uint_t, uint_t, uint_t> slotPortStats(&portStats);
Slot<SysPort&, AutoDiscovery::Type> slotPortDiscoveryStarted(&portDiscoveryStarted);
Slot<SysPort&, const ConnectionMeta&, AutoDiscovery::Type, uint_t> slotPortDiscoveryEvent(&portDiscoveryEvent);
Slot<SysPort&, AutoDiscovery::Type, uint_t, bool_t> slotPortDiscoveryFinished(&portDiscoveryFinished);
Slot<SysPort&, const uint8_t*, uint_t> slotPortData(&portData);
Slot<SysPort&> slotPortDeleted(&portDeleted);

void COM1_reader();
void COM2_reader();

//--------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    Platform::setTerminalMode();
    const std::string appPath = Platform::getExePath(argv[0]);
    Sdk sdk;                                                                    // Create the SDK instance

    Debug::log(Debug::Severity::Notice, "Main", "Impact Subsea SDK version %s    press\033[31m x\033[36m to exit", sdk.version.c_str());
    Platform::sleepMs(1000);

    sdk.ports.onNew.connect(slotNewPort);                                       // Connect to the new port signal
    sdk.devices.onNew.connect(slotNewDevice);                                   // Connect to the new device signal
    sdk.nmeaDevices.onNew.connect(slotNewNmeaDevice);                           // Connect to the new NMEA device signal

    // Serial over Lan ports can be created using the following function, change the IP address and port to suit your needs
    // sdk.ports.createSol("SOL1", false, true, Utils::ipToUint(192, 168, 1, 215), 1001);
    //���ڵĳ�ʼ������
    serialPort.open("COM1", 115200);//ʵ��վ
    //serialPort.open("COM1", 9600);//����ʵ��վ
    serialPort2.open("COM2", 115200);
    char writeBuffer[] = "$HXXB,WAR,1*CK\r\n";
    char sendBuffer1[] = "No Sonar Message\r\n";
    char sendBuffer2024[] = "System Ready\r\n";
    serialPort.write(sendBuffer2024, 14, bytesWritten1);//�����ɹ���ʾ
    saveData("D:/ceshi/Seriallog.txt", sendBuffer2024, strlen(sendBuffer2024), "COM1 Send", 0);
    int counts_jishu = 0;
    int counts_gengxin = 0;
    int islDiscoveryCounter = 0; // ���������ڿ��� ISL �豸���ֵ�Ƶ��

    std::thread com1_reader_Thread(&COM1_reader);
    com1_reader_Thread.detach();

    std::thread com2_reader_Thread(&COM2_reader);
    com2_reader_Thread.detach();

    while (1)
    {
        //Platform::sleepMs(40);
        //������Ҫ�Ǽ�ʱ������ڴ����Ϣ
       
            
            //if (counts_jishu >= 10000)
            //{
            //    counts_jishu = 0;
            //   
            //    if (sendBuffer2[0] != '\0')

            //    {
            //       
            //        std::cerr << "�ж�0֮ǰ" << std::endl;
            //        serialPort.write(sendBuffer2, 28, bytesWritten2);//ʵ��վʱ��ע�ͣ�����ע��
            //        //saveData("D:/ceshi/output.txt", sendBuffer, 28, "COM1 Send Hex Data", 1);
            //    }
            // }
            //else 
            //{
            //    counts_jishu++;
            //}
          
        //   
        

        //if (counts_gengxin >= 10) {
        //    counts_gengxin = 0;
        //    //sdk.ports.onNew.connect(slotNewPort);
        //    //std::cout << "��ˢ��: "  << std::endl;
        //}
        //else {
        //    counts_gengxin++;
        //}

        sdk.run();                                                              // Run the SDK. This should be called regularly to process data

        // �����¼��������ƺ������û�кķѶ���ʱ��
        //if (Platform::keyboardPressed())                                        // Check if a key has been pressed and do some example tasks
        //{
        //    int_t key = Platform::getKey();

        //    if (key == 'x')
        //    {
        //        break;
        //    }
        //    else
        //    {
        //        for (App* app : apps)
        //        {
        //            app->doTask(key, appPath);
        //        }
        //    }
        //}
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------
void COM1_reader()
{
    while (1) {
        // auto start_main = std::chrono::high_resolution_clock::now();
        serialPort.read(readBuffer1, sizeof(readBuffer1), bytesRead1);//����ʵ��վָ��
        
        if (bytesRead1 > 0) {
            std::cout << "���յ��ⲿ��������: " << std::string(readBuffer1, bytesRead1) << std::endl;
            // ��������Ƿ��� 0x00 ��ͷ
            if (static_cast<unsigned char>(readBuffer1[0]) == 0x00) {
                std::cout << "���յ��� 0x00 ��ͷ�����ݣ��������档" << std::endl;
                continue;
            }
            saveData("D:/ceshi/Seriallog.txt", readBuffer1, strlen(readBuffer1), "COM1 Recieved", 0);
            processSYZCommand(readBuffer1);

            // serialPort.write(readBuffer1, bytesRead1, bytesWritten1);
        }

        // auto end_main = std::chrono::high_resolution_clock::now();
        // auto duration_main = std::chrono::duration_cast<std::chrono::milliseconds>(end_main - start_main).count();
        // printf("line 152 serialPort.read took %lld ms\n", duration_main);
    }
}

void COM2_reader()
{
    while (1) {
        // auto start_main = std::chrono::high_resolution_clock::now();

        serialPort2.read(readBuffer2, sizeof(readBuffer2), bytesRead2);//����DSP������432�ĵ�FSKGOTָ��

        if (bytesRead2 > 0) {
            std::string reply432read;
            //std::cout << "���յ�432��������: " << std::string(readBuffer2, bytesRead2) << std::endl;
            reply432read = readBuffer2;
            if (sizeof(reply432read) > 0 && reply432read[0] != '\0') {
                std::cout << "���յ�432��������: " << std::string(readBuffer2, bytesRead2) << std::endl;
                saveData("D:/ceshi/Seriallog.txt", readBuffer2, strlen(readBuffer2), "COM2 Recieved", 0);
                //if (reply432read.size() < 2 || reply432read.substr(reply432read.size() - 2) != "\r\n")
                //   {
                //       reply432read += "\r\n";//������Ҫɾ����������������Ѿ�����
                //   }
                reply432read = "$432readDSP:" + reply432read;
                processDSPCommand(readBuffer2);
                //sendReply("COM6", reply432read);
            }
        }

        // auto end_main = std::chrono::high_resolution_clock::now();
        // auto duration_main = std::chrono::duration_cast<std::chrono::milliseconds>(end_main - start_main).count();
        // printf("line 175 serialPort2.read took %lld ms\n", duration_main);
    }
}

//--------------------------------------------------------------------------------------------------
// This function is called when a new port is found. It's address has been initialised inside the slot class defined above.
void newPort(const SysPort::SharedPtr& sysPort)
{
    // Connect to the port signals
    sysPort->onOpen.connect(slotPortOpen);
    sysPort->onClose.connect(slotPortClosed);
    sysPort->onPortStats.connect(slotPortStats);
    sysPort->onDiscoveryStarted.connect(slotPortDiscoveryStarted);
    sysPort->onDiscoveryEvent.connect(slotPortDiscoveryEvent);
    sysPort->onDiscoveryFinished.connect(slotPortDiscoveryFinished);
    sysPort->onDelete.connect(slotPortDeleted);

    Debug::log(Debug::Severity::Info, "Main", "Found new SysPort %s", sysPort->name.c_str());

    /*
    To find devices connected to the port, we need to start discovery. There are a few virtual and
    overloaded functions that can be used to do this. The simplest is to call the virtual function in
    the SysPort base class sysPort->discoverIslDevices() which will start discovery with default parameters.
    The overloaded version of this function can be used to specify the PID, part number and serial number
    (found on the device label). This only need to be specified in RS485 multi-drop networks.
    eg.
        sysPort->discoverIslDevices();
        sysPort->discoverIslDevices(0, 1336, 135);        // a value of 0 means any PID, part number or serial number
     */

    // The following code will start discovery on the port depending on the port type.
    // It casts the SysPort to the derived type and call the discover function for that type.

    if (sysPort->type == SysPort::Type::Net)
    {
        NetPort& port = reinterpret_cast<NetPort&>(*sysPort);

        if (sysPort->name == "NETWORK")
        {
            uint32_t ipAddress = Utils::ipToUint(192, 168, 1, 255);
            port.discoverIslDevices(0xffff, 0xffff, 0xffff, ipAddress, 33005, 2000);    // These are the default parameters, same as port.discoverIslDevices()
        }
    }
    else if (sysPort->type == SysPort::Type::Serial)
    {
        UartPort& port = reinterpret_cast<UartPort&>(*sysPort);
        // This call is the same as calling port.discoverIslDevices(pid, pn, sn, baudrate, timeout) multiple times with different baudrates
        port.discoverIslDevices();

        // To discover NMEA devices, call port.discoverNmeaDevices() or port.discoverNmeaDevices(baudrate, timeout)
        // calling this function will stop discovery of ISL devices
    }
}
//--------------------------------------------------------------------------------------------------
// This function is called when a new Device is found. It's address has been initialised inside the slot class defined above.
void newDevice(const Device::SharedPtr& device, const SysPort::SharedPtr& sysPort, const ConnectionMeta& meta)
{
    App* app = nullptr;

    switch (device->info.pid)
    {
    case Device::Pid::Isa500:
        Debug::log(Debug::Severity::Notice, "Main", "Found ISA500 altimeter %04u.%04u on port %s", FMT_U(device->info.pn), FMT_U(device->info.sn), sysPort->name.c_str());
        app = new Isa500App();
        break;

    case Device::Pid::Isd4000:
        Debug::log(Debug::Severity::Notice, "Main", "Found ISD4000 depth sensor %04u.%04u on port %s", FMT_U(device->info.pn), FMT_U(device->info.sn), sysPort->name.c_str());
        app = new Isd4000App();
        break;

    case Device::Pid::Ism3d:
        Debug::log(Debug::Severity::Notice, "Main", "Found ISM3D ahrs sensor %04u.%04u on port %s", FMT_U(device->info.pn), FMT_U(device->info.sn), sysPort->name.c_str());
        app = new Ism3dApp();
        break;

    case Device::Pid::Sonar:
        Debug::log(Debug::Severity::Notice, "Main", "Found Sonar %04u.%04u on port %s", FMT_U(device->info.pn), FMT_U(device->info.sn), sysPort->name.c_str());

        // ��¼�豸��Ӧ�Ĵ�������
        sonar_port_name[StringUtils::pnSnToStr(device->info.pn, device->info.sn)] = sysPort->name;
        // ����״̬����Ϊ1����������
        sonar_status[StringUtils::pnSnToStr(device->info.pn, device->info.sn)] = 1;
        sonar_working_flag = 1;
        printf("%s �豸��״̬��%d\n", StringUtils::pnSnToStr(device->info.pn, device->info.sn).c_str(), sonar_status[StringUtils::pnSnToStr(device->info.pn, device->info.sn)]);
        if (sonar_status.find("2255.0024") != sonar_status.end() || sonar_status.find("2254.0025") != sonar_status.end()) //2���ڶ���
        {
                           if (sonar_status["2255.0024"] == 1 && sonar_status["2254.0025"] == 1)
                          {
                              globalstatus1 = 0x03;
                              globalstatus2 = 0x03;// ���Ĭ��ֵ
                          }
                         else 
                         {
                             if (sonar_status["2254.0025"] == 1)
                             {
                                 globalstatus2 = 0x02; // ���Ĭ��ֵ
                                 saveData("D:/ceshi/Seriallog.txt", sonar2, strlen(sonar2), "Work:", 0);
                             }
                             if (sonar_status["2255.0024"] == 1)//(sonar_status["2255.0024"] == 1)
                             {
                                 globalstatus1 = 0x01; // ���Ĭ��ֵ
                                 saveData("D:/ceshi/Seriallog.txt", sonar1, strlen(sonar1), "Work:", 0);
                             }
                         }
        }
        printf("�Ƿ����豸�ڹ�����%d\n", sonar_working_flag);
        // ����������������
        sonar_reconnect_count[StringUtils::pnSnToStr(device->info.pn, device->info.sn)] = 0;

        app = new SonarApp();
        break;

    default:
        Debug::log(Debug::Severity::Notice, "Main", "Found device %04u.%04u on port %s", device->info.pn, device->info.sn, sysPort->name.c_str());
        app = new App("Device");
        break;
    }

    if (app)
    {
        app->setDevice(device);
        apps.push_back(app);

        device->connect();
    }

}
//--------------------------------------------------------------------------------------------------
// This function is called when a new Nmea device is found. It's address has been initialised inside the slot class defined above.
void newNmeaDevice(const NmeaDevice::SharedPtr& device, const SysPort::SharedPtr& sysPort, const ConnectionMeta& meta)
{
    if (device->type == NmeaDevice::Type::Gps)
    {
        Debug::log(Debug::Severity::Notice, "Main", "Found GPS device on port %s", sysPort->name.c_str());
        gpsApp = std::make_shared<GpsApp>("GPS");
        gpsApp->setDevice(device);
    }
}
//--------------------------------------------------------------------------------------------------
void portOpen(SysPort& sysPort, bool_t isOpen)
{
    if (isOpen)
    {
        Debug::log(Debug::Severity::Info, "Main", "%s open", sysPort.name.c_str());
    }
    else
    {
        Debug::log(Debug::Severity::Warning, "Main", "%s failed to open", sysPort.name.c_str());
    }
}
//--------------------------------------------------------------------------------------------------
void portClosed(SysPort& sysPort)
{
    Debug::log(Debug::Severity::Info, "Main", "%s closed", sysPort.name.c_str());
}
//--------------------------------------------------------------------------------------------------
void portStats(SysPort& sysPort, uint_t txBytes, uint_t rxBytes, uint_t badPackets)
{
    //Debug::log(Debug::Severity::Info, "Main", "%s TX:%u, RX:%u, Bad packets:%u", sysPort.name.c_str(), FMT_U(txBytes), FMT_U(rxBytes), FMT_U(badPackets));
}
//--------------------------------------------------------------------------------------------------
void portDiscoveryStarted(SysPort& sysPort, AutoDiscovery::Type type)
{
    Debug::log(Debug::Severity::Info, "Main", "%s discovery started", sysPort.name.c_str());
}
//--------------------------------------------------------------------------------------------------
void portDiscoveryEvent(SysPort& sysPort, const ConnectionMeta& meta, AutoDiscovery::Type type, uint_t discoveryCount)
{
    if (sysPort.type == SysPort::Type::Net)
    {
        uint_t ip = meta.ipAddress;
        Debug::log(Debug::Severity::Info, "Main", "%s Discovering at IP %u.%u.%u.%u:%u", sysPort.name.c_str(), ip & 0xff, (ip >> 8) & 0xff, (ip >> 16) & 0xff, (ip >> 24) & 0xff, meta.port);
    }
    else
    {
        Debug::log(Debug::Severity::Info, "Main", "%s Discovering at baudrate %u", sysPort.name.c_str(), meta.baudrate);
    }
}
//--------------------------------------------------------------------------------------------------
void portDiscoveryFinished(SysPort& sysPort, AutoDiscovery::Type type, uint_t discoveryCount, bool_t wasCancelled)
{
    Debug::log(Debug::Severity::Info, "Main", "%s Discovery Finished", sysPort.name.c_str());
}
//--------------------------------------------------------------------------------------------------
void portData(SysPort& sysPort, const uint8_t* data, uint_t size)
{
    Debug::log(Debug::Severity::Info, "Main", "Port data, size: %u", size);
}
//--------------------------------------------------------------------------------------------------
void portDeleted(SysPort& sysPort)
{
    Debug::log(Debug::Severity::Warning, "Main", "Port %s deleted", sysPort.name.c_str());
}
//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
//���ڽ��մ�����(����ʵ��վ���ݲ�����ת������)
void processSYZCommand(const std::string& command)
{
    std::string reply;
    std::string reply432;


    if (command.find("$SMSN,ON,0") != std::string::npos)
    {
        reply = "$SMSN,ONOK,1*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,ON,0*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (command.find("$SMSN,OFF,0") != std::string::npos) {
        reply = "$SMSN,OFOK,1*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,OFF,0*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        smsn1_on_flag = 0;
        smsn2_on_flag = 0;
    }
    else if (command.find("$SMSN,ONONE,1") != std::string::npos) {
        reply = "$SMSN,ONOK,1*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,ONONE,1*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        smsn1_on_flag = 1;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (command.find("$SMSN,OFONE,1") != std::string::npos) {
        reply = "$SMSN,ONNO,1*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,OFONE,1*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        smsn1_on_flag = 0;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (command.find("$SMSN,ONTWO,2") != std::string::npos) {
        reply = "$SMSN,ONOK,2*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,ONTWO,2*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        smsn2_on_flag = 1;
        sendToNextLevel("COM10", reply432);
    }
    else if (command.find("$SMSN,OFTWO,2") != std::string::npos) {
        reply = "$SMSN,ONNO,2*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,OFTWO,2*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        smsn2_on_flag = 0;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (command.find("$SMSN,ZL1,1") != std::string::npos) {
        reply = "$SMSN,ZL1,0*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,ZL1,1*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (command.find("$SMSN,ZL2,1") != std::string::npos) {
        reply = "$SMSN,ZL2,0*";
        //reply = "$SMSN,ONOK,1*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,ZL2,1*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (command.find("$SMSN,ZL3,1") != std::string::npos) {
        reply = "$SMSN,ZL3,0*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,ZL3,1*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (command.find("$SMSN,ZL1,2") != std::string::npos) {
        reply = "$SMSN,ZL1,0*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,ZL1,2*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (command.find("$SMSN,ZL2,2") != std::string::npos) {
        reply = "$SMSN,ZL2,0*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,ZL2,2*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else if (command.find("$SMSN,ZL3,2") != std::string::npos)
    {
        reply = "$SMSN,ZL3,0*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$SMSN,ZL3,2*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    //else if (command.find("$SMSN,ONONE,1") != std::string::npos) //��$SMSN,ONONE,1���$SMSN,SONAR,1
    //{
    //    reply = "$SMSN,ONOK,1*";
    //    //reply = "$SMSN,WORK,1*";
    //    reply += SeriallPort::calculateChecksum(reply) + "\r\n";
    //    sendReply("COM6", reply);
    //    reply432 = "$SMSN,SONAR,1*CK\r\n";
    //    //reply432 +=  "\r\n";
    //    sendToNextLevel("COM10", reply432);
    //    std::this_thread::sleep_for(std::chrono::seconds(1));
    //}
    //else if (command.find("$SMSN,ONTWO,2") != std::string::npos)//��$SMSN,ONTWO,2���$SMSN,SONAR,2
    //{
    //    reply = "$SMSN,WORK,2*";
    //    //reply = "$SMSN,WORK,2*";
    //    reply += SeriallPort::calculateChecksum(reply) + "\r\n";
    //    sendReply("COM6", reply);
    //    reply432 = "$SMSN,SONAR,2*CK\r\n";
    //    //reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
    //    sendToNextLevel("COM10", reply432);
    //    std::this_thread::sleep_for(std::chrono::seconds(1));
    //}
    else if (command.find("$SMSN,CYCLE") != std::string::npos) //$SMSN,CYCLE,XXXX,XXXX,0*CK
    {
        std::regex regexPattern(R"(\$SMSN,CYCLE,(\d+),(\d+),0\*CK)");
        std::smatch match;
        if (std::regex_search(command, match, regexPattern)) {
            std::string firstPart = match[1];
            std::string secondPart = match[2];
            std::cout << "��ȡ��������: " << firstPart << "," << secondPart << std::endl;


            reply = "$SMSN,CYCLE,OK*";
            reply += SeriallPort::calculateChecksum(reply) + "\r\n";
            sendReply("COM6", reply);
            std::string reply432 = "$SMSN,CYCLE," + firstPart + "," + secondPart + ",0*";

            reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
            sendToNextLevel("COM10", reply432);
        }
    }
    else if (command.find("$HXXB,WAR,1") != std::string::npos) {
        reply = "$HXXB,REV,1*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$HXXB,WAR,1*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
    }
    else if (command.find("$HXXB,OFF,1") != std::string::npos) {
        reply = "$HXXB,REC,1*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);
        reply432 = "$HXXB,OFF,1*";
        reply432 += SeriallPort::calculateChecksum(reply432) + "\r\n";
        sendToNextLevel("COM10", reply432);
    }
    else if (command.find("$SMSN,XINXI,1") != std::string::npos) {
        reply = "$SMSN,XINXI,0*";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);

        if (sendBuffer[0] == '\0')
        {
            char sendBuffer1[] = "No Sonar Message1\r\n";
            serialPort.write(sendBuffer1, 18, bytesWritten1);
        }
        else
        {
            serialPort.write(sendBuffer, 28, bytesWritten1);
            memset(sendBuffer, 0, 256);
        }

        if (sendBuffer2[0] == '\0') {
            char sendBuffer1 [] = "No Sonar Message2\r\n";
            serialPort.write(sendBuffer1, 18, bytesWritten1);
        }
        else {
            serialPort.write(sendBuffer2, 28, bytesWritten1);
            memset(sendBuffer2, 0, 256);
        }

    }
    else
    {
        Debug::log(Debug::Severity::Warning, "SonarApp", "Unknown command received: %s", command.c_str());
        return;
    }




}
void processDSPCommand(const std::string& command)
{
    std::string reply;
    //std::string reply432;


    if (command.find("$HXXB,FSKGOT") != std::string::npos)
    {
        reply = "$HXXB,FSKGOT";
        reply += SeriallPort::calculateChecksum(reply) + "\r\n";
        sendReply("COM6", reply);

    }

}

void sendReply(const std::string& portName, const std::string& message)
{
    //std::lock_guard<std::mutex> lock(uartOperationMutex);
    uint32_t baudrate = 115200;  // ������Ҫ����������
    const char* buffer = message.c_str();
    serialPort.write(buffer, strlen(buffer), bytesWritten1);
    saveData("D:/ceshi/Seriallog.txt", buffer, strlen(buffer), "COM1 Tran", 0);
    //uartPort6_.write(reinterpret_cast<const uint8_t*>(message.c_str()), message.size(),115200);
}

void sendToNextLevel(const std::string& portName, const std::string& message) {
    uint32_t baudrate = 115200;  // ������Ҫ����������
    const char* buffer = message.c_str();
    serialPort2.write(buffer, strlen(buffer), bytesWritten2);
    saveData("D:/ceshi/Seriallog.txt", buffer, strlen(buffer), "COM2 Send", 0);
    // ���������ͣ1��
    std::this_thread::sleep_for(std::chrono::seconds(1));
    //uartPort10_.write(reinterpret_cast<const uint8_t*>(message.c_str()), message.size(), ConnectionMeta(115200));
}