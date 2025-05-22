//------------------------------------------ Includes ----------------------------------------------

#include "sonarApp.h"
#include "maths/maths.h"
#include <cmath>
#include "platform/debug.h"
#include "files/bmpFile.h"
#include "utils/utils.h"
#include <iostream> // ����ͷ�ļ�
#include <ctime> // For std::time_t, std::time, std::ctime
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <direct.h> 
#include <filesystem> // ʹ�� std::filesystem ������ _mkdir
#include "platform/windows/serialPort.h" // ȷ����������ȷ��ͷ�ļ�
#include <vector>
#include <string>
#include "devices/device.h"
#include "global.h" // ���� global.h
#include <thread> // �����߳̿�
#include <windows.h> // ���� Windows API ͷ�ļ�
#include <mutex>
#include <opencv2/opencv.hpp>
#include "data_logger.h"
#include "array_data.h"

// #include "sonarDataStore.h"
// #include "sonarImage.h"
// #include "palette.h"

using namespace IslSdk;
//���
bool isRecording = false;
std::string dataFolder_;  // �ļ���·��
using namespace IslSdk;
unsigned char mesbag[28] = { 0 };
std::mutex uartMutex;
// 90��������߽ǽǶȶ���/����1����764�и����޸�
//const float START_ANGLE = 315; // ���ο�ʼ�߽�
//const float END_ANGLE = 45.0;    // ���ν����߽�
//const float ANGLE_TOLERANCE = 0.5; // �ݲΧ ��1��
//const int ROWS = 160; // ����
//const int COLS = 101;  // ����

cv::Mat frame11Display;
cv::Mat frame22Display;
int jishucount = 0;
////180��������߽ǽǶȶ���
const float START_ANGLE = 270; // ���ο�ʼ�߽�
const float END_ANGLE = 90;    // ���ν����߽�
const float ANGLE_TOLERANCE = 0.5; // �ݲΧ ��1��
const int ROWS = 160; // ����
const int COLS = 201;  // ����

float shanxing[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float frame1[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float frame2[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float frame3[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float frame4[ROWS][COLS] = { 0 }; // ��ʼ��Ϊ 0
float* currentFrame = &frame1[0][0]; // ָ�� frame1 ���׵�ַ
float* nextFrame = &frame2[0][0];   // ָ�� frame2 ���׵�ַ

bool isCollecting = false; // ���ݲɼ���־
// ����ȫ�ֱ���
Message  msg;
extern SeriallPort serialPort; // ����ȫ�ֱ���
extern SeriallPort serialPort2; // ����ȫ�ֱ���
int bytesWritten12;
int bytesWritten21;
int bytesWritten111;
int bytesWritten222;
////ʵ��վ
//char writeBufferimage[] = "Image detection ready!\r\n";
//char writeBufferjinggao[] = "$HXXB,WAR,1*CK\r\n";
//char writeBufferxiaoshi[] = "$HXXB,OFF,1*CK\r\n";
char writeBufferjinggao2[] = "$HXXB,WAR,2*CK\r\n";
char writeBufferxiaoshi2[] = "$HXXB,OFF,2*CK\r\n";
//char writeBufferjingzhi[] = "It may be a stationary target\r\n";

//����ʵ��վ
char writeBufferimage[] = "Image detection ready!\r\n";
char writeBufferjinggao[] = "$HXXB,WAR,1*CK\r\n";
char writeBufferxiaoshi[] = "$HXXB,OFF,1*CK\r\n";
char writeBufferjingzhi[] = "It may be a stationary target\r\n";

int first_mubao = 0;
int sec_mubao = 0;
int chazhi_mubiao = 0;
int biaozhi = 0;

// ��ǰ������ sonar app ���
static int sonar_app_index = 3;
// ������ device.cpp ��
// ÿһ�����ŵĹ���״̬��1������������2�����������У�
extern std::map < std::string, int > sonar_status;
// ���Ŷ�Ӧ�Ķ˿�����:��һ��������id���ڶ����Ƕ˿�����
extern std::map < std::string, std::string > sonar_port_name;
// ÿһ�����ŵĶ�����������
extern std::map < std::string, int > sonar_reconnect_count;
// �Ƿ���һ���������ڹ���
extern bool sonar_working_flag;

//void convert_uint_32_array_to_opencv_mat(const uint32_t* image, uint_t width, uint_t height);

std::string IslSdk::messageToString(const Message& msg)
{
    std::ostringstream oss;

    // ��headerת��Ϊ�ַ���
    oss.write(msg.header, sizeof(msg.header));

    // �������ֶΰ�˳����ӵ��ַ�������
    oss.write(reinterpret_cast<const char*>(&msg.length), sizeof(msg.length));
    oss.write(reinterpret_cast<const char*>(&msg.year), sizeof(msg.year));
    oss.write(reinterpret_cast<const char*>(&msg.month), sizeof(msg.month));
    oss.write(reinterpret_cast<const char*>(&msg.day), sizeof(msg.day));
    oss.write(reinterpret_cast<const char*>(&msg.hour), sizeof(msg.hour));
    oss.write(reinterpret_cast<const char*>(&msg.minute), sizeof(msg.minute));
    oss.write(reinterpret_cast<const char*>(&msg.second), sizeof(msg.second));
    oss.write(reinterpret_cast<const char*>(&msg.status), sizeof(msg.status));
    oss.write(reinterpret_cast<const char*>(&msg.angle), sizeof(msg.angle));
    oss.write(reinterpret_cast<const char*>(&msg.speed), sizeof(msg.speed));
    oss.write(reinterpret_cast<const char*>(&msg.minrange), sizeof(msg.minrange));
    oss.write(reinterpret_cast<const char*>(&msg.maxrange), sizeof(msg.maxrange));
    oss.write(reinterpret_cast<const char*>(&msg.reserved), sizeof(msg.reserved));
    oss.write(reinterpret_cast<const char*>(&msg.checksum), sizeof(msg.checksum));
    oss.write(reinterpret_cast<const char*>(&msg.end1), sizeof(msg.end1));
    oss.write(reinterpret_cast<const char*>(&msg.end2), sizeof(msg.end2));

    return oss.str();
}

SeriallPort::SeriallPort()
{
    hComm = INVALID_HANDLE_VALUE;
    SecureZeroMemory(&dcbSerialParams, sizeof(DCB));
    SecureZeroMemory(&timeouts, sizeof(COMMTIMEOUTS));
}

SeriallPort::~SeriallPort() {
    close();
}

bool SeriallPort::open(const std::string& portName, int baudRate) {
    hComm = CreateFileA(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hComm == INVALID_HANDLE_VALUE) {
        std::cout << "�޷��򿪴���!" << std::endl;
        return false;
    }

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hComm, &dcbSerialParams)) {
        std::cout << "��ȡ����״̬ʧ��!" << std::endl;
        close();
        return false;
    }

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hComm, &dcbSerialParams)) {
        std::cout << "���ô���״̬ʧ��!" << std::endl;
        close();
        return false;
    }

    timeouts.ReadIntervalTimeout = 5;
    timeouts.ReadTotalTimeoutConstant = 5;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant = 5;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    if (!SetCommTimeouts(hComm, &timeouts)) {
        std::cout << "���ô��ڳ�ʱʧ��!" << std::endl;
        close();
        return false;
    }

    return true;
}

bool SeriallPort::close() {
    if (hComm != INVALID_HANDLE_VALUE) {
        CloseHandle(hComm);
        hComm = INVALID_HANDLE_VALUE;
    }
    return true;
}

bool SeriallPort::read(char* buffer, int bufferSize, int& bytesRead) {
    return ReadFile(hComm, buffer, bufferSize, (LPDWORD)&bytesRead, NULL);
}

bool SeriallPort::write(const char* buffer, int bufferSize, int& bytesWritten) {
    return WriteFile(hComm, buffer, bufferSize, (LPDWORD)&bytesWritten, NULL);
}
void SonarApp::initializeTime()
{
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);
    year = 1900 + now_tm->tm_year;
    month = now_tm->tm_mon + 1;
    day = now_tm->tm_mday;
    hour = now_tm->tm_hour;
    minute = now_tm->tm_min;
    second = now_tm->tm_sec;
}
uint16_t SonarApp::addcalculateChecksum(const uint8_t* data, size_t size)
{
    uint16_t checksum = 0;
    for (size_t i = 0; i < size; ++i) {
        checksum += data[i];
    }
    return checksum;
}
namespace msis_pkg
{

    struct RangeImageBeam
    {
        float angle_du;
        float angle_stepSize;
        float minRange;
        float max_range;
        int data_count;
        std::vector<float> beam_data;

        // �������캯��
        RangeImageBeam();
    };

    // ���幹�캯��
    RangeImageBeam::RangeImageBeam() : angle_du(0.0f), angle_stepSize(0.0f), minRange(0.0f), max_range(0.0f), data_count(0) 
    {
        // ��������������ĳ�ʼ������
    }

} // namespace msis_pkg

std::string SeriallPort::calculateChecksum(const std::string& message)
{
    uint8_t checksum = 0;
    size_t start = message.find('$');
    size_t end = message.find('*');
    if (start != std::string::npos && end != std::string::npos && start < end) {
        for (size_t i = start + 1; i < end; ++i) {
            checksum ^= message[i];
        }
    }

    // Convert checksum to corresponding ASCII characters
    char asciiChecksum = static_cast<char>(checksum);
    return std::string(1, asciiChecksum);
}

//------------------------------------------ ��ӽ��� ----------
//--------------------------------------------------------------------------------------------------
SonarApp::SonarApp(void) : App("SonarApp"), m_pingCount(0), m_scanning(false), sequenceNumber_(1)
{
    setDataFolder("D:\\ceshi");
    header = "$SMSNXX";
    length = 28;  // ������Ϣ���ȣ��ų�������
    initializeTime();
    status = 0x0F;  // ʾ��״̬
    angle = 90;     // ʾ���Ƕ�
    speed = 2;      // ʾ���ٶ�
    reserved = 0;
    end1 = 0x0D;    // CR
    end2 = 0x0A;    // LF

    //if (globalPn == 2254 && globalSn == 25)
    //{
    //    sonar_app_index = 1;
    //    /*globalPn = 0;
    //    globalSn = 0;*/
    // }

    
    //sonar_app_index++;
    is_goal = cv::Mat(101, 200, CV_8UC1, cv::Scalar(0));
    qiang_du_tu = cv::Mat(101, 200, CV_8UC1, cv::Scalar(0));

    Debug::log(Debug::Severity::Notice, name.c_str(), "created" NEW_LINE
                                                        "d -> Set settings to defualt" NEW_LINE
                                                        "s -> Save settings to file" NEW_LINE
                                                        "r -> Start scaning" NEW_LINE
                                                        "R -> Stop scaning" NEW_LINE
                                                        "p -> Save palette" NEW_LINE
                                                        "t -> Save sonar texture" NEW_LINE
                                                        "i -> Save sonar image" NEW_LINE
                                                        "c -> Check head is sync'ed" NEW_LINE);
    std::cout << globalPn << std::endl;
    std::cout << globalSn << std::endl;
    

    if (sonar_status["2254.0025"] == 1)//1��һ��
    {
        sonar_app_index = 1;
        globalPn = 0;
        globalSn = 0;
        std::cout << globalPn << std::endl;
        std::cout << globalSn << std::endl;
    }
    if (sonar_status["2255.0024"] == 1)
    {
        sonar_app_index = 0;
        globalPn = 0;
        globalSn = 0;
        std::cout << globalPn << std::endl;
        std::cout << globalSn << std::endl;
    }

    std::cout << sonar_app_index << std::endl;

    m_sonar_app_index = sonar_app_index;
    std::cout << m_sonar_app_index << std::endl;
    //�Զ�����
    std::thread([this]() {
        
        while (true) {
            
             
            //std::this_thread::sleep_for(std::chrono::seconds(2)); // �ȴ�1�룬ȷ���豸�ѳ�ʼ��
     
            jishucount++;
                if (jishucount == 5)
                {
                    //std::cout << "creat";
                    this->doTask('r', dataFolder_); // ��ʼɨ��
                    
                }

             

            
            //std::this_thread::sleep_for(std::chrono::seconds(2)); // �ȴ�1�룬ȷ���豸�ѳ�ʼ��
            //std::this_thread::sleep_for(std::chrono::minutes(4)); // �ȴ�4����
            //this->doTask('R', dataFolder_); // ֹͣɨ��
            //std::this_thread::sleep_for(std::chrono::minutes(1)); // �ȴ�4����
        }
        }).detach();
    
    std::cout << "creat sonar app!";
    std::thread consumerThread(&SonarApp::consumePingData, this);
    consumerThread.detach();
}
//--------------------------------------------------------------------------------------------------
SonarApp::~SonarApp(void)
{
}
//--------------------------------------------------------------------------------------------------
void SonarApp::renderPalette(const std::string& path)
{
    const uint_t w = 100, h = 1000;

    std::unique_ptr<Palette> p = std::make_unique<Palette>();
    std::vector<uint32_t> buf(w * h);

    p->render(&buf[0], w, h, false);
    BmpFile::save(path + "palette.bmp", &buf[0], 32, w, h);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::connectSignals(Device& device)
{
    Sonar& sonar = reinterpret_cast<Sonar&>(device);

    ahrs.connectSignals(sonar.ahrs, name);
    gyro.connectSignals(sonar.gyro, name);
    accel.connectSignals(sonar.accel, name);

    sonar.onSettingsUpdated.connect(slotSettingsUpdated);
    sonar.onHeadIndexesAcquired.connect(slotHeadIndexesAcquired);
    sonar.onPingData.connect(slotPingData);                 // Subscribing to this event causes ping data to be sent
    sonar.onEchoData.connect(slotEchoData);                 // Subscribing to this event causes profiling data to be sent
    sonar.onPwrAndTemp.connect(slotPwrAndTemp);             // Subscribing to this event causes data to be sent from the device at the rate defined by setSensorRates()
    sonar.onMotorSlip.connect(slotMotorSlip);
    sonar.onMotorMoveComplete.connect(slotMotorMoveComplete);
    
    Sonar::SensorRates rates;
    rates.ahrs = 100;
    rates.gyro = 0;
    rates.accel = 0;
    rates.mag = 0;
    rates.voltageAndTemp = 1000;
    sonar.setSensorRates(rates);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::disconnectSignals(Device& device)
{
    Sonar& sonar = reinterpret_cast<Sonar&>(device);

    ahrs.disconnectSignals();
    gyro.disconnectSignals();
    accel.disconnectSignals();

    sonar.onSettingsUpdated.disconnect(slotSettingsUpdated);
    sonar.onHeadIndexesAcquired.disconnect(slotHeadIndexesAcquired);
    sonar.onPingData.disconnect(slotPingData);
    sonar.onEchoData.disconnect(slotEchoData);
    sonar.onPwrAndTemp.disconnect(slotPwrAndTemp);
    sonar.onMotorSlip.disconnect(slotMotorSlip);
    sonar.onMotorMoveComplete.disconnect(slotMotorMoveComplete);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::doTask(int_t key, const std::string& path)
{
    if (m_device)
    {
        Sonar& sonar = reinterpret_cast<Sonar&>(*m_device);

        switch (key)
        {
        case 'd':
            sonar.setSystemSettings(Sonar::System(), true);
            sonar.setAcousticSettings(Sonar::Acoustic(), true);
            sonar.setSetupSettings(Sonar::Setup(), true);
            break;

        case 's':
            sonar.saveConfig(path + m_device->info.pnSnAsStr() + " settings.xml");
            break;

        case 'r':
            //std::cerr << "555555" << std::endl;
           
            
            if (!isRecording)
            {
                
                isRecording = true;
                //// �ڼ�¼��ʼʱ����ʱ���
                //auto now = std::chrono::system_clock::now();
                //std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
                //// �ڿ�ʼ��¼ʱ����һ���µ��ļ���
                //std::stringstream filenameStream;
                //filenameStream << dataFolder_ << "/ping_data_" << std::to_string(timestamp) << ".txt";
                //std::string filename = filenameStream.str();
                //// ���ļ���д��ʱ����������Ϣ
                //std::ofstream outputFile(filename);
                //if (outputFile.is_open()) {
                //    std::cerr << "5" << std::endl;
                //    outputFile << "Recording Started:" << std::endl;
                //    outputFile << "Timestamp: " << std::ctime(&timestamp);
                //    outputFile << "Sequence Number: 1" << std::endl;
                //    outputFile.close();
                //    std::cout << "Recording started. Data will be written to " << filename << std::endl;


                //}
                //else
                //{
                //    std::cerr << "Unable to open the file for writing." << std::endl;
                //}
                //std::cerr << "55" << std::endl;
            }
            sonar.startScanning();
            //std::cerr << "5" << std::endl;
            break;

        case 'R':
            sonar.stopScanning();
            if (isRecording) {
                isRecording = false;
                // �ڼ�¼����ʱ����ʱ���
                auto now = std::chrono::system_clock::now();
                std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
                // �ڽ�����¼ʱ����һ���µ��ļ���
                std::stringstream filenameStream;
                filenameStream << dataFolder_ << "/ping_data_" << std::to_string(timestamp) << ".txt";
                std::string filename = filenameStream.str();
                // ���ļ���д��ʱ����������Ϣ
                std::ofstream outputFile(filename, std::ios::app);  // ʹ��׷��ģʽ���ļ�
                if (outputFile.is_open()) {
                    // �ڽ�����¼ʱ���������Ϣ
                    outputFile << "Recording Ended:" << std::endl;
                    outputFile << "Timestamp: " << std::ctime(&timestamp);
                    outputFile << "Sequence Number: " << sequenceNumber_ << std::endl;
                    outputFile.close();
                    std::cout << "Recording ended. Data written to " << filename << std::endl;
                }
                else {
                    std::cerr << "Unable to open the file for writing." << std::endl;
                }
                // �������
                ++sequenceNumber_;
            }

            break;

        case 'p':
            renderPalette(path);
            break;

        case 'i':
            m_circular.render(sonarDataStore, m_palette, true);
            BmpFile::save("E:/program/SYZ/build/bin/Debug/sonar.bmp", reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), 32, m_circular.width, m_circular.height);
            break;

        case 't':
            m_texture.renderTexture(sonarDataStore, m_palette, false);
            BmpFile::save(path + "texture.bmp", reinterpret_cast<const uint32_t*>(&m_texture.buf[0]), 32, m_texture.width, m_texture.height);
            break;

        case 'c':
            sonar.acquireHeadIdx(false);
            break;

        default:
            break;
        }

        App::doTask(key, path);
    }
}
//--------------------------------------------------------------------------------------------------
void SonarApp::connectEvent(Device& device)
{
    Sonar& sonar = reinterpret_cast<Sonar&>(device);

    m_circular.setBuffer(1000, 1000, true);
    m_circular.setSectorArea(0, sonar.settings.setup.maxRangeMm, sonar.settings.setup.sectorStart, sonar.settings.setup.sectorSize);
    m_circular.useBilinerInterpolation = true;

    // Optimal texture size to pass to the GPU - each pixel represents a data point. The GPU can then map this texture to circle (triangle fan)
    m_texture.useBilinerInterpolation = false;
    m_texture.setBuffer(sonar.settings.setup.imageDataPoint, Sonar::maxAngle / Math::abs(sonar.settings.setup.stepSize), true);
    m_texture.setSectorArea(0, sonar.settings.setup.maxRangeMm, sonar.settings.setup.sectorStart, sonar.settings.setup.sectorSize);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackSettingsUpdated(Sonar& sonar, bool_t ok, Sonar::Settings::Type settingsType)//ϵͳ�¼�
{
    const char* settingsTypeStr[] = { "System", "Acostic", "Setup" };

    if (ok)
    {
        Debug::log(Debug::Severity::Info, name.c_str(), "%s settings updated", settingsTypeStr[static_cast<uint_t>(settingsType)]);

        if (settingsType == Sonar::Settings::Type::Setup)
        {
            sonar.connection->sysPort->close();
            m_circular.setSectorArea(0, sonar.settings.setup.maxRangeMm, sonar.settings.setup.sectorStart, sonar.settings.setup.sectorSize);

            // Optimal texture size to pass to the GPU - each pixel represents a data point. The GPU can then map this texture to circle (triangle fan)
            m_texture.setBuffer(sonar.settings.setup.imageDataPoint, Sonar::maxAngle / Math::abs(sonar.settings.setup.stepSize), true);
            m_texture.setSectorArea(0, sonar.settings.setup.maxRangeMm, sonar.settings.setup.sectorStart, sonar.settings.setup.sectorSize);
        }
    }
    else
    {
        Debug::log(Debug::Severity::Warning, name.c_str(), "%s settings failed to update", settingsTypeStr[static_cast<uint_t>(settingsType)]);
    }
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackHeadIndexesAcquired(Sonar& sonar, const Sonar::HeadIndexes& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Head Sync:%s. Slippage:%i", data.state == Sonar::HeadIndexes::State::Ok ? "ok" : "error", data.slippage);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackEchoData(Sonar& sonar, const Sonar::Echos& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Echo data, size:%u", data.data.size());
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackPwrAndTemp(Sonar& sonar, const Sonar::CpuPowerTemp& data)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "CPU:%.1f, SYS:%.1f, 1.0V:%.2f, 1.8V:%.2f, 1.35V:%.2f", data.cpuTemperature, data.auxTemperature, data.core1V0, data.aux1V8, data.ddr1V35);
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackMotorSlip(Sonar& sonar)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Motor has slipped");
}
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackMotorMoveComplete(Sonar& sonar, bool_t ok)
{
    Debug::log(Debug::Severity::Info, name.c_str(), "Motor move %s", ok ? "complete" : "busy");
}
//----------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
void SonarApp::callbackPingData(Sonar& iss360, const Sonar::Ping& ping)
{
    //Debug::log(Debug::Severity::Info, name.c_str(), "Ping data");
    std::cerr << "���ݻش�" << std::endl;
    m_piss360 = &iss360;
    if (isRecording) {
        static int pingDataCount = 0; // ��̬�������ڱ��ּ�������ֵ
        //Debug::log(Debug::Severity::Info, name.c_str(), "Ping data");
        Debug::log(Debug::Severity::Info, name.c_str(), "Ping data, Count: %d", ++pingDataCount); // ��ʾ��������ֵ
        uint_t txPulseLengthMm = static_cast<uint_t>(iss360.settings.system.speedOfSound * iss360.settings.acoustic.txPulseWidthUs * 0.001 * 0.5);
        txPulseLengthMm = Math::max<uint_t>(txPulseLengthMm, 150);

        // ��ȡ��
        std::unique_lock<std::mutex> lock(pingQueueMutex);
        if (pingQueue.size() < MAX_QUEUE_SIZE) {
            pingQueue.push(ping); // ��������ӵ�������
            consumerCondition.notify_one(); // ֪ͨ�������߳�
        }
        else {
            std::cout << "Ping queue is full. Dropping ping data!!!!!!" << std::endl;
            pingQueue.pop(); // �������������������ɵ�����
            pingQueue.push(ping); // ����µ�����
            consumerCondition.notify_one(); // ֪ͨ�������߳�
        }
        std::cout << "Producer thread add data!" << std::endl; // �������
        lock.unlock(); // ����������

        sonarDataStore.add(ping, txPulseLengthMm);
        //recordPingData(iss360, ping, txPulseLengthMm);
        m_pingCount++;

        // �����·������趨ָ��
        // if (pingDataCount == 10) {
        //     std::cout << "----------------------------------ready set parameter----------------------------------" << std::endl;
        //     Sonar& sonar = reinterpret_cast<Sonar&>(*m_device);
        //     Sonar::Setup temp_setup;
        //     temp_setup.stepSize = 16;
        //     sonar.setSetupSettings(temp_setup, false);
        //     std::cout << "----------------------------------ready set parameter----------------------------------" << std::endl;
        // }

        // ����ֱ�ӽ���ԭʼɨ������Ϊͼ�񣬲���ʧ����
        // m_circular.render(sonarDataStore, m_palette, true);
        // convert_uint_32_array_to_opencv_mat(reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), COLS, ROWS);

        // ʵʱɨ����ʾͼ��
        // this->doTask('i', dataFolder_); // ��ʼɨ��
        // cv::Mat temp_img = cv::imread("E:/program/SYZ/build/bin/Debug/sonar.bmp", cv::IMREAD_UNCHANGED);
        // cv::imshow("sonar", temp_img);
        // cv::waitKey(1); // �ȴ�1�����Ը��´���

        if (m_pingCount % (Sonar::maxAngle / iss360.settings.setup.stepSize) == 0) {
            m_pingCount = 0;
            /*
            m_texture.renderTexture(sonarDataStore, m_palette, false);
            BmpFile::save("snrTex.bmp", reinterpret_cast<const uint32_t*>(&m_texture.buf[0]), 32, m_texture.width, m_texture.height);

            m_circular.render(sonarDataStore, m_palette, true);
            BmpFile::save("snrCi.bmp", reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), 32, m_circular.width, m_circular.height);
            */
        }
        return;
    }
}

void SonarApp::recordPingData(const Sonar & iss360, const Sonar::Ping & ping, uint_t txPulseLengthMm)
{
    std::cerr << "���е�recordPingData" << std::endl;
    //���ж��Ƿ�ɼ�shanxing����
    Device::Info adevice;
    msis_pkg::RangeImageBeam temp_ping_;
    temp_ping_.angle_du = float(ping.angle) * 360 / 12800;
    temp_ping_.angle_stepSize = float(ping.stepSize) * 360 / 12800;
    temp_ping_.minRange = float(ping.minRangeMm) / 1000;
    temp_ping_.max_range = float(ping.maxRangeMm) / 1000;
    temp_ping_.data_count = static_cast<int>(ping.data.size());

    for (int i_ = 0; i_ < ping.data.size(); i_++) {
        temp_ping_.beam_data.push_back(ping.data[i_]);
    }
    // ���㵱ǰ�Ƕ�
    float current_angle = float(ping.angle) * 360 / 12800;
    // �ж��Ƿ�ӽ���ʼ�߽� (315��/270��)
    if (!isCollecting && std::abs(current_angle - START_ANGLE) <= ANGLE_TOLERANCE)
    {
        std::cout << "Starting data collection at angle: " << current_angle << " degrees." << std::endl;
        isCollecting = true;

    }

    //////  �жϵ�ǰ�Ƕ��Ƿ��ںϷ���Χ (315�� ~ 45��)
    // if (current_angle < 315.0 && current_angle>45)
    //{
    //     //isValidAngle = false;
    //     isCollecting = false;
    //     std::memset(shanxing, 0, sizeof(shanxing));
    //     currentCol = 0; // �������� currentCol = 0; ����������
    //}
      /*�жϵ�ǰ�Ƕ��Ƿ��ںϷ���Χ (270�� ~ 90��)*/
    if (current_angle < 270.0 && current_angle>90)
    {
        //isValidAngle = false;
        isCollecting = false;
        std::memset(shanxing, 0, sizeof(shanxing));
        currentCol = 0; // �������� currentCol = 0; ����������
    }

    if (isCollecting)
    {
        //if (currentCol >= COLS)
        //{
        //    std::cerr << "Array full, cannot store more data." << std::endl;
        //    currentCol = 0; // �������� currentCol = 0; ����������
        //    std::memset(shanxing, 0, sizeof(shanxing));
        //    std::cerr << "Array clear all data." << std::endl;
        //}
        // �� ping.data �洢�� shanxing ����ĵ�ǰ��
        for (int i = 0; i < ping.data.size() && i < ROWS; ++i) { //�����Ҷ�int���г�ʼ�޸ģ�ּ���˳�ǰ��i�������
            shanxing[i][currentCol] = static_cast<float>(ping.data[i]);
        }

        // ���µ�ǰ������
        currentCol++;
        if (currentCol == COLS)
        {
            //jushu_shanxing++;
            flag_shanxing = 1;
            //std::cout << "Displaying shanxing matrix:" << std::endl;

            //for (int i = 0; i < ROWS; ++i) {
            //    for (int j = 0; j < COLS; ++j) {
            //        // ���ù̶���ȱ��ڹ۲����ṹ
            //        std::cout << std::setw(5) << shanxing[i][j] << " ";
            //    }
            //    std::cout << std::endl; // ����
            //}

            //std::cout << "End of matrix display." << std::endl;
    //        // �洢���ļ�
    //
    //
    ////float shanxing[ROWS][COLS] = {0}; // ��������������
    //        saveShanxingToFile(&shanxing[0][0], ROWS, COLS, "shanxing_data.csv");
        }
    }
    // ��ȡ��ǰʱ���
    auto now = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&timestamp);
    uint16_t year = now_tm->tm_year + 1900; // ��ȡ���������
    uint8_t month = now_tm->tm_mon + 1;
    uint8_t day = now_tm->tm_mday;
    uint8_t hour = now_tm->tm_hour;
    uint8_t minute = now_tm->tm_min;
    uint8_t second = now_tm->tm_sec;
    uint8_t status = 0x00; // ʾ��״̬
    // ����״̬λ��Bit0, Bit1, Bit2, Bit3
    // ʹ��ȫ�ֱ��������ж�
    //std::string deviceInfo = std::to_string(globalPn) + "." + std::to_string(globalSn);
    //if (globalPn == 2254 && globalSn == 25)
    //{
    //    status |= 0x01; // ���� Bit0 Ϊ 1
    //}
    //if (globalPn == 2254 && globalSn == 23)
    //{
    //    status |= 0x01; // ���� Bit0 Ϊ 1
    //}
    //if (globalPn == 2255 && globalSn == 10)
    //{
    //    status |= 0x01; // ���� Bit0 Ϊ 1
    //}
    // ���� angle �� speed
    if (biaozhi == 1)
    {
        if (m_sonar_app_index == 0)
        {
            status = 0x04; //��Ŀ��
            status |= globalstatus1;//������
        }
        else
        {
            status = 0x04; //��Ŀ��
            status |= globalstatus2;//������
        }
    }
    else
    {
        if (m_sonar_app_index == 0)
        {
            status |= globalstatus1;//��Ŀ��
        }
        else
        {
            status |= globalstatus2;//��Ŀ��
       
        }
    }
    uint16_t angle = static_cast<uint16_t>((ping.angle * 65535) / 12800);
    uint8_t speed;
    float stepSizeInDegrees = ping.stepSize * 360.0 / 12800.0;
    if (stepSizeInDegrees < 0.5)
    {
        speed = 1;
    }
    else if (stepSizeInDegrees < 1.0)
    {
        speed = 2;
    }
    else
    {
        speed = 3;
    }
    // ������ݴ����߼�
    //processData(ping.data);
    //// �������ݴ��������з���
    //bool targetDetected = checkForTarget(temp_ping_.beam_data);
    //if (targetDetected) {
    //    status |= 0x04; // ���� Bit2 Ϊ 1 ��ʾ��Ŀ��
    //}
    // ʾ��Ŀ���⺯��
    //bool SonarApp::checkForTarget(const std::vector<uint16_t>&beam_data)
    //{
    //    // ���������Ŀ�����߼�
    //    // ���� true ��ʾ��⵽Ŀ�꣬���򷵻� false
    //    return std::any_of(beam_data.begin(), beam_data.end(), [](uint16_t value) {
    //        return value > 100; // ʾ��������ֵ���� 100 ��ʾ��Ŀ��
    //        });
    //}
    globalcount = temp_ping_.data_count;
    //globalx = 1;
    uint8_t minrange = temp_ping_.minRange;
    uint8_t maxrange = temp_ping_.max_range;
    
// #define USE_C_FILE_STREAM
#ifndef USE_C_FILE_STREAM
    // �����ļ���
    char filename[256];
    snprintf(filename, sizeof(filename), "%s/all_ping_data.txt", dataFolder_.c_str());

    
    // ���ļ�����׷��ģʽ��
    FILE* outputFile = fopen(allPingDataFilename_.c_str(), "a");
    if (outputFile != NULL) {
        // д�����ݵ��ļ�
        fprintf(outputFile, "Processed %d Ping Data:\n", m_sonar_app_index);

        // д��ʱ���
        char timeBuffer[64];
        struct tm* localTime = localtime(&timestamp);
        strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", localTime);

        fprintf(outputFile, "Timestamp: %s\n", timeBuffer);

        // д����������
        fprintf(outputFile, "Angle: %f\n", temp_ping_.angle_du);
        fprintf(outputFile, "Step Size: %f\n", temp_ping_.angle_stepSize);
        fprintf(outputFile, "Min Range (m): %2f\n", temp_ping_.minRange);
        fprintf(outputFile, "Max Range (m): %2f\n", temp_ping_.max_range);
        fprintf(outputFile, "Speed of Sound: %3f\n", iss360.settings.system.speedOfSound);
        fprintf(outputFile, "Tx Pulse Width (us): %f\n", iss360.settings.acoustic.txPulseWidthUs);
        fprintf(outputFile, "Tx Pulse Length (mm): %f\n", txPulseLengthMm);
        fprintf(outputFile, "Data Count: %d\n", ping.data.size());

        // д�� Beam Data
        fprintf(outputFile, "Beam Data: ");
        int temp;
        for (int i = 0; i < temp_ping_.data_count; ++i) {
            temp = static_cast<int>(temp_ping_.beam_data[i]);
            // fprintf(outputFile, "%1f ", temp_ping_.beam_data[i]);
            fprintf(outputFile, "%d ", temp); // ʹ�� %d ��ʽ���������
        }
        fprintf(outputFile, "\n");

        // �ر��ļ�
        fclose(outputFile);
        printf("Data appended to %s\n", allPingDataFilename_.c_str());
    }
    else {
        perror("Failed to open file");
    }
#else
    std::stringstream filenameStream;
    // �����ļ�����ʹ��ʱ�����Ϊ�ļ�����һ����

    filenameStream << dataFolder_ << "/all_ping_data1.txt";
    std::string filename = filenameStream.str();

    // ������д���ļ�
    std::ofstream m_outputFile(allPingDataFilename_, std::ios::app);  // ע������ʹ�� std::ios::app ����׷��д��
    if (m_outputFile.is_open())
    {
        // ��¼���ݵ��ļ�
        m_outputFile << "Processed Ping Data:" << std::endl;
        m_outputFile << "Timestamp: " << std::put_time(std::localtime(&timestamp), "%Y-%m-%d %H:%M:%S") << std::endl;
        m_outputFile << "Angle: " << temp_ping_.angle_du << std::endl;
        m_outputFile << "Step Size: " << temp_ping_.angle_stepSize << std::endl;
        m_outputFile << "Min Range (m): " << temp_ping_.minRange << std::endl;
        m_outputFile << "Max Range (m): " << temp_ping_.max_range << std::endl;
        m_outputFile << "Speed of Sound: " << iss360.settings.system.speedOfSound << std::endl;
        m_outputFile << "Tx Pulse Width (us): " << iss360.settings.acoustic.txPulseWidthUs << std::endl;
        m_outputFile << "Tx Pulse Length (mm): " << txPulseLengthMm << std::endl;
        m_outputFile << "Data Count: " << ping.data.size() << std::endl;
        m_outputFile << "Beam Data: ";
        for (int i = 0; i < temp_ping_.data_count; ++i) {
            m_outputFile << temp_ping_.beam_data[i] << " ";
        }
        m_outputFile << std::endl;
        m_outputFile.close();
        std::cout << "Data appended to " << allPingDataFilename_ << std::endl;


    }
    else
    {
        std::cerr << "Unable to open the file for writing." << std::endl;
    }

#endif 

    
    // ���͸�ʽ������
    

    sendFormattedData(year, month, day, hour, minute, second, status, angle, speed, minrange, maxrange);

    static int send_count = 0;
    send_count++;
    if (send_count % 20 == 0) {
        std::cout << "Send data count: " << send_count << std::endl;
        int temp ;
        send_count = 0;
        //std::cerr << "�ж�֮ǰ" << std::endl;
        if (globalstatus1 == 0x03|| globalstatus1==0x07 || globalstatus2 == 0x07 || globalstatus2 == 0x03)
        {
            send_count = 0;
             ;
        }
        else{
            //std::cerr << "����else" << std::endl;
            if (m_sonar_app_index == 0) 
            {
               // std::cerr << "����ǰ1" << std::endl;
                serialPort.write(sendBuffer, 28, temp);   // ʵ��վʱ��ע�ͣ�����ע��
               // memset(sendBuffer, 0, sizeof(sendBuffer));
                // saveData("D:/ceshi/output.txt", sendBuffer, 28, "COM1 Send Hex Data", 1);
                send_count = 0;
            }
            else {
                //std::cerr << "����ǰ2" << std::endl;
                serialPort.write(sendBuffer2, 28, bytesWritten222);   // ʵ��վʱ��ע�ͣ�����ע��
                //std::cerr << "���ͺ�" << std::endl;
                // saveData("D:/ceshi/output.txt", sendBuffer2, 28, "COM1 Send Hex Data", 1);
                send_count = 0;
            }
            // std::cerr << "22" << std::endl;
            // std::cerr << "222222" << std::endl;
        }
    }


}


//



void SonarApp::setDataFolder(const std::string& basePath)
{
    dataFolder_ = basePath;

    updatePingDataFilename();
    // writeInitialLog();
}
void SonarApp::updatePingDataFilename()
{
    // �����ļ�����ʹ��ʱ�����Ϊ�ļ�����һ����
    auto now = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

    std::stringstream filenameStream;
    filenameStream << dataFolder_ << "/all_ping_data_" << std::to_string(timestamp) << ".txt";
    allPingDataFilename_ = filenameStream.str();
    // ����ǰʱ��д���ļ�
    std::ofstream outputFile(allPingDataFilename_);
    if (outputFile.is_open()) {
        outputFile << "All Ping Data Log:" << std::endl;
        outputFile << "Timestamp: " << std::ctime(&timestamp) << std::endl;
        outputFile.close();
        std::cout << "Initial log created at " << allPingDataFilename_ << std::endl;
    }
    else {
        std::cerr << "Unable to open the file for writing." << std::endl;
    }
}

void SonarApp::writeInitialLog()
{

}
void SonarApp::sendUartData(const std::string& portName, uint32_t baudrate, const std::vector<uint8_t>& data)
{
    // ��ȡ����ӡ��ǰ���õĴ����б�
   /* std::vector<std::string> availablePorts = Uart::getNames();
    std::string portsList = "Available UART ports: ";
    for (const auto& port : availablePorts)
    {
        portsList += port + " ";
    }*/
    //  Debug::log(Debug::Severity::Info, "SonarApp", portsList.c_str());
    SerialPort uart(portName.c_str());
    if (uart.open())
    {
        // Debug::log(Debug::Severity::Info, "SonarApp", ("UART port opened successfully: " + portName).c_str());
        uart.config(baudrate, 8, Uart::Parity::None, Uart::StopBits::One);
        uart.write(data.data(), data.size(), baudrate);
        // Debug::log(Debug::Severity::Info, "SonarApp", "Data sent via UART");
        uart.close();
        //  Debug::log(Debug::Severity::Info, "SonarApp", ("UART port closed: " + portName).c_str());
    }
    else
    {
        Debug::log(Debug::Severity::Error, "SonarApp", ("Failed to open UART port: " + portName).c_str());
    }
}


void SonarApp::readUartData(const std::string& portName, uint32_t baudrate)
{
    SeriallPort serialPort;
    SerialPort uart("COM6");
    
    if (uart.open()) {
        if (!uart.config(baudrate, 8, Uart::Parity::None, Uart::StopBits::One)) {
            Debug::log(Debug::Severity::Error, "SonarApp", "Failed to configure UART port.");
            return;
        }

        while (true) {
            std::vector<uint8_t> buffer(1024);
            {
                std::lock_guard<std::mutex> lock(uartMutex);
                int bytesRead;
                if (!serialPort.read(reinterpret_cast<char*>(buffer.data()), buffer.size(), bytesRead)) {
                    Debug::log(Debug::Severity::Error, "SonarApp", "Failed to read data from UART port.");
                    continue;
                }
                if (bytesRead > 0) {
                    buffer.resize(bytesRead);
                }
                else {
                    continue;
                }
            }
            processReceivedData(buffer);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    else {
        Debug::log(Debug::Severity::Error, "SonarApp", "Failed to open UART port.");
    }
}

void SonarApp::processReceivedData(const std::vector<uint8_t>& data)
{
    // ��������Ӵ�����յ������ݵ��߼�
    std::string receivedData(data.begin(), data.end());
    std::cout << "Received Data: " << receivedData << std::endl;
}
void SonarApp::sendFormattedData
(

    uint16_t year, uint8_t month, uint8_t day,
    uint8_t hour, uint8_t minute, uint8_t second,
    uint8_t status, uint16_t angle, uint8_t speed,
    uint8_t minrange,
    uint8_t maxrange
)
{

    memset(&msg, 0, sizeof(msg));
    std::memcpy(msg.header, "$SMSNXX", 7);
    msg.length = 28;
    msg.year = year;
    msg.month = month;
    msg.day = day;
    msg.hour = hour;
    msg.minute = minute;
    msg.second = second;
    msg.status = status;
    msg.angle = angle;
    msg.speed = speed;
    msg.minrange = minrange;
    msg.maxrange = maxrange;
    msg.reserved = 0;
    msg.checksum = addcalculateChecksum(reinterpret_cast<uint8_t*>(&msg), sizeof(msg) - 4);
    msg.end1 = 0x0D;
    msg.end2 = 0x0A;
    std::string messageStr = messageToString(msg);

    if (m_sonar_app_index == 0)
        memcpy(sendBuffer, messageStr.c_str(), 28);
    else
        memcpy(sendBuffer2, messageStr.c_str(), 28);

    memset(&msg, 0, sizeof(msg));
}
void SonarApp::saveShanxingToFile(const float* shanxing, int rows, int cols, const std::string& filename)
{
    // ���ļ�
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // ��������д���ļ�
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            file << std::setw(10) << *(shanxing + i * cols + j); // ���ÿ�ȶ��룬���ڹ۲�
            if (j < cols - 1) {
                file << ","; // ��֮��ӿո�ָ�
            }
        }
        file << "\n"; // ÿ�н�������
    }

    file.close(); // �ر��ļ�
    std::cout << "Shanxing data saved to " << filename << std::endl;
}

//#include <ostream>
//#include <fstream>
//std::ofstream out("mat.txt", std::ios::out | std::ios::app); // ���ļ���׷��ģʽд��
//
//void convert_uint_32_array_to_opencv_mat(const uint32_t* image, uint_t width, uint_t height)
//{
//    // ����һ�� CV_8UC4 ���͵� Mat����ʾ 8 λ�޷���������4 ͨ����RGBA��
//    cv::Mat mat(height, width, CV_8UC4);
//
//    // ����ÿ�����أ��� BMP ���ݿ����� Mat ��
//    for (uint_t row = 0; row < height; row++) {
//        for (uint_t col = 0; col < width; col++) {
//            uint32_t pixel = image[row * width + col];
//
//            // ��ȡ RGBA ͨ��
//            uint8_t r = (pixel >> 16) & 0xFF;
//            uint8_t g = (pixel >> 8) & 0xFF;
//            uint8_t b = (pixel >> 0) & 0xFF;
//            uint8_t a = 0;
//            // uint8_t a = (pixel >> 24) & 0xFF;
//
//            // ������ֵ�洢�� Mat ��
//            cv::Vec4b& matPixel = mat.at<cv::Vec4b>(row, col);
//            matPixel[0] = b; // OpenCV ʹ�� BGRA ˳��
//            matPixel[1] = g;
//            matPixel[2] = r;
//            matPixel[3] = a;
//        }
//    }
//
//    cv::Mat resizedMat;
//    cv::resize(mat, resizedMat, cv::Size(), 4.0, 4.0, cv::INTER_LINEAR);
//    cv::imshow("ʵʱ����ɨ��ͼ�Ŵ�", resizedMat);


    // out << "-----------------------------------------------------------\n";
    // for (uint_t row = 0; row < height; row++) {
    //     for (uint_t col = 0; col < width; col++) {
    //         uint32_t pixel = image[row * width + col];

    //         // ��ȡ RGBA ͨ��
    //         uint8_t r = (pixel >> 16) & 0xFF;
    //         uint8_t g = (pixel >> 8) & 0xFF;
    //         uint8_t b = (pixel >> 0) & 0xFF;
    //         uint8_t a = (pixel >> 24) & 0xFF;

    //         // ������ֵ�洢�� Mat ��
    //         out << "Pixel[" << row << "][" << col << "] = (" << static_cast<int>(r) << ", " << static_cast<int>(g) << ", " << static_cast<int>(b) << ", " << static_cast<int>(a) << ")" << std::endl;
    //     }
    // }

    // out << "-----------------------------------------------------------\n";

//    cv::imshow("ʵʱ����ɨ��ͼ", mat); // ��ʾͼ��
//    cv::waitKey(1); // �ȴ� 1 �����Ը��´���
//
//    return;
//}

// �Ƿ�ʹ��֡���ע�͵���ʹ��֡�����ʹ�ñ�����ģ��
#define USE_NEW_ALGORITHM
// �Ƿ����ģ�ͣ���������صĻ��������Զ�����ģ�ͣ�ע�͵�������򲻼��أ�ÿ��ѵ��
// #define LOAD_MODEL
// ģ�͵������ڶ����ֺ󽵵�ѧϰ��
#define MAKE_MODEL_TIMES 10
// ʹ�ü�����黹��PDF�����ܶȷֲ������ж���ע�͵������ͻ�ʹ�ü�����鷽��
#define USE_PDF
// �Ƿ����� debug ���Ե�ͼ����ʾ�������ʵ�ʲ����ʱ���뽫�˺���Ϊ0
#define USE_IMSHOW 0
// �Ƿ���ݾ��붯̬������ֵ����ֹĿ���Զ������ǿ�Ȳ����ߣ����±�����Ϊ����
#define USE_DYNAMIC_THRESHOLD 0
// cv::Mat qiang_du_tu(101, 200, CV_8UC1, cv::Scalar(0));
#define USE_SYZ 1
// �Ƿ�Ϊʵ��վ��Ŀʹ��
void SonarApp::consumePingData()
{
CONSUMER_START:
    std::cerr << "�������߳̿�ʼ����" << std::endl;
    std::unique_lock<std::mutex> lock(pingQueueMutex);
    consumerCondition.wait(lock, [this] { return !pingQueue.empty(); });
    std::cout << "Consumer thread comsume data!  Queue size = " << pingQueue.size() << std::endl; // �������
    Sonar::Ping ping = pingQueue.front(); // ��ȡ����ǰ�˵�Ԫ��
    pingQueue.pop(); // �Ƴ�����ǰ�˵�Ԫ��
    lock.unlock(); // ����������
    
#ifdef USE_NEW_ALGORITHM
    int temp_qiangdutu_index = convert_angle2index(ping.angle);
    for (int i = 0; i < ping.data.size(); i++) {
        int temp = (double(ping.data[i]) / 65536.0 * 255.0);
		qiang_du_tu.at<uchar>(temp_qiangdutu_index, i) = temp;
    }

    bool flag_have_goal = 0;//Ŀ���Ƿ����
// ͼ����ʾ debug
#if USE_IMSHOW == 1
    cv::imshow("qiang_du_tu" + std::to_string(m_sonar_app_index), qiang_du_tu);
    cv::waitKey(1);
    // �Ŵ�ͼ���Ա�ۿ�
    cv::Mat qiang_du_tu_resized;
    cv::resize(qiang_du_tu, qiang_du_tu_resized, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST); // �Ŵ�4��
    cv::imshow("qiang_du_tu_resized" + std::to_string(m_sonar_app_index), qiang_du_tu_resized);
    cv::waitKey(1); // �ȴ� 1 �����Ը��´���
#endif
    // ���ת��һȦ�ˣ�ͳ��תȦ�����ļ�����++��
    if (ping.angle == 0) //
    {
        // ����Ѿ��Ǽ��׶Σ����߻���ѧϰ�׶Σ���ʾͼ��
        if (flag_making_model == 0 || flag_making_model == 2) 
        {
#if USE_IMSHOW == 1
            cv::imshow("goal" + std::to_string(m_sonar_app_index), is_goal);
            cv::waitKey(1); // �ȴ� 1 �����Ը��´���
            cv::Mat is_goal_resized;
            cv::resize(is_goal, is_goal_resized, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST); // �Ŵ�4��
            cv::imshow("is_goal_resized" + std::to_string(m_sonar_app_index), is_goal_resized);
            cv::waitKey(1); // �ȴ� 1 �����Ը��´���

            // ������
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // �����
            cv::Mat result;
            cv::morphologyEx(is_goal, result, cv::MORPH_OPEN, kernel); // ������
            cv::imshow("After Opening" + std::to_string(m_sonar_app_index), result);
            cv::waitKey(1);

            //// ����ͼ�񣬿�����û�а�ɫ�ĵ㣬������Ϊ��Ŀ��
            int temp_have_goal = 0;
            //for (int i = 0; i < 101; i++)
            //    for (int j = 0; j < 200; j++) {
            //        if (is_goal.at<uchar>(i, j) == 255) 
            //        {
            //            temp_have_goal = 1;
            //            // ��Ŀ��Ĳ�����i�ǽǶ�����j�Ǿ���ֵ
            //            // < 50 �� 270~360 >50 �� 0~90
            //            // temp_angle ����ŵľ��ǻ�ԭ���ʵ�ʽǶ�
            //            int temp_angle = (i < 50) ? ((i * 64 + 9600.00) / 35.5) : ((i - 50) * 64.0 / 35.5);//����ǰ���ʽ�Ӿ���������ж�s
            //        }

            //    }
            std::vector<std::vector<cv::Point>> contours; // �洢����
            std::vector<cv::Vec4i> hierarchy; // �洢�����Ĳ�νṹ
            cv::findContours(result, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // ��������
            // ����������������Ӿ���
            for (size_t i = 0; i < contours.size(); i++)
            {
                cv::Rect boundingBox = cv::boundingRect(contours[i]); // ������Ӿ���
                // cv::rectangle(is_goal, boundingBox, cv::Scalar(255), 2); // ������Ӿ���
                int centerX = boundingBox.x + boundingBox.width / 2; // �������ĵ�� x ����
                int centerY = boundingBox.y + boundingBox.height / 2; // �������ĵ�� y ���꣬�����뵥λ
                // �������ĵĽǶ�
                double temp_angle = (centerY < 50) ? ((centerY * 64 + 9600.00) / 35.5) : ((centerY - 50) * 64.0 / 35.5);
                double temp_dis = 100.0 * centerX / 200.0; // ���뵥λ
                std::string angele_str = std::to_string(temp_angle);
                std::string dis_str = std::to_string(temp_dis);
                std::string temp_buffer;
                std::cout << "CenterX: " << centerX << " CenterY: " << centerY << " Angle: " << angele_str << " Distance: " << dis_str << std::endl;
                temp_buffer = angele_str + "," + dis_str + "\n";
                saveData("D:/ceshi/Seriallog.txt", temp_buffer.c_str(), strlen(temp_buffer.c_str()), "mubiaojuli", 0);
                //saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send xiaoshi2", 0);
            }
            if (contours.empty()) {
                temp_have_goal = 0;
                std::cout << "No contours found." << std::endl;
            }
            else {
                std::cout << "Contours found: " << contours.size() << std::endl;
                temp_have_goal = 1;
                saveImageWithTimestamp_mubiao(is_goal);
                saveImageWithTimestamp_mubiao(result);
            }

            if (temp_have_goal == 1)
            {
                flag_have_goal = 1;
                biaozhi = temp_have_goal;
            }
            else
            {
                flag_have_goal = 0;
                flag_zhiling = 1;//��һ�η���Ϣ�ı�־
            }

            if (flag_have_goal == 1 && flag_zhiling == 1)
            {


                if (m_sonar_app_index == 0)
                {
                    //serialPort.write(writeBufferjinggao, 16, bytesWritten21);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM1 Send jinagao1", 0);
                    serialPort2.write(writeBufferjinggao, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM2 Send jinagao1", 0);

                }
                else
                {
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao), "COM1 Send jinagao1", 0);
                    serialPort2.write(writeBufferjinggao2, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao), "COM2 Send jinagao1", 0);

                }
                flag_zhiling = 0;
            }
            if (flag_have_goal == 0 && flag_zhiling == 0)
            {
                if (m_sonar_app_index == 0)
                {
                    //serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM1 Send xiaoshi1", 0);
                    //serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send xiaoshi1", 0);
                }
                else
                {
                    //serialPort.write(writeBufferxiaoshi2, 16, bytesWritten21);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM1 Send xiaoshi2", 0);
                    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send xiaoshi2", 0);
                }
                biaozhi = 0;
                flag_zhiling = 1;
            }
            cv::imshow("Contours", is_goal);

           
            // ��������б�ǿ��ͼ
            cv::Mat zui_da_qiang_du_tu(101, 200, CV_8UC1, cv::Scalar(0));
            for (int i = 0; i < 101; i++)
                for (int j = 0; j < 200; j++) {
                    double temp_sigma = std::sqrt(model_array[i][j].sigma2); // ��׼��
                    double temp = std::exp(model_array[i][j].mu + 2.5 * temp_sigma);//���������ֵ����һ��
                    temp = (temp > 65535) ? 65535 : temp;
                    temp /= 65535.0;
                    temp *= 255.0;
                    int temp2 = static_cast<int>(temp);
                    zui_da_qiang_du_tu.at<uchar>(i, j) = temp2;
                }
            cv::imshow("zui_da_qiang_du_tu" + std::to_string(m_sonar_app_index), zui_da_qiang_du_tu);
            cv::waitKey(1);

            cv::Mat zui_da_qiang_du_tu4x;
            cv::resize(zui_da_qiang_du_tu, zui_da_qiang_du_tu4x, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST); // �Ŵ�4��
            cv::imshow("zui_da_qiang_du_tu4x" + std::to_string(m_sonar_app_index), zui_da_qiang_du_tu4x);
            cv::waitKey(1); // �ȴ� 1 �����Ը��´���
#else
            
            //cv::imshow("goal" + std::to_string(m_sonar_app_index), is_goal);
            //cv::waitKey(1); // �ȴ� 1 �����Ը��´���
            //cv::Mat is_goal_resized;
            //cv::resize(is_goal, is_goal_resized, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST); // �Ŵ�4��
            //cv::imshow("is_goal_resized" + std::to_string(m_sonar_app_index), is_goal_resized);
            //cv::waitKey(1); // �ȴ� 1 �����Ը��´���
            std::cerr << "ͼ����" << std::endl;
            // ������
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // �����
            cv::Mat result;
            cv::morphologyEx(is_goal, result, cv::MORPH_OPEN, kernel); // ������
            //cv::imshow("After Opening" + std::to_string(m_sonar_app_index), result);
            //cv::waitKey(1);
            
            //// ����ͼ�񣬿�����û�а�ɫ�ĵ㣬������Ϊ��Ŀ��
            int temp_have_goal = 0;
            //for (int i = 0; i < 101; i++)
            //    for (int j = 0; j < 200; j++) {
            //        if (is_goal.at<uchar>(i, j) == 255) 
            //        {
            //            temp_have_goal = 1;
            //            // ��Ŀ��Ĳ�����i�ǽǶ�����j�Ǿ���ֵ
            //            // < 50 �� 270~360 >50 �� 0~90
            //            // temp_angle ����ŵľ��ǻ�ԭ���ʵ�ʽǶ�
            //            int temp_angle = (i < 50) ? ((i * 64 + 9600.00) / 35.5) : ((i - 50) * 64.0 / 35.5);//����ǰ���ʽ�Ӿ���������ж�s
            //        }

            //    }
            std::vector<std::vector<cv::Point>> contours; // �洢����
            std::vector<cv::Vec4i> hierarchy; // �洢�����Ĳ�νṹ
            cv::findContours(result, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // ��������


            // ����������������Ӿ���
            for (size_t i = 0; i < contours.size(); i++)
            {
                cv::Rect boundingBox = cv::boundingRect(contours[i]); // ������Ӿ���
                // cv::rectangle(is_goal, boundingBox, cv::Scalar(255), 2); // ������Ӿ���
                int centerX = boundingBox.x + boundingBox.width / 2; // �������ĵ�� x ����
                int centerY = boundingBox.y + boundingBox.height / 2; // �������ĵ�� y ���꣬�����뵥λ
                // �������ĵĽǶ�
                double temp_angle = (centerY < 50) ? ((centerY * 64 + 9600.00) / 35.5) : ((centerY - 50) * 64.0 / 35.5);
                double temp_dis = 100.0 * centerX / 200.0; // ���뵥λ
                std::string angele_str = std::to_string(temp_angle);
                std::string dis_str = std::to_string(temp_dis);
                std::string temp_buffer;
                std::cout << "CenterX: " << centerX << " CenterY: " << centerY << " Angle: " << angele_str << " Distance: " << dis_str << std::endl;
                temp_buffer = angele_str + "," + dis_str + "\n";
                saveData("D:/ceshi/Seriallog.txt", temp_buffer.c_str(), strlen(temp_buffer.c_str()), "mubiaojuli", 0);
                //saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send xiaoshi2", 0);
            }
            if (contours.empty()) {
                temp_have_goal = 0;
                std::cout << "No contours found." << std::endl;
            }
            else {
                std::cout << "Contours found: " << contours.size() << std::endl;
                temp_have_goal = 1;
                saveImageWithTimestamp_mubiao(is_goal);
                saveImageWithTimestamp_mubiao(result);
            }

            if (temp_have_goal == 1)
            {
                flag_have_goal = 1;
                biaozhi = temp_have_goal;
            }
            else
            {
                flag_have_goal = 0;
                flag_zhiling = 1;//��һ�η���Ϣ�ı�־
            }

            if (flag_have_goal == 1 && flag_zhiling == 1)
            {


                if (m_sonar_app_index == 0)
                {
                    //serialPort.write(writeBufferjinggao, 16, bytesWritten21);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM1 Send jinagao1", 0);
                    serialPort2.write(writeBufferjinggao, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM2 Send jinagao1", 0);

                }
                else
                {
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao), "COM1 Send jinagao1", 0);
                    serialPort2.write(writeBufferjinggao2, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao), "COM2 Send jinagao1", 0);

                }
                flag_zhiling = 0;
            }
            if (flag_have_goal == 0 && flag_zhiling == 0)
            {
                if (m_sonar_app_index == 0)
                {
                    //serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM1 Send xiaoshi1", 0);
                    //serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send xiaoshi1", 0);
                }
                else
                {
                    //serialPort.write(writeBufferxiaoshi2, 16, bytesWritten21);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM1 Send xiaoshi2", 0);
                    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send xiaoshi2", 0);
                }
                biaozhi = 0;
                flag_zhiling = 1;
            }
#endif

        }
        circle_times++;
        std::cout << "circle_times = " << circle_times << std::endl;
        
#ifndef LOAD_MODEL
        // ����ÿһ�ֵ�ģ�Ͳ���
        //if (circle_times <= MAKE_MODEL_TIMES + 1)
            //write_model_param("model_" + std::to_string(circle_times) + ".txt");
#endif

        // �������
        if (flag_making_model == -1)
            flag_making_model = 1; // ����ǵ�һ��תȦ����ʼ��ģ
    }

    // ���ת��20Ȧ����Ϊģ���Ѿ�������ȡ����ģ������Ԥ��״̬
    if (circle_times == MAKE_MODEL_TIMES) {
        flag_making_model = 2; // ���뻺��ѧϰ�׶�
    }

    // �����ǰ���ڽ�ģ
    if (flag_making_model == 1 || flag_making_model == 2) {
#ifdef LOAD_MODEL
        std::fstream in("model_11.txt", std::ios::in); // ���ļ��Զ�ȡ
        if (!in) 
        {
            std::cerr << "Error opening file for reading." << std::endl;
            return;
        }
        // ��ȡģ�Ͳ���
        for (auto& i : model_array) {
            for (auto& j : i) {
                in >> j.mu >> j.sigma2;
            }
        }
        in.close(); // �ر��ļ�
        flag_making_model = 0;
#else
        // ����ǵ�һ��תȦ��ֱ��ȡ��ֵ��Ϊ��ֵ�����ֵΪ0.1
        if (circle_times == 1)
        {
            int temp_index = convert_angle2index(ping.angle);//���ｫ���ݷŽ�temp
            for (int i = 0; i < ping.data.size(); i++) 
            {
                model_array[temp_index][i].mu = std::log(ping.data[i]);
                model_array[temp_index][i].sigma2 = 0.1;
                model_array[temp_index][i].t = 1;
            }
        }
        else
        { // ���򣬵�����ģ
            // ���ң����ڲ���Ϊ 1.8 ��״̬�£�����Ǳ�������Ī������ɨ��ֵ��� 0.9 �����
            if (ping.stepSize == 64) {
                #if USE_IMSHOW == 1
                std::cout << "angle = " << ping.angle << "\n";
                #endif
                int temp_index = convert_angle2index(ping.angle);
                #if USE_IMSHOW == 1
                std::cout << "temp_index = " << temp_index << "\n";
                #endif

                // �����ǰû��Ŀ�꣬��ô������ǰ�Ƕ��µ�ÿһ�����룬���е������£���Ŀ��Ͳ�ѧϰ��
                //if (flag_have_goal == 0)
                for (int i = 0; i < ping.data.size(); i++)
                    // recursive_update(model_array[temp_index][i], ping.data[i], (flag_making_model == 1) ? 0.5 : 0.2);
                    recursive_update(model_array[temp_index][i], ping.data[i], 0.5);
                //else
                //    printf("[DEBUG]: There is a target, do not update the model!!!!!!!\n");
            }
            else
                std::cout << "[ERROR]: Step size is " << ping.stepSize << std::endl;
        }
#endif
    }

    // ����ǻ�����ͼ�׶λ��߼��׶Σ�����Ŀ����
    if (flag_making_model == 0 || flag_making_model == 2) {
        if (ping.stepSize == 64) {
            int temp_index = convert_angle2index(ping.angle);

            int temp_flag = 0;

            for (int i = 10; i < ping.data.size(); i++) {
                temp_flag = 0;
#ifndef USE_PDF
                temp_flag = detect_anomaly(model_array[temp_index][i], ping.data[i]);
#else
                // ��ԭ��С��������
                if (std::log(ping.data[i]) < model_array[temp_index][i].mu)
                    continue;

                double temp_p = lognormalPDF(ping.data[i], model_array[temp_index][i].mu, model_array[temp_index][i].sigma2);
                double temp_sigma = std::sqrt(model_array[temp_index][i].sigma2);
#if USE_DYNAMIC_THRESHOLD == 1
                double max_value = std::exp(model_array[temp_index][i].mu + (5.5 + (-2 / 190.0) * i)* temp_sigma);
#else
                double max_value = std::exp(model_array[temp_index][i].mu + 2* temp_sigma);//�������sigma��ֵ x* temp_sigma
#endif
                max_value < 0 ? max_value = 0 : 65536;
                max_value > 65535 ? max_value = 65535 : max_value;
                double p_min = lognormalPDF(max_value, model_array[temp_index][i].mu, model_array[temp_index][i].sigma2);

                if (temp_p < p_min) {
                    temp_flag = 1;
                    printf("[DEBUG]:value: %d, upper: %f, mu: %f, f(x): %f, f(mu+sigma): %f\n", ping.data[i], max_value, std::exp(model_array[temp_index][i].mu), temp_p, p_min);
                }
#endif
                is_goal.at<uchar>(temp_index, i) = (temp_flag == 0) ? 0 : 255; // 0��ʾ��Ŀ�꣬1��ʾ��Ŀ��
                //if (temp_flag == 1)
                //    printf("[GOAL]: value = %d, mu = %f, sigma2 = %f\n", (int) ping.data[i], model_array[temp_index][i].mu, model_array[temp_index][i].sigma2);
            }
        }
        else
            std::cout << "[ERROR]: Step size is not 1.8, please check!" << std::endl;
    }
    
    flag_shanxing = 0;
#endif

    // ���� pingData
    uint_t txPulseLengthMm = static_cast<uint_t>((*m_piss360).settings.system.speedOfSound * (*m_piss360).settings.acoustic.txPulseWidthUs * 0.001 * 0.5);
    txPulseLengthMm = Math::max<uint_t>(txPulseLengthMm, 150);
    recordPingData(*m_piss360, ping, txPulseLengthMm);


    if (flag_shanxing == 1) 
    {
        jishu_shanxing = jishu_shanxing + 1;
        if (jishu_shanxing == 1) {
            memcpy(frame1, shanxing, sizeof(shanxing));
            cv::Mat frame11 = cv::Mat(ROWS, COLS, CV_32FC1, frame1).clone(); // ��¡��֤���ݶ���
            //cv::Mat frame11Display;
            cv::normalize(frame11, frame11Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            // ���ű���
            double scale1 = 2.0;
            cv::Mat  frame11Resized;
            cv::resize(frame11Display, frame11Resized, cv::Size(), scale1, scale1, cv::INTER_NEAREST);
            saveImageWithTimestamp_beijing(frame11Display);

            //memcpy(frame1, frame4, sizeof(frame1));
        }
        jishi_mubiao++;
        //memcpy(frame1, frame2, sizeof(frame1));
        memcpy(frame2, shanxing, sizeof(shanxing));
        /*        }*/

        if (jishu_shanxing > 1000) {
            jishu_shanxing = 1;
            jishi_mubiao = 1;
        }
        //frame2 = shanxing;
        flag_shanxing = 0;
        // ����ά����ת��Ϊ cv::Mat

        cv::Mat frame22 = cv::Mat(ROWS, COLS, CV_32FC1, frame2).clone();

        // ��һ���� 0~255 ������ʾ
        //cv::Mat frame22Display;
        //cv::normalize(frame11, frame11Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::normalize(frame22, frame22Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        // ���ű���
        double scale = 2.0;
        // ���ͼ���Ƿ�Ϊ��
        if (frame22Display.empty()) {
            std::cerr << "Error: frame22Display is empty!" << std::endl;
            return;
        }

        // �Ŵ�ͼ��
        cv::Mat  frame22Resized;
        // cv::resize(frame11Display, frame11Resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
        cv::resize(frame22Display, frame22Resized, cv::Size(), scale, scale, cv::INTER_NEAREST);// ��ʾͼ��
        // cv::imshow("Frame11", frame11Display);
       // std::cout << "frame22 data: " << frame22Display << std::endl;
        bool result = cv::imwrite("D:/ceshi/frame22_image.png", frame22Resized);
        if (result) {
            std::cout << "ͼ���ѱ��浽�ļ���!" << std::endl;
        }
        else {
            std::cerr << "����ͼ��2ʧ��!" << std::endl;
        }



        // 
        //ͼ����
        if (jishu_shanxing > 1) 
        {
            int no_target = 0;  // ��ʼ����Ŀ���־
            int no_target1 = 0;  // ��ʼ����Ŀ���־
            Centroid centroid;  // ��ʼ�����Ľṹ��

            if (jishu_shanxing == 2 || jishu_shanxing == 3 || jishu_shanxing == 4) {
                serialPort.write(writeBufferimage, 24, bytesWritten21);//���������б�
                saveData("D:/ceshi/Seriallog.txt", writeBufferimage, strlen(writeBufferimage), "COM1 Send", 0);
            }
            // ���� process_frame_difference ����
            int result = process_frame_difference(frame11Display, frame22Display, no_target, centroid);//���ݴ���
            flag_target = no_target;


            if (flag_zhiling == 0) {



                jishu_guding++;
                if (jishu_guding > 15) {
                    //ʵ��վ
                    //if (globalPn == 2255 && globalSn == 10)
                    //{
                    //    //serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                    //    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
                    //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send jingzhi2", 0);
                    //    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                    //    //serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                    //    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi2", 0);
                    //}
                    //else if (globalPn == 2254 && globalSn == 25)
                    //{
                    //    // serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                    //    // serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                    //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send jingzhi1", 0);
                    //    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                    //   // serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                    //    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi1", 0);
                    //}
                    ////����ʵ��վ˫����
                    //if (globalPn == 2255 && globalSn == 10)
                    //{
                    //    serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                    //    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
                    //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send jingzhi2", 0);
                    //    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                    //    //serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                    //    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi2", 0);
                    //}
                    //else if (globalPn == 2254 && globalSn == 25)
                    //{
                    //     serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                    //    // serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                    //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send jingzhi1", 0);
                    //    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                    //   // serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                    //    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi1", 0);
                    //}
                    //����ʵ��վ������
                    serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                    // serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send jingzhi", 0);
                    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                   // serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi", 0);


                    memcpy(frame1, frame2, sizeof(frame1));
                    cv::Mat frame11 = cv::Mat(ROWS, COLS, CV_32FC1, frame1).clone(); // ��¡��֤���ݶ���
                    //cv::Mat frame11Display;
                    cv::normalize(frame11, frame11Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                    // ���ű���
                    double scale1 = 2.0;
                    cv::Mat  frame11Resized;
                    cv::resize(frame11Display, frame11Resized, cv::Size(), scale1, scale1, cv::INTER_NEAREST);
                    saveImageWithTimestamp_beijing(frame11Display);
                    //flag_jingzhi = 0;
                    //flag_yundong = 1;
                    jishu_guding = 0;
                    flag_zhiling = 1;
                    flag_target = 0;
                    biaozhi = 0;
                }
            }
        }

        if (flag_target == 1 && flag_zhiling == 1) {

            //serialPort2.open("COM2", 115200);

            // int bytesWritten1;
            // ʵ��վ
            //if (globalPn == 2255 && globalSn == 10)
            //{
            //    //serialPort.write(writeBufferjinggao2, 16, bytesWritten21);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao2), "COM1 Send jinagao2", 0);
            //    serialPort2.write(writeBufferjinggao2, 16, bytesWritten12);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao2), "COM2 Send jinagao2", 0);
            //}
            //else if (globalPn == 2254 && globalSn == 25)
            //{
            //    //serialPort.write(writeBufferjinggao, 16, bytesWritten21);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM1 Send jinagao1", 0);
            //    serialPort2.write(writeBufferjinggao, 16, bytesWritten12);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM2 Send jinagao1", 0);
            //}
            //char writeBuffer[] = "$HXXB,WAR,1*CK\r\n";
            ////����ʵ��վ˫����
            //if (globalPn == 2255 && globalSn == 10)
            //{
            //    serialPort.write(writeBufferjinggao, 16, bytesWritten21);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao2), "COM1 Send jinagao2", 0);
            //    //serialPort2.write(writeBufferjinggao2, 16, bytesWritten12);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao2, strlen(writeBufferjinggao2), "COM2 Send jinagao2", 0);
            //}
            //else if (globalPn == 2254 && globalSn == 25)
            //{
            //    serialPort.write(writeBufferjinggao, 16, bytesWritten21);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM1 Send jinagao1", 0);
            //    //serialPort2.write(writeBufferjinggao, 16, bytesWritten12);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao), "COM2 Send jinagao1", 0);
            //}
            //����ʵ��վ
            serialPort.write(writeBufferjinggao, 16, bytesWritten21);
            saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao2), "COM1 Send jinagao", 0);
            //serialPort2.write(writeBufferjinggao2, 16, bytesWritten12);
            saveData("D:/ceshi/Seriallog.txt", writeBufferjinggao, strlen(writeBufferjinggao2), "COM2 Send jinagao", 0);

            //memcpy(frame3, frame1, sizeof(frame1));

            flag_zhiling = 0;


        }
        if (flag_target == 0 && flag_zhiling == 0) {
            //SerialPort serialPort;
            //serialPort22.open("COM2", 115200);
            //int bytesWritten1;
           // char writeBuffer[] = "$HXXB,OFF,1*CK\r\n";
            //ʵ��վ
            //if (globalPn == 2255 && globalSn == 10)
            //{
            //    //serialPort.write(writeBufferxiaoshi2, 16, bytesWritten21);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM1 Send xiaoshi2", 0);
            //    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send xiaoshi2", 0);
            //}
            //else if (globalPn == 2254 && globalSn == 25)
            //{
            //    //serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM1 Send xiaoshi1", 0);
            //    //serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send xiaoshi1", 0);
            //}

            //����ʵ��վ
            //if (globalPn == 2255 && globalSn == 10)
            //{
            //    serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM1 Send xiaoshi2", 0);
            //    //serialPort2.write(writeBufferxiaoshi2, 16, bytesWritten12);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi2, strlen(writeBufferxiaoshi2), "COM2 Send xiaoshi2", 0);
            //}
            //else if (globalPn == 2254 && globalSn == 25)
            //{
            //    serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM1 Send xiaoshi1", 0);
            //    //serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
            //    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send xiaoshi1", 0);
            //}
            serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
            saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM1 Send xiaoshi", 0);
            //serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
            saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send xiaoshi", 0);

            biaozhi = 0;
            flag_zhiling = 1;

        }

    }
goto CONSUMER_START;
}

void SonarApp::saveImageWithTimestamp_beijing(const cv::Mat& image)
{
    // ��ȡ��ǰʱ��
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // ��ʽ��ʱ���
    std::ostringstream oss;
    oss << "D:/mubiao/beijing_"
        << (now_tm->tm_year + 1900) << "-"
        << (now_tm->tm_mon + 1) << "-"
        << now_tm->tm_mday << "_"
        << now_tm->tm_hour << "-"
        << now_tm->tm_min << "-"
        << now_tm->tm_sec << ".png";

    std::string filename = oss.str();

    // ����ͼ��
    bool result2 = cv::imwrite(filename, image);
    if (result2) {
        std::cout << "ͼ���ѱ��浽�ļ�: " << filename << std::endl;
    }
    else {
        std::cerr << "����ͼ��ʧ��!" << std::endl;
    }
}

// �ݹ���¶�����̬�ֲ�����
void SonarApp::recursive_update(model_param& params, double x, double learning_rate)
{
    // ���㵱ǰ�۲�ֵ�Ķ���
    double y = log(x);

    // ����֡������
    params.t += 1;

    // ��̬����ѧϰ��
    // double learning_rate = 0.5; // ѧϰ�������������С

    // ���¾�ֵ
    double delta = y - params.mu;
    params.mu += learning_rate * delta / params.t;

    // ���·���
    double delta2 = (y - params.mu) * delta; // ʹ�� delta �����ظ�����
    params.sigma2 += learning_rate * (delta2 - params.sigma2) / params.t;
}

// ����Ƿ�����쳣ֵ
bool SonarApp::detect_anomaly(const model_param& params, double current_value)
{
    double alpha = 0.001; // ������ˮƽ

    // �ӷ�������׼��
    double sigma = std::sqrt(params.sigma2);

    // ����������
    double z = inverse_normal_cdf(1 - alpha / 2); // ������ˮƽ��Ӧ��zֵ
    double lower_limit = std::exp(params.mu - z * sigma);
    double upper_limit = std::exp(params.mu + z * sigma);

    // �����ļ�����飬Ӧ�����ж������Ƿ��ޣ��������������ֻ��Ҫ�ж��Ƿ�����޴�ͺ���
    // bool flag = current_value < lower_limit || current_value > upper_limit;
    bool flag = current_value > upper_limit;

#if USE_IMSHOW == 1
    if (flag == 1)
        printf("[DEBUG]: mu: %f, sigma: %f, lower_limit: %f, upper_limit: %f, current_value: %f\n", std::exp(params.mu), params.sigma2, lower_limit, upper_limit, current_value);
#endif
    return flag;
}

int SonarApp::convert_angle2index(int angle)//
{
    // ӳ�䵱ǰ�Ƕ�; 270~360 -> 0~50; 0~90 -> 51~100
    int temp_index = 0;

    // ż�����г������Ƶ�
    if (angle > 9000 && angle < 9600)
        angle = 9600;

    if (angle > 3200 && angle < 4000)
        angle = 3200;

    // �Ƕ�ӳ��
    if (angle >= 9600 && angle <= 12800)
        temp_index = (angle - 9600) / 64;
    else
        temp_index = angle / 64 + 50;
    
    return temp_index;
}

// ����ʵ�ֱ�׼��̬�ֲ������ۻ��ֲ�������Phi^-1��
double SonarApp::inverse_normal_cdf(double p)
{
    // Abramowitz and Stegun's approximation for the inverse normal CDF
    if (p <= 0.0 || p >= 1.0) {
        throw std::invalid_argument("p must be in the range (0, 1)");
    }

    const double a1 = -3.969683028665376e+01;
    const double a2 = 2.209460984245205e+02;
    const double a3 = -2.759285104469687e+02;
    const double a4 = 1.383577518672690e+02;
    const double a5 = -3.066479806614716e+01;
    const double a6 = 2.506628277459239e+00;

    const double b1 = -5.447609879822406e+01;
    const double b2 = 1.615858368580409e+02;
    const double b3 = -1.556989798598866e+02;
    const double b4 = 6.680131188771972e+01;
    const double b5 = -1.328068155288572e+01;

    const double c1 = -7.784894002430293e-03;
    const double c2 = -3.223964580411365e-01;
    const double c3 = -2.400758277161838e+00;
    const double c4 = -2.549732539343734e+00;
    const double c5 = 4.374664141464968e+00;
    const double c6 = 2.938163982698783e+00;

    const double d1 = 7.784695709041462e-03;
    const double d2 = 3.224671290700398e-01;
    const double d3 = 2.445134137142996e+00;
    const double d4 = 3.754408661907416e+00;

    const double p_low = 0.02425;
    const double p_high = 1.0 - p_low;

    double q, r, result;

    if (p < p_low) {
        // Rational approximation for lower region
        q = std::sqrt(-2 * std::log(p));
        result = (((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
            ((((d1 * q + d2) * q + d3) * q + d4) * q + 1);
    }
    else if (p <= p_high) {
        // Rational approximation for central region
        q = p - 0.5;
        r = q * q;
        result = (((((a1 * r + a2) * r + a3) * r + a4) * r + a5) * r + a6) * q /
            (((((b1 * r + b2) * r + b3) * r + b4) * r + b5) * r + 1);
    }
    else {
        // Rational approximation for upper region
        q = std::sqrt(-2 * std::log(1 - p));
        result = -(((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
            ((((d1 * q + d2) * q + d3) * q + d4) * q + 1);
    }

    return result;
}

void SonarApp::write_model_param(std::string file_path)
{
    std::ofstream out(file_path, std::ios::out); // ���ļ���
    for (auto& i : model_array) {
        for (auto& j : i) {
            out << j.mu << " " << j.sigma2 << "\n";
        }
    }
    out.close();
    return;
}
void SonarApp::saveImageWithTimestamp_mubiao(const cv::Mat& image) {
    // ��ȡ��ǰʱ��
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // ��ʽ��ʱ���
    std::ostringstream oss;
    oss << "D:/mubiao/target_"
        << (now_tm->tm_year + 1900) << "-"
        << (now_tm->tm_mon + 1) << "-"
        << now_tm->tm_mday << "_"
        << now_tm->tm_hour << "-"
        << now_tm->tm_min << "-"
        << now_tm->tm_sec << ".png";

    std::string filename = oss.str();

    // ����ͼ��
    bool result2 = cv::imwrite(filename, image);
    if (result2) {
        std::cout << "ͼ���ѱ��浽�ļ�: " << filename << std::endl;
    }
    else {
        std::cerr << "����ͼ��ʧ��!" << std::endl;
    }
}

// ���������������̬�ֲ��ĸ����ܶ�ֵ
double SonarApp::lognormalPDF(double x, double mu, double variance)
{
    if (x <= 0) {
        // ������̬�ֲ�������Ϊ x > 0
        return 0.0;
    }
    // ���ݷ�������׼��
    double sigma = std::sqrt(variance);

    // ������̬�ֲ��ĸ����ܶȺ�����ʽ
    double exponent = -(std::pow(std::log(x) - mu, 2)) / (2 * std::pow(sigma, 2));
    double denominator = x * sigma * std::sqrt(2 * 3.141592653); // 2 * pi
    return std::exp(exponent) / denominator;
}