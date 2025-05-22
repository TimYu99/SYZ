//------------------------------------------ Includes ----------------------------------------------

#include "sonarApp.h"
#include "maths/maths.h"
#include <cmath>
#include "platform/debug.h"
#include "files/bmpFile.h"
#include "utils/utils.h"
#include <iostream> // 包含头文件
#include <ctime> // For std::time_t, std::time, std::ctime
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <direct.h> 
#include <filesystem> // 使用 std::filesystem 来代替 _mkdir
#include "platform/windows/serialPort.h" // 确保包含了正确的头文件
#include <vector>
#include <string>
#include "devices/device.h"
#include "global.h" // 包含 global.h
#include <thread> // 包含线程库
#include <windows.h> // 包含 Windows API 头文件
#include <mutex>
#include <opencv2/opencv.hpp>
#include "data_logger.h"
#include "array_data.h"

// #include "sonarDataStore.h"
// #include "sonarImage.h"
// #include "palette.h"

using namespace IslSdk;
//后加
bool isRecording = false;
std::string dataFolder_;  // 文件夹路径
using namespace IslSdk;
unsigned char mesbag[28] = { 0 };
std::mutex uartMutex;
// 90°扇形最边角角度定义/还有1处在764行附近修改
//const float START_ANGLE = 315; // 扇形开始边角
//const float END_ANGLE = 45.0;    // 扇形结束边角
//const float ANGLE_TOLERANCE = 0.5; // 容差范围 ±1度
//const int ROWS = 160; // 行数
//const int COLS = 101;  // 列数

cv::Mat frame11Display;
cv::Mat frame22Display;
int jishucount = 0;
////180°扇形最边角角度定义
const float START_ANGLE = 270; // 扇形开始边角
const float END_ANGLE = 90;    // 扇形结束边角
const float ANGLE_TOLERANCE = 0.5; // 容差范围 ±1度
const int ROWS = 160; // 行数
const int COLS = 201;  // 列数

float shanxing[ROWS][COLS] = { 0 }; // 初始化为 0
float frame1[ROWS][COLS] = { 0 }; // 初始化为 0
float frame2[ROWS][COLS] = { 0 }; // 初始化为 0
float frame3[ROWS][COLS] = { 0 }; // 初始化为 0
float frame4[ROWS][COLS] = { 0 }; // 初始化为 0
float* currentFrame = &frame1[0][0]; // 指向 frame1 的首地址
float* nextFrame = &frame2[0][0];   // 指向 frame2 的首地址

bool isCollecting = false; // 数据采集标志
// 定义全局变量
Message  msg;
extern SeriallPort serialPort; // 声明全局变量
extern SeriallPort serialPort2; // 声明全局变量
int bytesWritten12;
int bytesWritten21;
int bytesWritten111;
int bytesWritten222;
////实验站
//char writeBufferimage[] = "Image detection ready!\r\n";
//char writeBufferjinggao[] = "$HXXB,WAR,1*CK\r\n";
//char writeBufferxiaoshi[] = "$HXXB,OFF,1*CK\r\n";
char writeBufferjinggao2[] = "$HXXB,WAR,2*CK\r\n";
char writeBufferxiaoshi2[] = "$HXXB,OFF,2*CK\r\n";
//char writeBufferjingzhi[] = "It may be a stationary target\r\n";

//考古实验站
char writeBufferimage[] = "Image detection ready!\r\n";
char writeBufferjinggao[] = "$HXXB,WAR,1*CK\r\n";
char writeBufferxiaoshi[] = "$HXXB,OFF,1*CK\r\n";
char writeBufferjingzhi[] = "It may be a stationary target\r\n";

int first_mubao = 0;
int sec_mubao = 0;
int chazhi_mubiao = 0;
int biaozhi = 0;

// 当前创建的 sonar app 编号
static int sonar_app_index = 3;
// 定义在 device.cpp 中
// 每一个声呐的工作状态；1：正常工作；2：断线重连中；
extern std::map < std::string, int > sonar_status;
// 声呐对应的端口名字:第一个是声呐id，第二个是端口名字
extern std::map < std::string, std::string > sonar_port_name;
// 每一个声呐的断线重连次数
extern std::map < std::string, int > sonar_reconnect_count;
// 是否有一个声呐正在工作
extern bool sonar_working_flag;

//void convert_uint_32_array_to_opencv_mat(const uint32_t* image, uint_t width, uint_t height);

std::string IslSdk::messageToString(const Message& msg)
{
    std::ostringstream oss;

    // 将header转换为字符串
    oss.write(msg.header, sizeof(msg.header));

    // 将其他字段按顺序添加到字符串流中
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
        std::cout << "无法打开串口!" << std::endl;
        return false;
    }

    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hComm, &dcbSerialParams)) {
        std::cout << "获取串口状态失败!" << std::endl;
        close();
        return false;
    }

    dcbSerialParams.BaudRate = baudRate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hComm, &dcbSerialParams)) {
        std::cout << "设置串口状态失败!" << std::endl;
        close();
        return false;
    }

    timeouts.ReadIntervalTimeout = 5;
    timeouts.ReadTotalTimeoutConstant = 5;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant = 5;
    timeouts.WriteTotalTimeoutMultiplier = 1;
    if (!SetCommTimeouts(hComm, &timeouts)) {
        std::cout << "设置串口超时失败!" << std::endl;
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

        // 声明构造函数
        RangeImageBeam();
    };

    // 定义构造函数
    RangeImageBeam::RangeImageBeam() : angle_du(0.0f), angle_stepSize(0.0f), minRange(0.0f), max_range(0.0f), data_count(0) 
    {
        // 在这里进行其他的初始化操作
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

//------------------------------------------ 后加结束 ----------
//--------------------------------------------------------------------------------------------------
SonarApp::SonarApp(void) : App("SonarApp"), m_pingCount(0), m_scanning(false), sequenceNumber_(1)
{
    setDataFolder("D:\\ceshi");
    header = "$SMSNXX";
    length = 28;  // 计算消息长度，排除结束符
    initializeTime();
    status = 0x0F;  // 示例状态
    angle = 90;     // 示例角度
    speed = 2;      // 示例速度
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
    

    if (sonar_status["2254.0025"] == 1)//1第一处
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
    //自动运行
    std::thread([this]() {
        
        while (true) {
            
             
            //std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待1秒，确保设备已初始化
     
            jishucount++;
                if (jishucount == 5)
                {
                    //std::cout << "creat";
                    this->doTask('r', dataFolder_); // 开始扫描
                    
                }

             

            
            //std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待1秒，确保设备已初始化
            //std::this_thread::sleep_for(std::chrono::minutes(4)); // 等待4分钟
            //this->doTask('R', dataFolder_); // 停止扫描
            //std::this_thread::sleep_for(std::chrono::minutes(1)); // 等待4分钟
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
                //// 在记录开始时生成时间戳
                //auto now = std::chrono::system_clock::now();
                //std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
                //// 在开始记录时生成一个新的文件名
                //std::stringstream filenameStream;
                //filenameStream << dataFolder_ << "/ping_data_" << std::to_string(timestamp) << ".txt";
                //std::string filename = filenameStream.str();
                //// 打开文件并写入时间戳和序号信息
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
                // 在记录结束时生成时间戳
                auto now = std::chrono::system_clock::now();
                std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
                // 在结束记录时生成一个新的文件名
                std::stringstream filenameStream;
                filenameStream << dataFolder_ << "/ping_data_" << std::to_string(timestamp) << ".txt";
                std::string filename = filenameStream.str();
                // 打开文件并写入时间戳和序号信息
                std::ofstream outputFile(filename, std::ios::app);  // 使用追加模式打开文件
                if (outputFile.is_open()) {
                    // 在结束记录时生成序号信息
                    outputFile << "Recording Ended:" << std::endl;
                    outputFile << "Timestamp: " << std::ctime(&timestamp);
                    outputFile << "Sequence Number: " << sequenceNumber_ << std::endl;
                    outputFile.close();
                    std::cout << "Recording ended. Data written to " << filename << std::endl;
                }
                else {
                    std::cerr << "Unable to open the file for writing." << std::endl;
                }
                // 递增序号
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
void SonarApp::callbackSettingsUpdated(Sonar& sonar, bool_t ok, Sonar::Settings::Type settingsType)//系统新加
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
    std::cerr << "数据回传" << std::endl;
    m_piss360 = &iss360;
    if (isRecording) {
        static int pingDataCount = 0; // 静态变量用于保持计数器的值
        //Debug::log(Debug::Severity::Info, name.c_str(), "Ping data");
        Debug::log(Debug::Severity::Info, name.c_str(), "Ping data, Count: %d", ++pingDataCount); // 显示计数器的值
        uint_t txPulseLengthMm = static_cast<uint_t>(iss360.settings.system.speedOfSound * iss360.settings.acoustic.txPulseWidthUs * 0.001 * 0.5);
        txPulseLengthMm = Math::max<uint_t>(txPulseLengthMm, 150);

        // 获取锁
        std::unique_lock<std::mutex> lock(pingQueueMutex);
        if (pingQueue.size() < MAX_QUEUE_SIZE) {
            pingQueue.push(ping); // 将数据添加到队列中
            consumerCondition.notify_one(); // 通知消费者线程
        }
        else {
            std::cout << "Ping queue is full. Dropping ping data!!!!!!" << std::endl;
            pingQueue.pop(); // 如果队列已满，丢弃最旧的数据
            pingQueue.push(ping); // 添加新的数据
            consumerCondition.notify_one(); // 通知消费者线程
        }
        std::cout << "Producer thread add data!" << std::endl; // 调试输出
        lock.unlock(); // 解锁互斥量

        sonarDataStore.add(ping, txPulseLengthMm);
        //recordPingData(iss360, ping, txPulseLengthMm);
        m_pingCount++;

        // 测试下发参数设定指令
        // if (pingDataCount == 10) {
        //     std::cout << "----------------------------------ready set parameter----------------------------------" << std::endl;
        //     Sonar& sonar = reinterpret_cast<Sonar&>(*m_device);
        //     Sonar::Setup temp_setup;
        //     temp_setup.stepSize = 16;
        //     sonar.setSetupSettings(temp_setup, false);
        //     std::cout << "----------------------------------ready set parameter----------------------------------" << std::endl;
        // }

        // 尝试直接解码原始扫描数据为图像，不过失败了
        // m_circular.render(sonarDataStore, m_palette, true);
        // convert_uint_32_array_to_opencv_mat(reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), COLS, ROWS);

        // 实时扫描显示图像
        // this->doTask('i', dataFolder_); // 开始扫描
        // cv::Mat temp_img = cv::imread("E:/program/SYZ/build/bin/Debug/sonar.bmp", cv::IMREAD_UNCHANGED);
        // cv::imshow("sonar", temp_img);
        // cv::waitKey(1); // 等待1毫秒以更新窗口

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
    std::cerr << "运行到recordPingData" << std::endl;
    //先判断是否采集shanxing数据
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
    // 计算当前角度
    float current_angle = float(ping.angle) * 360 / 12800;
    // 判断是否接近起始边角 (315°/270°)
    if (!isCollecting && std::abs(current_angle - START_ANGLE) <= ANGLE_TOLERANCE)
    {
        std::cout << "Starting data collection at angle: " << current_angle << " degrees." << std::endl;
        isCollecting = true;

    }

    //////  判断当前角度是否在合法范围 (315° ~ 45°)
    // if (current_angle < 315.0 && current_angle>45)
    //{
    //     //isValidAngle = false;
    //     isCollecting = false;
    //     std::memset(shanxing, 0, sizeof(shanxing));
    //     currentCol = 0; // 或者重置 currentCol = 0; 来覆盖数据
    //}
      /*判断当前角度是否在合法范围 (270° ~ 90°)*/
    if (current_angle < 270.0 && current_angle>90)
    {
        //isValidAngle = false;
        isCollecting = false;
        std::memset(shanxing, 0, sizeof(shanxing));
        currentCol = 0; // 或者重置 currentCol = 0; 来覆盖数据
    }

    if (isCollecting)
    {
        //if (currentCol >= COLS)
        //{
        //    std::cerr << "Array full, cannot store more data." << std::endl;
        //    currentCol = 0; // 或者重置 currentCol = 0; 来覆盖数据
        //    std::memset(shanxing, 0, sizeof(shanxing));
        //    std::cerr << "Array clear all data." << std::endl;
        //}
        // 将 ping.data 存储到 shanxing 数组的当前列
        for (int i = 0; i < ping.data.size() && i < ROWS; ++i) { //这里我对int进行初始修改，旨在滤除前面i组的数据
            shanxing[i][currentCol] = static_cast<float>(ping.data[i]);
        }

        // 更新当前列索引
        currentCol++;
        if (currentCol == COLS)
        {
            //jushu_shanxing++;
            flag_shanxing = 1;
            //std::cout << "Displaying shanxing matrix:" << std::endl;

            //for (int i = 0; i < ROWS; ++i) {
            //    for (int j = 0; j < COLS; ++j) {
            //        // 设置固定宽度便于观察矩阵结构
            //        std::cout << std::setw(5) << shanxing[i][j] << " ";
            //    }
            //    std::cout << std::endl; // 换行
            //}

            //std::cout << "End of matrix display." << std::endl;
    //        // 存储到文件
    //
    //
    ////float shanxing[ROWS][COLS] = {0}; // 假设数据在这里
    //        saveShanxingToFile(&shanxing[0][0], ROWS, COLS, "shanxing_data.csv");
        }
    }
    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
    std::tm* now_tm = std::localtime(&timestamp);
    uint16_t year = now_tm->tm_year + 1900; // 获取完整的年份
    uint8_t month = now_tm->tm_mon + 1;
    uint8_t day = now_tm->tm_mday;
    uint8_t hour = now_tm->tm_hour;
    uint8_t minute = now_tm->tm_min;
    uint8_t second = now_tm->tm_sec;
    uint8_t status = 0x00; // 示例状态
    // 设置状态位：Bit0, Bit1, Bit2, Bit3
    // 使用全局变量进行判断
    //std::string deviceInfo = std::to_string(globalPn) + "." + std::to_string(globalSn);
    //if (globalPn == 2254 && globalSn == 25)
    //{
    //    status |= 0x01; // 设置 Bit0 为 1
    //}
    //if (globalPn == 2254 && globalSn == 23)
    //{
    //    status |= 0x01; // 设置 Bit0 为 1
    //}
    //if (globalPn == 2255 && globalSn == 10)
    //{
    //    status |= 0x01; // 设置 Bit0 为 1
    //}
    // 计算 angle 和 speed
    if (biaozhi == 1)
    {
        if (m_sonar_app_index == 0)
        {
            status = 0x04; //有目标
            status |= globalstatus1;//创两个
        }
        else
        {
            status = 0x04; //有目标
            status |= globalstatus2;//创两个
        }
    }
    else
    {
        if (m_sonar_app_index == 0)
        {
            status |= globalstatus1;//无目标
        }
        else
        {
            status |= globalstatus2;//无目标
       
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
    // 添加数据处理逻辑
    //processData(ping.data);
    //// 根据数据处理结果进行反馈
    //bool targetDetected = checkForTarget(temp_ping_.beam_data);
    //if (targetDetected) {
    //    status |= 0x04; // 设置 Bit2 为 1 表示有目标
    //}
    // 示例目标检测函数
    //bool SonarApp::checkForTarget(const std::vector<uint16_t>&beam_data)
    //{
    //    // 在这里添加目标检测逻辑
    //    // 返回 true 表示检测到目标，否则返回 false
    //    return std::any_of(beam_data.begin(), beam_data.end(), [](uint16_t value) {
    //        return value > 100; // 示例条件：值大于 100 表示有目标
    //        });
    //}
    globalcount = temp_ping_.data_count;
    //globalx = 1;
    uint8_t minrange = temp_ping_.minRange;
    uint8_t maxrange = temp_ping_.max_range;
    
// #define USE_C_FILE_STREAM
#ifndef USE_C_FILE_STREAM
    // 构造文件名
    char filename[256];
    snprintf(filename, sizeof(filename), "%s/all_ping_data.txt", dataFolder_.c_str());

    
    // 打开文件（以追加模式）
    FILE* outputFile = fopen(allPingDataFilename_.c_str(), "a");
    if (outputFile != NULL) {
        // 写入数据到文件
        fprintf(outputFile, "Processed %d Ping Data:\n", m_sonar_app_index);

        // 写入时间戳
        char timeBuffer[64];
        struct tm* localTime = localtime(&timestamp);
        strftime(timeBuffer, sizeof(timeBuffer), "%Y-%m-%d %H:%M:%S", localTime);

        fprintf(outputFile, "Timestamp: %s\n", timeBuffer);

        // 写入其他参数
        fprintf(outputFile, "Angle: %f\n", temp_ping_.angle_du);
        fprintf(outputFile, "Step Size: %f\n", temp_ping_.angle_stepSize);
        fprintf(outputFile, "Min Range (m): %2f\n", temp_ping_.minRange);
        fprintf(outputFile, "Max Range (m): %2f\n", temp_ping_.max_range);
        fprintf(outputFile, "Speed of Sound: %3f\n", iss360.settings.system.speedOfSound);
        fprintf(outputFile, "Tx Pulse Width (us): %f\n", iss360.settings.acoustic.txPulseWidthUs);
        fprintf(outputFile, "Tx Pulse Length (mm): %f\n", txPulseLengthMm);
        fprintf(outputFile, "Data Count: %d\n", ping.data.size());

        // 写入 Beam Data
        fprintf(outputFile, "Beam Data: ");
        int temp;
        for (int i = 0; i < temp_ping_.data_count; ++i) {
            temp = static_cast<int>(temp_ping_.beam_data[i]);
            // fprintf(outputFile, "%1f ", temp_ping_.beam_data[i]);
            fprintf(outputFile, "%d ", temp); // 使用 %d 格式化输出整数
        }
        fprintf(outputFile, "\n");

        // 关闭文件
        fclose(outputFile);
        printf("Data appended to %s\n", allPingDataFilename_.c_str());
    }
    else {
        perror("Failed to open file");
    }
#else
    std::stringstream filenameStream;
    // 生成文件名，使用时间戳作为文件名的一部分

    filenameStream << dataFolder_ << "/all_ping_data1.txt";
    std::string filename = filenameStream.str();

    // 将参数写入文件
    std::ofstream m_outputFile(allPingDataFilename_, std::ios::app);  // 注意这里使用 std::ios::app 进行追加写入
    if (m_outputFile.is_open())
    {
        // 记录数据到文件
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

    
    // 发送格式化数据
    

    sendFormattedData(year, month, day, hour, minute, second, status, angle, speed, minrange, maxrange);

    static int send_count = 0;
    send_count++;
    if (send_count % 20 == 0) {
        std::cout << "Send data count: " << send_count << std::endl;
        int temp ;
        send_count = 0;
        //std::cerr << "判断之前" << std::endl;
        if (globalstatus1 == 0x03|| globalstatus1==0x07 || globalstatus2 == 0x07 || globalstatus2 == 0x03)
        {
            send_count = 0;
             ;
        }
        else{
            //std::cerr << "进入else" << std::endl;
            if (m_sonar_app_index == 0) 
            {
               // std::cerr << "发送前1" << std::endl;
                serialPort.write(sendBuffer, 28, temp);   // 实验站时不注释，考古注释
               // memset(sendBuffer, 0, sizeof(sendBuffer));
                // saveData("D:/ceshi/output.txt", sendBuffer, 28, "COM1 Send Hex Data", 1);
                send_count = 0;
            }
            else {
                //std::cerr << "发送前2" << std::endl;
                serialPort.write(sendBuffer2, 28, bytesWritten222);   // 实验站时不注释，考古注释
                //std::cerr << "发送后" << std::endl;
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
    // 生成文件名，使用时间戳作为文件名的一部分
    auto now = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(now);

    std::stringstream filenameStream;
    filenameStream << dataFolder_ << "/all_ping_data_" << std::to_string(timestamp) << ".txt";
    allPingDataFilename_ = filenameStream.str();
    // 将当前时间写入文件
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
    // 获取并打印当前可用的串口列表
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
    // 在这里添加处理接收到的数据的逻辑
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
    // 打开文件
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // 遍历矩阵并写入文件
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            file << std::setw(10) << *(shanxing + i * cols + j); // 设置宽度对齐，便于观察
            if (j < cols - 1) {
                file << ","; // 列之间加空格分隔
            }
        }
        file << "\n"; // 每行结束换行
    }

    file.close(); // 关闭文件
    std::cout << "Shanxing data saved to " << filename << std::endl;
}

//#include <ostream>
//#include <fstream>
//std::ofstream out("mat.txt", std::ios::out | std::ios::app); // 打开文件以追加模式写入
//
//void convert_uint_32_array_to_opencv_mat(const uint32_t* image, uint_t width, uint_t height)
//{
//    // 创建一个 CV_8UC4 类型的 Mat，表示 8 位无符号整数，4 通道（RGBA）
//    cv::Mat mat(height, width, CV_8UC4);
//
//    // 遍历每个像素，将 BMP 数据拷贝到 Mat 中
//    for (uint_t row = 0; row < height; row++) {
//        for (uint_t col = 0; col < width; col++) {
//            uint32_t pixel = image[row * width + col];
//
//            // 提取 RGBA 通道
//            uint8_t r = (pixel >> 16) & 0xFF;
//            uint8_t g = (pixel >> 8) & 0xFF;
//            uint8_t b = (pixel >> 0) & 0xFF;
//            uint8_t a = 0;
//            // uint8_t a = (pixel >> 24) & 0xFF;
//
//            // 将像素值存储到 Mat 中
//            cv::Vec4b& matPixel = mat.at<cv::Vec4b>(row, col);
//            matPixel[0] = b; // OpenCV 使用 BGRA 顺序
//            matPixel[1] = g;
//            matPixel[2] = r;
//            matPixel[3] = a;
//        }
//    }
//
//    cv::Mat resizedMat;
//    cv::resize(mat, resizedMat, cv::Size(), 4.0, 4.0, cv::INTER_LINEAR);
//    cv::imshow("实时声呐扫描图放大", resizedMat);


    // out << "-----------------------------------------------------------\n";
    // for (uint_t row = 0; row < height; row++) {
    //     for (uint_t col = 0; col < width; col++) {
    //         uint32_t pixel = image[row * width + col];

    //         // 提取 RGBA 通道
    //         uint8_t r = (pixel >> 16) & 0xFF;
    //         uint8_t g = (pixel >> 8) & 0xFF;
    //         uint8_t b = (pixel >> 0) & 0xFF;
    //         uint8_t a = (pixel >> 24) & 0xFF;

    //         // 将像素值存储到 Mat 中
    //         out << "Pixel[" << row << "][" << col << "] = (" << static_cast<int>(r) << ", " << static_cast<int>(g) << ", " << static_cast<int>(b) << ", " << static_cast<int>(a) << ")" << std::endl;
    //     }
    // }

    // out << "-----------------------------------------------------------\n";

//    cv::imshow("实时声呐扫描图", mat); // 显示图像
//    cv::waitKey(1); // 等待 1 毫秒以更新窗口
//
//    return;
//}

// 是否不使用帧差法，注释掉则使用帧差，否则使用背景建模法
#define USE_NEW_ALGORITHM
// 是否加载模型，如果不加载的话，将会自动创建模型，注释掉这个宏则不加载，每次训练
// #define LOAD_MODEL
// 模型迭代将在多少轮后降低学习率
#define MAKE_MODEL_TIMES 10
// 使用假设检验还是PDF概率密度分布函数判定，注释掉这个宏就会使用假设检验方法
#define USE_PDF
// 是否启用 debug 调试的图像显示和输出，实际部署的时候请将此宏设为0
#define USE_IMSHOW 0
// 是否根据距离动态调整阈值，防止目标过远，反射强度不够高，导致被误判为背景
#define USE_DYNAMIC_THRESHOLD 0
// cv::Mat qiang_du_tu(101, 200, CV_8UC1, cv::Scalar(0));
#define USE_SYZ 1
// 是否为实验站项目使用
void SonarApp::consumePingData()
{
CONSUMER_START:
    std::cerr << "消费者线程开始运行" << std::endl;
    std::unique_lock<std::mutex> lock(pingQueueMutex);
    consumerCondition.wait(lock, [this] { return !pingQueue.empty(); });
    std::cout << "Consumer thread comsume data!  Queue size = " << pingQueue.size() << std::endl; // 调试输出
    Sonar::Ping ping = pingQueue.front(); // 获取队列前端的元素
    pingQueue.pop(); // 移除队列前端的元素
    lock.unlock(); // 解锁互斥量
    
#ifdef USE_NEW_ALGORITHM
    int temp_qiangdutu_index = convert_angle2index(ping.angle);
    for (int i = 0; i < ping.data.size(); i++) {
        int temp = (double(ping.data[i]) / 65536.0 * 255.0);
		qiang_du_tu.at<uchar>(temp_qiangdutu_index, i) = temp;
    }

    bool flag_have_goal = 0;//目标是否存在
// 图像显示 debug
#if USE_IMSHOW == 1
    cv::imshow("qiang_du_tu" + std::to_string(m_sonar_app_index), qiang_du_tu);
    cv::waitKey(1);
    // 放大图像以便观看
    cv::Mat qiang_du_tu_resized;
    cv::resize(qiang_du_tu, qiang_du_tu_resized, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST); // 放大4倍
    cv::imshow("qiang_du_tu_resized" + std::to_string(m_sonar_app_index), qiang_du_tu_resized);
    cv::waitKey(1); // 等待 1 毫秒以更新窗口
#endif
    // 如果转完一圈了，统计转圈次数的计数器++；
    if (ping.angle == 0) //
    {
        // 如果已经是检测阶段，或者缓慢学习阶段，显示图像
        if (flag_making_model == 0 || flag_making_model == 2) 
        {
#if USE_IMSHOW == 1
            cv::imshow("goal" + std::to_string(m_sonar_app_index), is_goal);
            cv::waitKey(1); // 等待 1 毫秒以更新窗口
            cv::Mat is_goal_resized;
            cv::resize(is_goal, is_goal_resized, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST); // 放大4倍
            cv::imshow("is_goal_resized" + std::to_string(m_sonar_app_index), is_goal_resized);
            cv::waitKey(1); // 等待 1 毫秒以更新窗口

            // 开运算
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 定义核
            cv::Mat result;
            cv::morphologyEx(is_goal, result, cv::MORPH_OPEN, kernel); // 开运算
            cv::imshow("After Opening" + std::to_string(m_sonar_app_index), result);
            cv::waitKey(1);

            //// 遍历图像，看还有没有白色的点，有则认为是目标
            int temp_have_goal = 0;
            //for (int i = 0; i < 101; i++)
            //    for (int j = 0; j < 200; j++) {
            //        if (is_goal.at<uchar>(i, j) == 255) 
            //        {
            //            temp_have_goal = 1;
            //            // 是目标的操作，i是角度数，j是距离值
            //            // < 50 是 270~360 >50 是 0~90
            //            // temp_angle 里面放的就是还原后的实际角度
            //            int temp_angle = (i < 50) ? ((i * 64 + 9600.00) / 35.5) : ((i - 50) * 64.0 / 35.5);//？：前面的式子决定后面的判断s
            //        }

            //    }
            std::vector<std::vector<cv::Point>> contours; // 存储轮廓
            std::vector<cv::Vec4i> hierarchy; // 存储轮廓的层次结构
            cv::findContours(result, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // 查找轮廓
            // 遍历轮廓，计算外接矩形
            for (size_t i = 0; i < contours.size(); i++)
            {
                cv::Rect boundingBox = cv::boundingRect(contours[i]); // 计算外接矩形
                // cv::rectangle(is_goal, boundingBox, cv::Scalar(255), 2); // 绘制外接矩形
                int centerX = boundingBox.x + boundingBox.width / 2; // 计算中心点的 x 坐标
                int centerY = boundingBox.y + boundingBox.height / 2; // 计算中心点的 y 坐标，即距离单位
                // 计算质心的角度
                double temp_angle = (centerY < 50) ? ((centerY * 64 + 9600.00) / 35.5) : ((centerY - 50) * 64.0 / 35.5);
                double temp_dis = 100.0 * centerX / 200.0; // 距离单位
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
                flag_zhiling = 1;//第一次发消息的标志
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

           
            // 画出最大判别强度图
            cv::Mat zui_da_qiang_du_tu(101, 200, CV_8UC1, cv::Scalar(0));
            for (int i = 0; i < 101; i++)
                for (int j = 0; j < 200; j++) {
                    double temp_sigma = std::sqrt(model_array[i][j].sigma2); // 标准差
                    double temp = std::exp(model_array[i][j].mu + 2.5 * temp_sigma);//与上述设计值保持一致
                    temp = (temp > 65535) ? 65535 : temp;
                    temp /= 65535.0;
                    temp *= 255.0;
                    int temp2 = static_cast<int>(temp);
                    zui_da_qiang_du_tu.at<uchar>(i, j) = temp2;
                }
            cv::imshow("zui_da_qiang_du_tu" + std::to_string(m_sonar_app_index), zui_da_qiang_du_tu);
            cv::waitKey(1);

            cv::Mat zui_da_qiang_du_tu4x;
            cv::resize(zui_da_qiang_du_tu, zui_da_qiang_du_tu4x, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST); // 放大4倍
            cv::imshow("zui_da_qiang_du_tu4x" + std::to_string(m_sonar_app_index), zui_da_qiang_du_tu4x);
            cv::waitKey(1); // 等待 1 毫秒以更新窗口
#else
            
            //cv::imshow("goal" + std::to_string(m_sonar_app_index), is_goal);
            //cv::waitKey(1); // 等待 1 毫秒以更新窗口
            //cv::Mat is_goal_resized;
            //cv::resize(is_goal, is_goal_resized, cv::Size(), 4.0, 4.0, cv::INTER_NEAREST); // 放大4倍
            //cv::imshow("is_goal_resized" + std::to_string(m_sonar_app_index), is_goal_resized);
            //cv::waitKey(1); // 等待 1 毫秒以更新窗口
            std::cerr << "图像处理" << std::endl;
            // 开运算
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)); // 定义核
            cv::Mat result;
            cv::morphologyEx(is_goal, result, cv::MORPH_OPEN, kernel); // 开运算
            //cv::imshow("After Opening" + std::to_string(m_sonar_app_index), result);
            //cv::waitKey(1);
            
            //// 遍历图像，看还有没有白色的点，有则认为是目标
            int temp_have_goal = 0;
            //for (int i = 0; i < 101; i++)
            //    for (int j = 0; j < 200; j++) {
            //        if (is_goal.at<uchar>(i, j) == 255) 
            //        {
            //            temp_have_goal = 1;
            //            // 是目标的操作，i是角度数，j是距离值
            //            // < 50 是 270~360 >50 是 0~90
            //            // temp_angle 里面放的就是还原后的实际角度
            //            int temp_angle = (i < 50) ? ((i * 64 + 9600.00) / 35.5) : ((i - 50) * 64.0 / 35.5);//？：前面的式子决定后面的判断s
            //        }

            //    }
            std::vector<std::vector<cv::Point>> contours; // 存储轮廓
            std::vector<cv::Vec4i> hierarchy; // 存储轮廓的层次结构
            cv::findContours(result, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // 查找轮廓


            // 遍历轮廓，计算外接矩形
            for (size_t i = 0; i < contours.size(); i++)
            {
                cv::Rect boundingBox = cv::boundingRect(contours[i]); // 计算外接矩形
                // cv::rectangle(is_goal, boundingBox, cv::Scalar(255), 2); // 绘制外接矩形
                int centerX = boundingBox.x + boundingBox.width / 2; // 计算中心点的 x 坐标
                int centerY = boundingBox.y + boundingBox.height / 2; // 计算中心点的 y 坐标，即距离单位
                // 计算质心的角度
                double temp_angle = (centerY < 50) ? ((centerY * 64 + 9600.00) / 35.5) : ((centerY - 50) * 64.0 / 35.5);
                double temp_dis = 100.0 * centerX / 200.0; // 距离单位
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
                flag_zhiling = 1;//第一次发消息的标志
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
        // 保存每一轮的模型参数
        //if (circle_times <= MAKE_MODEL_TIMES + 1)
            //write_model_param("model_" + std::to_string(circle_times) + ".txt");
#endif

        // 对齐零点
        if (flag_making_model == -1)
            flag_making_model = 1; // 如果是第一次转圈，开始建模
    }

    // 如果转了20圈，认为模型已经收敛，取消建模，进入预测状态
    if (circle_times == MAKE_MODEL_TIMES) {
        flag_making_model = 2; // 进入缓慢学习阶段
    }

    // 如果当前正在建模
    if (flag_making_model == 1 || flag_making_model == 2) {
#ifdef LOAD_MODEL
        std::fstream in("model_11.txt", std::ios::in); // 打开文件以读取
        if (!in) 
        {
            std::cerr << "Error opening file for reading." << std::endl;
            return;
        }
        // 读取模型参数
        for (auto& i : model_array) {
            for (auto& j : i) {
                in >> j.mu >> j.sigma2;
            }
        }
        in.close(); // 关闭文件
        flag_making_model = 0;
#else
        // 如果是第一次转圈，直接取其值作为均值，方差赋值为0.1
        if (circle_times == 1)
        {
            int temp_index = convert_angle2index(ping.angle);//这里将数据放进temp
            for (int i = 0; i < ping.data.size(); i++) 
            {
                model_array[temp_index][i].mu = std::log(ping.data[i]);
                model_array[temp_index][i].sigma2 = 0.1;
                model_array[temp_index][i].t = 1;
            }
        }
        else
        { // 否则，迭代建模
            // 并且，处于步长为 1.8 的状态下，这个是避免声呐莫名其妙扫描值变成 0.9 的情况
            if (ping.stepSize == 64) {
                #if USE_IMSHOW == 1
                std::cout << "angle = " << ping.angle << "\n";
                #endif
                int temp_index = convert_angle2index(ping.angle);
                #if USE_IMSHOW == 1
                std::cout << "temp_index = " << temp_index << "\n";
                #endif

                // 如果当前没有目标，那么遍历当前角度下的每一个距离，进行迭代更新，有目标就不学习了
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

    // 如果是缓慢建图阶段或者检测阶段，进行目标检测
    if (flag_making_model == 0 || flag_making_model == 2) {
        if (ping.stepSize == 64) {
            int temp_index = convert_angle2index(ping.angle);

            int temp_flag = 0;

            for (int i = 10; i < ping.data.size(); i++) {
                temp_flag = 0;
#ifndef USE_PDF
                temp_flag = detect_anomaly(model_array[temp_index][i], ping.data[i]);
#else
                // 比原来小，不管它
                if (std::log(ping.data[i]) < model_array[temp_index][i].mu)
                    continue;

                double temp_p = lognormalPDF(ping.data[i], model_array[temp_index][i].mu, model_array[temp_index][i].sigma2);
                double temp_sigma = std::sqrt(model_array[temp_index][i].sigma2);
#if USE_DYNAMIC_THRESHOLD == 1
                double max_value = std::exp(model_array[temp_index][i].mu + (5.5 + (-2 / 190.0) * i)* temp_sigma);
#else
                double max_value = std::exp(model_array[temp_index][i].mu + 2* temp_sigma);//这里更改sigma阈值 x* temp_sigma
#endif
                max_value < 0 ? max_value = 0 : 65536;
                max_value > 65535 ? max_value = 65535 : max_value;
                double p_min = lognormalPDF(max_value, model_array[temp_index][i].mu, model_array[temp_index][i].sigma2);

                if (temp_p < p_min) {
                    temp_flag = 1;
                    printf("[DEBUG]:value: %d, upper: %f, mu: %f, f(x): %f, f(mu+sigma): %f\n", ping.data[i], max_value, std::exp(model_array[temp_index][i].mu), temp_p, p_min);
                }
#endif
                is_goal.at<uchar>(temp_index, i) = (temp_flag == 0) ? 0 : 255; // 0表示无目标，1表示有目标
                //if (temp_flag == 1)
                //    printf("[GOAL]: value = %d, mu = %f, sigma2 = %f\n", (int) ping.data[i], model_array[temp_index][i].mu, model_array[temp_index][i].sigma2);
            }
        }
        else
            std::cout << "[ERROR]: Step size is not 1.8, please check!" << std::endl;
    }
    
    flag_shanxing = 0;
#endif

    // 处理 pingData
    uint_t txPulseLengthMm = static_cast<uint_t>((*m_piss360).settings.system.speedOfSound * (*m_piss360).settings.acoustic.txPulseWidthUs * 0.001 * 0.5);
    txPulseLengthMm = Math::max<uint_t>(txPulseLengthMm, 150);
    recordPingData(*m_piss360, ping, txPulseLengthMm);


    if (flag_shanxing == 1) 
    {
        jishu_shanxing = jishu_shanxing + 1;
        if (jishu_shanxing == 1) {
            memcpy(frame1, shanxing, sizeof(shanxing));
            cv::Mat frame11 = cv::Mat(ROWS, COLS, CV_32FC1, frame1).clone(); // 克隆保证数据独立
            //cv::Mat frame11Display;
            cv::normalize(frame11, frame11Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            // 缩放比例
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
        // 将二维数组转换为 cv::Mat

        cv::Mat frame22 = cv::Mat(ROWS, COLS, CV_32FC1, frame2).clone();

        // 归一化到 0~255 方便显示
        //cv::Mat frame22Display;
        //cv::normalize(frame11, frame11Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        cv::normalize(frame22, frame22Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        // 缩放比例
        double scale = 2.0;
        // 检查图像是否为空
        if (frame22Display.empty()) {
            std::cerr << "Error: frame22Display is empty!" << std::endl;
            return;
        }

        // 放大图像
        cv::Mat  frame22Resized;
        // cv::resize(frame11Display, frame11Resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
        cv::resize(frame22Display, frame22Resized, cv::Size(), scale, scale, cv::INTER_NEAREST);// 显示图像
        // cv::imshow("Frame11", frame11Display);
       // std::cout << "frame22 data: " << frame22Display << std::endl;
        bool result = cv::imwrite("D:/ceshi/frame22_image.png", frame22Resized);
        if (result) {
            std::cout << "图像已保存到文件夹!" << std::endl;
        }
        else {
            std::cerr << "保存图像2失败!" << std::endl;
        }



        // 
        //图像处理
        if (jishu_shanxing > 1) 
        {
            int no_target = 0;  // 初始化无目标标志
            int no_target1 = 0;  // 初始化无目标标志
            Centroid centroid;  // 初始化质心结构体

            if (jishu_shanxing == 2 || jishu_shanxing == 3 || jishu_shanxing == 4) {
                serialPort.write(writeBufferimage, 24, bytesWritten21);//进入声纳判别
                saveData("D:/ceshi/Seriallog.txt", writeBufferimage, strlen(writeBufferimage), "COM1 Send", 0);
            }
            // 调用 process_frame_difference 函数
            int result = process_frame_difference(frame11Display, frame22Display, no_target, centroid);//数据处理
            flag_target = no_target;


            if (flag_zhiling == 0) {



                jishu_guding++;
                if (jishu_guding > 15) {
                    //实验站
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
                    ////考古实验站双声纳
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
                    //考古实验站单声纳
                    serialPort.write(writeBufferxiaoshi, 16, bytesWritten21);
                    // serialPort2.write(writeBufferxiaoshi, 16, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferxiaoshi, strlen(writeBufferxiaoshi), "COM2 Send jingzhi", 0);
                    //serialPort.write(writeBufferjingzhi, 31, bytesWritten21);
                   // serialPort2.write(writeBufferjingzhi, 31, bytesWritten12);
                    saveData("D:/ceshi/Seriallog.txt", writeBufferjingzhi, strlen(writeBufferjingzhi), "COM1 Send jingzhi", 0);


                    memcpy(frame1, frame2, sizeof(frame1));
                    cv::Mat frame11 = cv::Mat(ROWS, COLS, CV_32FC1, frame1).clone(); // 克隆保证数据独立
                    //cv::Mat frame11Display;
                    cv::normalize(frame11, frame11Display, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                    // 缩放比例
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
            // 实验站
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
            ////考古实验站双声纳
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
            //考古实验站
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
            //实验站
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

            //考古实验站
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
    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // 格式化时间戳
    std::ostringstream oss;
    oss << "D:/mubiao/beijing_"
        << (now_tm->tm_year + 1900) << "-"
        << (now_tm->tm_mon + 1) << "-"
        << now_tm->tm_mday << "_"
        << now_tm->tm_hour << "-"
        << now_tm->tm_min << "-"
        << now_tm->tm_sec << ".png";

    std::string filename = oss.str();

    // 保存图像
    bool result2 = cv::imwrite(filename, image);
    if (result2) {
        std::cout << "图像已保存到文件: " << filename << std::endl;
    }
    else {
        std::cerr << "保存图像失败!" << std::endl;
    }
}

// 递归更新对数正态分布参数
void SonarApp::recursive_update(model_param& params, double x, double learning_rate)
{
    // 计算当前观测值的对数
    double y = log(x);

    // 增加帧计数器
    params.t += 1;

    // 动态调整学习率
    // double learning_rate = 0.5; // 学习率随迭代次数减小

    // 更新均值
    double delta = y - params.mu;
    params.mu += learning_rate * delta / params.t;

    // 更新方差
    double delta2 = (y - params.mu) * delta; // 使用 delta 避免重复计算
    params.sigma2 += learning_rate * (delta2 - params.sigma2) / params.t;
}

// 检测是否存在异常值
bool SonarApp::detect_anomaly(const model_param& params, double current_value)
{
    double alpha = 0.001; // 显著性水平

    // 从方差计算标准差
    double sigma = std::sqrt(params.sigma2);

    // 计算上下限
    double z = inverse_normal_cdf(1 - alpha / 2); // 显著性水平对应的z值
    double lower_limit = std::exp(params.mu - z * sigma);
    double upper_limit = std::exp(params.mu + z * sigma);

    // 正常的假设检验，应该是判断两边是否超限，但是这个场景下只需要判断是否比上限大就好了
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
    // 映射当前角度; 270~360 -> 0~50; 0~90 -> 51~100
    int temp_index = 0;

    // 偶尔会有超出限制的
    if (angle > 9000 && angle < 9600)
        angle = 9600;

    if (angle > 3200 && angle < 4000)
        angle = 3200;

    // 角度映射
    if (angle >= 9600 && angle <= 12800)
        temp_index = (angle - 9600) / 64;
    else
        temp_index = angle / 64 + 50;
    
    return temp_index;
}

// 近似实现标准正态分布的逆累积分布函数（Phi^-1）
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
    std::ofstream out(file_path, std::ios::out); // 打开文件以
    for (auto& i : model_array) {
        for (auto& j : i) {
            out << j.mu << " " << j.sigma2 << "\n";
        }
    }
    out.close();
    return;
}
void SonarApp::saveImageWithTimestamp_mubiao(const cv::Mat& image) {
    // 获取当前时间
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // 格式化时间戳
    std::ostringstream oss;
    oss << "D:/mubiao/target_"
        << (now_tm->tm_year + 1900) << "-"
        << (now_tm->tm_mon + 1) << "-"
        << now_tm->tm_mday << "_"
        << now_tm->tm_hour << "-"
        << now_tm->tm_min << "-"
        << now_tm->tm_sec << ".png";

    std::string filename = oss.str();

    // 保存图像
    bool result2 = cv::imwrite(filename, image);
    if (result2) {
        std::cout << "图像已保存到文件: " << filename << std::endl;
    }
    else {
        std::cerr << "保存图像失败!" << std::endl;
    }
}

// 函数：计算对数正态分布的概率密度值
double SonarApp::lognormalPDF(double x, double mu, double variance)
{
    if (x <= 0) {
        // 对数正态分布定义域为 x > 0
        return 0.0;
    }
    // 根据方差计算标准差
    double sigma = std::sqrt(variance);

    // 对数正态分布的概率密度函数公式
    double exponent = -(std::pow(std::log(x) - mu, 2)) / (2 * std::pow(sigma, 2));
    double denominator = x * sigma * std::sqrt(2 * 3.141592653); // 2 * pi
    return std::exp(exponent) / denominator;
}