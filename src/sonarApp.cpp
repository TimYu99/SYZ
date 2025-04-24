//------------------------------------------ Includes ----------------------------------------------

#include "sonarApp.h"
#include "maths/maths.h"
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

////180°扇形最边角角度定义
const float START_ANGLE = 270; // 扇形开始边角
const float END_ANGLE = 90;    // 扇形结束边角
const float ANGLE_TOLERANCE = 0.5; // 容差范围 ±1度
const int ROWS = 200; // 行数
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
    RangeImageBeam::RangeImageBeam() : angle_du(0.0f), angle_stepSize(0.0f), minRange(0.0f), max_range(0.0f), data_count(0) {
        // 在这里进行其他的初始化操作
    }

} // namespace msis_pkg
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

    Debug::log(Debug::Severity::Notice, name.c_str(), "created" NEW_LINE
        "d -> Set settings to defualt" NEW_LINE
        "s -> Save settings to file" NEW_LINE
        "r -> Start scaning" NEW_LINE
        "R -> Stop scaning" NEW_LINE
        "p -> Save palette" NEW_LINE
        "t -> Save sonar texture" NEW_LINE
        "i -> Save sonar image" NEW_LINE
        "c -> Check head is sync'ed" NEW_LINE);
    //自动运行
    std::thread([this]() {
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待1秒，确保设备已初始化
            this->doTask('r', dataFolder_); // 开始扫描
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
            sonar.startScanning();
            if (!isRecording)
            {
                isRecording = true;
                // 在记录开始时生成时间戳
                auto now = std::chrono::system_clock::now();
                std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
                // 在开始记录时生成一个新的文件名
                std::stringstream filenameStream;
                filenameStream << dataFolder_ << "/ping_data_" << std::to_string(timestamp) << ".txt";
                std::string filename = filenameStream.str();
                // 打开文件并写入时间戳和序号信息
                std::ofstream outputFile(filename);
                if (outputFile.is_open()) {
                    outputFile << "Recording Started:" << std::endl;
                    outputFile << "Timestamp: " << std::ctime(&timestamp);
                    outputFile << "Sequence Number: 1" << std::endl;
                    outputFile.close();
                    std::cout << "Recording started. Data will be written to " << filename << std::endl;


                }
                else
                {
                    std::cerr << "Unable to open the file for writing." << std::endl;
                }
            }
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
            BmpFile::save(path + "sonar.bmp", reinterpret_cast<const uint32_t*>(&m_circular.buf[0]), 32, m_circular.width, m_circular.height);
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
void SonarApp::consumePingData()
{
    // Sonar::Ping ping;
CONSUMER_START:
    std::unique_lock<std::mutex> lock(pingQueueMutex);
    consumerCondition.wait(lock, [this] { return !pingQueue.empty(); });
    std::cout << "Consumer thread comsume data!  Queue size = " << pingQueue.size() << std::endl; // 调试输出
    Sonar::Ping ping = pingQueue.front(); // 获取队列前端的元素
    pingQueue.pop(); // 移除队列前端的元素
    lock.unlock(); // 解锁互斥量

    // 处理 pingData
    uint_t txPulseLengthMm = static_cast<uint_t>((*m_piss360).settings.system.speedOfSound * (*m_piss360).settings.acoustic.txPulseWidthUs * 0.001 * 0.5);
    txPulseLengthMm = Math::max<uint_t>(txPulseLengthMm, 150);
    recordPingData(*m_piss360, ping, txPulseLengthMm);

    if (flag_shanxing == 1)
    {
        jishu_shanxing = jishu_shanxing + 1;
        if (jishu_shanxing == 1)
        {
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

        if (jishu_shanxing > 1000)
        {
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
        if (frame22Display.empty())
        {
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
        if (result)
        {
            std::cout << "图像已保存到文件夹!" << std::endl;
        }
        else
        {
            std::cerr << "保存图像2失败!" << std::endl;
        }



        // 
        //图像处理
        if (jishu_shanxing > 1)
        {
            int no_target = 0;  // 初始化无目标标志
            int no_target1 = 0;  // 初始化无目标标志
            Centroid centroid;  // 初始化质心结构体

            if (jishu_shanxing == 2 || jishu_shanxing == 3 || jishu_shanxing == 4)
            {
                serialPort.write(writeBufferimage, 24, bytesWritten21);//进入声纳判别
                saveData("D:/ceshi/Seriallog.txt", writeBufferimage, strlen(writeBufferimage), "COM1 Send", 0);
            }
            // 调用 process_frame_difference 函数
            int result = process_frame_difference(frame11Display, frame22Display, no_target, centroid);//数据处理
            flag_target = no_target;


            if (flag_zhiling == 0)
            {



                jishu_guding++;
                if (jishu_guding > 15)
                {
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
        if (flag_target == 1 && flag_zhiling == 1)
        {

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
        if (flag_target == 0 && flag_zhiling == 0)
        {
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

//--------------------------------------------------------------------------------------------------
void SonarApp::callbackPingData(Sonar& iss360, const Sonar::Ping& ping)
{
    //Debug::log(Debug::Severity::Info, name.c_str(), "Ping data");
    m_piss360 = &iss360;
    if (isRecording)
    {
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
        // recordPingData(iss360, ping, txPulseLengthMm);
        m_pingCount++;

        if (m_pingCount % (Sonar::maxAngle / iss360.settings.setup.stepSize) == 0)
        {
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




void SonarApp::recordPingData(const Sonar& iss360, const Sonar::Ping& ping, uint_t txPulseLengthMm)
{
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
        for (int i = 0; i < ping.data.size() && i < ROWS; ++i) {
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
        status = 0x04; //有目标
        status |= globalstatus;
    }
    else
    {
        status |= globalstatus;//无目标
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

    // 发送格式化数据
    std::string portName = "COM6"; // 根据需要调整
    uint32_t baudrate = 115200; // 根据需要调整
    sendFormattedData(portName, baudrate, year, month, day, hour, minute, second, status, angle, speed, minrange, maxrange);


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
    const std::string& portName,
    uint32_t baudrate,
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
    memcpy(sendBuffer, messageStr.c_str(), 28);
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