#ifndef _SERIAL_CLASS_
#define _SERIAL_CLASS_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <iostream>
#include <jsoncpp/json/json.h>
#include <fstream>
#include <termios.h>
#include "stewart_control_function_V4_part_test_1.h"

using namespace std;
using namespace boost::asio;

struct matlabV6Planning
{
    int dataLenger;
    double *data;
    int *dataSize;
};


class SERIAL
{
private:
    io_service io;
    serial_port serial;
    bool is_open_serial, is_asyncRuning;
    string portName{}, matlabV6Async{}, witeBuff{};
    int serial_fd;

    bool is_updata;

    int asyncBuffLenger{};
    vector<char> readBuff;
    matlabV6Planning planning_;

public:
    /// @brief 构造函数，串口参数设置
    /// @param port 端口号“/dev/ttyUSB0”
    SERIAL(const string &portName);


    /// @brief 析构，关闭串口
    ~SERIAL();

    /// @brief 串口参数设置
    /// @param baudRate 波特率 115200
    /// @param character_size 数据位 8
    /// @param Calibration 校验位 0：none，1：odd，2：even
    /// @param stop 停止位 0：onepointfive, 1：one， 2：two
    /// @return 
    void set_serail_option(int baudRate, int character_size, int Calibration, int stop);

    /// @brief 打开串口
    /// @return true：成功；false：失败
    bool openSerial();

    /// @brief 同步读取串口数据
    /// @param outData 输出读取到的数据
    /// @param cache_ 读取数据长度 默认1024
    void readSome(string *outData, size_t cache_);


    /// @brief 同步写入串口数据
    /// @param witeBuff 写入的数据
    void witeSome(double *witeBuff);


    /// @brief 监听io流
    void asyncRun();

    /// @brief 暂停监听io流
    void asyncStop();

    /// @brief 关闭串口
    void Close();


    void handleRead(const boost::system::error_code &error_, size_t byte_read);

    void startAsyncRead();

    void serialAsyncRunSerive();
    void setAsyncBuffLenger(int lenger_);

    void setPlanningStorage(double *planningData, int dataLenger, int *dataSize);

    void handleWite(const boost::system::error_code &error_);

    void AsyncWite(string witBuff);

    void speedPlanning();
};

/// @brief 读取参数文件，加载串口配置
/// @param root 输出参数配置
void getParam(Json::Value *root);

#endif
