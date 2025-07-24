#include "serial.hpp"
#include <csignal>
#include <atomic>
#include "stewart_control_function_V4_part_test_1.h"
#include <unistd.h>


atomic<bool> got_sigint(false);

void hander(int s)
{
    if (s == SIGINT)
        got_sigint.store(true);
    else
        got_sigint.store(false);
}


int main(int, char**)
{
    signal(SIGINT, hander);
    Json::Value serailParam{};
    getParam(&serailParam);
    cout << "参数: \n" << serailParam << endl;

    SERIAL serial_(serailParam["port"].asString());
    serial_.set_serail_option(serailParam["baud_rate"].asUInt(), serailParam["character_size"].asInt(), serailParam["parity"].asInt(), serailParam["stop_bits"].asInt());
    if (!serial_.openSerial())
    {
        cout << "串口打开失败！！！" << endl;
        return -1;
    }
    cout << "串口打开成功" << endl;

    double Pf[6] = {0, 0 , 700};

    while (!got_sigint)
    {
        string matlabV6{};
        serial_.readSome(&matlabV6);
        cout << "读 ： " << matlabV6.data() << endl;

        // Pf[3] = matlabV6[1];
        // Pf[4] = matlabV6[3];
        // Pf[5] = matlabV6[5];

        // double sensor_lenger[6]{};
        // size_t j = 7;
        // for (size_t i = 0; i < 6; i++)
        // {
        //     sensor_lenger[i] = matlabV6[j];
        //     j += 2;
        // }
        
        // //读-->数据处理-->计算-->写   测试暂用同步处理，效果不理想改异步处理，可能改善效果
        // double vec[6]{};
        // stewart_control_function_V4_part_test_1(sensor_lenger, Pf, vec);
        // serial_.witeSome(vec, 6);

        matlabV6.clear();
        //usleep(200 * 1000);
    }
    return 0;
}
