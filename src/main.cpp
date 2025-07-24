#include "serial.hpp"
#include <csignal>
#include <atomic>
#include "stewart_control_function_V4_part_test_1.h"


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
        double matlabV6[9];
        serial_.readSome(matlabV6);
        cout << "读 ： " ;
        for (size_t i = 0; i < 9; i++)
            cout << matlabV6[i] << " ";
        cout << endl;

        Pf[3] = matlabV6[0];
        Pf[4] = matlabV6[1];
        Pf[5] = matlabV6[2];

        double sensor_lenger[6]{};
        size_t j = 3;
        for (size_t i = 0; i < 6; i++)
        {
            sensor_lenger[i] = matlabV6[j];
            j++;
        }
        
        //读-->数据处理-->计算-->写   测试暂用同步处理，效果不理想改异步处理，可能改善效果
        double vec[6]{};
        stewart_control_function_V4_part_test_1(sensor_lenger, Pf, vec);
        serial_.witeSome(vec, 6);

        memset(matlabV6, 0, 9);
    }
    return 0;
}
