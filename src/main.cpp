#include "serial.hpp"
#include <csignal>
#include <atomic>
#include <unistd.h>
#include <stdio.h>
#include <thread>


atomic<bool> got_sigint(false);

void hander(int s)
{
    if (s == SIGINT)
        got_sigint.store(true);
    else
        got_sigint.store(false);
}


/// @brief 同步处理规划运动轨迹  读->解析->规划->写
/// @param serial_ 串口类对象
/// @param planningData 规划数据缓存空间
/// @param dataLenger 规划数据缓存空间大小
/// @param planningDataSize 规划数据大小
void matlabV6(void * serial_, double *planningData, int dataLenger, int *planningDataSize)
{
    SERIAL * serialMatlabV6 = (SERIAL *) serial_;
    string matlabV6{};
    float Pf[6] = {0, 0 , 700};
    serialMatlabV6->readSome(&matlabV6, 192);
    cout << "读 ： " << matlabV6.data() << endl;
    float sensor_lenger[6]{};

    sscanf(matlabV6.c_str(), "x%fy%fz%fp1%fp2%fp3%fp4%fp5%fp6%fm", &Pf[3], &Pf[4], &Pf[5], 
            &sensor_lenger[0], &sensor_lenger[1], &sensor_lenger[2], &sensor_lenger[3], &sensor_lenger[4], &sensor_lenger[5]);

    double sensor_lenger_[6]{}, Pf_[6]{};
        
    for (size_t i = 0; i < 6; i++)
    {
        sensor_lenger_[i] = sensor_lenger[i];
        Pf_[i] = Pf[i]; 
    }

    cout << "Pf: [ ";
    for (size_t i_ = 0; i_ < 6; i_++)
    {
        cout << Pf_[i_] << " ";
    }
    cout << " ]\nsensor_lenger: [ ";
    for (size_t i = 0; i < 6; i++)
    {
        cout << sensor_lenger_[i] << " ";
    }
    cout << " ]\n";
        
        
    //读-->数据处理-->计算-->写   测试暂用同步处理，效果不理想改异步处理，可能改善效果
    double vec[6]{};
    stewart_control_function_V4_part_test_1(planningData, dataLenger, planningDataSize, sensor_lenger_, Pf_, vec);
    serialMatlabV6->witeSome(vec);
        
    matlabV6.clear();
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

/********************************同步******************************************** */
    int dataLenger = 65536;
    double planningData[dataLenger]{};
    int planningDataSize[4]{};
    // while (!got_sigint)
    // {
    //     matlabV6((void *)&serial_, planningData, dataLenger, planningDataSize);
    // }
/******************************************************************************** */

/*------------------------------------异步--------------------------------------------*/
    serial_.setPlanningStorage(planningData, dataLenger, planningDataSize);
    std::thread serialAsyncRun([&](){serial_.serialAsyncRunSerive();});
    serialAsyncRun.detach();

    char q = '0';

    do
    {
        q = getchar();
        /* code */
    } while ('q' != q);

    if (serialAsyncRun.joinable())
        serialAsyncRun.join();

    serial_.Close();
/*-----------------------------------------------------------------------------------*/


    return 0;
}
