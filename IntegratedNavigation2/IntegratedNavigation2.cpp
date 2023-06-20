// IntegratedNavigation2.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include"ImrFileLoader.h"
#include"LooseCombination.h"

int main()
{
    //初始化载体位置速度姿态(单位：[m] [m/s] [deg])
    //BLH Blh(30.5278108404, 114.3557126448, 22.312);//开阔
    BLH Blh(30.5282960229, 114.3557510106, 23.141);//遮蔽
    Vector Vel(3, 0.0);
    ANGLE Angle;

    ImrFileLoader imrfileloader("D:\\专业实习2\\3_2cover_environment.imr");/* static.imr  3_2cover_environment.imr  3_2open_environment.imr*/
    //初对准（并使用粗对准结束时刻作为解算起始时刻）
    double STARTTIME = imrfileloader.InitialAlignment(300, &Angle, &Blh);
    NavState inistate(Blh, Vel, Angle);
    cout << '\n' << "Calculating..." << "\n";

    //初始化载体位置速度姿态的误差(单位：[m] [m/s] [deg])
    BLH Blh_std(0.009, 0.008, -0.022);
    Vector Vel_std(3, '|', 0.000, 0.000, 0.000);
    ANGLE Angle_std(0.05, 0.05, 0.05);
    NavState inistate_std(Blh_std, Vel_std, Angle_std);

    //GPS天线杆臂
    Vector al(3, '|', -0.1000, 0.2350, -0.8900);
    //里程计杆臂
    Vector ol(3, '|', -0.05, -0.842, 0.5);

    //惯导传感器噪声参数
    Vector arw(3, 0.2);//输入参数单位为[deg/sqrt(h)]
    Vector vrw(3, 0.4);//输入参数单位为[m/s/sqrt(h)]
    Vector gb_std(3, 24.0);//输入参数单位为[deg/h]
    Vector ab_std(3, 400.0);//输入参数单位为[mGal]
    Vector gs_std(3, 1000.0);//输入参数单位为[ppm]
    Vector as_std(3, 1000.0);//输入参数单位为[ppm]

    IMUNoise ImuNoise(arw, vrw, gb_std, ab_std, gs_std, as_std, 3600.0);

    //构造运载体
    Body body(inistate, inistate_std, ImuNoise, al, ol);

    //GNSS数据加载器
    FileLoader Gnssfileloader("D:\\专业实习2\\cover.pos", 14, TEXT);/*cover.pos wide.pos*/
    fstream fout("D:\\专业实习2\\遮蔽环境解算结果.txt",ios::out);
    vector<double> OneLineImu(7, 0.0);//一个历元的IMU数据
    vector<double> OneLineGNSS(14, 0.0);//一个历元的GNSS数据

    //GNSS跳过起始时刻之前的数据
    Gnssfileloader.Skip_n_Line(2);
    while (!Gnssfileloader.isEof())
    {
        Gnssfileloader.load(OneLineGNSS);
        if (OneLineGNSS[1] < STARTTIME)
            continue;
        else
            break;
    }

    body.SetTimestamp(STARTTIME+1.0);

    while (!imrfileloader.isEof()&&!Gnssfileloader.isEof())
    {
        imrfileloader.load(OneLineImu);

        //跳过起始时刻之前的数据
        if (OneLineImu[0] < STARTTIME+1.0)
        {
            body.AddIMUData(OneLineImu.data());
            continue;
        }

        //添加IMU数据
        body.AddIMUData(OneLineImu.data());
        //body.PureINSwork();

        //进行松组合算法传播误差状态和测量更新工作
        body.work();
        //body.BodyShow();
        body.BodyOutputToFile(fout);

        //若GNSS更新后读取一组新GNSS数据
        if (body.IsGNSSDataAvailable() == false)
        {
            Gnssfileloader.load(OneLineGNSS);

            //简单调整GNSS数据格式，以便使用添加函数直接将数据加入载体(删去第一列GPS周，将东北天换为北东地)
            OneLineGNSS.erase(OneLineGNSS.begin());
            double t = OneLineGNSS[7];
            OneLineGNSS[7] = OneLineGNSS[8];
            OneLineGNSS[8] = t;
            OneLineGNSS[9] = -OneLineGNSS[9];

            body.AddGNSSData(OneLineGNSS.data());

            OneLineGNSS.resize(14);
        }


    }
    cout << '\n' << "Successfully!" << '\n';
    cout << "End Time:  " << OneLineImu[0] << "s";

}

