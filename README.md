ROMIOL-SCNT Project

该程序对各层进行了抽象：设备层(Devices)-部件层(Components)-任务层(Tasks)

硬件的通讯在设备层(Devices)完成，高级功能在任务层(Tasks)完成；这两部分也是该程序的关键之处。
部件层(Components)起到中间抽象、连接的作用，使得两部分的代码可以单独工作。
部件在界面初始化时创建，界面关闭时销毁；因此任务层只使用硬件资源，不进行销毁操作。

串口类的内容：
打开串口
发送指令
接受信息
信号：打开串口失败、连接开始

平台RJ45通信：
设置IPV4地址为192.168.0.2

固高电机：
1. test.cfg 加载路径: :/
QFile file(":/config/test.cfg");
file.copy("googol.cfg");
2. 每次open, reset...


设备类：
1. 包括串口设备（调焦、MP285、气压传感器）、网口设备（Trio电机驱动、Googol电机驱动、相机），
其中串口设备继承了同一个串口类，网口设备则各自通讯方式不同，Trio, Googol都需要引单独的头文件，Googol还需要dll文件(gts.dll 添加到PATH, gts.lib)以及配置文件(test.cfg/googol.cfg)
连接设备
（处理返回信息）
更新状态，通知 UI

设备的方向：以opencv图片的XY正方向定义设备的正方向，靠近显微镜的方向为Z轴正方向

注：
1. 为了防止命名空间之间的冲突，头文件中不要用 using namespace std/cv; 可以放到各个 cpp 中；
2. Clips: GetFocus -> ClipsGetFocus; TokenType -> ClipsTokenType