一、总体原则

- 一致性优先：同一项目内风格统一高于个人偏好
- 可读性至上：命名清晰、结构清楚、注释完整
- 安全优先：不牺牲内存与线程安全
- 性能感知：在不影响可读性的前提下关注性能
- 模块化：便于测试、复用与维护（仅在不影响逻辑前提下做最小化重组）

二、命名规范

- 文件命名
    - 头文件：snake_case.h（例：lidar_odometry.h）
    - 源文件：与头文件同名.cpp（例：lidar_odometry.cpp）
    - 应用程序：功能描述_app.cpp（例：mapping_3d_app.cpp）
    - 工具程序：功能描述.cpp（例：pcl2smap.cpp）
- 类型命名（类、结构体、类型别名、枚举、类型模板参数）
    - PascalCase，禁止下划线（例：LidarOdometry, VoxelMap）
- 函数与方法
    - PascalCase，禁止下划线（例：ProcessPointCloud, GetCurrentPose, SetConfiguration）
    - 取值/赋值函数与变量同名的动词化形式（GetX/SetX）
- 命名空间
    - 全小写，单词以下划线分隔（例：mapping_backend）
- 变量（包括参数、局部变量、数据成员）
    - snake_case（例：current_pose, voxel_map）
    - 类私有成员以单个下划线结尾（例：voxel_map_）
- 宏
    - 全部大写，单词以下划线分隔（例：MAX_BUFFER_SIZE）

三、头文件与包含

- 头文件保护
    - 推荐使用 #pragma once
    - 或采用 include guard：文件名大写+_H_，并在endif处保留注释（例：LIDAR_ODOMETRY_H_）
- 头文件包含顺序（自上而下）
    1) 对应的头文件（仅在.cpp中）
    2) 系统C头文件（例：<cmath>）
    3) 系统C++头文件（例：<vector> <memory> <thread> <mutex>）
    4) 第三方库头文件（例：<pcl/...> <yaml-cpp/yaml.h> <Eigen/Dense>）
    5) Protobuf生成的头文件
    6) 项目公共头文件（例：common_type.h, utils.h）
    7) 本模块内部头文件（例：voxel_map.h, imu_process.h）
- 前置声明
    - 优先使用前置声明以减少编译依赖（例：pcl::PointCloud 模板前置，YAML::Node，项目内类）
    - 仅在需要完整类型时再包含头文件
    - 与之配合可使用指针/智能指针成员（如unique_ptr）

四、注释与文档

- 文件头注释
    - 包含：@file, @brief, @author, @date, @version，以及文件职责的简要说明
- 类注释
    - 包含：@class, @brief，主要功能点（列表），@note（如线程安全性），@see（相关类）
- 函数注释
    - 包含：@brief，@param（注明in/out），@return，@throw（异常说明），@note，@warning（前置条件/副作用）
    - 参数名与签名保持一致；删除过时或错误注释
- 内联注释
    - 使用 //，且独立一行，禁止写在代码行尾
    - 合理使用 TODO 与 FIXME 标记后续改进点与已知问题

