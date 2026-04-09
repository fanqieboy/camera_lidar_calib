###  新任务

用户要求分析`@thirdparty/camera_model/src/camera_models/EquidistantCamera.cc`中的`initUndistortRectifyMap`函数，并提出如何根据已有的真实内参和去畸变图像恢复原始畸变图像的问题。我们正在制定解决方案。

已完成：
- [x] 读取并分析 EquidistantCamera.cc 中的 initUndistortRectifyMap 函数
- [ ] 理解从去畸变图像恢复到原始畸变图像的原理
- [ ] 分析当前函数中的坐标变换步骤
- [ ] 设计逆向映射的算法
- [ ] 编写反向转换的代码实现
- [ ] 测试方案的可行性
