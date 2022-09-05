## 我修改了哪些内容？
仅仅修改了XYZ的轴朝向使其可以适配任何轴对齐的外挂磁力计方案SlimeVR[可参考这个PCB](https://github.com/tianrui233/SlimeVR-PCB-KitKat)。本分支还做了磁力计混合方案串口输出的汉化

# 适用于ESP的SlimeVR固件

适配ESP8266/ESP32微控制器和不同惯性传感器的固件，可将它们用作VR中的类似vive的跟踪器。

需要[SlimeVR Server](https://github.com/SlimeVR/SlimeVR-Server)与SteamVR一起工作并解析姿势。 应该兼容[owoTrack](https://github.com/abb128/owo-track-driver)，但不能保证其可靠性。

## 配置

固件配置位于`defines.h`文件中。有关如何配置固件的更多信息，请参阅[配置 SlimeVR 文档的固件项目部分](https://docs.slimevr.dev/firmware/configuring-project.html)。  

## 兼容性

固件支持以下惯性传感器及其对应的“IMU”值：
* BNO085 & BNO086 (IMU_BNO085)
  * 在内部DMP中使用任何融合。如果在良好的磁场环境中，使用ARVR Stabilized Game Rotation Vector或ARVR Stabilized Rotation Vector可获得最佳结果。
* BNO080 (IMU_BNO080)
  * 在内部DMP中使用任何融合。没有BNO085的ARVR稳定，但仍能提供良好的效果。
* MPU-6500 (IMU_MPU6500) & MPU-6050 (IMU_MPU6050)
  * 使用内部DMP融合陀螺仪和加速度计。会随着时间偏差越来越大，大概10分钟左右重置一次。
  * 注意：目前 MPU 会在开机时自动校准。您*必须*将它放在平面上并且*不要*移动它，直到校准完成（几秒钟）。 **校准结束后，ESP 上的 LED 将闪烁 5 次。**
* BNO055 (IMU_BNO055)
  * 尽管硬件相同，但性能比BNO080差得多。不推荐使用。
* 磁力计混合方案 (IMU_MPU9250)
  * 使用陀螺仪、磁力计和加速度计的Mahony传感器融合，需要良好的磁环境。
  * 有关校准此传感器的信息，请参阅下面的*传感器校准*，在磁场环境允许的情况下大概半小时~一小时之间才需要重置。
  * 在 `defines.h` 中指定 `IMU_MPU6500` 以在 6DoF 模式下不使用磁力计。
  * 实验支持！
* BMI160 (IMU_BMI160)
  * 使用陀螺仪和加速度计的Mahony传感器融合
  * 有关校准此传感器的信息，请参阅下面的*传感器校准*。
  * 实验支持！
* ICM-20948 (IMU_ICM20948)
  * 在内部DMP中使用fision进行6DoF或9DoF，9DoF模式需要良好的磁环境。
  * 在9DoF模式的`debug.h`中注释掉`USE_6DOF`。
  * 实验支持！

固件兼容ESP8266和ESP32。请编辑`defines.h`并根据您连接IMU的方式正确设置您的引脚。
## 传感器校准

*通常建议打开跟踪器并让它们在平坦的表面上放置几秒钟。**这将更好地校准它们。

**一些跟踪器在启动时需要特殊的校准步骤：**
* 磁力计混合方案、BMI160 [可参考哔哩哔哩的视频教程](https://www.bilibili.com/video/BV1314y1b7Fz)
  * 芯片朝下打开它们。翻转并放在表面上几秒钟，LED 将亮起。
  * 闪烁几下后LED将再次亮起
  * 以∞动作缓慢旋转跟踪器，面向不同方向持续约 30 秒，同时 LED 闪烁
  * 校准完成后LED将关闭
  * 下次关机不用再校准，校准值会保存下来以备下次使用
  
## 使用网页刷机（推荐！）
* 安装完驱动后打开[此网站](https://slimevr-firmware.bscotch.ca/)并选择此分支即可！

## 在 Linux 上刷机

按照此链接[Platformio](https://docs.platformio.org/en/latest//faq.html#platformio-udev-rules)中的说明进行操作，这应该可以解决任何权限被拒绝错误

## 贡献

通过为本项目做出贡献，您将所有代码置于 MIT 或限制较少的许可下，并且您证明您使用的代码与这些许可兼容或由您创作。如果您在工作时间这样做，您证明您的雇主对此表示同意。

有关如何贡献的说明，请参阅 [`CONTRIBUTING.md`](CONTRIBUTING.md)
