## 我修改了哪些内容？
仅仅修改了XYZ的轴朝向使其可以适配任何轴对齐的外挂磁力计方案SlimeVR，并且做了汉化部分串口输出[this PCB](https://github.com/tianrui233/SlimeVR-PCB-KitKat)

# 适用于ESP的SlimeVR固件

用于 ESP8266 / ESP32 微控制器和不同 IMU 传感器的固件，可将它们用作 VR 中的类似 vive 的跟踪器。

需要[SlimeVR Server](https://github.com/SlimeVR/SlimeVR-Server)与 SteamVR 一起工作并解析姿势。 应该兼容[owoTrack](https://github.com/abb128/owo-track-driver)，但不能保证其可靠性。

## 配置

固件配置位于 `defines.h` 文件中。 有关如何配置固件的更多信息，请参阅 [配置 SlimeVR 文档的固件项目部分](https://docs.slimevr.dev/firmware/configuring-project.html)。  

## 兼容性

固件支持以下 IMU 及其对应的“IMU”值：
* BNO085 & BNO086 (IMU_BNO085)
  * 在内部 DMP 中使用任何融合。如果在良好的磁场环境中，使用 ARVR Stabilized Game Rotation Vector 或 ARVR Stabilized Rotation Vector 可获得最佳结果。
* BNO080 (IMU_BNO080)
  * 在内部 DMP 中使用任何融合。没有 BNO085 的 ARVR 稳定，但仍能提供良好的效果。
* MPU-6500 (IMU_MPU6500) & MPU-6050 (IMU_MPU6050)
  * 使用内部 DMP 融合陀螺仪和加速度计。可以大幅度漂移。
  * 注意：目前 MPU 会在开机时自动校准。您*必须*将它放在平面上并且*不要*移动它，直到校准完成（几秒钟）。 **校准结束后，ESP 上的 LED 将闪烁 5 次。**
* BNO055 (IMU_BNO055)
  * 尽管硬件相同，但性能比 BNO080 差得多。不推荐使用。
* 磁力计混合方案 (IMU_MPU9250)
  * 使用陀螺仪、磁力计和加速度计的 Mahony 传感器融合，需要良好的磁环境。
  * 有关校准此传感器的信息，请参阅下面的*传感器校准*。
  * 在 `defines.h` 中指定 `IMU_MPU6500` 以在 6DoF 模式下不使用磁力计。
  * 实验支持！
* BMI160 (IMU_BMI160)
  * 使用陀螺仪和加速度计的 Mahony 传感器融合
  * 有关校准此传感器的信息，请参阅下面的*传感器校准*。
  * 实验支持！
* ICM-20948 (IMU_ICM20948)
  * 在内部 DMP 中使用 fision 进行 6DoF 或 9DoF，9DoF 模式需要良好的磁环境。
  * 在 9DoF 模式的 `debug.h` 中注释掉 `USE_6DOF`。
  * 实验支持！

固件兼容ESP8266和ESP32。请编辑 `defines.h` 并根据您连接IMU的方式正确设置您的引脚。
## 传感器校准

*通常建议打开跟踪器并让它们在平坦的表面上放置几秒钟。**这将更好地校准它们。

**一些跟踪器在启动时需要特殊的校准步骤：**
* 磁力计混合方案、BMI160 [可参考哔哩哔哩的视频教程](https://www.bilibili.com/video/BV1314y1b7Fz)
  * 芯片朝下打开它们。翻转并放在表面上几秒钟，LED 将亮起。
  * 闪烁几下后，LED 将再次亮起
  * 以 8 种动作缓慢旋转跟踪器，面向不同方向持续约 30 秒，同时 LED 闪烁
  * 校准完成后 LED 将关闭
  * 下次关机不用再校准，校准值会保存下来以备下次使用
  
## 使用网页刷机（推荐！）
* 安装完驱动后打开[此网站](https://slimevr-firmware.bscotch.ca/)并选择此分支即可！

## 在 Linux 上刷机

按照此链接[Platformio](https://docs.platformio.org/en/latest//faq.html#platformio-udev-rules)中的说明进行操作，这应该可以解决任何权限被拒绝错误

## 贡献

通过为本项目做出贡献，您将所有代码置于 MIT 或限制较少的许可下，并且您证明您使用的代码与这些许可兼容或由您创作。如果您在工作时间这样做，您证明您的雇主对此表示同意。

有关如何贡献的说明，请参阅 [`CONTRIBUTING.md`](CONTRIBUTING.md)
