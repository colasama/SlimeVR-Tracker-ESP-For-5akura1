/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain, S.J. Remington & SlimeVR contributors

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "mpu9250sensor.h"
#include "network/network.h"
#include "globals.h"
#include "helper_3dmath.h"
#include <i2cscan.h>
#include "calibration.h"
#include "magneto1.4.h"
#include "GlobalVars.h"
// #include "mahony.h"
// #include "madgwick.h"
#if not (defined(_MAHONY_H_) || defined(_MADGWICK_H_))
#include "dmpmag.h"
#endif

// See AK8693 datasheet for sensitivity scales in different mode
// We use 16-bit continuous reading mode
#define MAG_LSB_TO_MG_8G .333f
//#define MAG_UT_LSB_16_BIT 0.15f

// 131 LSB/deg/s = 250 deg/s
#define TYPICAL_GYRO_SENSITIVITY 131
// 16384 LSB/G = 2G
#define TYPICAL_ACCEL_SENSITIVITY 16384.

#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
// Gyro scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_GYRO_SENSITIVITY) / 32768.) * (PI / 180.0);
// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE = ((32768. / TYPICAL_ACCEL_SENSITIVITY) / 32768.) * SENSORS_GRAVITY_EARTH;
#endif
//#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
constexpr float gscale = (250. / 32768.0) * (PI / 180.0); //gyro default 250 LSB per d/s -> rad/s
//#endif

#define MAG_CORR_RATIO 0.02

#define ACCEL_SENSITIVITY_2G 16384.0f

// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE_2G = ((32768. / ACCEL_SENSITIVITY_2G) / 32768.) * EARTH_GRAVITY;

void MPU9250Sensor::motionSetup() {
    // initialize device
    imu.initialize(addr);
    if(!imu.testConnection()) {
        m_Logger.fatal("无法在混合方案上使用该传感器（当前传感器地址为：0x%02x）我们需要兼容0x%02x的惯性传感器！", imu.getDeviceID(), addr);
        return;
    }

    m_Logger.info("已成功连接混合方案所需的惯性传感器（当前传感器地址为：0x%02x）兼容0x%02x", imu.getDeviceID(), addr);

    uint8_t magId = imu.getMagnetometerDeviceID();
    if (magId != 0xFF) {
        m_Logger.fatal("无法在混合方案上使用该传感器（当前传感器地址为：0x%02x）我们需要兼容0x%02x的地磁传感器！", magId, 0x0D);
    } else {
        m_Logger.info("已成功连接混合方案所需的地磁传感器（当前传感器地址为：0x%02x）兼容0x%02x", magId, 0x0D);
    }

    int16_t ax,ay,az;

    // turn on while flip back to calibrate. then, flip again after 5 seconds.
    // TODO: Move calibration invoke after calibrate button on slimeVR server available
    imu.getAcceleration(&ax, &ay, &az);
    float g_az = (float)az / TYPICAL_ACCEL_SENSITIVITY; // For 2G sensitivity
    if(g_az < -0.75f) {
        ledManager.on();
        m_Logger.info("翻转到正面进入校准模式");
        delay(5000);
        ledManager.off();

        imu.getAcceleration(&ax, &ay, &az);
        g_az = (float)az / TYPICAL_ACCEL_SENSITIVITY;
        if(g_az > 0.75f) {
            m_Logger.debug("开始校准...");
            startCalibration(0);
        }
    }

    // Initialize the configuration
    {
        SlimeVR::Configuration::CalibrationConfig sensorCalibration = configuration.getCalibration(sensorId);
        // If no compatible calibration data is found, the calibration data will just be zero-ed out
        switch (sensorCalibration.type) {
        case SlimeVR::Configuration::CalibrationConfigType::MPU9250:
            m_Calibration = sensorCalibration.data.mpu9250;
            break;

        case SlimeVR::Configuration::CalibrationConfigType::NONE:
            m_Logger.warn("在编号为%d的传感器上并没有找到校准数据，已跳过...", sensorId);
            m_Logger.info("请至少校准一次，否则将无法使用");
            break;

        default:
            m_Logger.warn("在编号为%d的校准数据与现在连接的传感器不兼容，跳过...", sensorId);
            m_Logger.info("建议校准");
        }
    }

#if not (defined(_MAHONY_H_) || defined(_MADGWICK_H_))
    uint8_t devStatus = imu.dmpInitialize();
    if(devStatus == 0){
        ledManager.pattern(50, 50, 5);

        // turn on the DMP, now that it's ready
        m_Logger.debug("启用DMP...");
        imu.setDMPEnabled(true);

        // TODO: Add interrupt support
        // mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        m_Logger.debug("DMP准备就绪！ 等待第一次中断...");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();
        working = true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        m_Logger.error("DMP初始化失败（错误代码：)%d）", devStatus);
    }
#else
    // NOTE: could probably combine these into less total writes, but this should work, and isn't time critical.
    imu.setAccelFIFOEnabled(true);
    imu.setXGyroFIFOEnabled(true);
    imu.setYGyroFIFOEnabled(true);
    imu.setZGyroFIFOEnabled(true);
    imu.setSlave0FIFOEnabled(true);

    // TODO: set a rate we prefer instead of getting the current rate from the device.
    deltat = 1.0 / 1000.0 * (1 + imu.getRate());
    //imu.setRate(blah);

    imu.resetFIFO();
    imu.setFIFOEnabled(true);

    working = true;
    configured = true;
#endif
}

void MPU9250Sensor::motionLoop() {
#if ENABLE_INSPECTION
    {
        int16_t rX, rY, rZ, aX, aY, aZ, mX, mY, mZ;
        imu.getRotation(&rX, &rY, &rZ);
        imu.getAcceleration(&aX, &aY, &aZ);
        imu.getMagnetometer(&mX, &mY, &mZ);

        Network::sendInspectionRawIMUData(sensorId, rX, rY, rZ, 255, aX, aY, aZ, 255, mX, mY, mZ, 255);
    }
#endif

#if not (defined(_MAHONY_H_) || defined(_MADGWICK_H_))
    // Update quaternion
    if(!dmpReady)
        return;
    Quaternion rawQuat{};
    uint8_t dmpPacket[packetSize];
    if(!imu.GetCurrentFIFOPacket(dmpPacket, packetSize)) return;
    if(imu.dmpGetQuaternion(&rawQuat, dmpPacket)) return; // FIFO CORRUPTED
    Quat quat(-rawQuat.y,rawQuat.x,rawQuat.z,rawQuat.w);
    
    getMPUScaled();

    if (Mxyz[0] == 0.0f && Mxyz[1] == 0.0f && Mxyz[2] == 0.0f) {
        return;
    }

    VectorFloat grav;
    imu.dmpGetGravity(&grav, &rawQuat);

    float Grav[] = {grav.x, grav.y, grav.z};

    if (correction.length_squared() == 0.0f) {
        correction = getCorrection(Grav, Mxyz, quat);
    } else {
        Quat newCorr = getCorrection(Grav, Mxyz, quat);

        if(!__isnanf(newCorr.w)) {
            correction = correction.slerp(newCorr, MAG_CORR_RATIO);
        }
    }

#if SEND_ACCELERATION
    {
        // dmpGetGravity returns a value that is the percentage of gravity that each axis is experiencing.
        // dmpGetLinearAccel by default compensates this to be in 4g mode because of that
        // we need to multiply by the gravity scale by two to convert to 2g mode ()
        grav.x *= 2;
        grav.y *= 2;
        grav.z *= 2;

        this->imu.dmpGetAccel(&this->rawAccel, dmpPacket);
        this->imu.dmpGetLinearAccel(&this->rawAccel, &this->rawAccel, &grav);

        // convert acceleration to m/s^2 (implicitly casts to float)
        this->acceleration[0] = this->rawAccel.x * ASCALE_2G;
        this->acceleration[1] = this->rawAccel.y * ASCALE_2G;
        this->acceleration[2] = this->rawAccel.z * ASCALE_2G;
    }
#endif

    quaternion = correction * quat;
#else

    union fifo_sample_raw buf;
    uint16_t remaining_samples;
    // TODO: would it be faster to read multiple samples at once
    while (getNextSample (&buf, &remaining_samples)) {
        parseAccelData(buf.sample.accel);
        parseGyroData(buf.sample.gyro);
        parseMagData(buf.sample.mag);

        // TODO: monitor magnetometer status
        // buf.sample.mag_status;
        // TODO: monitor interrupts
        // imu.getIntStatus();
        // TODO: monitor remaining_samples to ensure that the number is going down, not up.
        // remaining_samples

        #if defined(_MAHONY_H_)
        mahonyQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
        #elif defined(_MADGWICK_H_)
        madgwickQuaternionUpdate(q, Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], Mxyz[0], Mxyz[1], Mxyz[2], deltat * 1.0e-6);
        #endif
    }
    
    quaternion.set(-q[2], q[1], q[3], q[0]);

#endif
    quaternion *= sensorOffset;

#if ENABLE_INSPECTION
    {
        Network::sendInspectionFusedIMUData(sensorId, quaternion);
    }
#endif

    if(!lastQuatSent.equalsWithEpsilon(quaternion)) {
        newData = true;
        lastQuatSent = quaternion;
    }
}

void MPU9250Sensor::getMPUScaled()
{
    float temp[3];
    int i;

#if defined(_MAHONY_H_) || defined(_MADGWICK_H_)
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = ((float)gx - m_Calibration.G_off[0]) * GSCALE;
    Gxyz[1] = ((float)gy - m_Calibration.G_off[1]) * GSCALE;
    Gxyz[2] = ((float)gz - m_Calibration.G_off[2]) * GSCALE;

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;

    //apply offsets (bias) and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
        Axyz[0] = (m_Calibration.A_Ainv[0][0] * temp[0] + m_Calibration.A_Ainv[0][1] * temp[1] + m_Calibration.A_Ainv[0][2] * temp[2]) * ASCALE;
        Axyz[1] = (m_Calibration.A_Ainv[1][0] * temp[0] + m_Calibration.A_Ainv[1][1] * temp[1] + m_Calibration.A_Ainv[1][2] * temp[2]) * ASCALE;
        Axyz[2] = (m_Calibration.A_Ainv[2][0] * temp[0] + m_Calibration.A_Ainv[2][1] * temp[1] + m_Calibration.A_Ainv[2][2] * temp[2]) * ASCALE;
    #else
        for (i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - m-Calibration.A_B[i]);
    #endif

#else
    int16_t mx, my, mz;
    // with DMP, we just need mag data
    imu.getMagnetometer(&mx, &my, &mz);
#endif

    // Orientations of axes are set in accordance with the datasheet
    // See Section 9.1 Orientation of Axes
    // https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
    if(sensorId == 1 && SECOND_IMU_AXIS_ALIGN) {
        Mxyz[0] = (float)mx;
        Mxyz[1] = (float)my;
        Mxyz[2] = (float)mz;
    } else {
        Mxyz[0] = - (float)my;
        Mxyz[1] = - (float)mx;
        Mxyz[2] = (float)mz;
    }
    //apply offsets and scale factors from Magneto
    #if useFullCalibrationMatrix == true
        for (i = 0; i < 3; i++)
            temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
        Mxyz[0] = (m_Calibration.M_Ainv[0][0] * temp[0] + m_Calibration.M_Ainv[0][1] * temp[1] + m_Calibration.M_Ainv[0][2] * temp[2]) * MAG_LSB_TO_MG_8G * .1f;
        Mxyz[1] = (m_Calibration.M_Ainv[1][0] * temp[0] + m_Calibration.M_Ainv[1][1] * temp[1] + m_Calibration.M_Ainv[1][2] * temp[2]) * MAG_LSB_TO_MG_8G * .1f;
        Mxyz[2] = (m_Calibration.M_Ainv[2][0] * temp[0] + m_Calibration.M_Ainv[2][1] * temp[1] + m_Calibration.M_Ainv[2][2] * temp[2]) * MAG_LSB_TO_MG_8G * .1f;
    #else
        for (i = 0; i < 3; i++)
            Mxyz[i] = (Mxyz[i] - m_Calibration.M_B[i]);
    #endif
    
    uint32_t t = micros();
    if(sensorId == 1 && SECOND_IMU_AXIS_ALIGN) {
        Mxyz[0] = f_mag_y.filter(Mxyz[0], t);
        Mxyz[1] = f_mag_z.filter(Mxyz[1], t);
        Mxyz[2] = f_mag_x.filter(Mxyz[2], t);
    } else {
        Mxyz[0] = - f_mag_y.filter(Mxyz[0], t);
        Mxyz[1] = f_mag_z.filter(Mxyz[1], t);
        Mxyz[2] = - f_mag_x.filter(Mxyz[2], t);
    }
}

void MPU9250Sensor::startCalibration(int calibrationType) {
    ledManager.on();
#if not (defined(_MAHONY_H_) || defined(_MADGWICK_H_))
    // with DMP, we just need mag data
    constexpr int calibrationSamples = 300;

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("Gently rotate the device while it's gathering magnetometer data");
    ledManager.pattern(15, 300, 3000/310);
    float *calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    for (int i = 0; i < calibrationSamples; i++) {
        ledManager.on();
        int16_t mx,my,mz;
        imu.getMagnetometer(&mx, &my, &mz);
        if(sensorId == 1 && SECOND_IMU_AXIS_ALIGN) {
            calibrationDataMag[i * 3 + 0] = mx;
            calibrationDataMag[i * 3 + 1] = my;
            calibrationDataMag[i * 3 + 2] = mz;
        } else {
            calibrationDataMag[i * 3 + 0] = - my;
            calibrationDataMag[i * 3 + 1] = - mx;
            calibrationDataMag[i * 3 + 2] = mz;
        }
        Network::sendRawCalibrationData(calibrationDataMag, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        ledManager.off();
        delay(250);
    }
    m_Logger.debug("Calculating calibration data...");

    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataMag);

    m_Logger.debug("[INFO] Magnetometer calibration matrix:");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    m_Logger.debug("}");

#else

    m_Logger.debug("为追踪器校准收集原始数据...");
    constexpr int calibrationSamples = 300;
    // Reset values
    Gxyz[0] = 0;
    Gxyz[1] = 0;
    Gxyz[2] = 0;

    // Wait for sensor to calm down before calibration
    m_Logger.info("静置追踪器，等待陀螺仪自动校准");
    delay(2000);

    union fifo_sample_raw buf;
    
    imu.resetFIFO(); // fifo is sure to have filled up in the seconds of delay, don't try reading it.
    for (int i = 0; i < calibrationSamples; i++) {
        // wait for new sample
        while (!getNextSample(&buf, nullptr)) { ; }

        Gxyz[0] += float(buf.sample.gyro[0]);
        Gxyz[1] += float(buf.sample.gyro[1]);
        Gxyz[2] += float(buf.sample.gyro[2]);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;

#ifdef DEBUG_SENSOR
    m_Logger.trace("陀螺校准结果：%f %f %f", Gxyz[0], Gxyz[1], Gxyz[2]);
#endif

    Network::sendRawCalibrationData(Gxyz, CALIBRATION_TYPE_EXTERNAL_GYRO, 0);
    // TODO: use offset registers?
    m_Calibration.G_off[0] = Gxyz[0];
    m_Calibration.G_off[1] = Gxyz[1];
    m_Calibration.G_off[2] = Gxyz[2];

    // Blink calibrating led before user should rotate the sensor
    m_Logger.info("在收集加速度计和磁力计数据时画8字或各个面都旋转一圈");
    ledManager.pattern(15, 300, 3000/310);
    float *calibrationDataAcc = (float*)malloc(calibrationSamples * 3 * sizeof(float));
    float *calibrationDataMag = (float*)malloc(calibrationSamples * 3 * sizeof(float));

    // NOTE: we don't use the FIFO here on *purpose*. This makes the difference between a calibration that takes a second or three and a calibration that takes much longer.
    for (int i = 0; i < calibrationSamples; i++) {
        ledManager.on();
        int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        calibrationDataAcc[i * 3 + 0] = ax;
        calibrationDataAcc[i * 3 + 1] = ay;
        calibrationDataAcc[i * 3 + 2] = az;
       if(sensorId == 1 && SECOND_IMU_AXIS_ALIGN) {
            calibrationDataMag[i * 3 + 0] = mx;
            calibrationDataMag[i * 3 + 1] = my;
            calibrationDataMag[i * 3 + 2] = mz;
        } else {
            calibrationDataMag[i * 3 + 0] = - my;
            calibrationDataMag[i * 3 + 1] = - mx;
            calibrationDataMag[i * 3 + 2] = mz;
        }
        
        Network::sendRawCalibrationData(calibrationDataAcc + i * 3, CALIBRATION_TYPE_EXTERNAL_ACCEL, 0);
        Network::sendRawCalibrationData(calibrationDataMag + i * 3, CALIBRATION_TYPE_EXTERNAL_MAG, 0);
        ledManager.off();
        delay(250);
    }
    m_Logger.debug("正在计算校准数据...");

    float A_BAinv[4][3];
    float M_BAinv[4][3];
    CalculateCalibration(calibrationDataAcc, calibrationSamples, A_BAinv);
    free(calibrationDataAcc);
    CalculateCalibration(calibrationDataMag, calibrationSamples, M_BAinv);
    free(calibrationDataMag);
    m_Logger.debug("校准数据计算完成");
    m_Logger.debug("加速度计校准矩阵：");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++)
    {
        m_Calibration.A_B[i] = A_BAinv[0][i];
        m_Calibration.A_Ainv[0][i] = A_BAinv[1][i];
        m_Calibration.A_Ainv[1][i] = A_BAinv[2][i];
        m_Calibration.A_Ainv[2][i] = A_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", A_BAinv[0][i], A_BAinv[1][i], A_BAinv[2][i], A_BAinv[3][i]);
    }
    m_Logger.debug("}");
    m_Logger.debug("[信息] 磁力计校准矩阵：");
    m_Logger.debug("{");
    for (int i = 0; i < 3; i++) {
        m_Calibration.M_B[i] = M_BAinv[0][i];
        m_Calibration.M_Ainv[0][i] = M_BAinv[1][i];
        m_Calibration.M_Ainv[1][i] = M_BAinv[2][i];
        m_Calibration.M_Ainv[2][i] = M_BAinv[3][i];
        m_Logger.debug("  %f, %f, %f, %f", M_BAinv[0][i], M_BAinv[1][i], M_BAinv[2][i], M_BAinv[3][i]);
    }
    m_Logger.debug("}");
#endif

    m_Logger.debug("保存校准数据");

    SlimeVR::Configuration::CalibrationConfig calibration;
    calibration.type = SlimeVR::Configuration::CalibrationConfigType::MPU9250;
    calibration.data.mpu9250 = m_Calibration;
    configuration.setCalibration(sensorId, calibration);
    configuration.save();

    ledManager.off();
    Network::sendCalibrationFinished(CALIBRATION_TYPE_EXTERNAL_ALL, 0);
    m_Logger.debug("校准数据保存成功");

    m_Logger.info("已收集校准数据");
    // fifo will certainly have overflown due to magnetometer calibration, reset it.
    imu.resetFIFO();
}

void MPU9250Sensor::parseMagData(int16_t data[3]) {
    // reading *little* endian int16
    Mxyz[0] = (float)data[0];
    Mxyz[1] = (float)data[1];
    Mxyz[2] = -(float)data[2];

    float temp[3];

    //apply offsets and scale factors from Magneto
    for (unsigned i = 0; i < 3; i++) {
        temp[i] = (Mxyz[i] - m_Calibration.M_B[i]);
        #if useFullCalibrationMatrix == true
            Mxyz[i] = m_Calibration.M_Ainv[i][0] * temp[0] + m_Calibration.M_Ainv[i][1] * temp[1] + m_Calibration.M_Ainv[i][2] * temp[2];
        #else
            Mxyz[i] = temp[i];
        #endif
    }
}

void MPU9250Sensor::parseAccelData(int16_t data[3]) {
    // reading big endian int16
    Axyz[0] = (float)data[0];
    Axyz[1] = (float)data[1];
    Axyz[2] = (float)data[2];

    float temp[3];

    //apply offsets (bias) and scale factors from Magneto
    for (unsigned i = 0; i < 3; i++) {
        temp[i] = (Axyz[i] - m_Calibration.A_B[i]);
        #if useFullCalibrationMatrix == true
            Axyz[i] = m_Calibration.A_Ainv[i][0] * temp[0] + m_Calibration.A_Ainv[i][1] * temp[1] + m_Calibration.A_Ainv[i][2] * temp[2];
        #else
            Axyz[i] = temp[i];
        #endif
    }
}

// TODO: refactor so that calibration/conversion to float is only done in one place.
void MPU9250Sensor::parseGyroData(int16_t data[3]) {
    // reading big endian int16
    Gxyz[0] = ((float)data[0] - m_Calibration.G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)data[1] - m_Calibration.G_off[1]) * gscale;
    Gxyz[2] = ((float)data[2] - m_Calibration.G_off[2]) * gscale;
}

// really just an implementation detail of getNextSample...
void MPU9250Sensor::swapFifoData(union fifo_sample_raw* sample) {
    #if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
        // byteswap the big endian integers
        for (unsigned iii = 0; iii < 12; iii += 2) {
            uint8_t tmp = sample->raw[iii + 0];
            sample->raw[iii + 0] = sample->raw[iii + 1];
            sample->raw[iii + 1] = tmp;
        }
    #elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        // byteswap the little endian integers
        for (unsigned iii = 12; iii < 18; iii += 2) {
            uint8_t tmp = sample->raw[iii + 0];
            sample->raw[iii + 0] = sample->raw[iii + 1];
            sample->raw[iii + 1] = tmp;
        }
    #else
        #error "Endian isn't Little endian or big endian, are we not using GCC or is this a PDP?"
    #endif

    // compiler hint for the union, should be optimized away for optimization >= -O1 according to compiler explorer
    memmove(&sample->sample, sample->raw, sensor_data_len);
}

// thought experiments:
// is a single burst i2c transaction faster than multiple? by how much?
// how does that compare to the performance of a memcpy?
// how does that compare to the performance of data fusion?
// if we read an extra byte from the magnetometer (or otherwise did something funky)
// we could read into a properly aligned array of fifo_samples (and not require a memcpy?)
// TODO: strict aliasing might not be violated if we just read directly into a fifo_sample*.
//  which means the union approach may be overcomplicated. *shrug*
bool MPU9250Sensor::getNextSample(union fifo_sample_raw *buffer, uint16_t *remaining_count) {
    uint16_t count = imu.getFIFOCount();
    if (count < sensor_data_len) {
        // no samples to read
        remaining_count = 0;
        return false;
    }

    if (remaining_count) {
        *remaining_count = (count / sensor_data_len) - 1;
    }

    imu.getFIFOBytes(buffer->raw, sensor_data_len);
    swapFifoData(buffer);
    return true;
}