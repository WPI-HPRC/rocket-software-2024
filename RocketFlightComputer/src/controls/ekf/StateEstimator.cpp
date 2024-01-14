#include "StateEstimator.h"

/**
 * @brief Construct a new Quat State Estimator:: Quat State Estimator object
 * 
 * @param initialOrientation 
 * @param dt 
 */
QuatStateEstimator::QuatStateEstimator(BLA::Matrix<4> initialOrientation, float dt) {
    this->x = initialOrientation;
    this->x_min = initialOrientation;
    this->dt = dt;
};

/**
 * @brief Run every loop of the state machine to perform the predict and update step of the EKF
 * 
 * @param sensorPacket Sensor Frame
 * @return BLA::Matrix<4> State Vector
 */
BLA::Matrix<4> QuatStateEstimator::onLoop(Utility::SensorPacket dataPacket) {

    /* Read Data from Sensors and Convert to SI Units */
    // Convert Accel values to m/s/s
    dataPacket.accelX *= 9.81;
    dataPacket.accelY *= 9.81;
    dataPacket.accelZ *= 9.81;
    // Convert gyro values from deg/s to rad/s
    dataPacket.gyroX *= (PI / 180);
    dataPacket.gyroY *= (PI / 180);
    dataPacket.gyroY *= (PI / 180);

    // Convert Gauss to microTesla
    // sensorPacket.mag_x = sensorPacket.mag_x * 100000;
    // sensorPacket.mag_y = sensorPacket.mag_y * 100000;
    // sensorPacket.mag_z = sensorPacket.mag_z * 100000;

    /* PREDICTION STEP 
    x_min = f(x,dt)
    A = df(x,dt) / dx
    P_min = A*P*A' + sigma_gyr^2*W*W'
    */

    // Apply measurement function to predict priori state of the system
    x_min = measurementFunction(dataPacket);

    // Take the jacobian to obtain the covariance of the prediction step
    BLA::Matrix<4,4> A = measurementJacobian(dataPacket);

    // Update model covariance from previous state
    BLA::Matrix<4,3> W = updateModelCovariance(dataPacket);

    // Apply updated model covariance to process noise covariance matrix
    Q = W * gyroVar * BLA::MatrixTranspose<BLA::Matrix<4,3>>(W);

    // Update Priori Error Covariance
    P_min = A*P*BLA::MatrixTranspose<BLA::Matrix<4,4>>(A) + Q;

    // Apply Soft and Hard Iron Calibration to magnetometer data
    // **IMPORTANT** THIS MAY NEED TO BE RE-CALIBRATED UPON POSITION CHANGE
    BLA::Matrix<3> magVector = {
        dataPacket.magX, dataPacket.magY, dataPacket.magZ
    };
    
    // Serial.println(String(magVector(0))+","+String(magVector(1))+","+String(magVector(2)));

    // magVector = softIronCal * (magVector - hardIronCal);

    BLA::Matrix<3> accelVector = {dataPacket.accelX,dataPacket.accelY,dataPacket.accelZ};
    // Normalize Accel and Mag for use in correction step
    magVector = magVector / BLA::Norm(magVector);
    accelVector = accelVector / BLA::Norm(accelVector);

    // Calculate update function with magnetometer readings to correct orientation
    BLA::Matrix<6> z = {
        accelVector(0), accelVector(1), accelVector(2), magVector(0), magVector(1), magVector(2)
    };

    BLA::Matrix<6> h = updateFunction(dataPacket);

    // Take the jacobian to obtain the covariance of the correction function
    BLA::Matrix<6,4> H = updateJacobian(dataPacket);

    // Compute the kalman gain from the magnetometer covariance readings
    BLA::Matrix<6> v = z - h;
    BLA::Matrix<6,6> S = H * P_min * BLA::MatrixTranspose<BLA::Matrix<6,4>>(H) + R;
    BLA::Matrix<4,6>  K = P_min * BLA::MatrixTranspose<BLA::Matrix<6,4>>(H) * BLA::Inverse(S);

    // Use our kalman gain and magnetometer readings to correct priori orientation
    x = x_min + K*v;
    // x = x_min;

    // Update error covariance matrix
    P = (eye4 - K*H)*P_min;
    // P = P_min;

    // Serial.println("<----- Prediction Covariance ----->");
    // for (int i = 0; i < A.Rows; i++) {
    //     for (int j = 0; j < A.Cols; j++) {
    //         Serial.print(String(A(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Serial.println("<----- State ----->");
    // for (int i = 0; i < x.Rows; i++) {
    //     for (int j = 0; j < x.Cols; j++) {
    //         Serial.print(String(x(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    x = x / BLA::Norm(x);

    // Serial.print(sensorPacket.ac_x); Serial.print(",");
    // Serial.print(sensorPacket.ac_y); Serial.print(",");
    // Serial.print(sensorPacket.ac_z); Serial.print(",");
    // Serial.print(sensorPacket.gy_x); Serial.print(",");
    // Serial.print(sensorPacket.gy_y); Serial.print(",");
    // Serial.print(sensorPacket.gy_z); Serial.print(",");
    // Serial.print(sensorPacket.mag_x); Serial.print(",");
    // Serial.print(sensorPacket.mag_y); Serial.print(",");
    // Serial.print(sensorPacket.mag_z); Serial.print(",");
    // Serial.print(x(0)); Serial.print(",");
    // Serial.print(x(1)); Serial.print(",");
    // Serial.print(x(2)); Serial.print(",");
    // Serial.print(x(3)); Serial.print(",");
    // Serial.println(millis());

    Serial.print("QUAT|");
    Serial.print(x(0)); Serial.print(",");
    Serial.print(x(1)); Serial.print(",");
    Serial.print(x(2)); Serial.print(",");
    Serial.println(x(3));
    // Serial.print("MAGV|");
    // Serial.print(magVector(0)); Serial.print(",");
    // Serial.print(magVector(1)); Serial.print(",");
    // Serial.println(magVector(2));
    // Serial.print("ACC|");
    // Serial.print(sensorPacket.ac_x); Serial.print(",");
    // Serial.print(sensorPacket.ac_y); Serial.print(",");
    // Serial.println(sensorPacket.ac_z);

    return this->x;
}

BLA::Matrix<4> QuatStateEstimator::measurementFunction(Utility::SensorPacket dataPacket) {
    float p = dataPacket.gyroX; float q = dataPacket.gyroY; float r = dataPacket.gyroZ;
    // BLA::Matrix<3> bodyAccel = {sensorPacket.ac_x, sensorPacket.ac_y, sensorPacket.ac_z};

    BLA::Matrix<4> f_q = {
        x(0) - (dt/2)*p*x(1) - (dt/2)*q*x(2) - (dt/2)*r*x(3),
        x(1) + (dt/2)*p*x(0) - (dt/2)*q*x(3) + (dt/2)*r*x(2),
        x(2) + (dt/2)*p*x(3) + (dt/2)*q*x(0) - (dt/2)*r*x(1),
        x(3) - (dt/2)*p*x(2) + (dt/2)*q*x(1) + (dt/2)*r*x(0)
    };

    // Serial.println("<----- State ----->");
    // for (int i = 0; i < f_q.Rows; i++) {
    //     for (int j = 0; j < f_q.Cols; j++) {
    //         Serial.print(String(f_q(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // BLA::Matrix<3> grav_T = {0, 0,-9.81};

    // // BLA::Matrix<4> quatConj = {x(0), -x(1), -x(2), -x(3)};

    // // BLA::Matrix<4> bodyGrav = quaternionMultiplication(quaternionMultiplication(x, gravNEDQuat), quatConj);

    // BLA::Matrix<3> accel_R = {dataPacket.accelX, dataPacket.accelY, dataPacket.accelZ};

    // BLA::Matrix<3,3> rot = quat2rotm(x);

    // BLA::Matrix<3> g_a = BLA::MatrixTranspose<BLA::Matrix<3,3>>(rot) * grav_T;

    // BLA::Matrix<3> accel_a = accel_R - g_a;

    // Serial.println("<----- Accel in Body ----->");
    // for (int i = 0; i < accel_a.Rows; i++) {
    //     for (int j = 0; j < accel_a.Cols; j++) {
    //         Serial.print(String(accel_a(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Normalize Quaternion Function
    f_q = f_q / BLA::Norm(f_q);

    BLA::Matrix<4> f = f_q;

    return f;
};

BLA::Matrix<4,4> QuatStateEstimator::measurementJacobian(Utility::SensorPacket dataPacket) {
    float p = dataPacket.gyroX; float q = dataPacket.gyroY; float r = dataPacket.gyroZ;

    BLA::Matrix<4,4> A = {
        1, -(dt/2)*p, -(dt/2)*q, -(dt/2)*r,
        (dt/2)*p, 1, (dt/2)*r, -(dt/2)*q,
        (dt/2)*q, -(dt/2)*r, 1, (dt/2)*p,
        (dt/2)*r, (dt/2)*q, -(dt/2)*p, 1
    };

    return A;
}

BLA::Matrix<6> QuatStateEstimator::updateFunction(Utility::SensorPacket dataPacket) {
    BLA::Matrix<3> r = {
        cos(inclination), 0, sin(inclination)
    };

    // Must normalize vector for corrections step
    r = r / BLA::Norm(r);

    BLA::Matrix<3> G = {
        0, 0, -9.81
    }; // NED Gravity Vector
    
    // Must normalize vector for correction step
    G = G / BLA::Norm(G);

    // Normalize quaternion prior to calculation
    BLA::Matrix<4> q = {x_min(0), x_min(1), x_min(2), x_min(3)};
    q = q / BLA::Norm(q);

    BLA::Matrix<3,3> quatRotm = quat2rotm(q);

    BLA::Matrix<3> a_hat = BLA::MatrixTranspose<BLA::Matrix<3,3>>(quatRotm) * G;
    BLA::Matrix<3> m_hat = BLA::MatrixTranspose<BLA::Matrix<3,3>>(quatRotm) * r;

    BLA::Matrix<6> h = {
        a_hat(0),
        a_hat(1),
        a_hat(2),
        m_hat(0),
        m_hat(1),
        m_hat(2)
    };

    return h;

};

BLA::Matrix<6,4> QuatStateEstimator::updateJacobian(Utility::SensorPacket dataPacket) {
    BLA::Matrix<3> r = {
        cos(inclination), 0, sin(inclination)
    }; // NED Mag vector

    // Must normalize vector for corrections step
    r = r / BLA::Norm(r);

    BLA::Matrix<3> G = {
        0, 0, -9.81
    }; // NED Gravity Vector
    
    // Must normalize vector for correction step
    G = G / BLA::Norm(G);

    // Normalize quaternion prior to calculation
    BLA::Matrix<4> q = {x_min(0), x_min(1), x_min(2), x_min(3)};
    q = q / BLA::Norm(q);

    float rx = r(0);
    float ry = r(1);
    float rz = r(2);
    float gx = G(0);
    float gy = G(1);
    float gz = G(2);

    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);

    BLA::Matrix<6,4> H = {
        gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz, -gx*qy+gy*qx-gz*qw, -gx*qz+gy*qw+gz*qx,
        -gx*qz+gy*qw+gz*qx, gx*qy-gy*qx+gz*qw, gx*qx+gy*qy+gz*qz, -gx*qw-gy*qz+gz*qy, 
        gx*qy-gy*qx+gz*qw, gx*qz-gy*qw-gz*qx, gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz,
        rx*qw+ry*qz-rz*qy, rx*qy+ry*qy+rz*qz, -rx*qy+ry*qx-rz*qw, -rx*qz+ry*qw+rz*qx,
        -rx*qz+ry*qw+rz*qx, rx*qy-ry*qx+rz*qw, rx*qx+ry*qy+rz*qz, -rx*qw-ry*qz+rz*qy, 
        rx*qy-ry*qx+rz*qw, rx*qz-ry*qw-rz*qx, rx*qw+ry*qz-rz*qy, rx*qx+ry*qy+rz*qz
    };

    return H * 2.0f;

};

BLA::Matrix<4,3> QuatStateEstimator::updateModelCovariance(Utility::SensorPacket dataPacket) {

    BLA::Matrix<4,3> W = {
        -x(1), -x(2), -x(3),
        x(0), -x(3), x(2),
        x(3), x(0), -x(1),
        -x(2), x(1), x(0)
    };

    return W * (dt/2);
};

BLA::Matrix<3,3> QuatStateEstimator::quat2rotm(BLA::Matrix<4> q) {
    // q = q / BLA::Norm(q);
    
    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);

    BLA::Matrix<3,3> rotm = {
        pow(qw,2)+pow(qx,2)-pow(qy,2)-pow(qz,2), 2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy),
        2*(qx*qy+qw*qz), pow(qw,2)-pow(qx,2)+pow(qy,2)-pow(qz,2), 2*(qy*qz-qw*qz),
        2*(qx*qz-qw*qy), 2*(qw*qx+qy*qz), pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2)
    };

    return rotm;
};

BLA::Matrix<4> QuatStateEstimator::quaternionMultiplication(BLA::Matrix<4> q1, BLA::Matrix<4> q2) {

    float w1 = q1(0);
    float i1 = q1(1);
    float j1 = q1(2);
    float k1 = q1(3);

    float w2 = q2(0);
    float i2 = q2(1);
    float j2 = q2(2);
    float k2 = q2(3);

    BLA::Matrix<4> res;

    res(0) = w1*w2 - i1*i2 - j1*j2 - k1*k2;
    res(1) = w1*i2 + i1*w2 + j1*k2 - k1*j2;
    res(2) = w1*j2 - i1*k2 + j1*w2 + k1*i2;
    res(3) = w1*k2 + i1*j2 - j1*i2 + k1*w2;

    return res;

};