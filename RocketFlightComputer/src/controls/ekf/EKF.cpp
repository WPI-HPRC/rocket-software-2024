#include "EKF.h"

/**
 * @brief Construct a new Quat State Estimator:: Quat State Estimator object
 * 
 * @param initialOrientation 
 * @param dt 
 */
StateEstimator::StateEstimator(BLA::Matrix<10> initialOrientation, float dt) {
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
BLA::Matrix<10> StateEstimator::onLoop(Utility::SensorPacket sensorPacket) {

    /* Read Data from Sensors and Convert to SI Units */
    // Convert Accel values to m/s/s
    sensorPacket.accelX = sensorPacket.accelX * 9.81;
    sensorPacket.accelY = sensorPacket.accelY * 9.81;
    sensorPacket.accelZ = sensorPacket.accelZ * 9.81;
    // Convert gyro values from deg/s to rad/s
    sensorPacket.gyroX = sensorPacket.gyroX * (PI / 180);
    sensorPacket.gyroY = sensorPacket.gyroY * (PI / 180);
    sensorPacket.gyroZ = sensorPacket.gyroZ * (PI / 180);
    // Convert Gauss to microTesla
    // sensorPacket.magX = ((sensorPacket.magX - 134920) / 134920) * 800000000;
    // sensorPacket.magY = ((sensorPacket.magY - 131820) / 131820) * 800000000;
    // sensorPacket.magZ = ((sensorPacket.magZ - 139070) / 139070) * 800000000;

    /* PREDICTION STEP 
    x_min = f(x,dt)
    A = df(x,dt) / dx
    P_min = A*P*A' + sigma_gyr^2*W*W'
    */

    // Apply measurement function to predict priori state of the system
    x_min = measurementFunction(sensorPacket);

    // Take the jacobian to obtain the covariance of the prediction step
    BLA::Matrix<10,10> A = measurementJacobian(sensorPacket);

    // Update model covariance from previous state
    BLA::Matrix<10,6> W = updateModelCovariance(sensorPacket);

    // Apply updated model covariance to process noise covariance matrix
    BLA::Matrix<10,10> Q = W * gyroAccelVar * BLA::MatrixTranspose<BLA::Matrix<10,6>>(W);

    // Update Priori Error Covariance
    P_min = A*P*BLA::MatrixTranspose<BLA::Matrix<10,10>>(A) + Q;

    // Apply Soft and Hard Iron Calibration to magnetometer data
    // **IMPORTANT** THIS MAY NEED TO BE RE-CALIBRATED UPON POSITION CHANGE
    BLA::Matrix<3> magVector = {
        sensorPacket.magX, sensorPacket.magY, sensorPacket.magZ
    };

    magVector = BLA::Inverse(magSoftIron) * magVector;

    magVector = magVector - hardIronCal;

    BLA::Matrix<3> accelVector = {sensorPacket.accelX,sensorPacket.accelX,sensorPacket.accelZ};
    // Normalize Accel and Mag for use in correction step
    magVector = magVector / BLA::Norm(magVector);
    accelVector = accelVector / BLA::Norm(accelVector);

    // Calculate update function with magnetometer readings to correct orientation
    BLA::Matrix<6> z = {
        accelVector(0), accelVector(1), accelVector(2), magVector(0), magVector(1), magVector(2)
    };

    BLA::Matrix<6> h = updateFunction(sensorPacket);

    // Take the jacobian to obtain the covariance of the correction function
    BLA::Matrix<6,10> H = updateJacobian(sensorPacket);

    // Compute the kalman gain from the magnetometer covariance readings
    BLA::Matrix<6> v = z - h;
    BLA::Matrix<6,6> S = H * P_min * BLA::MatrixTranspose<BLA::Matrix<6,10>>(H) + R;
    BLA::Matrix<10,6>  K = P_min * BLA::MatrixTranspose<BLA::Matrix<6,10>>(H) * BLA::Inverse(S);

    // Use our kalman gain and magnetometer readings to correct priori orientation
    x = x_min + K*v;
    // x = x_min;

    // Update error covariance matrix
    P = (eye10 - K*H)*P_min;
    // P = P_min;

    float quatNorm = sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));
    x(0) = x(0) / quatNorm;
    x(1) = x(1) / quatNorm;
    x(2) = x(2) / quatNorm;
    x(3) = x(3) / quatNorm;

    // Debug Printout to Data Logger
    // Serial.print(sensorPacket.accelX); Serial.print(",");
    // Serial.print(sensorPacket.accelY); Serial.print(",");
    // Serial.print(sensorPacket.accelZ); Serial.print(",");
    // Serial.print(sensorPacket.gyroX); Serial.print(",");
    // Serial.print(sensorPacket.gyroY); Serial.print(",");
    // Serial.print(sensorPacket.gyroZ); Serial.print(",");
    // Serial.print(magVector(0)); Serial.print(",");
    // Serial.print(magVector(1)); Serial.print(",");
    // Serial.print(magVector(2)); Serial.print(",");
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
    // Serial.print("ACC|");
    // Serial.print(sensorPacket.ac_x); Serial.print(",");
    // Serial.print(sensorPacket.ac_y); Serial.print(",");
    // Serial.println(sensorPacket.ac_z);

    return this->x;
}

BLA::Matrix<10> StateEstimator::measurementFunction(Utility::SensorPacket sensorPacket) {
    float p = sensorPacket.gyroX; float q = sensorPacket.gyroY; float r = sensorPacket.gyroZ;
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

    // BLA::Matrix<4> gravNEDQuat = {0, 0, 0,-9.81};

    // BLA::Matrix<4> quatConj = {x(0), -x(1), -x(2), -x(3)};

    // BLA::Matrix<4> bodyGrav = quaternionMultiplication(quaternionMultiplication(x, gravNEDQuat), quatConj);


    // Serial.println("<----- Gravity in Body ----->");
    // for (int i = 0; i < bodyGrav.Rows; i++) {
    //     for (int j = 0; j < bodyGrav.Cols; j++) {
    //         Serial.print(String(bodyGrav(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Serial.println("<----- Accel in Body ----->");
    // for (int i = 0; i < bodyAccel.Rows; i++) {
    //     for (int j = 0; j < bodyAccel.Cols; j++) {
    //         Serial.print(String(bodyAccel(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // Normalize Quaternion Function
    f_q = f_q / BLA::Norm(f_q);

    BLA::Matrix<3> f_v = {0,0,0};
    BLA::Matrix<3> f_p = {0,0,0};

    BLA::Matrix<10> f = {
        f_q(0),
        f_q(1),
        f_q(2),
        f_q(3),
        f_v(0),
        f_v(1),
        f_v(2),
        f_p(0),
        f_p(1),
        f_p(2)
    };

    return f;
};

BLA::Matrix<10,10> StateEstimator::measurementJacobian(Utility::SensorPacket sensorPacket) {
    float p = sensorPacket.gyroX; float q = sensorPacket.gyroY; float r = sensorPacket.gyroZ;

    BLA::Matrix<10,10> A = {
        1, -(dt/2)*p, -(dt/2)*q, -(dt/2)*r, 0,0,0,0,0,0,
        (dt/2)*p, 1, (dt/2)*r, -(dt/2)*q, 0,0,0,0,0,0,
        (dt/2)*q, -(dt/2)*r, 1, (dt/2)*p, 0,0,0,0,0,0,
        (dt/2)*r, (dt/2)*q, -(dt/2)*p, 1, 0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,
    };

    return A;
}

BLA::Matrix<6> StateEstimator::updateFunction(Utility::SensorPacket sensorPacket) {
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

BLA::Matrix<6,10> StateEstimator::updateJacobian(Utility::SensorPacket sensorPacket) {
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

    // float sigma1 = 2*rx*qx + 2*ry*qy + 2*rz*qz;
    // float sigma2 = 2*rx*qw + 2*ry*qz - 2*rz*qy;
    // float sigma3 = 2*ry*qw - 2*rx*qz + 2*rz*qx;
    // float sigma4 = 2*rx*qy - 2*ry*qx + 2*rz*qw;
    // float sigma5 = -2*gz*qz;
    // float sigma6 = -2*gz*qx;
    // float sigma7 = -2*gz*qw;

    // BLA::Matrix<6,10> H = {
    //     2*gz*qy, sigma5, 2*gz*qw, sigma6, 0,0,0,0,0,0,
    //     sigma6, sigma7, sigma5, -2*gz*qy, 0,0,0,0,0,0,
    //     sigma7, 2*gz*qx, 2*gz*qy, sigma5, 0,0,0,0,0,0,
    //     sigma2, sigma1, 2*ry*qx-2*rx*qy-2*rz*qw, sigma3, 0,0,0,0,0,0,
    //     sigma3, sigma4, sigma1, 2*rz*qy-2*ry*qz-2*rx*qw, 0,0,0,0,0,0,
    //     sigma4, 2*rx*qz-2*ry*qw-2*rx*qx, sigma2, sigma1, 0,0,0,0,0,0,
    // };

    BLA::Matrix<6,10> H = {
        gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz, -gx*qy+gy*qx-gz*qw, -gx*qz+gy*qw+gz*qx, 0,0,0,0,0,0,
        -gx*qz+gy*qw+gz*qx, gx*qy-gy*qx+gz*qw, gx*qx+gy*qy+gz*qz, -gx*qw-gy*qz+gz*qy, 0,0,0,0,0,0,
        gx*qy-gy*qx+gz*qw, gx*qz-gy*qw-gz*qx, gx*qw+gy*qz-gz*qy, gx*qx+gy*qy+gz*qz, 0,0,0,0,0,0,
        rx*qw+ry*qz-rz*qy, rx*qy+ry*qy+rz*qz, -rx*qy+ry*qx-rz*qw, -rx*qz+ry*qw+rz*qx, 0,0,0,0,0,0,
        -rx*qz+ry*qw+rz*qx, rx*qy-ry*qx+rz*qw, rx*qx+ry*qy+rz*qz, -rx*qw-ry*qz+rz*qy, 0,0,0,0,0,0,
        rx*qy-ry*qx+rz*qw, rx*qz-ry*qw-rz*qx, rx*qw+ry*qz-rz*qy, rx*qx+ry*qy+rz*qz, 0,0,0,0,0,0,
    };

    return H;

};

BLA::Matrix<10,6> StateEstimator::updateModelCovariance(Utility::SensorPacket sensorPacket) {

    BLA::Matrix<10,6> W = {
        -x(1), -x(2), -x(3), 0, 0, 0,
        x(0), -x(3), x(2), 0, 0, 0,
        x(3), x(0), -x(1), 0, 0, 0,
        -x(2), x(1), x(0), 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
    };

    return W * (dt/2);
};

BLA::Matrix<3,3> StateEstimator::quat2rotm(BLA::Matrix<4> q) {
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

BLA::Matrix<4> StateEstimator::quaternionMultiplication(BLA::Matrix<4> q1, BLA::Matrix<4> q2) {

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