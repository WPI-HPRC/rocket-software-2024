#include "AttitudeEKF.h"

// TODO: Fix comments, do be wrong - Dan Pearson

/**
 * @brief Construct a new Quat State Estimator:: Quat State Estimator object
 *
 */
AttitudeStateEstimator::AttitudeStateEstimator() {}

/**
 * @param initialOrientation
 * @param dt
 */
void AttitudeStateEstimator::init(BLA::Matrix<4> initialOrientation, float dt) {
    // this->x = initialOrientation;

    // Intialize initial orientation and bias estimate
    this->x(1) = initialOrientation(1);
    this->x(2) = initialOrientation(2);
    this->x(3) = initialOrientation(3);
    this->x(4) = initialOrientation(4);
    // Initialize bias whole state to be zero
    this->x(5) = 0; this->x(6) = 0; this->x(7) = 0;
    this->x(8) = 0; this->x(9) = 0; this->x(10) = 0;

    /* ---- Initialize error covariance ----- */
    P.Fill(0.0f);

    // Initialize attitude as gyro covariance
    for(int i=0; i < 4; i++) {
        P(i,i) = gyroVar * gyroVar;
    }

    // Initialize random small number (tunable) for gyro bias covariance
    for (int i = 4; i < 7; i++) {
        P(i, i) = 0.01;
    }

    // Intialize random small number (tunable) for accel bias covariance
    for (int i = 7; i < 10; i++) {
        P(i, i) = 0.01;
    }

    /* ---- Initialize Process Noise covariance ----- */
    Q_k.Fill(0.0f);

    // Initialize attitude as gyro covariance
    for(int i=0; i < 4; i++) {
        Q_k(i,i) = gyroVar * gyroVar;
    }

    // Initialize random small number (tunable) for gyro bias covariance
    for (int i = 4; i < 7; i++) {
        Q_k(i, i) = std_dev_gyrBias;
    }

    // Intialize random small number (tunable) for accel bias covariance
    for (int i = 7; i < 10; i++) {
        Q_k(i, i) = std_dev_accBias;
    }

    this->dt = dt;
    this->initialized = true;
}

/**
 * @brief Run every loop of the state machine to perform the predict and update step of the EKF
 *
 * @param telemPacket Sensor Frame
 * @return BLA::Matrix<4> State Vector
 */
void AttitudeStateEstimator::onLoop(Utility::TelemPacket telemPacket)
{
    /* Read Data from Sensors and Convert to SI Units */

    // Convert Accel values to m/s/s
        // Subtract current bias estimate
    float accX = telemPacket.accelX * g - x(8);
    float accY = telemPacket.accelY * g - x(9);
    float accZ = telemPacket.accelZ * g - x(10);

    // Convert gyro values from deg/s to rad/s
        // Subtract current bias estimate
    float gyrX = telemPacket.gyroX * (PI / 180) - x(5);
    float gyrY = telemPacket.gyroY * (PI / 180) - x(6);
    float gyrZ = telemPacket.gyroZ * (PI / 180) - x(7);

    float magX = telemPacket.magX;
    float magY = telemPacket.magY;
    float magZ = telemPacket.magZ;

    BLA::Matrix<6> u = {gyrX, gyrY, gyrZ, accX, accY, accZ};

    BLA::Matrix<10> x_temp = x;
    
    BLA::Matrix<10> k1 = measurementFunction(x_temp, u) * dt;
    BLA::Matrix<10> k2 = measurementFunction(x_temp + (k1 * 0.5f), u) * dt;
    BLA::Matrix<10> k3 = measurementFunction(x_temp + (k2 * 0.5f), u) * dt;
    BLA::Matrix<10> k4 = measurementFunction(x_temp + k3, u) * dt;

    // x_min = x_temp + k1 * (1.0f/6.0f) + k2 * (1.0f/3.0f) + k3 * (1.0f/3.0f) + k4 * (1.0f/6.0f);
    x_min = x + k1;

    BLA::Matrix<10,10> F = measurementJacobian(x, u);

    BLA::Matrix<10,10> phi = BLA::Eye<10,10>() + F * dt;

    P_min = phi * P * BLA::MatrixTranspose<BLA::Matrix<10,10>>(phi) + Q_k;

    x = x_min;

    P = P_min;

    Serial.println("<----- State ----->");
    for (int i = 0; i < x.Rows; i++) {
        for (int j = 0; j < x.Cols; j++) {
            Serial.print(String(x(i,j)) + "\t");
        }
        Serial.println("");
    }

    // x_min = measurementFunction(u);

    // BLA::Matrix<6> u = {accX, accY, accZ, gyrX, gyrY, gyrZ};

    // // Apply measurement function to predict priori state of the system
    // x_min = measurementFunction(u);

    // // Take the jacobian to obtain the covariance of the prediction step
    // BLA::Matrix<4, 4> A = measurementJacobian(u);

    // // Update model covariance from previous state
    // BLA::Matrix<4, 3> W = updateModelCovariance(telemPacket);
    // // BLA::Matrix<4,3> W = {0,0,0, 0,0,0, 0,0,0};

    // // Apply updated model covariance to process noise covariance matrix
    // BLA::Matrix<4, 4> Q = W * Sigma_gyro * BLA::MatrixTranspose<BLA::Matrix<4, 3>>(W);

    // // Update Priori Error Covariance
    // P_min = A * P * BLA::MatrixTranspose<BLA::Matrix<4,4>>(A) + Q;

    // BLA::Matrix<3> magVector = {magX, magY, magZ};
    // float magVectorLen = BLA::Norm(magVector);

    // BLA::Matrix<3> accelVector = {accX, accY, accZ};
    // float accelVectorLen = BLA::Norm(accelVector);

    // // Normalize Accel and Mag for use in correction step
    // if (magVectorLen != 0) {
    //     magVector /= magVectorLen;
    // }
    // if (accelVectorLen != 0) {
    //     accelVector /= accelVectorLen;
    // }

    // // Calculate update function with magnetometer readings to correct orientation
    // BLA::Matrix<6> z = {
    //     accelVector(0), accelVector(1), accelVector(2), magVector(0), magVector(1), magVector(2)
    // };
    // // BLA::Matrix<6> z = {
    // //     accelVector(0), accelVector(1), accelVector(2), 0, 0, 0
    // // };

    // BLA::Matrix<6> h = updateFunction();

    // // Take the jacobian to obtain the covariance of the correction function
    // BLA::Matrix<6, 4> H = updateJacobian();

    // // Compute the kalman gain from the magnetometer covariance readings
    // BLA::Matrix<6> v = z - h;
    // BLA::Matrix<6, 6> S = H * P_min * BLA::MatrixTranspose<BLA::Matrix<6, 4>>(H) + R;
    // BLA::Matrix<4, 6> K = P_min * BLA::MatrixTranspose<BLA::Matrix<6, 4>>(H) * BLA::Inverse(S);

    // // Use our kalman gain and magnetometer readings to correct priori orientation
    // x = x_min + K * v;

    // // Update error covariance matrix
    // P = (eye4 - K * H) * P_min;

    // Serial.println("<----- State ----->");
    // for (int i = 0; i < x.Rows; i++) {
    //     for (int j = 0; j < x.Cols; j++) {
    //         Serial.print(String(x(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    // float quatNorm = sqrt(x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3));
    // if (quatNorm != 0) {
    //     x(0) = x(0) / quatNorm;
    //     x(1) = x(1) / quatNorm;
    //     x(2) = x(2) / quatNorm;
    //     x(3) = x(3) / quatNorm;
    // }
}

BLA::Matrix<10> AttitudeStateEstimator::measurementFunction(BLA::Matrix<10> x_temp, BLA::Matrix<6> u)
{
    float p = u(0);
    float q = u(1);
    float r = u(2);

    BLA::Matrix<4,4> quatMat = {
        0, -p, -q, -r,
        p,  0,  r, -q,
        q, -r,  0,  p,
        r,  q, -p,  0,
    };

    BLA::Matrix<4,1> quat = {x_temp(0), x_temp(1), x_temp(2), x_temp(3)};

    BLA::Matrix<4,1> f_q = quatMat * quat;

    BLA::Matrix<10,1> f = {f_q(0), f_q(1), f_q(2), f_q(3), 0, 0, 0, 0, 0, 0};

    return f;
};

BLA::Matrix<10,10> AttitudeStateEstimator::measurementJacobian(BLA::Matrix<10> x_temp, BLA::Matrix<6> u) {
    float p = u(0);
    float q = u(1);
    float r = u(2);

    BLA::Matrix<10,10> F = {
        0, x(4)/2 - p/2, x(4)/2 - q/2, x(6)/2 - r/2,   x(1)/2 + x(2)/2, 0,  x(3)/2, 0, 0, 0,
        p/2 - x(4)/2,          0, r/2 - x(6)/2, x(4)/2 - q/2,   x(3)/2 - x(0)/2, 0, -x(2)/2, 0, 0, 0,
        q/2 - x(4)/2, x(6)/2 - r/2,          0, p/2 - x(4)/2, - x(0)/2 - x(3)/2, 0,  x(1)/2, 0, 0, 0,
        r/2 - x(6)/2, q/2 - x(4)/2, x(4)/2 - p/2,          0,   x(2)/2 - x(1)/2, 0, -x(0)/2, 0, 0, 0,
        0,          0,          0,          0,             0, 0,     0, 0, 0, 0,
        0,          0,          0,          0,             0, 0,     0, 0, 0, 0,
        0,          0,          0,          0,             0, 0,     0, 0, 0, 0,
        0,          0,          0,          0,             0, 0,     0, 0, 0, 0,
        0,          0,          0,          0,             0, 0,     0, 0, 0, 0,
        0,          0,          0,          0,             0, 0,     0, 0, 0, 0,
    };


}


BLA::Matrix<3, 3> AttitudeStateEstimator::quat2rotm(BLA::Matrix<4> q)
{

    float qw = q(0);
    float qx = q(1);
    float qy = q(2);
    float qz = q(3);

    BLA::Matrix<3, 3> rotm = {
        qw*qw + qx*qx - qy*qy - qz*qz, 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
        2 * (qx * qy + qw * qz), qw*qw - qx*qx + qy*qy - qz*qz, 2 * (qy * qz - qw * qx),
        2 * (qx * qz - qw * qy), 2 * (qw * qx + qy * qz), qw*qw - qx*qx - qy*qy + qz*qz
    };

    return rotm;
};

BLA::Matrix<4> AttitudeStateEstimator::quaternionMultiplication(BLA::Matrix<4> q1, BLA::Matrix<4> q2)
{

    float w1 = q1(0);
    float i1 = q1(1);
    float j1 = q1(2);
    float k1 = q1(3);

    float w2 = q2(0);
    float i2 = q2(1);
    float j2 = q2(2);
    float k2 = q2(3);

    BLA::Matrix<4> res;

    res(0) = w1 * w2 - i1 * i2 - j1 * j2 - k1 * k2;
    res(1) = w1 * i2 + i1 * w2 + j1 * k2 - k1 * j2;
    res(2) = w1 * j2 - i1 * k2 + j1 * w2 + k1 * i2;
    res(3) = w1 * k2 + i1 * j2 - j1 * i2 + k1 * w2;

    return res;
};
