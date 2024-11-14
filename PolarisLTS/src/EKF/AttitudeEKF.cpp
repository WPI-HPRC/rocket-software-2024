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
    this->x(0) = initialOrientation(0);
    this->x(1) = initialOrientation(1);
    this->x(2) = initialOrientation(2);
    this->x(3) = initialOrientation(3);
    // Initialize bias whole state to be zero
    this->x(4) = 0; this->x(5) = 0; this->x(6) = 0;
    this->x(7) = 0; this->x(8) = 0; this->x(9) = 0;

    /* ---- Initialize error covariance ----- */
    P.Fill(0.0f);

    // Initialize attitude as gyro covariance
    for(int i=0; i < 4; i++) {
        P(i,i) = gyroVar * gyroVar;
    }

    // Initialize random small number (tunable) for gyro bias covariance
    for (int i = 4; i < 7; i++) {
        P(i, i) = 0.1;
    }

    // Intialize random small number (tunable) for accel bias covariance
    for (int i = 7; i < 10; i++) {
        P(i, i) = 0.1;
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
    float accX = telemPacket.accelX * g;
    float accY = telemPacket.accelY * g;
    float accZ = telemPacket.accelZ * g;

    BLA::Matrix<3> accBias = {x(7), x(8), x(9)};

    // Convert gyro values from deg/s to rad/s
        // Subtract current bias estimate
    float gyrX = telemPacket.gyroX * (PI / 180);
    float gyrY = telemPacket.gyroY * (PI / 180);
    float gyrZ = telemPacket.gyroZ * (PI / 180);

    float magX = telemPacket.magX;
    float magY = telemPacket.magY;
    float magZ = telemPacket.magZ;

    BLA::Matrix<6> u = {gyrX, gyrY, gyrZ, accX, accY, accZ};

    // First order, forward euler
    BLA::Matrix<10> k1 = measurementFunction(u) * dt;
    // BLA::Matrix<10> k2 = measurementFunction(x_temp + (k1 * 0.5f), u) * dt;
    // BLA::Matrix<10> k3 = measurementFunction(x_temp + (k2 * 0.5f), u) * dt;
    // BLA::Matrix<10> k4 = measurementFunction(x_temp + k3, u) * dt;

    x_min = x + k1;

    BLA::Matrix<10,10> F = measurementJacobian(u);

    // // x_min = x_temp + k1 * (1.0f/6.0f) + k2 * (1.0f/3.0f) + k3 * (1.0f/3.0f) + k4 * (1.0f/6.0f);
    // x_min = x + k1;

    BLA::Matrix<10,10> phi = BLA::Eye<10,10>() + (F * dt);

    P_min = phi * P * BLA::MatrixTranspose<BLA::Matrix<10,10>>(phi) + Q_k;

    BLA::Matrix<3> z = {accX, accY, accZ};
    z = z - accBias;

    BLA::Matrix<3> h = updateFunction();
    BLA::Matrix<3,10> H = updateJacobian();

    BLA::Matrix<3,3> S = H*P_min*BLA::MatrixTranspose<BLA::Matrix<3,10>>(H) + R;
    BLA::Matrix<10,3> K = P_min * BLA::MatrixTranspose<BLA::Matrix<3,10>>(H) * BLA::Inverse(S);

    // x = x_min;
    // P = P_min;

    x = x_min + K * (z - h);

    P = (BLA::Eye<10,10>() - K*H) * P_min;

    Serial.print(">GbX:");
    Serial.println(x(4));
    Serial.print(">GbY:");
    Serial.println(x(5));
    Serial.print(">GbZ:");
    Serial.println(x(6));

    Serial.print(">AbX:");
    Serial.println(x(7));
    Serial.print(">AbY:");
    Serial.println(x(8));
    Serial.print(">AbZ:");
    Serial.println(x(9));

    // Serial.println("<----- State ----->");
    // for (int i = 0; i < x.Rows; i++) {
    //     for (int j = 0; j < x.Cols; j++) {
    //         Serial.print(String(x(i,j)) + "\t");
    //     }
    //     Serial.println("");
    // }

    Serial.println("<----- Error Covariance ----->");
    for (int i = 0; i < P.Rows; i++) {
        for (int j = 0; j < P.Cols; j++) {
            Serial.print(String(P(i,j)) + "\t");
        }
        Serial.println("");
    }

    float quatNorm = sqrt(x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3));
    if (quatNorm != 0) {
        x(0) = x(0) / quatNorm;
        x(1) = x(1) / quatNorm;
        x(2) = x(2) / quatNorm;
        x(3) = x(3) / quatNorm;
    }
}

BLA::Matrix<10> AttitudeStateEstimator::measurementFunction(BLA::Matrix<6> u)
{
    float p = u(0) - x(4);
    float q = u(1) - x(5);
    float r = u(2) - x(6);

    BLA::Matrix<4,3> quatMat = {
        -x(1), -x(2), -x(3),
         x(0), -x(3),  x(2),
         x(3),  x(0), -x(1),
        -x(2),  x(1),  x(0)
    };

    quatMat = quatMat * 0.5f;

    BLA::Matrix<3> w_ib_b = {p, q, r};

    BLA::Matrix<4> f_q = quatMat * w_ib_b;

    f_q = f_q / BLA::Norm(f_q);

    BLA::Matrix<10> f = {f_q(0), f_q(1), f_q(2), f_q(3), 0, 0, 0, 0, 0, 0};

    return f;
};

BLA::Matrix<10,10> AttitudeStateEstimator::measurementJacobian(BLA::Matrix<6> u) {
    float p = u(0) - x(4);
    float q = u(1) - x(5);
    float r = u(2) - x(6);

    float gbx = x(4);
    float gby = x(5);
    float gbz = x(6);

    float qw = x(0);
    float qx = x(1);
    float qy = x(2);
    float qz = x(3);

    BLA::Matrix<10,10> F = {
        0, gbx - p, gby - q, gbz - r,  qx,  qy,  qz, 0, 0, 0,
        p - gbx,           0, r - gbz, gby - q, -qw,  qz, -qy, 0, 0, 0,
        q - gby, gbz - r,           0, p - gbx, -qz, -qw,  qx, 0, 0, 0,
        r - gbz, q - gby, gbx - p,           0,  qy, -qx, -qw, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0,
        0,           0,           0,           0,     0,     0,     0, 0, 0, 0,
    };
    
    return (F*0.5f);

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

BLA::Matrix<3> AttitudeStateEstimator::updateFunction() {
    BLA::Matrix<3> G_NED = {0, 0, -g};

    BLA::Matrix<4> q = {x_min(0), x_min(1), x_min(2), x_min(3)};

    BLA::Matrix<3,3> R_TB = quat2rotm(q);

    BLA::Matrix<3> accelBias = {x(7), x(8), x(9)};

    BLA::Matrix<3> h_accel = BLA::MatrixTranspose<BLA::Matrix<3,3>>(R_TB) * G_NED + accelBias;

    return h_accel;

}

BLA::Matrix<3,10> AttitudeStateEstimator::updateJacobian() {

    BLA::Matrix<3,10> H_accel = {
        2*g*x_min(2), -2*g*x_min(3),  2*g*x_min(0), -2*g*x_min(1), 0, 0, 0, 1, 0, 0,
       -2*g*x_min(1), -2*g*x_min(0), -2*g*x_min(3), -2*g*x_min(2), 0, 0, 0, 0, 1, 0,
       -4*g*x_min(0),  0,             0,            -4*g*x_min(4), 0, 0, 0, 0, 0, 1,
    };

    return H_accel;
}