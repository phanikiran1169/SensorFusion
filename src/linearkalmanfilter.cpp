// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// CONSTANTS
constexpr bool INIT_ON_FIRST_PREDICTION = false;
constexpr double INIT_POS_STD = 0.0;
constexpr double INIT_VEL_STD = 10.0;
constexpr double ACCEL_STD = 0.5;
constexpr double GPS_POS_STD = 3.0;
// -------------------------------------------------- //

void KalmanFilter::predictionStep(double dt)
{
    if (!isInitialised() && INIT_ON_FIRST_PREDICTION)
    {
        // The state vector has the form [X,Y,VX,VY].
        // ----------------------------------------------------------------------- //
            VectorXd state = Vector4d::Zero();
            MatrixXd cov = Matrix4d::Zero();

            // The initial position is (X,Y) = (0,0) m
            // The initial velocity is 5 m/s at 45 degrees (VX,VY) = (5*cos(45deg),5*sin(45deg)) m/s
            state << 0, 0, 5.0*cos(M_PI/4), 5.0*sin(M_PI/4);

            // State Covariance Matrix
            cov(0,0) = INIT_POS_STD*INIT_POS_STD;
            cov(1,1) = INIT_POS_STD*INIT_POS_STD;
            cov(2,2) = INIT_VEL_STD*INIT_VEL_STD;
            cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;
            
            setState(state);
            setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }

    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Kalman Filter Prediction Step for the system 
        // ----------------------------------------------------------------------- //
        
        // State Transition Matrix
        MatrixXd F = Matrix4d::Zero();
        F(0,0) = 1.0;
        F(1,1) = 1.0;
        F(2,2) = 1.0;
        F(3,3) = 1.0;

        F(0,2) = dt;
        F(1,3) = dt;

        // Process Noise Covariance Matrix
        MatrixXd Q = Matrix2d::Zero();
        Q(0,0) = (ACCEL_STD*ACCEL_STD);
        Q(1,1) = (ACCEL_STD*ACCEL_STD);

        // Process Noise Sensitivity Matrix
        MatrixXd L = MatrixXd(4,2);
        L << (0.5*dt*dt), 0, 0,( 0.5*dt*dt), dt, 0, 0, dt;

        // State Prediction - Linear Dynamics
        state = F * state;
        
        // Covarince Prediction
        cov = F * cov * F.transpose() + L * Q * L.transpose();

        // ----------------------------------------------------------------------- //
        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Kalman Filter Update Step for the GPS Measurements
        // GPS sensor has a 3m (1 sigma) position uncertainty.
        // ----------------------------------------------------------------------- //
        
        // Measurement Matrix
        MatrixXd H = MatrixXd(2,4);
        H << 1, 0, 0, 0, 0, 1, 0, 0;

        // Measurement Noise Covariance Matrix
        MatrixXd R = Matrix2d::Zero();
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        // Sensor Measurement Vector
        VectorXd z = Vector2d();
        z << meas.x, meas.y;

        // Mesurement Update Equations
        VectorXd z_hat = Vector2d();
        z_hat = H * state;

        // Innovation
        VectorXd y = z - z_hat;

        // Innovation Covariance update
        MatrixXd S = H * cov * H.transpose() + R;
        
        // Kalman Gain
        MatrixXd K = cov*H.transpose()*S.inverse();

        // State update
        state = state + K*y;
        
        // State error covariance update
        MatrixXd I_KH = MatrixXd::Identity(4,4) - K*H;
        cov = (I_KH) * cov * (I_KH.transpose()) + K * R * K.transpose();

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
    else
    {
        // ----------------------------------------------------------------------- //
            VectorXd state = Vector4d::Zero();
            MatrixXd cov = Matrix4d::Zero();

            state << meas.x, meas.y, 0, 0;
            cov(0,0) = GPS_POS_STD*GPS_POS_STD;
            cov(1,1) = GPS_POS_STD*GPS_POS_STD;
            cov(2,2) = INIT_VEL_STD*INIT_VEL_STD;
            cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;

            setState(state);
            setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }        
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,VX,VY]
        double psi = std::atan2(state[3],state[2]);
        double V = std::sqrt(state[2]*state[2] + state[3]*state[3]);
        return VehicleState(state[0],state[1],psi,V);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt){predictionStep(dt);}
void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map){}
void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map){}

