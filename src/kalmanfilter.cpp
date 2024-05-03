// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Extended Kalman Filter
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// CONSTANTS
constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //

void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset) {handleLidarMeasurement(meas, map);}
}

void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Kalman Filter Update Step for the Lidar Measurements
        // HINT: The mapped-matched beacon position can be accessed by the variables
        // map_beacon.x and map_beacon.y
        // ----------------------------------------------------------------------- //

        BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id
        if (meas.id != -1 && map_beacon.id != -1)
        {

        }

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Kalman Filter Prediction Step for the system in the  
        // The state vector has the form [PX, PY, PSI, V].
        // ----------------------------------------------------------------------- //
        
        double x = state(0);
        double y = state(1);
        double psi = state(2);
        double V = state(3);
        double psi_dot = gyro.psi_dot;

        // State Update
        state(0) += dt*V*std::cos(psi);
        state(1) += dt*V*std::sin(psi);
        state(2) += dt*gyro.psi_dot;

        // Covariance Update
        
        // F Matrix
        MatrixXd F = Matrix4d::Zero();
        F << 1, 0, -dt*V*sin(psi), dt*cos(psi), 0, 1, dt*V*cos(psi), dt*sin(psi), 0, 0, 1, 0, 0, 0, 0, 1;

        // Q Matrix
        MatrixXd Q = Matrix4d::Zero();
        Q(2,2) = dt*dt*GYRO_STD*GYRO_STD;
        Q(3,3) = dt*dt*ACCEL_STD*ACCEL_STD;

        cov = F * cov * F.transpose() + Q;
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
        VectorXd state = getState(); // STATE VECTOR [X,Y,PSI,V,...]
        return VehicleState(state[0],state[1],state[2],state[3]);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt){}
