#include "rpp_localization/inekf/inertial_process.hpp"
#include "rpp_localization/inekf/so3.hpp"
#include "rpp_localization/inekf/se3.hpp"
#include "rpp_localization/inekf/lie_group.hpp"
#include "rpp_localization/ros/ros_filter_utilities.hpp"



Eigen::Matrix3d rotation_from_euler(double roll, double pitch, double yaw){
    // roll and pitch and yaw in radians
    double su = sin(roll);
    double cu = cos(roll);
    double sv = sin(pitch);
    double cv = cos(pitch);
    double sw = sin(yaw);
    double cw = cos(yaw);
    Eigen::Matrix3d Rot_matrix(3, 3);
    Rot_matrix(0, 0) = cv*cw;
    Rot_matrix(0, 1) = su*sv*cw - cu*sw;
    Rot_matrix(0, 2) = su*sw + cu*sv*cw;
    Rot_matrix(1, 0) = cv*sw;
    Rot_matrix(1, 1) = cu*cw + su*sv*sw;
    Rot_matrix(1, 2) = cu*sv*sw - su*cw;
    Rot_matrix(2, 0) = -sv;
    Rot_matrix(2, 1) = su*cv;
    Rot_matrix(2, 2) = cu*cv;
    return Rot_matrix;
}

Eigen::VectorXd euler_from_rotation(const Eigen::Matrix3d& R)
{
    Eigen::VectorXd euler(3);
    euler << atan2(R(2, 1), R(2, 2)), asin(-R(2, 0)), atan2(R(1, 0), R(0, 0));
    return euler;
}

namespace rpp_localization::InEKF
{
    InertialProcess::InertialProcess(int state_dim)
    : NavModelBase(state_dim)
    {
        // Setting Initial Covariance Process Matrix to Zero (TBD)
        Q_ = MatrixCov::Zero();
        _use_control = false;
    }

    InertialProcess::~InertialProcess(){}

    void InertialProcess::init(std::shared_ptr<rclcpp::Node> node)
    {
        NavModelBase::init(node);

        // Set error type here in load params and initial position state and covariance
        load_params();
        _initialized = true;

        // SET THIS TO
        _error = ERROR::RIGHT;
    }

    void InertialProcess::step(Eigen::VectorXd& state, Eigen::MatrixXd& state_covariance, const rclcpp::Time & reference_time, const double delta_sec)
    {
        // Prepare input control vector
        prepareControl(reference_time, delta_sec);

        // (1) Apply control terms, which are actually accelerations
        state(StateMemberVroll) +=
            control_acceleration_(ControlMemberVroll) * delta_sec;
        state(StateMemberVpitch) +=
            control_acceleration_(ControlMemberVpitch) * delta_sec;
        state(StateMemberVyaw) +=
            control_acceleration_(ControlMemberVyaw) * delta_sec;

        state(StateMemberAx) = (_control_update_vector[ControlMemberVx] ?
            control_acceleration_(ControlMemberVx) :
            state(StateMemberAx));
        state(StateMemberAy) = (_control_update_vector[ControlMemberVy] ?
            control_acceleration_(ControlMemberVy) :
            state(StateMemberAy));
        state(StateMemberAz) = (_control_update_vector[ControlMemberVz] ?
            control_acceleration_(ControlMemberVz) :
            state(StateMemberAz));

        // Create input control vector
        Eigen::Matrix<double,6,1> u;
        u << state(StateMemberVroll), state(StateMemberVpitch), state(StateMemberVyaw),  // angular velocity
             state(StateMemberAx)   , state(StateMemberAy)    , state(StateMemberAz);  // linear acceleration


        set_lie_state(state);
        set_lie_covariance(state_covariance,
            rotation_from_euler(state(StateMemberRoll), state(StateMemberPitch), state(StateMemberYaw)));


        // START PROPAGATION
        // COVARIANCE PROPAGATION
        // Predict Sigma
        auto& current_state = get_lie_state();

        propagateState(u, delta_sec, current_state);

        MatrixCov Sigma = current_state.cov();
        MatrixCov Phi = makePhi(u, delta_sec, current_state, _error);
        MatrixCov Q = getQ();
        if (_error == ERROR::RIGHT)
        {
            MatrixCov Adj_X = current_state.Ad(current_state);
            Q = Adj_X * Q * Adj_X.transpose();
        }
        Sigma += Q * delta_sec;
        if (!Phi.isIdentity())
        {
            Sigma = Phi * Sigma * Phi.transpose();
        }
        current_state.setCov(Sigma);


        /**
         * SET PREDICTED VARIABLES TO RPP_LOCALIZATION STATES
        */
        // New state predicted, now map into rpp_localization state
        Eigen::Vector3d predicted_vel = current_state[0];
        Eigen::Vector3d predicted_pos = current_state[1];

        // Prediction a part for attitude
        auto attitude = euler_from_rotation(current_state.R()());

        // Now Compute new state
        // Position (Global frame)
        state(StateMemberX) = predicted_pos[0];
        state(StateMemberY) = predicted_pos[1];
        state(StateMemberZ) = predicted_pos[2];

        // Orientation
        state(StateMemberRoll)  = attitude[0];
        state(StateMemberPitch) = attitude[1];
        state(StateMemberYaw)   = attitude[2];

        // Wrap State Angles before computing Local Velocity
        filter_utilities::wrapStateAngles(state);

        auto Rot = current_state.R()();
        // Transform Velocity from Global to Local
        Eigen::Vector3d predicted_local_vel = Rot.inverse() * predicted_vel;

        // Velocity
        state(StateMemberVx) = predicted_local_vel[0];
        state(StateMemberVy) = predicted_local_vel[1];
        state(StateMemberVz) = predicted_local_vel[2];

        // Remap covariance matrix
        state_covariance.block(0, 0, 3, 3) = Sigma.block(6, 6, 3, 3); // pos
        state_covariance.block(3, 3, 3, 3) = Sigma.block(0, 0, 3, 3); // angle
        state_covariance.block(6, 6, 3, 3) = Rot.transpose() * Sigma.block(3, 3, 3, 3) * Rot; // vel

        // add angvel and acceleration process noise covariance
        state_covariance.block(9, 9, 3, 3) += process_noise_covariance_.block(9, 9, 3, 3) * delta_sec;
        state_covariance.block(12, 12, 3, 3) += process_noise_covariance_.block(12, 12, 3, 3) * delta_sec;

        MB_DEBUG(
        "---------------------- Ekf::predict ----------------------\n" <<
        "delta is " << delta_sec << "\n" <<
        "state is " << state << "\n");

    }

    void InertialProcess::propagateState(const Vector6d& u, double dt, SE3<2,6>& state)
    {
        Vector6d u_shifted = u - state.aug();
        Eigen::Matrix3d R = state.R()();
        Eigen::Vector3d omega = u.head(3);
        Eigen::Vector3d a = R * u.tail(3) + g_;
        Eigen::Vector3d v = state[0];

        Eigen::Vector3d p = state[1];

        // Calculate
        MatrixState S = MatrixState::Identity();
        S.block(0,0,3,3) = R * SO3<>::exp(omega*dt)();
        S.block(0,3,3,1) = v + a*dt;
        S.block(0,4,3,1) = p + v*dt + a*dt*dt/2;

        state.setMat(S);
    }

    void InertialProcess::set_lie_covariance(const Eigen::MatrixXd& cov, const Eigen::MatrixXd& R)
    {
        MatrixCov initial_covariance = MatrixCov::Zero();
        initial_covariance.block(0, 0, 3, 3) = cov.block(3, 3, 3, 3);
        initial_covariance.block(3, 3, 3, 3) = R * cov.block(6, 6, 3, 3) * R.transpose();
        initial_covariance.block(6, 6, 3, 3) = cov.block(0, 0, 3, 3);
        state_lie_.setCov(initial_covariance);
    }

    void InertialProcess::set_lie_state(const Eigen::VectorXd& state)
    {
        auto roll = state(StateMemberRoll);
        auto pitch = state(StateMemberPitch);
        auto yaw = state(StateMemberYaw);
        Eigen::Matrix3d Rot = rotation_from_euler(roll, pitch, yaw);

        Eigen::Vector3d v;
        Eigen::Vector3d p;
        v << state(StateMemberVx), state(StateMemberVy), state(StateMemberVz);
        p << state(StateMemberX), state(StateMemberY), state(StateMemberZ);
        v = Rot * v;

        MatrixState S = MatrixState::Identity();
        S.block(0,0,3,3) = Rot;
        S.block(0,3,3,1) = v;
        S.block(0,4,3,1) = p;

        state_lie_.setMat(S);
    }

    typedef Eigen::Matrix<double,15,15> MatrixCov;
    typedef Eigen::Matrix<double,6,1> Vector6d;
    MatrixCov InertialProcess::makePhi(const Eigen::Matrix<double, 6, 1>& u, double dt, const SE3<2,6>& state, InEKF::ERROR error)
    {
        MatrixCov A = MatrixCov::Zero();

        if(error == InEKF::ERROR::RIGHT){
            // Get everything we need
            Eigen::Matrix3d R = state.R()();
            Eigen::Matrix3d v_cross = SO3<>::wedge( state[0] );
            Eigen::Matrix3d p_cross = SO3<>::wedge( state[1] );

            A.block<3,3>(3,0) = SO3<>::wedge(g_);
            A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();

            A.block<3,3>(0,9) = -R;
            A.block<3,3>(3,9) = -v_cross * R;
            A.block<3,3>(6,9) = -p_cross * R;
            A.block<3,3>(3,12) = -R;

            return MatrixCov::Identity() + A*dt + A*A*dt*dt/2 + A*A*A*dt*dt*dt/6;
        }
        else{
            Vector6d u_shifted = u - state.aug();
            Eigen::Matrix3d w_cross = SO3<>::wedge( u_shifted.head(3) );
            Eigen::Matrix3d a_cross = SO3<>::wedge( u_shifted.tail(3) );

            A.block<3,3>(0,0) = -w_cross;
            A.block<3,3>(3,3) = -w_cross;
            A.block<3,3>(6,6) = -w_cross;
            A.block<3,3>(3,0) = -a_cross;
            A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();

            A.block<3,3>(0,9) = -Eigen::Matrix3d::Identity();
            A.block<3,3>(3,12) = -Eigen::Matrix3d::Identity();

            return (A*dt).exp();
        }
    }

    void InertialProcess::setGyroNoise(Eigen::Matrix3d M){
        Q_.block<3,3>(0,0) = M;
    }

    void InertialProcess::setAccelNoise(Eigen::Matrix3d M){
        Q_.block<3,3>(3,3) = M;
    }

    void InertialProcess::setGyroBiasNoise(double std){
        Q_.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * std*std;
    }

    void InertialProcess::setAccelBiasNoise(double std){
        Q_.block<3,3>(12,12) = Eigen::Matrix3d::Identity() * std*std;
    }

    void InertialProcess::load_params()
    {
        /**
         * LOAD ROS PARAMETERS FOR INITIALIZING INEKF FILTER
        */
        std::vector<double> initial_state;
        if (_node->get_parameter("initial_state", initial_state)) {
            if (initial_state.size() != STATE_SIZE) {
            RCLCPP_ERROR_STREAM(
                _node->get_logger(),
                "Initial state must be of size " << STATE_SIZE << ". Provided config was of size " <<
                initial_state.size() << ". The initial state will be ignored.");
            }
            else
            {
                Eigen::VectorXd initial_state_as_vec = Eigen::VectorXd::Map(initial_state.data(), initial_state.size());
                // INITIALIZE STATE MATRIX LIE
                // Compute everything: velocity is calculated non in local but in global frame
                // Prepare state SE2(3) Matrix Lie Group
                double roll     = initial_state[StateMemberRoll];
                double pitch    = initial_state[StateMemberPitch];
                double yaw      = initial_state[StateMemberYaw];

                this->set_state(initial_state_as_vec);
                this->set_lie_state(initial_state_as_vec);

                auto R = rotation_from_euler(roll, pitch, yaw);
                this->set_lie_covariance(_state_covariance, R);
                this->setQ(process_noise_covariance_);
            }
        }
    }
}