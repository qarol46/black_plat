#include "utility.hpp"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

/**
 * @brief Custom preintegration class for combined Wheel Odometry and IMU model
 */
class PreintegratedWheelInertialMeasurements : public gtsam::PreintegratedImuMeasurements {
private:
    gtsam::Vector3 deltaP_;      // Position delta [p_x, p_y, p_z]
    gtsam::Vector3 deltaV_;      // Velocity delta [v_x, v_y, v_z]
    gtsam::Vector3 deltaTheta_;  // Rotation delta [theta_x, theta_y, theta_z]
    double deltaTij_;            // Total integration time
    
    gtsam::Matrix3 deltaRotCov_;
    gtsam::Matrix3 deltaVelCov_;
    gtsam::Matrix3 deltaPosCov_;

public:
    PreintegratedWheelInertialMeasurements(
        const boost::shared_ptr<gtsam::PreintegrationParams>& p,
        const gtsam::imuBias::ConstantBias& priorBias)
        : gtsam::PreintegratedImuMeasurements(p, priorBias),
          deltaP_(0, 0, 0), deltaV_(0, 0, 0), deltaTheta_(0, 0, 0), deltaTij_(0.0),
          deltaRotCov_(gtsam::I_3x3), deltaVelCov_(gtsam::I_3x3), deltaPosCov_(gtsam::I_3x3) {}

    ~PreintegratedWheelInertialMeasurements() override = default;

    void resetIntegration() override {
        gtsam::PreintegratedImuMeasurements::resetIntegration();
        deltaP_.setZero();
        deltaV_.setZero();
        deltaTheta_.setZero();
        deltaTij_ = 0.0;
        deltaRotCov_ = gtsam::I_3x3;
        deltaVelCov_ = gtsam::I_3x3;
        deltaPosCov_ = gtsam::I_3x3;
    }

    void resetIntegrationAndSetBias(const gtsam::imuBias::ConstantBias& bias) {
        gtsam::PreintegratedImuMeasurements::resetIntegrationAndSetBias(bias);
        deltaP_.setZero();
        deltaV_.setZero();
        deltaTheta_.setZero();
        deltaTij_ = 0.0;
        deltaRotCov_ = gtsam::I_3x3;
        deltaVelCov_ = gtsam::I_3x3;
        deltaPosCov_ = gtsam::I_3x3;
    }

    void integrateMeasurement(double dt,
                              const gtsam::Vector3& gyro,
                              double wheelLinearX,
                              double wheelAngularZ) {
        
        deltaTij_ += dt;
        
        // Attitude integration
        gtsam::Vector3 deltaTheta_inc;
        deltaTheta_inc(0) = gyro.x() * dt;
        deltaTheta_inc(1) = gyro.y() * dt;
        deltaTheta_inc(2) = wheelAngularZ * dt;
        
        deltaTheta_ += deltaTheta_inc;
        
        gtsam::Vector3 gyro_for_base = gyro;
        gyro_for_base(2) = wheelAngularZ;
        
        gtsam::PreintegratedImuMeasurements::integrateMeasurement(
            gtsam::Vector3(0, 0, 0),
            gyro_for_base,
            dt);
        
        // Velocity integration
        gtsam::Rot3 currentRot = gtsam::Rot3::Expmap(deltaTheta_);
        gtsam::Vector3 velocityBody(wheelLinearX, 0, 0);
        gtsam::Vector3 velocityNav = currentRot * velocityBody;
        deltaV_ += velocityNav * dt;
        
        // Position integration
        deltaP_ += deltaV_ * dt;
        
        // Covariance update (simplified)
        const double gyro_noise = 1e-4;
        const double wheel_noise_linear = 0.01;
        const double wheel_noise_angular = 0.01;
        
        deltaRotCov_ += gtsam::Matrix33::Identity() * pow(gyro_noise * dt, 2);
        deltaVelCov_ += gtsam::Matrix33::Identity() * pow(wheel_noise_linear * dt, 2);
        deltaPosCov_ += gtsam::Matrix33::Identity() * pow(wheel_noise_linear * dt * dt, 2);
    }

    gtsam::NavState predict(const gtsam::NavState& state_i,
                            const gtsam::imuBias::ConstantBias& bias_i) const {
        
        gtsam::Vector3 bias_correction(0, 0, 0);
        bias_correction(0) = bias_i.gyroscope().x() * deltaTij_;
        bias_correction(1) = bias_i.gyroscope().y() * deltaTij_;
        
        gtsam::Rot3 deltaRot = gtsam::Rot3::Expmap(deltaTheta_ - bias_correction);
        
        gtsam::Rot3 newR = state_i.attitude() * deltaRot;
        gtsam::Vector3 newV = state_i.velocity() + deltaV_;
        gtsam::Point3 newP = state_i.position() + deltaP_;
        
        return gtsam::NavState(newR, newP, newV);
    }

    double deltaTij() const { return deltaTij_; }
    const gtsam::Vector3& deltaP() const { return deltaP_; }
    const gtsam::Vector3& deltaV() const { return deltaV_; }
    const gtsam::Vector3& deltaTheta() const { return deltaTheta_; }
};

// TransformFusion class (unchanged, but make sure it doesn't cause issues)
class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometry;

    rclcpp::CallbackGroup::SharedPtr callbackGroupImuOdometry;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLaserOdometry;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

    Eigen::Isometry3d lidarOdomAffine;
    Eigen::Isometry3d imuOdomAffineFront;
    Eigen::Isometry3d imuOdomAffineBack;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    tf2::Stamped<tf2::Transform> lidar2Baselink;

    double lidarOdomTime = -1;
    std::deque<nav_msgs::msg::Odometry> imuOdomQueue;

    TransformFusion(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_transformFusion", options)
    {
        tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        callbackGroupImuOdometry = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupLaserOdometry = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imuOdomOpt = rclcpp::SubscriptionOptions();
        imuOdomOpt.callback_group = callbackGroupImuOdometry;
        auto laserOdomOpt = rclcpp::SubscriptionOptions();
        laserOdomOpt.callback_group = callbackGroupLaserOdometry;

        subLaserOdometry = create_subscription<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry", qos,
            std::bind(&TransformFusion::lidarOdometryHandler, this, std::placeholders::_1),
            laserOdomOpt);
        subImuOdometry = create_subscription<nav_msgs::msg::Odometry>(
            odomTopic+"_incremental", qos_imu,
            std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1),
            imuOdomOpt);

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic, qos_imu);
        pubImuPath = create_publisher<nav_msgs::msg::Path>("lio_sam/imu/path", qos);

        tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    ~TransformFusion() = default;

    Eigen::Isometry3d odom2affine(nav_msgs::msg::Odometry odom)
    {
        tf2::Transform t;
        tf2::fromMsg(odom.pose.pose, t);
        return tf2::transformToEigen(tf2::toMsg(t));
    }

    void lidarOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        lidarOdomAffine = odom2affine(*odomMsg);
        lidarOdomTime = stamp2Sec(odomMsg->header.stamp);
    }

    void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        if (lidarOdomTime == -1)
            return;
            
        while (!imuOdomQueue.empty())
        {
            if (stamp2Sec(imuOdomQueue.front().header.stamp) <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        
        if (imuOdomQueue.empty())
            return;
            
        Eigen::Isometry3d imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Isometry3d imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Isometry3d imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Isometry3d imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        auto t = tf2::eigenToTransform(imuOdomAffineLast);
        tf2::Stamped<tf2::Transform> tCur;
        tf2::convert(t, tCur);

        nav_msgs::msg::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = t.transform.translation.x;
        laserOdometry.pose.pose.position.y = t.transform.translation.y;
        laserOdometry.pose.pose.position.z = t.transform.translation.z;
        laserOdometry.pose.pose.orientation = t.transform.rotation;
        pubImuOdometry->publish(laserOdometry);

        if(lidarFrame != baselinkFrame)
        {
            try
            {
                tf2::fromMsg(tfBuffer->lookupTransform(
                    lidarFrame, baselinkFrame, rclcpp::Time(0)), lidar2Baselink);
            }
            catch (tf2::TransformException ex)
            {
                RCLCPP_ERROR(get_logger(), "%s", ex.what());
            }
            tf2::Stamped<tf2::Transform> tb(
                tCur * lidar2Baselink, tf2_ros::fromMsg(odomMsg->header.stamp), odometryFrame);
            tCur = tb;
        }
        
        if (publishOdomToBaseTF)
        {
            geometry_msgs::msg::TransformStamped ts;
            tf2::convert(tCur, ts);
            ts.child_frame_id = baselinkFrame;
            tfBroadcaster->sendTransform(ts);
        }

        static nav_msgs::msg::Path imuPath;
        static double last_path_time = -1;
        double imuTime = stamp2Sec(imuOdomQueue.back().header.stamp);
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while(!imuPath.poses.empty() && stamp2Sec(imuPath.poses.front().header.stamp) < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath->get_subscription_count() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath->publish(imuPath);
            }
        }
    }
};

// Main IMUPreintegration class
class IMUPreintegration : public ParamServer
{
public:
    std::mutex mtx;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subWheelOdom;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;

    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupWheel;

    bool systemInitialized = false;

    gtsam::SharedNoiseModel priorPoseNoise;
    gtsam::SharedNoiseModel priorVelNoise;
    gtsam::SharedNoiseModel priorBiasNoise;
    gtsam::SharedNoiseModel correctionNoise;
    gtsam::SharedNoiseModel correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;

    // Use shared_ptr for automatic memory management
    std::shared_ptr<PreintegratedWheelInertialMeasurements> wheelInertialIntegratorOpt_;
    std::shared_ptr<PreintegratedWheelInertialMeasurements> wheelInertialIntegratorImu_;

    std::deque<sensor_msgs::msg::Imu> imuQueOpt;
    std::deque<sensor_msgs::msg::Imu> imuQueImu;
    std::deque<nav_msgs::msg::Odometry> wheelOdomQueOpt;
    std::deque<nav_msgs::msg::Odometry> wheelOdomQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    double lastWheelT_imu = -1;
    double lastWheelT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    int key = 1;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration(const rclcpp::NodeOptions & options) :
            ParamServer("lio_sam_imu_preintegration", options)
    {
        RCLCPP_INFO(get_logger(), "IMUPreintegration constructor started");
        
        callbackGroupImu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupWheel = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = callbackGroupImu;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;
        auto wheelOpt = rclcpp::SubscriptionOptions();
        wheelOpt.callback_group = callbackGroupWheel;

        RCLCPP_INFO(get_logger(), "Creating subscribers...");
        
        subImu = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, qos_imu,
            std::bind(&IMUPreintegration::imuHandler, this, std::placeholders::_1),
            imuOpt);
            
        subOdometry = create_subscription<nav_msgs::msg::Odometry>(
            "lio_sam/mapping/odometry_incremental", qos,
            std::bind(&IMUPreintegration::odometryHandler, this, std::placeholders::_1),
            odomOpt);
            
        subWheelOdom = create_subscription<nav_msgs::msg::Odometry>(
            "/diff_drive_controller/odom", qos_imu,
            std::bind(&IMUPreintegration::wheelOdomHandler, this, std::placeholders::_1),
            wheelOpt);

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(odomTopic+"_incremental", qos_imu);

        RCLCPP_INFO(get_logger(), "Setting up preintegration parameters...");
        
        boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2);
        p->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2);
        p->integrationCovariance = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2);
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
        
        RCLCPP_INFO(get_logger(), "Creating custom integrators...");
        
        // Use make_shared for automatic memory management
        wheelInertialIntegratorImu_ = std::make_shared<PreintegratedWheelInertialMeasurements>(p, prior_imu_bias);
        wheelInertialIntegratorOpt_ = std::make_shared<PreintegratedWheelInertialMeasurements>(p, prior_imu_bias);
        
        if (!wheelInertialIntegratorImu_ || !wheelInertialIntegratorOpt_) {
            RCLCPP_ERROR(get_logger(), "Failed to create integrators");
        }
        
        RCLCPP_INFO(get_logger(), "IMUPreintegration constructor finished");
    }

    ~IMUPreintegration() {
        RCLCPP_INFO(get_logger(), "Destructor called");
        // shared_ptr automatically cleans up
    }

    void wheelOdomHandler(const nav_msgs::msg::Odometry::SharedPtr wheelOdomMsg)
    {
        if (!wheelOdomMsg) {
            RCLCPP_ERROR(get_logger(), "wheelOdomHandler: null message");
            return;
        }
        
        std::lock_guard<std::mutex> lock(mtx);
        
        try {
            double currentTime = stamp2Sec(wheelOdomMsg->header.stamp);
            
            wheelOdomQueOpt.push_back(*wheelOdomMsg);
            wheelOdomQueImu.push_back(*wheelOdomMsg);
            
            // Keep only recent messages
            while (!wheelOdomQueOpt.empty() && 
                   stamp2Sec(wheelOdomQueOpt.front().header.stamp) < currentTime - 1.0)
            {
                wheelOdomQueOpt.pop_front();
            }
            
            while (!wheelOdomQueImu.empty() && 
                   stamp2Sec(wheelOdomQueImu.front().header.stamp) < currentTime - 1.0)
            {
                wheelOdomQueImu.pop_front();
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception in wheelOdomHandler: %s", e.what());
        }
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        graphFactors.resize(0);
        graphValues.clear();
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        lastWheelT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        if (!odomMsg || !wheelInertialIntegratorOpt_ || !wheelInertialIntegratorImu_) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(mtx);

        try {
            double currentCorrectionTime = stamp2Sec(odomMsg->header.stamp);

            if (imuQueOpt.empty() || wheelOdomQueOpt.empty()) {
                return;
            }

            float p_x = odomMsg->pose.pose.position.x;
            float p_y = odomMsg->pose.pose.position.y;
            float p_z = odomMsg->pose.pose.position.z;
            float r_x = odomMsg->pose.pose.orientation.x;
            float r_y = odomMsg->pose.pose.orientation.y;
            float r_z = odomMsg->pose.pose.orientation.z;
            float r_w = odomMsg->pose.pose.orientation.w;
            bool degenerate = (int)odomMsg->pose.covariance[0] == 1;
            gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

            if (!systemInitialized)
            {
                RCLCPP_INFO(get_logger(), "Initializing system...");
                
                resetOptimization();

                while (!imuQueOpt.empty() && stamp2Sec(imuQueOpt.front().header.stamp) < currentCorrectionTime)
                {
                    lastImuT_opt = stamp2Sec(imuQueOpt.front().header.stamp);
                    imuQueOpt.pop_front();
                }
                while (!wheelOdomQueOpt.empty() && stamp2Sec(wheelOdomQueOpt.front().header.stamp) < currentCorrectionTime)
                {
                    lastWheelT_opt = stamp2Sec(wheelOdomQueOpt.front().header.stamp);
                    wheelOdomQueOpt.pop_front();
                }

                prevPose_ = lidarPose.compose(lidar2Imu);
                gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
                graphFactors.add(priorPose);
                
                prevVel_ = gtsam::Vector3(0, 0, 0);
                gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
                graphFactors.add(priorVel);
                
                prevBias_ = gtsam::imuBias::ConstantBias();
                gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
                graphFactors.add(priorBias);
                
                graphValues.insert(X(0), prevPose_);
                graphValues.insert(V(0), prevVel_);
                graphValues.insert(B(0), prevBias_);
                
                optimizer.update(graphFactors, graphValues);
                graphFactors.resize(0);
                graphValues.clear();

                wheelInertialIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
                wheelInertialIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
                
                key = 1;
                systemInitialized = true;
                
                RCLCPP_INFO(get_logger(), "System initialized");
                return;
            }

            // Integrate measurements
            while (!imuQueOpt.empty() && !wheelOdomQueOpt.empty())
            {
                double imuTime = stamp2Sec(imuQueOpt.front().header.stamp);
                double wheelTime = stamp2Sec(wheelOdomQueOpt.front().header.stamp);
                
                if (imuTime < currentCorrectionTime && wheelTime < currentCorrectionTime)
                {
                    sensor_msgs::msg::Imu& imu = imuQueOpt.front();
                    nav_msgs::msg::Odometry& wheel = wheelOdomQueOpt.front();
                    
                    double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                    
                    double wheelLinearX = wheel.twist.twist.linear.x;
                    double wheelAngularZ = wheel.twist.twist.angular.z;
                    
                    gtsam::Vector3 gyro(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
                    
                    wheelInertialIntegratorOpt_->integrateMeasurement(
                        dt, gyro, wheelLinearX, wheelAngularZ);
                    
                    lastImuT_opt = imuTime;
                    lastWheelT_opt = wheelTime;
                    
                    imuQueOpt.pop_front();
                    wheelOdomQueOpt.pop_front();
                }
                else
                    break;
            }

            // Add factors
            const PreintegratedWheelInertialMeasurements& preint_wheel = *wheelInertialIntegratorOpt_;
            
            gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_wheel);
            graphFactors.add(imu_factor);
            
            graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
                B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                gtsam::noiseModel::Diagonal::Sigmas(sqrt(preint_wheel.deltaTij()) * noiseModelBetweenBias)));

            gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
            graphFactors.add(pose_factor);

            gtsam::NavState propState_ = wheelInertialIntegratorOpt_->predict(prevState_, prevBias_);
            graphValues.insert(X(key), propState_.pose());
            graphValues.insert(V(key), propState_.v());
            graphValues.insert(B(key), prevBias_);

            // Optimize
            optimizer.update(graphFactors, graphValues);
            optimizer.update();
            graphFactors.resize(0);
            graphValues.clear();

            gtsam::Values result = optimizer.calculateEstimate();
            prevPose_  = result.at<gtsam::Pose3>(X(key));
            prevVel_   = result.at<gtsam::Vector3>(V(key));
            prevState_ = gtsam::NavState(prevPose_, prevVel_);
            prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));

            wheelInertialIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            if (failureDetection(prevVel_, prevBias_))
            {
                resetParams();
                return;
            }

            // Update for IMU odometry
            prevStateOdom = prevState_;
            prevBiasOdom  = prevBias_;
            
            while (!imuQueImu.empty() && stamp2Sec(imuQueImu.front().header.stamp) < currentCorrectionTime)
            {
                imuQueImu.pop_front();
            }
            while (!wheelOdomQueImu.empty() && stamp2Sec(wheelOdomQueImu.front().header.stamp) < currentCorrectionTime)
            {
                wheelOdomQueImu.pop_front();
            }

            if (!imuQueImu.empty() && !wheelOdomQueImu.empty())
            {
                wheelInertialIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
                
                lastImuT_imu = -1;
                lastWheelT_imu = -1;
                
                size_t imuIdx = 0;
                size_t wheelIdx = 0;
                
                while (imuIdx < imuQueImu.size() && wheelIdx < wheelOdomQueImu.size())
                {
                    sensor_msgs::msg::Imu& imu = imuQueImu[imuIdx];
                    nav_msgs::msg::Odometry& wheel = wheelOdomQueImu[wheelIdx];
                    
                    double imuTime = stamp2Sec(imu.header.stamp);
                    double wheelTime = stamp2Sec(wheel.header.stamp);
                    
                    if (imuTime <= currentCorrectionTime)
                    {
                        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
                        
                        double wheelLinearX = wheel.twist.twist.linear.x;
                        double wheelAngularZ = wheel.twist.twist.angular.z;
                        gtsam::Vector3 gyro(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
                        
                        wheelInertialIntegratorImu_->integrateMeasurement(
                            dt, gyro, wheelLinearX, wheelAngularZ);
                        
                        lastImuT_imu = imuTime;
                        lastWheelT_imu = wheelTime;
                        
                        imuIdx++;
                        
                        if (wheelTime < imuTime - 0.005)
                        {
                            wheelIdx++;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                
                if (imuIdx > 0)
                {
                    imuQueImu.erase(imuQueImu.begin(), imuQueImu.begin() + imuIdx);
                }
                if (wheelIdx > 0)
                {
                    wheelOdomQueImu.erase(wheelOdomQueImu.begin(), wheelOdomQueImu.begin() + wheelIdx);
                }
            }

            ++key;
            doneFirstOpt = true;
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception in odometryHandler: %s", e.what());
        }
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw)
    {
        if (!imu_raw || !wheelInertialIntegratorImu_) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(mtx);

        try {
            sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);
            
            imuQueOpt.push_back(thisImu);
            imuQueImu.push_back(thisImu);

            if (!doneFirstOpt)
                return;

            double imuTime = stamp2Sec(thisImu.header.stamp);
            double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
            lastImuT_imu = imuTime;
            
            gtsam::Vector3 gyro(thisImu.angular_velocity.x, 
                                thisImu.angular_velocity.y, 
                                thisImu.angular_velocity.z);
            
            double wheelLinearX = 0.0;
            double wheelAngularZ = 0.0;
            
            if (!wheelOdomQueImu.empty())
            {
                nav_msgs::msg::Odometry& wheel = wheelOdomQueImu.front();
                double wheelTime = stamp2Sec(wheel.header.stamp);
                
                wheelLinearX = wheel.twist.twist.linear.x;
                wheelAngularZ = wheel.twist.twist.angular.z;
                
                double wheelPeriod = 0.01;
                if (imuTime - wheelTime > wheelPeriod * 1.5)
                {
                    while (!wheelOdomQueImu.empty())
                    {
                        double frontWheelTime = stamp2Sec(wheelOdomQueImu.front().header.stamp);
                        if (imuTime - frontWheelTime > wheelPeriod * 1.5)
                        {
                            wheelOdomQueImu.pop_front();
                        }
                        else
                        {
                            wheel = wheelOdomQueImu.front();
                            wheelLinearX = wheel.twist.twist.linear.x;
                            wheelAngularZ = wheel.twist.twist.angular.z;
                            lastWheelT_imu = frontWheelTime;
                            break;
                        }
                    }
                }
                else if (imuTime - wheelTime < -wheelPeriod * 1.5)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                        "Wheel timestamp ahead: %.3fs", wheelTime - imuTime);
                }
                else
                {
                    lastWheelT_imu = wheelTime;
                }
                
                while (!wheelOdomQueImu.empty() && 
                       stamp2Sec(wheelOdomQueImu.front().header.stamp) < imuTime - 0.1)
                {
                    wheelOdomQueImu.pop_front();
                }
            }
            
            wheelInertialIntegratorImu_->integrateMeasurement(
                dt, gyro, wheelLinearX, wheelAngularZ);

            gtsam::NavState currentState = wheelInertialIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

            auto odometry = nav_msgs::msg::Odometry();
            odometry.header.stamp = thisImu.header.stamp;
            odometry.header.frame_id = odometryFrame;
            odometry.child_frame_id = "odom_imu";

            gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
            gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

            odometry.pose.pose.position.x = lidarPose.translation().x();
            odometry.pose.pose.position.y = lidarPose.translation().y();
            odometry.pose.pose.position.z = lidarPose.translation().z();
            odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
            odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
            odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
            odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();
            
            odometry.twist.twist.linear.x = currentState.velocity().x();
            odometry.twist.twist.linear.y = currentState.velocity().y();
            odometry.twist.twist.linear.z = currentState.velocity().z();
            odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
            odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
            odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
            
            pubImuOdometry->publish(odometry);
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception in imuHandler: %s", e.what());
        }
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            RCLCPP_WARN(get_logger(), "Large velocity, reset!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            RCLCPP_WARN(get_logger(), "Large bias, reset!");
            return true;
        }

        return false;
    }
};

int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor e;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Creating nodes...");
    
    auto ImuP = std::make_shared<IMUPreintegration>(options);
    auto TF = std::make_shared<TransformFusion>(options);
    
    e.add_node(ImuP);
    e.add_node(TF);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Wheel-Inertial Preintegration Started.\033[0m");

    e.spin();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}