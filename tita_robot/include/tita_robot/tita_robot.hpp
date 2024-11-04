#include "can_receiver.hpp"
#include "can_sender.hpp"

class tita_robot
{
public:
    enum ReadyNext
    {
        READY_WAITING = 0x00U,
        AUTO_LOCOMOTION = 0x01U,
        FORCE_LOCOMOTION = 0x02U,
        FORCE_DIRECT = 0x03U,
        MOTOR_ZERO_CAL = 0x04U,
        BOOT_RECOVERY_MODE = 0x05U,
        ESTOP_MODE = 0x06U,
    };
    struct ChannelInput
    {
        uint32_t timestamp;
        float forward;
        float yaw;
        float pitch;
        float roll;
        float height;
        float split;
        float tilt;
        float forward_accel;
        float yaw_accel;
    };
    tita_robot(size_t num_motors)
    {
        // Initialize the CAN receiver
        can_receiver_ = std::make_unique<can_device::MotorsImuCanReceiveApi>(num_motors);
        can_sender_ = std::make_unique<can_device::MotorsCanSendApi>(num_motors);
        motor_num_ = num_motors;
    }

    /**
     * @brief Get the current joint positions in joint space.
     * @return std::vector<double>: current joint positions in radians.
     */
    std::vector<double> get_joint_q() const;

    /**
     * @brief Get the current joint velocities in joint space.
     * @return std::vector<double>: current joint velocities in radians per second.
     */
    std::vector<double> get_joint_v() const;

    /**
     * @brief Get the current joint torques in joint space.
     * @return std::vector<double>: current joint torques in Newton meters.
     */
    std::vector<double> get_joint_t() const;

    /**
     * @brief Get the current joint status.
     * @note Joints status now is in bool value, true means the joint is online(TODO).
     * @return std::vector<uint8_t>: current joint status.
     */
    std::vector<uint8_t> get_joint_status() const;

    /**
     * @brief Get the current imu quaternion of mcu.
     * @note Quaternion sequence is x y z w.
     * @return std::array<double, 4>: current quaternion.
     */
    std::array<double, 4> get_imu_quaternion() const; // x y z w

    /**
     * @brief Get the current imu acceleration of mcu.
     * @return std::array<double, 3>: current acceleration.
     */
    std::array<double, 3> get_imu_acceleration() const;

    /**
     * @brief Get the current imu angular velocity of mcu.
     * @return std::array<double, 3>: current angular velocity.
     */
    std::array<double, 3> get_imu_angular_velocity() const;

    /**
     * @brief Set the target joint feed-forward torques.
     * @param t the target joint feed-forward torques.
     * @return return true if the target is set successfully.
     */
    bool set_target_joint_t(const std::vector<double> &t);

    /**
     * @brief MIT control method. Set the target joint positions, velocities, kp, kd and feed-forward torques of the
     motors.
     * @param q the target joint positions in radians.
     * @param v the target joint velocities in radians per second.
     * @param kp the target joint proportional gains.
     * @param kd the target joint derivative gains.
     * @param t the target joint feed-forward torques.
     *
     * @return return true if the target is set successfully
     */
    bool set_target_joint_mit(const std::vector<double> &q, const std::vector<double> &v, const std::vector<double> &kp, const std::vector<double> &kd, const std::vector<double> &t);

    // /**
    //  * @brief Set mcu board control mode, maybe remove in the future.
    //  * @param mode the target mcu mode, option: READY_WAITING AUTO_LOCOMOTION FORCE_DIRECT.
    //  *
    //  * @return return true if the target is set successfully
    //  */
    // bool set_board_mode(ReadyNext mode);
    /**
     * @brief Set mcu board can direct drive motors, default is auto_locomotion(use mcu control).
     * @param if_sdk true means use control motors sdk, false means use mcu control.
     *
     * @return return true if the target is set successfully
     */
    bool set_motors_sdk(bool if_sdk);
    /**
     * @brief Set rc input to mcu. Only used in board mode is AUTO_LOCOMOTION.
     * @param forward the target forward.
     * @param yaw the target yaw.
     * @param pitch the target pitch.
     * @param roll the target roll.
     * @param height the target height.
     * @param split the target split.
     * @param tilt the target tilt.
     * @param forward_accel the target forward acceleration.
     * @param yaw_accel the target yaw acceleration.
     * @return return true if the target is set successfully
     */
    bool set_rc_input(ChannelInput input);    
    /**
     * @brief Set rc input to mcu. Only used in board mode is AUTO_LOCOMOTION.
     * @param stand true if stand.
     * @return return true if the target is set successfully
     */
    bool set_robot_stand(bool stand);

    /**
     * @brief Set rc input to mcu. Only used in board mode is AUTO_LOCOMOTION.
     * @param jump true to start robot charge, then false change to start jump.
     * @return return true if the target is set successfully
     */
    bool set_robot_jump(bool jump);
    /**
     * @brief Set rc input to mcu. Only used in board mode is AUTO_LOCOMOTION.
     * @return return true if the target is set successfully
     */
    bool set_robot_stop();
private:
    std::unique_ptr<can_device::MotorsImuCanReceiveApi> can_receiver_;
    std::unique_ptr<can_device::MotorsCanSendApi> can_sender_;
    size_t motor_num_;
};

// class tita_robot
