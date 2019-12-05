/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef VISO2_WITH_IMU_TASK_TASK_HPP
#define VISO2_WITH_IMU_TASK_TASK_HPP

#include "viso2_with_imu/TaskBase.hpp"
#include "base-logging/Logging.hpp"
#include <ctime>

namespace viso2_with_imu {

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
		base::samples::RigidBodyState delta_pose, pose_out, 
			previous_imu_pose, imu_pose, imu_extra_pose, previous_imu_extra_pose,
			reset_pose;
		Eigen::Affine3d pose; //accumulated pose
		double gyro_offset;
		bool pose_valid, start;
        std::clock_t begin;

    protected:
        virtual void delta_pose_samples_inTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &delta_pose_samples_in_sample);
        virtual void pose_samples_imuTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_imu_sample);

    public:
        Task(std::string const& name = "viso2_with_imu::Task");

        Task(std::string const& name, RTT::ExecutionEngine* engine);

    	~Task();

        bool configureHook();

        bool startHook();

        void updateHook();

        void errorHook();

        void stopHook();

        void cleanupHook();
        
        bool resetPose();
    };
}

#endif

