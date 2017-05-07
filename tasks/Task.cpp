/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace viso2_with_imu;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    
	// init pose for test purpose
	pose = Eigen::Affine3d::Identity();
	Eigen::Quaternion <double> orientation_init(Eigen::AngleAxisd(1.69, Eigen::Vector3d::UnitZ())*
									Eigen::AngleAxisd(-0.001745328, Eigen::Vector3d::UnitY()) *
									Eigen::AngleAxisd(0.054977825, Eigen::Vector3d::UnitX()));
	Eigen::Translation<double,3> translation_init(-5.44,1.33,-1.77);
	pose = translation_init * orientation_init;
       
	// Rigid body state output initialization
	pose_out.invalidate();
	pose_out.orientation = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
	pose_out.position = Eigen::Vector3d::Zero();
	pose_out.velocity = Eigen::Vector3d::Zero();
	
	imu_pose.invalidate();
	imu_pose.orientation = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
	imu_pose.position = Eigen::Vector3d::Zero();
	imu_pose.velocity = Eigen::Vector3d::Zero();
		
	previous_imu_pose.invalidate();
	previous_imu_pose.orientation = orientation_init;
	previous_imu_pose.position = Eigen::Vector3d::Zero();
	previous_imu_pose.velocity = Eigen::Vector3d::Zero();
	
	imu_extra_pose.invalidate();
	imu_extra_pose.orientation = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
	imu_extra_pose.position = Eigen::Vector3d::Zero();
	imu_extra_pose.velocity = Eigen::Vector3d::Zero();
		
	previous_imu_extra_pose.invalidate();
	previous_imu_extra_pose.orientation = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
	previous_imu_extra_pose.position = Eigen::Vector3d::Zero();
	previous_imu_extra_pose.velocity = Eigen::Vector3d::Zero();
	
	
	// frame def for transformer
	pose_out.sourceFrame = "body";
    pose_out.targetFrame = "visiodom_world";
	
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    
    if(_delta_pose_samples_in.read(delta_pose) == RTT::NewData)
    {
		// read IMU sensors
		double yaw, pitch, roll;
		_pose_samples_imu.read(imu_pose); // non blocking because IMU is fast enough
		
		// TODO this IMU rotation must be sorted out one day
		yaw = delta_pose.getYaw();
		//pitch = (-imu_pose.getPitch())-(-previous_imu_pose.getPitch());
		//roll = (-imu_pose.getRoll())-(-previous_imu_pose.getRoll());
		pitch = imu_pose.getPitch()-previous_imu_pose.getPitch();
		roll = imu_pose.getRoll()-previous_imu_pose.getRoll();
		
		if(_pose_samples_imu_extra.connected()) // read second imu specifically for yaw
		{
			yaw = imu_extra_pose.getYaw()-previous_imu_extra_pose.getYaw();
		}	
		// from rigidbody state to eigen affine3d 	
		// combine delta pitch roll from imu with the ones of viso2

		Eigen::Quaternion <double> orientation_delta(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*
                                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
										Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
		
		Eigen::Translation<double,3> translation_delta(delta_pose.position.x(),
													delta_pose.position.y(),
													delta_pose.position.z());
		Eigen::Transform<double,3,Eigen::Affine> eigen_delta_pose = orientation_delta * translation_delta;

		// update pose
		pose = pose * eigen_delta_pose;

		// revert pitch and roll to imu pitch and roll by reading imu
		previous_imu_pose = imu_pose;
		previous_imu_extra_pose = imu_extra_pose;
		Eigen::Quaternion<double> attitude(pose.rotation());
		Eigen::Vector3d euler = attitude.toRotationMatrix().eulerAngles(2, 1, 0);
		//yaw = euler[0];
		//pitch = -imu_pose.getPitch();
		//roll = -imu_pose.getRoll();
		
		// Store in RigidBodyState to port out the pose					
		/*pose_out.orientation = Eigen::Quaternion <double> (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*
                                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
										Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));*/
		pose_out.orientation = attitude;
		
		
        pose_out.time = delta_pose.time;
        pose_out.position = pose.translation();
        
        _pose_samples_out.write(pose_out);
	}

}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
