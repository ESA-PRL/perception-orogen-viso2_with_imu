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
	/*pose = Eigen::Affine3d::Identity();
	Eigen::Quaternion <double> orientation_init(Eigen::AngleAxisd(1.54, Eigen::Vector3d::UnitZ())*
									Eigen::AngleAxisd(-0.0087, Eigen::Vector3d::UnitY()) *
									Eigen::AngleAxisd(0.0157, Eigen::Vector3d::UnitX()));
	Eigen::Translation<double,3> translation_init(-4.58,6.668,1.528);
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
	previous_imu_extra_pose.velocity = Eigen::Vector3d::Zero();*/
	
	
	// frame def for transformer
	pose_out.sourceFrame = "body";
    pose_out.targetFrame = "visiodom_world";
	
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
        
    reset_pose = _initial_pose.get();
	this->resetPose();
        
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
        
    if(_reset_pose.connected())
    {
		if(_reset_pose.read(reset_pose) == RTT::NewData)
		{
			this->resetPose();
		}
	}
    
    if(_delta_pose_samples_in.read(delta_pose) == RTT::NewData)
    {
		// read IMU sensors
		double delta_yaw, delta_pitch, delta_roll;
		_pose_samples_imu.read(imu_pose); // non blocking because IMU is fast enough
		
		// TODO this IMU rotation must be sorted out one day
		delta_yaw = delta_pose.getYaw();
		delta_pitch = (-imu_pose.getPitch())-(-previous_imu_pose.getPitch());
		delta_roll = (-imu_pose.getRoll())-(-previous_imu_pose.getRoll());
		previous_imu_pose = imu_pose;
		//pitch = imu_pose.getPitch()-previous_imu_pose.getPitch();
		//roll = imu_pose.getRoll()-previous_imu_pose.getRoll();
		
		if(_pose_samples_imu_extra.connected()) // read second imu specifically for yaw
		{
			_pose_samples_imu_extra.read(imu_extra_pose); // non blocking because IMU is fast enough
			delta_yaw = imu_extra_pose.getYaw() - previous_imu_extra_pose.getYaw();
			previous_imu_extra_pose = imu_extra_pose;
		}	
		// from rigidbody state to eigen affine3d 	
		// combine delta pitch roll from imu with the ones of viso2

		Eigen::Quaternion <double> orientation_delta(Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ())*
                                        Eigen::AngleAxisd(delta_pitch, Eigen::Vector3d::UnitY()) *
										Eigen::AngleAxisd(delta_roll, Eigen::Vector3d::UnitX()));
										
		
		Eigen::Translation<double,3> translation_delta(delta_pose.position.x(),
													delta_pose.position.y(),
													delta_pose.position.z());
		Eigen::Transform<double,3,Eigen::Affine> eigen_delta_pose = orientation_delta * translation_delta;		
		
		// update pose
		pose = pose * eigen_delta_pose;
		
		Eigen::Quaternion<double> attitude(pose.rotation());		
		
        pose_out.time = delta_pose.time;
        pose_out.position = pose.translation();
		pose_out.orientation = attitude;

		// force attitude to IMU attitude
		Eigen::Quaternion <double> imu_orientation;
										
		if(_pose_samples_imu_extra.connected()) // read second imu specifically for yaw
		{
			Eigen::Quaternion <double> tmp_orient(Eigen::AngleAxisd(imu_extra_pose.getYaw()+gyro_offset, Eigen::Vector3d::UnitZ())*
									Eigen::AngleAxisd(-imu_pose.getPitch(), Eigen::Vector3d::UnitY()) *
									Eigen::AngleAxisd(-imu_pose.getRoll(), Eigen::Vector3d::UnitX()));
			imu_orientation = tmp_orient;
		}	
		else
		{
			Eigen::Quaternion <double> tmp_orient(Eigen::AngleAxisd(pose_out.getYaw(), Eigen::Vector3d::UnitZ())*
                                        Eigen::AngleAxisd(-imu_pose.getPitch(), Eigen::Vector3d::UnitY()) *
										Eigen::AngleAxisd(-imu_pose.getRoll(), Eigen::Vector3d::UnitX()));
			imu_orientation = tmp_orient;
		}
		
		Eigen::Translation<double,3> translation_total(pose.translation());

		pose = translation_total * imu_orientation;

        
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

void Task::resetPose()
{
	// init pose for test purpose
	pose = Eigen::Affine3d::Identity();
	Eigen::Quaternion <double> orientation_init(Eigen::AngleAxisd(reset_pose.getYaw(), Eigen::Vector3d::UnitZ())*
                                        Eigen::AngleAxisd(-imu_pose.getPitch(), Eigen::Vector3d::UnitY()) *
										Eigen::AngleAxisd(-imu_pose.getRoll(), Eigen::Vector3d::UnitX()));
	
	Eigen::Translation<double,3> translation_init(reset_pose.position.x(),
												reset_pose.position.y(),
												reset_pose.position.z());

										
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
	previous_imu_pose.orientation = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
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
	
	// Init previous IMU with current IMU status and load gyro_offset as well for absolute gyro position
	while(_pose_samples_imu.read(previous_imu_pose) != RTT::NewData)
		;
    while(_pose_samples_imu_extra.read(previous_imu_extra_pose) != RTT::NewData)
		;
	
	gyro_offset = reset_pose.getYaw()-previous_imu_extra_pose.getYaw();
}
