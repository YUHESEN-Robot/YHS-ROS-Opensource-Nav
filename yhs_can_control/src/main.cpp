#include "yhs_dt_can_control.h"
#include "yhs_mk_can_control.h"
#include "yhs_fw_can_control.h"
#include "yhs_fr_can_control.h"

int main(int argc, char ** argv)
{

	ros::init(argc, argv, "yhs_can_control_node");
	
  ros::NodeHandle nh("~");
  
  std::string chassis_type;
  nh.param("/common/chassis_type",chassis_type,std::string("FW"));
  
	std::shared_ptr<yhs_chassis::CanControl> cc;
	
  if (chassis_type == "DT") {
      cc = std::make_shared<can_control::DgtCanControl>();
  } else if (chassis_type == "FW") {
      cc = std::make_shared<can_control::FwCanControl>();
  } else if (chassis_type == "FR") {
      cc = std::make_shared<can_control::FrCanControl>();
  } else if (chassis_type == "MK") {
      cc = std::make_shared<can_control::MkCanControl>();
  } else {
      ROS_ERROR( "Invalid chassis type. Please specify DGT, FW, FR, or MK.");
      return 1;
  }

  ROS_INFO("Chassis type: %s", chassis_type.c_str());


	if (!cc->Run())
	{
		ROS_ERROR("Failed to initialize yhs_can_control_node");
		return 0;
	}

	ROS_INFO("yhs_can_control_node initialized successfully");
	
	ros::MultiThreadedSpinner spinner(3);

	spinner.spin();
	
	cc->Stop();
	
	std::cout << "yhs_can_control_node stopped" << std::endl;
	return 0;
	
}
