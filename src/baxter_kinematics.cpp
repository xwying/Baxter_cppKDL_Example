#include "baxter_cppkdl/baxter_cppkdl.h"

using namespace CPPKDL;

int main(int argc, char **argv)
{
  int i;
  ros::init(argc, argv, "baxter_cppKDL_example");
  CPPKDL::baxter_kinematics kin = baxter_kinematics("right");
  double fk_result[7];
  double ik_position_result[7];
  double ik_pose_result[7];
//===================FK=======================
  printf("\nBaxter Position FK:\n");
  //double joint_values[7] = {-4.797576,0.879523,-8.680395,-4.213394,-0.527894,22.773351,34.232391};
  kin.forward_kinematics(fk_result);
  for(i=0;i<7;i++)
  {
    printf("%f,",fk_result[i]);
  }
  printf("\n");
//================IK=============================
  double pos[3] = {0.582583, -0.180819, 0.216003};
  double rot[4] = {0.03085, 0.9945, 0.0561, 0.0829};
//=================IK with position=============
  printf("\nBaxter Position IK:\n");
  kin.inverse_kinematics(ik_position_result,pos);
  for(i=0;i<7;i++)
  {
    printf("%f,",ik_position_result[i]);
  }
  printf("\n");

//=================IK with position and orientation=============
  printf("\nBaxter Pose IK:\n");
  kin.inverse_kinematics(ik_pose_result,pos,rot);
  for(i=0;i<7;i++)
  {
    printf("%f,",ik_pose_result[i]);
  }
  printf("\n");


  return 0;
}
