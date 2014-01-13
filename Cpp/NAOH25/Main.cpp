#include <iostream>
#include <vector>
#include "NAOKinematics.h"
#include "KMat.hpp"

using namespace std;
using namespace KMath::KMat;
using namespace KDeviceLists;

int main(){
	NAOKinematics nkin;
	NAOKinematics::kmatTable output1, output2, output3, output4, output5,T;
	
	std::vector<float> joints(NUMOFJOINTS);
	double pi = KMath::KMat::transformations::PI;
	//Left Hand
	joints[L_ARM+SHOULDER_PITCH]=0.2;
	joints[L_ARM+SHOULDER_ROLL]=0.1;
	joints[L_ARM+ELBOW_YAW]=0;
	joints[L_ARM+ELBOW_ROLL]=0;
	joints[L_ARM+WRIST_YAW]=0.0;
	//Right Hand
	joints[R_ARM+SHOULDER_PITCH]=M_PI_2;
	joints[R_ARM+SHOULDER_ROLL]=-M_PI_4;
	joints[R_ARM+ELBOW_YAW]=0;
	joints[R_ARM+ELBOW_ROLL]=0;
	joints[R_ARM+WRIST_YAW]=M_PI/3535;
	//Left Leg
	joints[L_LEG+HIP_YAW_PITCH]=0;
	joints[L_LEG+HIP_ROLL]=0;
	joints[L_LEG+HIP_PITCH]=0;
	joints[L_LEG+KNEE_PITCH]=M_PI/4;
	joints[L_LEG+ANKLE_PITCH]=-M_PI/4;
	joints[L_LEG+ANKLE_ROLL]=0;
	//Right Leg
	joints[R_LEG+HIP_YAW_PITCH]=0;
	joints[R_LEG+HIP_ROLL]=0;
	joints[R_LEG+HIP_PITCH]=0;
	joints[R_LEG+KNEE_PITCH]=0;
	joints[R_LEG+ANKLE_PITCH]=0;
	joints[R_LEG+ANKLE_ROLL]=0;
	//Head
	joints[HEAD+YAW]=M_PI_2;
	joints[HEAD+PITCH]=0;
	
	//Now we can set the joints to the class
	nkin.setJoints(joints);
	
	output1= nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_L_ARM);
	
	//Right Hand
	output2 = nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_R_ARM);
	
	//Left Leg
	output3 = nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_L_LEG);
	
	//Right Leg
	output4 = nkin.getForwardEffector((NAOKinematics::Effectors)CHAIN_R_LEG);
	
	//Camera
	output5 = nkin.getForwardEffector(NAOKinematics::EFF_CAMERA_BOT);
	
	std::cout << "x = " << output1(0,3) << " y = " << output1(1,3) << " z = " << output1(2,3) <<  std::endl;
	std::cout << "x = " << output2(0,3) << " y = " << output2(1,3) << " z = " << output2(2,3) <<  std::endl;
	std::cout << "x = " << output3(0,3) << " y = " << output3(1,3) << " z = " << output3(2,3) <<  std::endl;
	std::cout << "x = " << output4(0,3) << " y = " << output4(1,3) << " z = " << output4(2,3) <<  std::endl;
	std::cout << "x = " << output5(0,3) << " y = " << output5(1,3) << " z = " << output5(2,3) <<  std::endl;
	
	vector<vector<float> > result;

	result = nkin.inverseLeftHand(output1);
	if(!result.empty()){
		cout << "--Solution exists 1" << endl;
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
		}
		cout << endl;
		//Under construction
		/*joints[L_ARM+SHOULDER_PITCH]=0;
		joints[L_ARM+SHOULDER_ROLL]=0;
		joints[L_ARM+ELBOW_YAW]=0;
		joints[L_ARM+ELBOW_ROLL]=0;
		joints[L_ARM+WRIST_YAW]=0.0;
		nkin.setJoints(joints);
		result = nkin.jacobianInverseLeftHand(output1);
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
		}
		cout << endl;*/
	}
	result = nkin.inverseRightHand(output2);
	if(!result.empty()){
		cout << "--Solution exists 2" << endl;
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
		}
		cout << endl;
		//Under construction
		/*
		joints[R_ARM+SHOULDER_PITCH]=0;
		joints[R_ARM+SHOULDER_ROLL]=0;
		joints[R_ARM+ELBOW_YAW]=0;
		joints[R_ARM+ELBOW_ROLL]=0;
		joints[R_ARM+WRIST_YAW]=0.0;
		nkin.setJoints(joints);
		result = nkin.jacobianInverseRightHand(output2);
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
		}
		cout << endl;
		*/
	}
	result = nkin.inverseLeftLeg(output3);
	if(!result.empty()){
		cout << "--Solution exists 3" << endl;
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
		}
		cout << endl;
		joints[L_LEG+HIP_YAW_PITCH]=0;
		joints[L_LEG+HIP_ROLL]=0;
		joints[L_LEG+HIP_PITCH]=0;
		joints[L_LEG+KNEE_PITCH]=0;
		joints[L_LEG+ANKLE_PITCH]=0;
		joints[L_LEG+ANKLE_ROLL]=0;
		nkin.setJoints(joints);
		result = nkin.jacobianInverseLeftLeg(output3);
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
		}
		cout << endl;
	}
	result = nkin.inverseRightLeg(output4);
	if(!result.empty()){
		cout << "--Solution exists 4" << endl;
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
		}
		cout << endl;
		joints[R_LEG+HIP_YAW_PITCH]=0;
		joints[R_LEG+HIP_ROLL]=0;
		joints[R_LEG+HIP_PITCH]=0;
		joints[R_LEG+KNEE_PITCH]=0;
		joints[R_LEG+ANKLE_PITCH]=0;
		joints[R_LEG+ANKLE_ROLL]=0;
		nkin.setJoints(joints);
		result = nkin.jacobianInverseRightLeg(output4);
		for(int j=0; j<result[0].size(); j++){
			cout << "angle" << j << " = " << result[0][j] << " ";
		}
		cout << endl;
	}
	return 0;
}
