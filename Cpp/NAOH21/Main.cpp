#include <iostream>
#include <vector>
#include "NAOKinematics.h"
#include "KMat.h"
using namespace std;
int main(){
	NAOKinematics *nkin = new NAOKinematics();

	NAOKinematics::FKvars output1, output2, output3, output4, output5;
	std::vector<double> flh,frh,fll,frl,fc,empty;
	double pi = KMath::KMat::transformations::PI;

	//Left Hand
	flh.push_back(0.1);flh.push_back(pi/4);flh.push_back(pi/3);flh.push_back(-0.1);
	output1 = nkin->filterForwardFromTo("Torso","LeftArm",empty,flh);
	
	//Right Hand
	frh.push_back(0.1);frh.push_back(-pi/4);frh.push_back(pi/3);frh.push_back(0.1);
	output2 = nkin->filterForwardFromTo("Torso","RightArm",empty,frh);
	
	//Left Leg
	fll.push_back(0);fll.push_back(0);fll.push_back(0);fll.push_back(1.2);fll.push_back(0.1);fll.push_back(0);
	output3 = nkin->filterForwardFromTo("Torso","LeftLeg",empty,fll);
	
	//Right Leg
	frl.push_back(-pi/10);frl.push_back(0);frl.push_back(0);frl.push_back(pi/2);frl.push_back(0);frl.push_back(0);
	output4 = nkin->filterForwardFromTo("Torso","RightLeg",empty,frl);
	
	//Camera
	fc.push_back(pi/2);fc.push_back(pi/3);
	output5 = nkin->filterForwardFromTo("Torso","CameraBot",empty,fc);
	
	std::cout << "x = " << output1.pointX << " y = " << output1.pointY << " z = " << output1.pointZ << " ax = " << output1.angleX << " ay = " << output1.angleY << " az = " << output1.angleZ << std::endl;
	std::cout << "x = " << output2.pointX << " y = " << output2.pointY << " z = " << output2.pointZ << " ax = " << output2.angleX << " ay = " << output2.angleY << " az = " << output2.angleZ << std::endl;
	std::cout << "x = " << output3.pointX << " y = " << output3.pointY << " z = " << output3.pointZ  << " ax = " << output3.angleX << " ay = " << output3.angleY << " az = " << output3.angleZ << std::endl;
	std::cout << "x = " << output4.pointX << " y = " << output4.pointY << " z = " << output4.pointZ << " ax = " << output4.angleX << " ay = " << output4.angleY << " az = " << output4.angleZ << std::endl;
	
	std::cout << "x = " << output5.pointX << " y = " << output5.pointY << " z = " << output5.pointZ << " angleZ " << output5.angleZ << " angleY " << output5.angleY << " angleX " << output5.angleX << std::endl;
	
	
	
	vector<vector<double> > result;

	result = nkin->inverseLeftHand(output1.pointX,output1.pointY,output1.pointZ,output1.angleX,output1.angleY,output1.angleZ);
	if(!result.empty())
		cout << "--Solution exists 1" << endl;
	result = nkin->inverseRightHand(output2.pointX,output2.pointY,output2.pointZ,output2.angleX,output2.angleY,output2.angleZ);
	if(!result.empty())
		cout << "--Solution exists 2" << endl;
	result = nkin->inverseLeftLeg(output3.pointX,output3.pointY,output3.pointZ,output3.angleX,output3.angleY,output3.angleZ);
	if(!result.empty())
		cout << "--Solution exists 3" << endl;
	result = nkin->inverseRightLeg(output4.pointX,output4.pointY,output4.pointZ,output4.angleX,output4.angleY,output4.angleZ);
	if(!result.empty())
		cout << "--Solution exists 4" << endl;
	return 0;
}
