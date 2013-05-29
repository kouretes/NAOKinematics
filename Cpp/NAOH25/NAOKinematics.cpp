#include "NAOKinematics.h"
using namespace KMath;

void NAOKinematics::forwardLeftHand(kmatTable & EndTransf, double ShoulderPitch, double ShoulderRoll, double ElbowYaw, double ElbowRoll, double EwristYaw)
{
	KMatTransf::makeDHTransformation(T1, 0.0, -PI / 2, 0.0, ShoulderPitch);
	KMatTransf::makeDHTransformation(T2, 0.0, PI / 2, 0.0, ShoulderRoll - PI / 2);
	KMatTransf::makeDHTransformation(T3, 0.0, -PI / 2, UpperArmLength, -ElbowYaw);
	KMatTransf::makeDHTransformation(T4, 0.0, PI / 2, 0.0, ElbowRoll);
	KMatTransf::makeDHTransformation(T5, 0.0, -PI / 2, LowerArmLength, -EwristYaw);
	Tend = TBaseLArm;
	Tend *= T1;
	Tend *= T2;
	Tend *= T3;
	Tend *= T4;
	Tend *= T5;
	Tend *= RotLArm;
	Tend *= TEndLArm;
	EndTransf = Tend;
}


void NAOKinematics::forwardRightHand(kmatTable & EndTransf, double ShoulderPitch, double ShoulderRoll, double ElbowYaw, double ElbowRoll, double EwristYaw)
{
	KMatTransf::makeDHTransformation(T1, 0.0, -PI / 2, 0.0, ShoulderPitch);
	KMatTransf::makeDHTransformation(T2, 0.0, PI / 2, 0.0, ShoulderRoll + PI / 2); //Allagh apo matlab
	KMatTransf::makeDHTransformation(T3, 0.0, -PI / 2, -UpperArmLength, ElbowYaw);
	KMatTransf::makeDHTransformation(T4, 0.0, PI / 2, 0.0, ElbowRoll); //Allagh apo matlab
	KMatTransf::makeDHTransformation(T5, 0.0, -PI / 2, -LowerArmLength, EwristYaw);
	Tend = TBaseRArm;
	Tend *= T1;
	Tend *= T2;
	Tend *= T3;
	Tend *= T4;
	Tend *= T5;
	Tend *= RotRArm;
	Tend *= TEndRArm;
	Tend *= RotRArmFix;
	EndTransf = Tend;
}

void  NAOKinematics::forwardLeftLeg(kmatTable & EndTransf, double HipYawPitch, double HipRoll, double HipPitch, double KneePitch, double AnklePitch, double AnkleRoll)
{
	KMatTransf::makeDHTransformation(T1, 0.0, -3 * PI / 4, 0.0, HipYawPitch - PI / 2);
	KMatTransf::makeDHTransformation(T2, 0.0, -PI / 2, 0.0, HipRoll + PI / 4);
	KMatTransf::makeDHTransformation(T3, 0.0, PI / 2, 0.0, HipPitch);
	KMatTransf::makeDHTransformation(T4, -ThighLength, 0.0, 0.0, KneePitch);
	KMatTransf::makeDHTransformation(T5, -TibiaLength, 0.0, 0.0, AnklePitch);
	KMatTransf::makeDHTransformation(T6, 0.0, -PI / 2, 0.0, AnkleRoll);
	Tend = TBaseLLeg;
	Tend *= T1;
	Tend *= T2;
	Tend *= T3;
	Tend *= T4;
	Tend *= T5;
	Tend *= T6;
	Tend *= RotLLeg;
	Tend *= TEndLLeg;
	EndTransf = Tend;
}

void NAOKinematics::forwardRightLeg(kmatTable & EndTransf, double HipYawPitch, double HipRoll, double HipPitch, double KneePitch, double AnklePitch, double AnkleRoll)
{
	KMatTransf::makeDHTransformation(T1, 0.0, -PI / 4, 0.0, HipYawPitch - PI / 2);
	KMatTransf::makeDHTransformation(T2, 0.0, -PI / 2, 0.0, HipRoll - PI / 4); //allagh
	KMatTransf::makeDHTransformation(T3, 0.0, PI / 2, 0.0, HipPitch);
	KMatTransf::makeDHTransformation(T4, -ThighLength, 0.0, 0.0, KneePitch);
	KMatTransf::makeDHTransformation(T5, -TibiaLength, 0.0, 0.0, AnklePitch); //allagh
	KMatTransf::makeDHTransformation(T6, 0.0, -PI / 2, 0.0, AnkleRoll);
	Tend = TBaseBRLeg;
	Tend *= T1;
	Tend *= T2;
	Tend *= T3;
	Tend *= T4;
	Tend *= T5;
	Tend *= T6;
	Tend *= RotRLeg;
	Tend *= TEndRLeg;
	EndTransf = Tend;
}

void NAOKinematics::forwardCamera(kmatTable & EndTransf, double HeadYaw, double HeadPitch, bool topCamera)
{
	KMatTransf::makeDHTransformation(T1, 0.0, 0.0, 0.0, HeadYaw);
	KMatTransf::makeDHTransformation(T2, 0.0, -PI / 2, 0.0, HeadPitch - PI / 2);
	Tend = TBaseHead;
	Tend *= T1;
	Tend *= T2;
	Tend *= RotHead;

	if(!topCamera)
		Tend *= TEndHead1;
	else
		Tend *= TEndHead2;

	EndTransf = Tend;
}

void NAOKinematics::filterForward(kmatTable & Tmatrix, std::string WhatForward, std::vector<double> joints)
{
	Tmatrix.zero();
	std::vector<double>::iterator iter;
	iter = joints.begin();

	if(!WhatForward.compare("CameraTop"))
	{
		double HP = *iter;
		iter++;
		double HY = *iter;
		forwardCamera(Tmatrix, HP, HY, true);
	}
	else if(!WhatForward.compare("CameraBot"))
	{
		double HP = *iter;
		iter++;
		double HY = *iter;
		forwardCamera(Tmatrix, HP, HY, false);
	}
	else if(!WhatForward.compare("LeftArm") || !WhatForward.compare("LeftHand"))
	{
		double SP = *iter;
		iter++;
		double SR = *iter;
		iter++;
		double EY = *iter;
		iter++;
		double ER = *iter;
		iter++;
		double WY = *iter;
		forwardLeftHand(Tmatrix, SP, SR, EY, ER, WY);
	}
	else if(!WhatForward.compare("RightArm") || !WhatForward.compare("RightHand"))
	{
		double SP = *iter;
		iter++;
		double SR = *iter;
		iter++;
		double EY = *iter;
		iter++;
		double ER = *iter;
		iter++;
		double WY = *iter;
		forwardRightHand(Tmatrix, SP, SR, EY, ER, WY);
	}
	else if(!WhatForward.compare("LeftLeg"))
	{
		double HYP = *iter;
		iter++;
		double HR = *iter;
		iter++;
		double HP = *iter;
		iter++;
		double KP = *iter;
		iter++;
		double AP = *iter;
		iter++;
		double AR = *iter;
		forwardLeftLeg(Tmatrix, HYP, HR, HP, KP, AP, AR);
	}
	else if(!WhatForward.compare("RightLeg"))
	{
		double HYP = *iter;
		iter++;
		double HR = *iter;
		iter++;
		double HP = *iter;
		iter++;
		double KP = *iter;
		iter++;
		double AP = *iter;
		iter++;
		double AR = *iter;
		forwardRightLeg(Tmatrix, HYP, HR, HP, KP, AP, AR);
	}

	Tmatrix.check();
}

NAOKinematics::FKvars NAOKinematics::filterForwardFromTo(std::string start, std::string stop, std::vector<double> jointsStart, std::vector<double> jointsEnd)
{
	if(!start.compare("Torso"))
	{
		Tmatrix1.identity();
	}
	else
		filterForward(Tmatrix1, start, jointsStart);

	if(!stop.compare("Torso"))
	{
		Tmatrix2.identity();
	}
	else
		filterForward(Tmatrix2, stop, jointsEnd);

	Tmatrix1.fast_invert();
	Tmatrix1 *= Tmatrix2;
	FKvars FKVariables;
	FKVariables.pointX = Tmatrix1(0, 3);
	FKVariables.pointY = Tmatrix1(1, 3);
	FKVariables.pointZ = Tmatrix1(2, 3);
	FKVariables.angleZ = atan2(Tmatrix1(1, 0), Tmatrix1(0, 0));
	FKVariables.angleY = atan2(-Tmatrix1(2, 0), sqrt(pow(Tmatrix1(2, 1), 2) + pow(Tmatrix1(2, 2), 2)));
	FKVariables.angleX = atan2(Tmatrix1(2, 1), Tmatrix1(2, 2));
	return FKVariables;
}

NAOKinematics::kmatTable NAOKinematics::forwardFromTo(std::string start, std::string stop, std::vector<double> jointsStart, std::vector<double> jointsEnd)
{
	if(!start.compare("Torso"))
	{
		Tmatrix1.identity();
	}
	else
		filterForward(Tmatrix1, start, jointsStart);

	if(!stop.compare("Torso"))
	{
		Tmatrix2.identity();
	}
	else
		filterForward(Tmatrix2, stop, jointsEnd);

	Tmatrix1.fast_invert();
	Tmatrix1 *= Tmatrix2;
	return Tmatrix1;
}

NAOKinematics::FKvars NAOKinematics::calculateCenterOfMass(std::vector<double> allJoints)
{
	kmatTable endTr1, endTr2, endTr3, endTr4, endTr5, endTr6, temp;
	KMath::KMat::GenMatrix<double, 3, 1> lh1, lh2, lh3, lh4, lh5, rh1, rh2, rh3, rh4, rh5, ll1, ll2, ll3, ll4, ll5, ll6, rl1, rl2, rl3, rl4, rl5, rl6, h1, h2, t;
	double PI = KMatTransf::PI;
	
	if(allJoints.size() != 24)
		std::cout << "Kinematics CoM fatal: joints vector not equal to 22" << std::endl;
	
	//Head
	KMatTransf::makeTranslation(endTr1, HeadYawX, HeadYawY, HeadYawZ);
	KMatTransf::makeTranslation(endTr2, HeadPitchX, HeadPitchY, HeadPitchZ);
	KMatTransf::makeTranslation(base, 0.0, 0.0, NeckOffsetZ);
	KMatTransf::makeDHTransformation(T1, 0.0, 0.0, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0, -PI / 2, 0.0, allJoints.front() - PI / 2);
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= endTr1;
	h1 = temp.get_translation();
	h1.scalar_mult(HeadYawMass);
	base *= T2;
	temp = base;
	temp *= RotHead;
	temp *= endTr2;
	h2 = temp.get_translation();
	h2.scalar_mult(HeadPitchMass);
	h1 += h2;

	//Left Hand
	KMatTransf::makeTranslation(endTr1, LShoulderPitchX, LShoulderPitchY, LShoulderPitchZ);
	KMatTransf::makeTranslation(endTr2, LShoulderRollX, LShoulderRollY, LShoulderRollZ);
	KMatTransf::makeTranslation(endTr3, LElbowYawX, LElbowYawY, LElbowYawZ);
	KMatTransf::makeTranslation(endTr4, LElbowRollX, LElbowRollY, LElbowRollZ);
	KMatTransf::makeTranslation(endTr5, LWristYawX, LWristYawY, LWristYawZ);
	KMatTransf::makeTranslation(base, 0.0, ShoulderOffsetY + ElbowOffsetY, ShoulderOffsetZ);
	KMatTransf::makeDHTransformation(T1, 0.0, -PI / 2, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0, PI / 2, 0.0, allJoints.front() - PI / 2);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T3, 0.0, -PI / 2, UpperArmLength, -allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T4, 0.0, PI / 2, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T5, 0.0, -PI / 2, LowerArmLength, -allJoints.front());
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= RotLHshouldP;
	temp *= endTr1;
	lh1 = temp.get_translation();
	lh1.scalar_mult(LShoulderPitchMass);
	base *= T2;
	temp = base;
	temp *= RotLHshoulR;
	temp *= endTr2;
	lh2 = temp.get_translation();
	lh2.scalar_mult(LShoulderRollMass);
	base *= T3;
	temp = base;
	temp *= RotLHelbowY;
	temp *= endTr3;
	lh3 = temp.get_translation();
	lh3.scalar_mult(LElbowYawMass);
	base *= T5;
	temp = base;
	temp *= RotLHelbowR;
	temp *= endTr4;
	lh4 = temp.get_translation();
	lh4.scalar_mult(LElbowRollMass);
	base *= T5;
	temp = base;
	temp *= RotLArm;
	temp *= endTr5;
	lh5 = temp.get_translation();
	lh5.scalar_mult(LWristYawMass);
	lh1 += lh2;
	lh1 += lh3;
	lh1 += lh4;
	lh1 += lh5;

	//Left Leg
	KMatTransf::makeTranslation(endTr1, LHipYawPitchX, LHipYawPitchY, LHipYawPitchZ);
	KMatTransf::makeTranslation(endTr2, LHipRollX, LHipRollY, LHipRollZ);
	KMatTransf::makeTranslation(endTr3, LHipPitchX, LHipPitchY, LHipPitchZ);
	KMatTransf::makeTranslation(endTr4, LKneePitchX, LKneePitchY, LKneePitchZ);
	KMatTransf::makeTranslation(endTr5, LAnklePitchX, LAnklePitchY, LAnklePitchZ);
	KMatTransf::makeTranslation(endTr6, LAnkleRollX, LAnkleRollY, LAnkleRollZ);
	KMatTransf::makeTranslation(base, 0.0, HipOffsetY, -HipOffsetZ);
	KMatTransf::makeDHTransformation(T1, 0.0, -3 * PI / 4, 0.0, allJoints.front() - PI / 2);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0, -PI / 2, 0.0, allJoints.front() + PI / 4);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T3, 0.0, PI / 2, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T4, -ThighLength, 0.0, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T5, -TibiaLength, 0.0, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T6, 0.0, -PI / 2, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= RotLLhipYP;
	temp *= endTr1;
	ll1 = temp.get_translation();
	ll1.scalar_mult(LHipYawPitchMass);
	base *= T2;
	temp = base;
	temp *= RotLLhipR;
	temp *= endTr2;
	ll2 = temp.get_translation();
	ll2.scalar_mult(LHipRollMass);
	base *= T3;
	temp = base;
	temp *= RotLLallPitchs;
	temp *= endTr3;
	ll3 = temp.get_translation();
	ll3.scalar_mult(LHipPitchMass);
	base *= T4;
	temp = base;
	temp *= RotLLallPitchs;
	temp *= endTr4;
	ll4 = temp.get_translation();
	ll4.scalar_mult(LKneePitchMass);
	base *= T5;
	temp = base;
	temp *= RotLLallPitchs;
	temp *= endTr5;
	ll5 = temp.get_translation();
	ll5.scalar_mult(LAnklePitchMass);
	base *= T6;
	temp = base;
	temp *= RotLLeg;
	temp *= endTr6;
	ll6 = temp.get_translation();
	ll6.scalar_mult(LAnkleRollMass);
	ll1 += ll2;
	ll1 += ll3;
	ll1 += ll4;
	ll1 += ll5;
	ll1 += ll6;

	//Right Leg
	KMatTransf::makeTranslation(endTr1, RHipYawPitchX, RHipYawPitchY, RHipYawPitchZ);
	KMatTransf::makeTranslation(endTr2, RHipRollX, RHipRollY, RHipRollZ);
	KMatTransf::makeTranslation(endTr3, RHipPitchX, RHipPitchY, RHipPitchZ);
	KMatTransf::makeTranslation(endTr4, RKneePitchX, RKneePitchY, RKneePitchZ);
	KMatTransf::makeTranslation(endTr5, RAnklePitchX, RAnklePitchY, RAnklePitchZ);
	KMatTransf::makeTranslation(endTr6, RAnkleRollX, RAnkleRollY, RAnkleRollZ);
	KMatTransf::makeTranslation(base, 0.0, -HipOffsetY, -HipOffsetZ);
	KMatTransf::makeDHTransformation(T1, 0.0, -PI / 4, 0.0, allJoints.front() - PI / 2);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0, -PI / 2, 0.0, allJoints.front() - PI / 4);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T3, 0.0, PI / 2, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T4, -ThighLength, 0.0, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T5, -TibiaLength, 0.0, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T6, 0.0, -PI / 2, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= RotRLhipYP;
	temp *= endTr1;
	rl1 = temp.get_translation();
	rl1.scalar_mult(RHipYawPitchMass);
	base *= T2;
	temp = base;
	temp *= RotRLhipR;
	temp *= endTr2;
	rl2 = temp.get_translation();
	rl2.scalar_mult(RHipRollMass);
	base *= T3;
	temp = base;
	temp *= RotRLallPitchs;
	temp *= endTr3;
	rl3 = temp.get_translation();
	rl3.scalar_mult(RHipPitchMass);
	base *= T4;
	temp = base;
	temp *= RotRLallPitchs;
	temp *= endTr4;
	rl4 = temp.get_translation();
	rl4.scalar_mult(RKneePitchMass);
	base *= T5;
	temp = base;
	temp *= RotRLallPitchs;
	temp *= endTr5;
	rl5 = temp.get_translation();
	rl5.scalar_mult(RAnklePitchMass);
	base *= T6;
	temp = base;
	temp *= RotRLeg;
	temp *= endTr6;
	rl6 = temp.get_translation();
	rl6.scalar_mult(RAnkleRollMass);
	rl1 += rl2;
	rl1 += rl3;
	rl1 += rl4;
	rl1 += rl5;
	rl1 += rl6;

	//Right Hand
	KMatTransf::makeTranslation(endTr1, RShoulderPitchX, RShoulderPitchY, RShoulderPitchZ);
	KMatTransf::makeTranslation(endTr2, RShoulderRollX, RShoulderRollY, RShoulderRollZ);
	KMatTransf::makeTranslation(endTr3, RElbowYawX, RElbowYawY, RElbowYawZ);
	KMatTransf::makeTranslation(endTr4, RElbowRollX, RElbowRollY, RElbowRollZ);
	KMatTransf::makeTranslation(endTr5, RWristYawX, RWristYawY, RWristYawZ);
	KMatTransf::makeTranslation(base, 0.0, -(ShoulderOffsetY + ElbowOffsetY), ShoulderOffsetZ);
	KMatTransf::makeDHTransformation(T1, 0.0, -PI / 2, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0, PI / 2, 0.0, allJoints.front() + PI / 2);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T3, 0.0, -PI / 2, -UpperArmLength, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T4, 0.0, PI / 2, 0.0, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T5, 0.0, -PI / 2, -LowerArmLength, allJoints.front());
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= RotRHshouldP;
	temp *= endTr1;
	rh1 = temp.get_translation();
	rh1.scalar_mult(RShoulderPitchMass);
	base *= T2;
	temp = base;
	temp *= RotRHshoulR;
	temp *= endTr2;
	rh2 = temp.get_translation();
	rh2.scalar_mult(RShoulderRollMass);
	base *= T3;
	temp = base;
	temp *= RotRHelbowY;
	temp *= endTr3;
	rh3 = temp.get_translation();
	rh3.scalar_mult(RElbowYawMass);
	base *= T4;
	temp = base;
	temp *= RotRHelbowR;
	temp *= endTr4;
	rh4 = temp.get_translation();
	rh4.scalar_mult(RElbowRollMass);
	base *= T5;
	temp = base;
	temp *= RotRHwristY;
	temp *= endTr5;
	rh5 = temp.get_translation();
	rh5.scalar_mult(RWristYawMass);
	rh1 += rh2;
	rh1 += rh3;
	rh1 += rh4;
	rh1 += rh5;

	//Torso
	t(0, 0) = TorsoX;
	t(1, 0) = TorsoY;
	t(2, 0) = TorsoZ;
	t.scalar_mult(TorsoMass);
	t += lh1;
	t += rh1;
	t += ll1;
	t += rl1;
	t += h1;
	double tmass = 1 / TotalMassH21;
	t.scalar_mult(tmass);
	FKvars FKVariables;
	FKVariables.pointX = t(0, 0);
	FKVariables.pointY = t(1, 0);
	FKVariables.pointZ = t(2, 0);
	FKVariables.angleZ = 0;
	FKVariables.angleY = 0;
	FKVariables.angleX = 0;
	return FKVariables;
}

std::vector<std::vector<double> > NAOKinematics::inverseHead(double px, double py, double pz, double rx, double ry, double rz, bool withAngles, bool topCamera){
	KMatTransf::makeTransformation(T, px, py, pz, rx, ry, rz);
	return inverseHead(T, withAngles, topCamera);
}

std::vector<std::vector<double> > NAOKinematics::inverseHead(kmatTable targetPoint, bool withAngles, bool topCamera)
{
	std::vector<double> fc, empty;
	std::vector<std::vector<double> > returnResult;
	FKvars output;
	T = targetPoint;
	double theta1, theta2;

	if(T(0,3) != 0.0 && withAngles)
		return returnResult;

	if(withAngles)
	{
		theta1 = atan2(T(1,0), T(0,0));
		theta2 = atan2(-T(2,0), sqrt(pow(T(2,1), 2) + pow(T(2,2), 2)));
	}
	else
	{
		double up = - T(2,3) + NeckOffsetZ;
		double downt2, downt1, psi;

		if(topCamera)
		{
			downt2 = sqrt(pow(CameraTopX, 2) + pow(CameraTopZ, 2));
			psi = atan2(CameraTopX, CameraTopZ);
		}
		else
		{
			downt2 = sqrt(pow(CameraBotomX, 2) + pow(CameraBotomZ, 2));
			psi = atan2(CameraBotomX, CameraBotomZ);
		}

		double tempTheta2 = asin(up / downt2); //-psi;
		double posOrNegPI = (tempTheta2 >= 0) ? PI : -PI;

		for(int j = 0; j < 2; j++)
		{
			if(j == 0 && (tempTheta2 - psi > HeadYawHigh || tempTheta2 - psi < HeadYawLow))
				continue;
			else if(j == 1 && (posOrNegPI - tempTheta2 - psi > HeadYawHigh || posOrNegPI - tempTheta2 - psi < HeadYawLow))
				continue;
			else if(j == 1)
				theta2 = posOrNegPI - tempTheta2 - psi;
			else
				theta2 = tempTheta2 - psi;

			if(topCamera)
			{
				downt1 = CameraTopZ * cos(theta2) - CameraTopX * sin(theta2);
			}
			else
			{
				downt1 = CameraBotomZ * cos(theta2) - CameraBotomX * sin(theta2);
			}

			theta2 = theta2 + PI / 2;
			theta1 = acos(T(0,3) / downt1);

			for(int i = 0; i < 2; i++)
			{
				if(i == 0 && (theta1 > HeadPitchHigh || theta1 < HeadPitchLow))
					continue;
				else if(i == 1 && (-theta1 > HeadPitchHigh || -theta1 < HeadPitchLow))
					continue;
				else if(i == 0)
					theta1 = theta1;
				else
					theta1 = -theta1;
			}

			//---------------------------Forward validation step--------------------------------------------------------------------------------------
			fc.clear();
			fc.push_back(theta1);
			fc.push_back(theta2);

			if(topCamera)
				output = filterForwardFromTo("Torso", "CameraTop", empty, fc);
			else
				output = filterForwardFromTo("Torso", "CameraBot", empty, fc);

			double x = output.pointX, y = output.pointY, z = output.pointZ;

			//Validate only the points
			if(T(0,3) - 1 <= x && x <= T(0,3) + 1 && T(1,3) - 1 <= y && y <= T(1,3) + 1 && T(2,3) - 1 <= z && z <= T(2,3) + 1)
			{
				returnResult.push_back(fc);
			}

			//-----------------------------------------------------------------------------------------------------------------------------------------
		}
	}

	return returnResult;
}

std::vector<std::vector<double> > NAOKinematics::inverseLeftHand(double px, double py, double pz, double rx, double ry, double rz)
{
	KMatTransf::makeTransformation(T, px, py, pz, rx, ry, rz);
	return inverseLeftHand(T);
}

std::vector<std::vector<double> > NAOKinematics::inverseLeftHand(kmatTable targetPoint)
{
	std::vector<double> flh, empty;
	std::vector<std::vector<double> > returnResult;
	Tinit = targetPoint;
	double startX = 0;
	double startY = ShoulderOffsetY + ElbowOffsetY;
	double startZ = ShoulderOffsetZ;
	double side1 = UpperArmLength;
	double side2 = LowerArmLength + HandOffsetX;
	double value1 = ShoulderOffsetY + ElbowOffsetY; //113
	double value2 = LowerArmLength + HandOffsetX; //113.7
	double value3 = UpperArmLength; //to allo
	//Calculate Theta 4
	double distance = sqrt(pow(startX - Tinit(0,3), 2) + pow(startY - Tinit(1,3), 2) + pow(startZ - Tinit(2,3), 2));
	double theta4 = PI - acos((pow(side1, 2) + pow(side2, 2) - pow(distance, 2)) / (2 * side1 * side2));
	theta4 = - theta4;

	if(theta4 != theta4 || theta4 > LElbowRollHigh || theta4 < LElbowRollLow)
		return returnResult;

	double cth4 = cos(theta4);
	double sth4 = sin(theta4);
	
	kmatTable Tt1;
	Tt1 = TBaseLArmInv;
	Tt1 *= Tinit;
	Tt1 *= TEndAndRotLArmInv;
	Tt1.fast_invert();
	double theta5 = -atan2(-Tt1(1,3),Tt1(0,3));
	
	Tt1 = Tinit;
	Tt1 *= TEndAndRotLArmInv;
	KMatTransf::makeDHTransformation(T5, 0.0, -PI / 2, LowerArmLength, -theta5);
	T5.fast_invert();
	
	Tt1 *= T5;	
	Tt1 *= TLArmOldR;
	Tt1 *= TLArmOldEnd;
	
	//Calculate Theta 2
	double equationForTheta2 = Tt1(1,1);
	double upForTheta2 = Tt1(1,3) - value1 - (equationForTheta2 * value2 * sth4) / cth4;
	double downForTheta2 = value3 + value2 * cth4 + value2 * pow(sth4, 2) / cth4;
	double theta2temp = acos(upForTheta2 / downForTheta2);
	for(int i = 0; i < 2; i++)
	{
		double theta2 = theta2temp;

		if(i == 0 && (theta2 + PI / 2 > LShoulderRollHigh || theta2 + PI / 2 < LShoulderRollLow))
			continue;
		else if(i == 1 && (-theta2 + PI / 2 > LShoulderRollHigh || -theta2 + PI / 2 < LShoulderRollLow))
			continue;
		else if(i == 0)
			theta2 = theta2 + PI / 2;
		else
			theta2 = -theta2 + PI / 2;

		//Calculate Theta 3
		double equationForTheta3 = Tt1(1,2);
		double upForTheta3 = equationForTheta3;
		double downForTheta3 = sin(theta2 - PI / 2);
		double theta3temp = asin(upForTheta3 / downForTheta3);
		
		if(theta3temp != theta3temp && upForTheta3 / downForTheta3 < 0)
			theta3temp = -PI / 2;
		else if(theta3temp != theta3temp)
			theta3temp = PI / 2;
			
		double posOrNegPI = (theta3temp >= 0) ? PI : -PI;

		for(int j = 0; j < 2; j++)
		{
			double theta3 = theta3temp;

			if(j == 0 && (theta3 > LElbowYawHigh || theta3 < LElbowYawLow))
				continue;
			else if(j == 1 && (posOrNegPI - theta3 > LElbowYawHigh || posOrNegPI - theta3 < LElbowYawLow))
				continue;
			else if(j == 1)
				theta3 = -(posOrNegPI - theta3);
			else if(j == 0)
				theta3 = -theta3;

			//Calculate Theta 1
			double equation1ForTheta1 = Tt1(0,2);
			double equation2ForTheta1 = Tt1(2,2);
			double theta1temp;

			if(cos(-theta3) < 10e-10 && cos(-theta3) > -10e-10 && cos(theta2-PI/2) < 10e-10 && cos(theta2-PI/2) > -10e-10)
			{
				kmatTable temp = Tt1;
				kmatTable temp2;
				KMatTransf::makeTranslation(temp2, HandOffsetX + LowerArmLength, 0.0, 0.0);
				temp2.fast_invert();
				temp *= temp2;
				theta1temp = acos(temp(0,3)/value3);
				
				if(theta1temp != theta1temp && temp(0,3)/value3 < 0)
					theta1temp = -PI;
				else if(theta1temp != theta1temp)
					theta1temp = 0;
			}else if(cos(-theta3) < 10e-10 && cos(-theta3) > -10e-10){
			
				theta1temp = acos(equation1ForTheta1 / cos(theta2 - PI / 2));
				cout << theta1temp << endl;
			}else{
				double upForTheta1 = equation2ForTheta1 + equation1ForTheta1 * sin(-theta3) * cos(theta2 - PI / 2) / cos(-theta3);
				double downForTheta1 = cos(-theta3) + pow(cos(theta2 - PI / 2), 2) * pow(sin(-theta3), 2) / cos(-theta3);
				theta1temp = acos(upForTheta1 / downForTheta1);
			}

			for(int l = 0; l < 2; l++)
			{
				double theta1 = theta1temp;
				//cout << theta1 << " and " << l << endl;
				if(l == 0 && (theta1 > LShoulderPitchHigh || theta1 < LShoulderPitchLow))
					continue;
				else if(l == 1 && (-theta1 > LShoulderPitchHigh || -theta1 < LShoulderPitchLow))
					continue;
				else if(l == 1)
					theta1 = -theta1;

				//---------------------------Forward validation step--------------------------------------------------------------------------------------
				flh.clear();
				flh.push_back(theta1);
				flh.push_back(theta2);
				flh.push_back(theta3);
				flh.push_back(theta4);
				flh.push_back(theta5);
				kmatTable test;
				filterForward(test, "LeftHand", flh);

				if(test.almostEqualTo(Tinit))
				{
					//std::cout << "Niki!!\nTheta 1 = " << theta1 << "\nTheta 2 = " << theta2 << "\nTheta 3 = " << theta3 << "\nTheta 4 = " << theta4 << std::endl;
					returnResult.push_back(flh);
				}
				else
				{
					//std::cout << "Hta!!\nTheta 1 = " << theta1 << " Theta 2 = " << theta2 << " Theta 3 = " << theta3 << " Theta 4 = " << theta4 << " Theta 5 = " << theta5 << std::endl;
				}

				//------------------------------------------------------------------------------------------------------------------------------------------
			}
		}
	}

	return returnResult;
}

std::vector<std::vector<double> > NAOKinematics::inverseRightHand(double px, double py, double pz, double rx, double ry, double rz)
{
	KMatTransf::makeTransformation(T, px, py, pz, rx, ry, rz);
	return inverseRightHand(T);
}

std::vector<std::vector<double> > NAOKinematics::inverseRightHand(kmatTable targetPoint)
{
	std::vector<double> frh, empty;
	std::vector<std::vector<double> > returnResult;
	//Rotate input to remvoe Rfix
	T = targetPoint;
	Tinit = T;
	T *= RotRArmFixInv;
	//continue with the rotated input
	double startX = 0;
	double startY = -ShoulderOffsetY - ElbowOffsetY;
	double startZ = ShoulderOffsetZ;
	double side1 = UpperArmLength;
	double side2 = LowerArmLength + HandOffsetX;
	double value1 = -ShoulderOffsetY - ElbowOffsetY; //113
	double value2 = -LowerArmLength - HandOffsetX; //113.7
	double value3 = -UpperArmLength; //to allo
	//Calculate Theta 4
	double distance = sqrt(pow(startX - T(0,3), 2) + pow(startY - T(1,3), 2) + pow(startZ - T(2,3), 2));
	double theta4 = PI - acos((pow(side1, 2) + pow(side2, 2) - pow(distance, 2)) / (2 * side1 * side2));
	theta4 = theta4;

	if(theta4 != theta4 || theta4 > RElbowRollHigh || theta4 < RElbowRollLow)
		return returnResult;

	double cth4 = cos(theta4);
	double sth4 = sin(theta4);
	
	kmatTable Tt1;
	Tt1 = TBaseRArmInv;
	Tt1 *= T;
	Tt1 *= TEndAndRotRArmInv;
	Tt1.fast_invert();
	double theta5 = atan2(-Tt1(1,3),Tt1(0,3));

	Tt1 = T;
	Tt1 *= TEndAndRotRArmInv;
	KMatTransf::makeDHTransformation(T5, 0.0, -PI / 2, -LowerArmLength, theta5);
	T5.fast_invert();
	
	Tt1 *= T5;	
	Tt1 *= TRArmOldR;
	Tt1 *= TRArmOldEnd;
	
	//Calculate Theta 2
	double equationForTheta2 = Tt1(1,1);
	double upForTheta2 = Tt1(1,3) - value1 - (equationForTheta2 * value2 * sth4) / cth4;
	double downForTheta2 = value3 + value2 * cth4 + value2 * pow(sth4, 2) / cth4;
	double theta2temp = acos(upForTheta2 / downForTheta2);

	for(int i = 0; i < 2; i++)
	{
		double theta2 = theta2temp;

		if(i == 0 && (theta2 - PI / 2 > RShoulderRollHigh || theta2 - PI / 2 < RShoulderRollLow))
			continue;
		else if(i == 1 && (-theta2 - PI / 2 > RShoulderRollHigh || -theta2 - PI / 2 < RShoulderRollLow))
			continue;
		else if(i == 0)
			theta2 = theta2 - PI / 2;
		else
			theta2 = -theta2 - PI / 2;

		//Calculate Theta 3
		double equationForTheta3 = Tt1(1,2);
		double upForTheta3 = equationForTheta3;
		double downForTheta3 = sin(theta2 + PI / 2);
		double theta3temp = asin(upForTheta3 / downForTheta3);
		
		if(theta3temp != theta3temp && upForTheta3 / downForTheta3 < 0)
			theta3temp = -PI / 2;
		else if(theta3temp != theta3temp)
			theta3temp = PI / 2;
		
		double posOrNegPI = (theta3temp >= 0) ? PI : -PI;

		for(int j = 0; j < 2; j++)
		{
			double theta3 = theta3temp;

			if(j == 0 && (theta3 > RElbowYawHigh || theta3 < RElbowYawLow))
				continue;
			else if(j == 1 && (posOrNegPI - theta3 > RElbowYawHigh || posOrNegPI - theta3 < RElbowYawLow))
				continue;
			else if(j == 1)
				theta3 = posOrNegPI - theta3;

			//Calculate Theta 1
			double equation1ForTheta1 = Tt1(0,2);
			double equation2ForTheta1 = Tt1(2,2);
			double theta1temp;

			if(cos(theta3) < 10e-10 && cos(theta3) > -10e-10 && cos(theta2+PI/2) < 10e-10 && cos(theta2+PI/2) > -10e-10)
			{
				kmatTable temp = Tt1;
				kmatTable temp2 = TEndRArm;
				temp2.fast_invert();
				temp *= temp2;
				theta1temp = acos(-temp(0,3)/value3);
				
				if(theta1temp != theta1temp && -temp(0,3)/value3 < 0)
					theta1temp = -PI;
				else if(theta1temp != theta1temp)
					theta1temp = 0;
			}else if(cos(theta3) < 10e-10 && cos(theta3) > -10e-10){
				theta1temp = acos(equation1ForTheta1 / cos(theta2 + PI / 2));
			}else{
				double upForTheta1 = equation2ForTheta1 + equation1ForTheta1 * sin(theta3) * cos(theta2 + PI / 2) / cos(theta3);
				double downForTheta1 = cos(theta3) + pow(cos(theta2 + PI / 2), 2) * pow(sin(theta3), 2) / cos(theta3);
				theta1temp = acos(upForTheta1 / downForTheta1);
			}

			for(int l = 0; l < 2; l++)
			{
				double theta1 = theta1temp;

				if(l == 0 && (theta1 > RShoulderPitchHigh || theta1 < RShoulderPitchLow))
					continue;
				else if(l == 1 && (-theta1 > RShoulderPitchHigh || -theta1 < RShoulderPitchLow))
					continue;
				else if(l == 1)
					theta1 = -theta1;

				//---------------------------Forward validation step--------------------------------------------------------------------------------------
				frh.clear();
				frh.push_back(theta1);
				frh.push_back(theta2);
				frh.push_back(theta3);
				frh.push_back(theta4);
				frh.push_back(theta5);
				kmatTable test;
				filterForward(test, "RightHand", frh);

				if(test.almostEqualTo(Tinit))
				{
					//std::cout << "Niki!!\nTheta 1 = " << theta1 << "\nTheta 2 = " << theta2 << "\nTheta 3 = " << theta3 << "\nTheta 4 = " << theta4 << std::endl;
					returnResult.push_back(frh);
				}
				else
				{
					//std::cout << "Htaa!!\nTheta 1 = " << theta1 << " Theta 2 = " << theta2 << " Theta 3 = " << theta3 << " Theta 4 = " << theta4 << " Theta 5 = " << theta5 << std::endl;
				}

				//-----------------------------------------------------------------------------------------------------------------------------------------
			}
		}
	}

	frh.clear();
	return returnResult;
}

std::vector<std::vector<double> > NAOKinematics::inverseLeftLeg(double px, double py, double pz, double rx, double ry, double rz)
{
	KMatTransf::makeTransformation(T, px, py, pz, rx, ry, rz);
	return inverseLeftLeg(T);
}

std::vector<std::vector<double> > NAOKinematics::inverseLeftLeg(kmatTable targetPoint)
{
	std::vector<double> fll, empty;
	std::vector<std::vector<double> > returnResult;
	T = targetPoint;
	Tinit = T;
	//Move the start point to the hipyawpitch point
	base = TBaseLLegInv;
	base *= T;
	//Move the end point to the anklePitch joint
	base *= TEndLLegInv;
	//Rotate hipyawpitch joint
	Rot = RotFixLLeg;
	Rot *= base;
	//Invert the table, because we need the chain from the ankle to the hip
	Tstart = Rot;
	Rot.fast_invert();
	T = Rot;
	//Build the rotation table
	double startX = 0;
	double startY = 0;
	double startZ = 0;
	double side1 = ThighLength;
	double side2 = TibiaLength;
	//Calculate Theta 4
	double distance = sqrt(pow(startX -  T(0,3), 2) + pow(startY -  T(1,3), 2) + pow(startZ - T(2,3), 2));
	double theta4 = PI - acos((pow(side1, 2) + pow(side2, 2) - pow(distance, 2)) / (2 * side1 * side2));

	if(theta4 != theta4)
		return returnResult;

	double theta6 = atan(T(1,3) / T(2,3));

	if(theta6 < LAnkleRollLow || theta6 > LAnkleRollHigh)
		return returnResult;

	KMatTransf::makeDHTransformation(T6i, 0.0, -PI / 2, 0.0, theta6);
	T6i *= RotRLeg;

	try
	{
		T6i.fast_invert();
		Tstart *= T6i;
		TtempTheta5 = Tstart;
		TtempTheta5.fast_invert();
	}
	catch(KMath::KMat::SingularMatrixInvertionException d)
	{
		return returnResult;
	}

	for(int itter = 0; itter < 2; itter++)
	{
		theta4 = (itter == 0) ? theta4 : -theta4;

		if(theta4 < RKneePitchLow || theta4 > RKneePitchHigh)
			continue;

		KMatTransf::makeDHTransformation(T4i, -ThighLength, 0.0, 0.0, theta4);
		double up = TtempTheta5(1, 3) * (TibiaLength + ThighLength * cos(theta4)) + ThighLength * TtempTheta5(0, 3) * sin(theta4);
		double down = pow(ThighLength, 2) * pow(sin(theta4), 2) + pow(TibiaLength + ThighLength * cos(theta4), 2);
		double theta5 = asin(-up / down);
		double posOrNegPIt5 = (theta5 >= 0) ? PI : -PI;

		if(theta5 != theta5 && up / down < 0)
			theta5 = -PI / 2;
		else if(theta5 != theta5)
			theta5 = PI / 2;

		for(int i = 0; i < 2; i++)
		{
			if(i == 0 && (theta5 > LAnklePitchHigh || theta5 < LAnklePitchLow))
				continue;
			else if(i == 1 && (posOrNegPIt5 - theta5 > LAnklePitchHigh || posOrNegPIt5 - theta5 < LAnklePitchLow))
				continue;
			else if(i == 1)
				theta5 = posOrNegPIt5 - theta5;

			KMatTransf::makeDHTransformation(T5i, -TibiaLength, 0.0, 0.0, theta5);
			Ttemp = T4i;
			Ttemp *= T5i;

			try
			{
				Ttemp.fast_invert();
			}
			catch(KMath::KMat::SingularMatrixInvertionException d)
			{
				continue;
			}

			Ttemp2 = Tstart;
			Ttemp2 *= Ttemp;
			double temptheta2 = acos(Ttemp2(1, 2));
			double theta2;

			for(int l = 0; l < 2; l++)
			{
				if(l == 0 && (temptheta2 - PI / 4 > LHipRollHigh || temptheta2 - PI / 4 < LHipRollLow))
					continue;
				else if(l == 1 && (-temptheta2 - PI / 4 > LHipRollHigh || -temptheta2 - PI / 4 < LHipRollLow))
					continue;
				else if(l == 0)
					theta2 = temptheta2 - PI / 4;
				else if(l == 1)
					theta2 = -temptheta2 - PI / 4;

				double theta3 = asin(Ttemp2(1, 1) / sin(theta2 + PI / 4));
				double posOrNegPIt3 = (theta3 >= 0) ? PI : -PI;

				if(theta3 != theta3 && Ttemp2(1, 1) / sin(theta2 + PI / 4) < 0)
					theta3 = -PI / 2;
				else if(theta3 != theta3)
					theta3 = PI / 2;

				for(int k = 0; k < 2; k++)
				{
					if(k == 0 && (theta3 > LHipPitchHigh || theta3 < LHipPitchLow))
						continue;
					else if(k == 1 && (posOrNegPIt3 - theta3 > LHipPitchHigh || posOrNegPIt3 - theta3 < LHipPitchLow))
						continue;
					else if(k == 1)
						theta3 = posOrNegPIt3 - theta3;

					double temptheta1 = acos(Ttemp2(0, 2) / sin(theta2 + PI / 4));

					if(temptheta1 != temptheta1)
						temptheta1 = 0;

					for(int p = 0; p < 2; p++)
					{
						double theta1;

						if(p == 0 && (temptheta1 + PI / 2 > LHipYawPitchHigh || -temptheta1 + PI / 2 < LHipYawPitchLow))
							continue;
						else if(p == 1 && (-temptheta1 + PI / 2 > LHipYawPitchHigh || - temptheta1 + PI / 2 < LHipYawPitchLow))
							continue;
						else if(p == 0)
							theta1 = temptheta1 + PI / 2;
						else if(p == 1)
							theta1 = -temptheta1 + PI / 2;

						//--------------------------------------Forward validation step------------------------------------------------------------------------------
						fll.clear();
						fll.push_back(theta1);
						fll.push_back(theta2);
						fll.push_back(theta3);
						fll.push_back(theta4);
						fll.push_back(theta5);
						fll.push_back(theta6);
						kmatTable test;
						filterForward(test, "LeftLeg", fll);

						if(test.almostEqualTo(Tinit))
						{
							//std::cout << "Niki!!\nTheta 1 = " << theta1 << "\nTheta 2 = " << theta2 << "\nTheta 3 = " << theta3 << "\nTheta 4 = " << theta4  << "\nTheta 5 = " << theta5 << "\nTheta 6 = " << theta6 << std::endl;
							returnResult.push_back(fll);
						}

						//-
						//---------------------------------------------------------------------------------------------------------------------------------------------
					}
				}
			}
		}
	}

	return returnResult;
}

std::vector<std::vector<double> > NAOKinematics::inverseRightLeg(double px, double py, double pz, double rx, double ry, double rz)
{
	KMatTransf::makeTransformation(T, px, py, pz, rx, ry, rz);
	return inverseRightLeg(T);
}

std::vector<std::vector<double> > NAOKinematics::inverseRightLeg(kmatTable targetPoint)
{
	std::vector<double> frl, empty;
	std::vector<std::vector<double> > returnResult;
	T = targetPoint;
	Tinit = T;
	//Move the start point to the hipyawpitch point
	base = TBaseRLegInv;
	base *= T;
	//Move the end point to the anklePitch joint
	base *= TEndRLegInv;
	//Rotate hipyawpitch joint
	Rot = RotFixRLeg;
	Rot *= base;
	//Invert the table, because we need the chain from the ankle to the hip
	Tstart = Rot;
	Rot.fast_invert();
	T = Rot;
	//Build the rotation table
	double startX = 0;
	double startY = 0;
	double startZ = 0;
	double side1 = ThighLength;
	double side2 = TibiaLength;
	//Calculate Theta 4
	double distance = sqrt(pow(startX - T(0,3), 2) + pow(startY - T(1,3), 2) + pow(startZ - T(2,3), 2));
	double theta4 = PI - acos((pow(side1, 2) + pow(side2, 2) - pow(distance, 2)) / (2 * side1 * side2));

	if(theta4 != theta4)
		return returnResult;

	double theta6 = atan(T(1,3) / T(2,3));

	if(theta6 < RAnkleRollLow || theta6 > RAnkleRollHigh)
		return returnResult;

	KMatTransf::makeDHTransformation(T6i, 0.0, -PI / 2, 0.0, theta6);
	T6i *= RotRLeg;

	try
	{
		T6i.fast_invert();
		Tstart *= T6i;
		TtempTheta5 = Tstart;
		TtempTheta5.fast_invert();
	}
	catch(KMath::KMat::SingularMatrixInvertionException d)
	{
		return returnResult;
	}

	for(int itter = 0; itter < 2; itter++)
	{
		theta4 = (itter == 0) ? theta4 : -theta4;

		if(theta4 < RKneePitchLow || theta4 > RKneePitchHigh)
			continue;

		KMatTransf::makeDHTransformation(T4i, -ThighLength, 0.0, 0.0, theta4);
		double up = TtempTheta5(1, 3) * (TibiaLength + ThighLength * cos(theta4)) + ThighLength * TtempTheta5(0, 3) * sin(theta4);
		double down = pow(ThighLength, 2) * pow(sin(theta4), 2) + pow(TibiaLength + ThighLength * cos(theta4), 2);
		double theta5 = asin(-up / down);
		double posOrNegPIt5 = (theta5 >= 0) ? PI : -PI;

		if(theta5 != theta5 && up / down < 0)
			theta5 = -PI / 2;
		else if(theta5 != theta5)
			theta5 = PI / 2;

		for(int i = 0; i < 2; i++)
		{
			if(i == 0 && (theta5 > RAnklePitchHigh || theta5 < RAnklePitchLow))
				continue;
			else if(i == 1 && (posOrNegPIt5 - theta5 > RAnklePitchHigh || posOrNegPIt5 - theta5 < RAnklePitchLow))
				continue;
			else if(i == 1)
				theta5 = posOrNegPIt5 - theta5;

			KMatTransf::makeDHTransformation(T5i, -TibiaLength, 0.0, 0.0, theta5);
			Ttemp = T4i;
			Ttemp *= T5i;

			try
			{
				Ttemp.fast_invert();
			}
			catch(KMath::KMat::SingularMatrixInvertionException d)
			{
				continue;
			}

			Ttemp2 = Tstart;
			Ttemp2 *= Ttemp;
			double temptheta2 = acos(Ttemp2(1, 2));
			double theta2;

			for(int l = 0; l < 2; l++)
			{
				if(l == 0 && (temptheta2 + PI / 4 > RHipRollHigh || temptheta2 + PI / 4 < RHipRollLow))
					continue;
				else if(l == 1 && (-temptheta2 + PI / 4 > RHipRollHigh || -temptheta2 + PI / 4 < RHipRollLow))
					continue;
				else if(l == 0)
					theta2 = temptheta2 + PI / 4;
				else if(l == 1)
					theta2 = -temptheta2 + PI / 4;

				double theta3 = asin(Ttemp2(1, 1) / sin(theta2 - PI / 4));
				double posOrNegPIt3 = (theta3 >= 0) ? PI : -PI;

				if(theta3 != theta3 && Ttemp2(1, 1) / sin(theta2 - PI / 4) < 0)
					theta3 = -PI / 2;
				else if(theta3 != theta3)
					theta3 = PI / 2;

				for(int k = 0; k < 2; k++)
				{
					if(k == 0 && (theta3 > RHipPitchHigh || theta3 < RHipPitchLow))
						continue;
					else if(k == 1 && (posOrNegPIt3 - theta3 > RHipPitchHigh || posOrNegPIt3 - theta3 < RHipPitchLow))
						continue;
					else if(k == 1)
						theta3 = posOrNegPIt3 - theta3;

					double temptheta1 = acos(Ttemp2(0, 2) / sin(theta2 - PI / 4));

					if(temptheta1 != temptheta1)
						temptheta1 = 0;

					for(int p = 0; p < 2; p++)
					{
						double theta1;

						if(p == 0 && (temptheta1 + PI / 2 > RHipYawPitchHigh || -temptheta1 + PI / 2 < RHipYawPitchLow))
							continue;
						else if(p == 1 && (-temptheta1 + PI / 2 > RHipYawPitchHigh || - temptheta1 + PI / 2 < RHipYawPitchLow))
							continue;
						else if(p == 0)
							theta1 = temptheta1 + PI / 2;
						else if(p == 1)
							theta1 = -temptheta1 + PI / 2;

						//--------------------------------------Forward validation step------------------------------------------------------------------------------
						frl.clear();
						frl.push_back(theta1);
						frl.push_back(theta2);
						frl.push_back(theta3);
						frl.push_back(theta4);
						frl.push_back(theta5);
						frl.push_back(theta6);
						kmatTable test;
						filterForward(test, "RightLeg", frl);

						if(test.almostEqualTo(Tinit))
						{
							//	std::cout << "Px = " << px << " Py = " << py << " Pz = " << pz << "Ax = " << rx << " Ay = " << ry << " Az = " << rz << std::endl;
							//	std::cout << "Niki!!\nTheta 1 = " << theta1 << "\nTheta 2 = " << theta2 << "\nTheta 3 = " << theta3 << "\nTheta 4 = " << theta4  << "\nTheta 5 = " << theta5 << "\nTheta 6 = " << theta6 << std::endl;
							returnResult.push_back(frl);
						}

						//---------------------------------------------------------------------------------------------------------------------------------------------
					}
				}
			}
		}
	}

	return returnResult;
}

