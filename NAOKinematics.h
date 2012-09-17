#ifndef __NAOKIN_H__
#define __NAOKIN_H__
#include <iostream>
#include <stdexcept>
#include <string.h>
#include <vector>
#include <iomanip>
#include <limits>
#include <math.h>
#include "KinematicsDefines.h"
#include "KMat.h"

/**
 * This is the code for the Forward and Inverse Kinematics for nao v3.3 robot.

 * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
 * special thanks to Orf or vosk for the KMat library
 	 * 1 = vector's head position\n
	 * Arms:
	 *			-# ShoulderPitch
	 * 			-# ShoulderRoll
	 *			-# ElbowYaw
	 *			-# ElbowRoll
	 * .
	 *
	 * Legs:
	 *			-# HipYawPitch
	 *			-# HipRoll
	 *			-# HipPitch
	 *			-# KneePitch
	 *			-# AnklePitch
	 *			-# AnkleRoll
	 *
	 * .
	 * Camera:
	 *			-# HeadYaw
	 *			-# HeadPitch
	 * .
	 * Format of inputs for filter forward
	 * "CameraTop" = forward for Top camera
	 * "CameraBot" = forward for Bottom camera
	 * "LeftLeg" = forward for left leg
	 * "RightLeg" = forward for right leg
	 * "LeftArm" = forward for left arm
	 * "RightArm" = forward for right arm
 * \file NAOKinematics.h
*/
using namespace std;
typedef KMat::transformations KMatTransf;
/**
*@struct FKvars
*@brief This struct contains all the cartesian points and angles that we extract from the forward kinematics
*/
struct FKvars
{
	float pointX, pointY, pointZ;
	float angleX, angleY, angleZ;
};
typedef KMat::ATMatrix<float, 4> kmatTable;
class NAOKinematics
{
public:
	float PI;
	//Predifined tables
	kmatTable TBaseHead, TEndHead1, TEndHead2, RotHead;
	kmatTable TBaseLArm, TEndLArm, RotLArm;
	kmatTable TBaseRArm, TEndRArm, RotRArm, RotRArmFix, RotRArmFixInv;
	kmatTable TBaseLLeg, TEndLLeg, RotLLeg, TBaseLLegInv, TEndLLegInv, RotFixLLeg;
	kmatTable TBaseBRLeg, TEndRLeg, RotRLeg, TBaseRLegInv, TEndRLegInv, RotFixRLeg;
	//Predifined only for forward kinematics
	kmatTable T1, T2, T3, T4, T5, T6, Tend;
	kmatTable Tmatrix1, Tmatrix2;
	//Predifined only for inverse kinematics
	kmatTable T4i, T5i, T6i;
	kmatTable T, base, Rot, Tstart, Ttemp, Ttemp2, TtempTheta5, Tinit, Rfix;

public:

	NAOKinematics()
	{
		PI = KMatTransf::PI;
		KMatTransf::makeTranslation(TBaseHead, 0.0f, 0.0f, NeckOffsetZ);
		KMatTransf::makeRotationXYZ(RotHead, PI / 2, PI / 2, 0.0f);
		KMatTransf::makeTranslation(TEndHead1, CameraBotomX, 0.0f, CameraBotomZ);
		KMatTransf::makeTranslation(TEndHead2, CameraTopX, 0.0f, CameraTopZ);
		KMatTransf::makeTranslation(TBaseLArm, 0.0f, ShoulderOffsetY + ElbowOffsetY, ShoulderOffsetZ);
		KMatTransf::makeRotationXYZ(RotLArm, 0.0f, 0.0f, PI / 2);
		KMatTransf::makeTranslation(TEndLArm, HandOffsetX + LowerArmLength, 0.0f, 0.0f);
		KMatTransf::makeTranslation(TBaseRArm, 0.0f, -(ShoulderOffsetY + ElbowOffsetY), ShoulderOffsetZ);
		KMatTransf::makeRotationXYZ(RotRArm, 0.0f, 0.0f, PI / 2);
		KMatTransf::makeTranslation(TEndRArm, -(HandOffsetX + LowerArmLength), 0.0f, 0.0f);
		KMatTransf::makeRotationXYZ(RotRArmFix, 0.0f, 0.0f, -PI);
		RotRArmFixInv = RotRArmFix.fast_invert();
		RotRArmFix.fast_invert();
		KMatTransf::makeTranslation(TBaseLLeg, 0.0f, HipOffsetY, -HipOffsetZ);
		KMatTransf::makeRotationZYX(RotLLeg, PI, -PI / 2, 0.0f);
		KMatTransf::makeTranslation(TEndLLeg, 0.0f, 0.0f, -FootHeight);
		KMatTransf::makeRotationXYZ(RotFixLLeg, PI / 4, 0.0f, 0.0f);
		TBaseLLegInv = TBaseLLeg.fast_invert();
		TBaseLLeg.fast_invert();
		TEndLLegInv = TEndLLeg.fast_invert();
		TEndLLeg.fast_invert();
		KMatTransf::makeTranslation(TBaseBRLeg, 0.0f, -HipOffsetY, -HipOffsetZ);
		KMatTransf::makeRotationZYX(RotRLeg, PI, -PI / 2, 0.0f);
		KMatTransf::makeTranslation(TEndRLeg, 0.0f, 0.0f, -FootHeight);
		KMatTransf::makeRotationXYZ(RotFixRLeg, -PI / 4, 0.0f, 0.0f);
		TBaseRLegInv = TBaseBRLeg.fast_invert();
		TBaseBRLeg.fast_invert();
		TEndRLegInv = TEndRLeg.fast_invert();
		TEndRLeg.fast_invert();
	}


	/**
	 * @fn void forwardLeftHand(kmatTable & EndTransf, float ShoulderPitch, float ShoulderRoll, float ElbowYaw, float ElbowRoll)
	 * @brief Forward kinematic for the left hand.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param ShoulderPitch. The value of the Left's arm shoulder pitch joint.
	 * @param ShoulderRoll. The value of the Left's arm shoulder roll joint.
	 * @param ElbowYaw. The value of the Left's arm elbow yaw joint.
	 * @param ElbowRoll. The value of the Left's arm elbow roll joint.
	 * */
	void forwardLeftHand(kmatTable & EndTransf, float ShoulderPitch, float ShoulderRoll, float ElbowYaw, float ElbowRoll);

	/**
	 * @fn void forwardRightHand(kmatTable & EndTransf, float ShoulderPitch, float ShoulderRoll, float ElbowYaw, float ElbowRoll)
	 * @brief Forward kinematic for the right hand.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param ShoulderPitch. The value of the Right's arm shoulder pitch joint.
	 * @param ShoulderRoll. The value of the Right's arm shoulder roll joint.
	 * @param ElbowYaw. The value of the Right's arm elbow yaw joint.
	 * @param ElbowRoll. The value of the Right's arm elbow roll joint.
	 * */
	void forwardRightHand(kmatTable & EndTransf, float ShoulderPitch, float ShoulderRoll, float ElbowYaw, float ElbowRoll);

	/**
	 * @fn void forwardLeftLeg(kmatTable & EndTransf, float HipYawPitch, float HipRoll, float HipPitch, float KneePitch, float AnklePitch, float AnkleRoll)
	 * @brief Forward kinematic for the left leg.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param HipYawPitch. The value of the Left's leg hip yaw pitch joint.
	 * @param HipRoll. The value of the Left's leg hip roll joint.
	 * @param HipPitch. The value of the Left's hip pitch yaw joint.
	 * @param KneePitch. The value of the Left's knee pitch roll joint.
	 * @param AnklePitch. The value of the Left's ankle pitch roll joint.
	 * @param AnkleRoll. The value of the Left's ankle elbow roll joint.
	 * */
	void forwardLeftLeg(kmatTable & EndTransf, float HipYawPitch, float HipRoll, float HipPitch, float KneePitch, float AnklePitch, float AnkleRoll);

	/**
	 * @fn void forwardRightLeg(kmatTable & EndTransf, float HipYawPitch, float HipRoll, float HipPitch, float KneePitch, float AnkleRoll, float AnklePitch)
	 * @brief Forward kinematic for the right leg.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param HipYawPitch. The value of the Right's leg hip yaw pitch joint.
	 * @param HipRoll. The value of the Right's leg hip roll joint.
	 * @param HipPitch. The value of the Right's hip pitch yaw joint.
	 * @param KneePitch. The value of the Right's knee pitch roll joint.
	 * @param AnklePitch. The value of the Right's ankle pitch roll joint.
	 * @param AnkleRoll. The value of the Right's ankle elbow roll joint.
	 * */
	void forwardRightLeg(kmatTable & EndTransf, float HipYawPitch, float HipRoll, float HipPitch, float KneePitch, float AnklePitch, float AnkleRoll);

	/**
	 * @fn void forwardCamera(kmatTable & EndTransf, float HeadYaw, float HeadPitch, bool topCamera)
	 * @brief Forward kinematic for the camera's on the head.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param HeadYaw. The value of the Head's yaw joint.
	 * @param HeadPitch. The value of the Head's pitch joint.
	 * @param topCamera. This value is true if we want forward kinematics for the top camera, or false if we want for the bottom camera.
	 * */
	void forwardCamera(kmatTable & EndTransf, float HeadYaw, float HeadPitch, bool topCamera);

	/**
	 * @fn void filterForward(kmatTable & Tmatrix, string WhatForward, std::vector<float> joints)
	 * @brief This function take the name of the end effector and one vector with joint and then it call's the apropriate function.
	 * @param Tmatrix. The matrix table that we will return the result.
	 * @param WhatForward. With this string, this function understands whate forward chain we want.
	 * @param joints. One vector with all the joints for the chain.
	 * */
	void filterForward(kmatTable & Tmatrix, string WhatForward, std::vector<float> joints);

	/**
	 * @fn FKvars filterForwardFromTo(std::string start, std::string stop, std::vector<float> jointsStart, std::vector<float> jointsEnd)
	 * @brief This function take's the name of the start point for the chain, the name for the end point and returns the cartesian values of the end effector.
	 * @param start. The name of the start point of the chain.
	 * @param stop. The name of the end point of the chain.
	 * @param jointsStart. One vector with all the joints for the chain of the start point.
	 * @param jointsEnd. One vector with all the joints for the chain of the end point.
	 * @returns FKVariables. The struct with the 3 cartesian points and with the 3 cartesian angles.
	 *
	 * @details Format of vector for filtering.
	 * */
	FKvars filterForwardFromTo(std::string start, std::string stop, std::vector<float> jointsStart, std::vector<float> jointsEnd);

	/**
	 * @fn FKvars forwardFromTo(std::string start, std::string stop, std::vector<float> jointsStart, std::vector<float> jointsEnd)
	 * @brief This function take's the name of the start point for the chain, the name for the end point and returns the transformation table.
	 * @param start. The name of the start point of the chain.
	 * @param stop. The name of the end point of the chain.
	 * @param jointsStart. One vector with all the joints for the chain of the start point.
	 * @param jointsEnd. One vector with all the joints for the chain of the end point.
	 * @returns Tamatrix1. The transformation matrix.
	 *
	 * @details Return the whole transformation table
	 * */
	kmatTable forwardFromTo(std::string start, std::string stop, std::vector<float> jointsStart, std::vector<float> jointsEnd);

	/**
	 * @fn FKvars calculateCenterOfMass(vector<float> allJoints)
	 * @brief Calculate the center of mass of the robot
	 * @param allJoints. all the joint of the robot. They must be Larm,Rarm,Lleg,Rleg,Head with that order.
	 * */
	//Makaronada code
	FKvars calculateCenterOfMass(vector<float> allJoints);

	/**
	 * vector<vector<float> > inverseHead(float px,float py,float pz, float rx, float ry, float rz, bool withAngles, bool topCamera)
	 * @brief Inverse Kinematics for the head (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation. For the head it's every time 0.0f so don't bother.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @param wihtAngles. If true, the problem will be solved only with angles, else only with the cartesian points.
	 * @param topCamera. If true, the top camera is chosen as end point, else the bottom camera is chosen.
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: HeadYaw,HeadPitch.
	 * */
	vector<vector<float> > inverseHead(float px, float py, float pz, float rx, float ry, float rz, bool withAngles, bool topCamera);

	/**
	 * vector<vector<float> > inverseLeftHand(float px,float py,float pz, float rx, float ry, float rz)
	 * @brief Inverse Kinematics for the left hand (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll
	 * */
	vector<vector<float> > inverseLeftHand(float px, float py, float pz, float rx, float ry, float rz);

	/**
	 * vector<vector<float> > inverseRightHand(float px,float py,float pz, float rx, float ry, float rz)
	 * @brief Inverse Kinematics for the right hand (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll
	 * */
	vector<vector<float> > inverseRightHand(float px, float py, float pz, float rx, float ry, float rz);
	/**
	 * vector<vector<float> > inverseLeftLeg(float px,float py,float pz, float rx, float ry, float rz)
	 * @brief Inverse Kinematics for the left leg (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: LHipYawPitch,LHipRoll,LHipPitch,LKneePitch,LAnklePitch,LAnkleRoll
	 * */
	vector<vector<float> > inverseLeftLeg(float px, float py, float pz, float rx, float ry, float rz);

	/**
	 * vector<vector<float> > inverseRightLeg(float px,float py,float pz, float rx, float ry, float rz)
	 * @brief Inverse Kinematics for the right leg (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: RHipYawPitch,RHipRoll,RHipPitch,RKneePitch,RAnklePitch,RAnkleRoll
	 * */
	vector<vector<float> > inverseRightLeg(float px, float py, float pz, float rx, float ry, float rz);

};

void NAOKinematics::forwardLeftHand(kmatTable & EndTransf, float ShoulderPitch, float ShoulderRoll, float ElbowYaw, float ElbowRoll)
{
	KMatTransf::makeDHTransformation(T1, 0.0f, -PI / 2, 0.0f, ShoulderPitch);
	KMatTransf::makeDHTransformation(T2, 0.0f, PI / 2, 0.0f, ShoulderRoll - PI / 2);
	KMatTransf::makeDHTransformation(T3, 0.0f, -PI / 2, UpperArmLength, ElbowYaw);
	KMatTransf::makeDHTransformation(T4, 0.0f, PI / 2, 0.0f, ElbowRoll);
	Tend = TBaseLArm;
	Tend *= T1;
	Tend *= T2;
	Tend *= T3;
	Tend *= T4;
	Tend *= RotLArm;
	Tend *= TEndLArm;
	EndTransf = Tend;
}


void NAOKinematics::forwardRightHand(kmatTable & EndTransf, float ShoulderPitch, float ShoulderRoll, float ElbowYaw, float ElbowRoll)
{
	KMatTransf::makeDHTransformation(T1, 0.0f, -PI / 2, 0.0f, ShoulderPitch);
	KMatTransf::makeDHTransformation(T2, 0.0f, PI / 2, 0.0f, ShoulderRoll + PI / 2); //Allagh apo matlab
	KMatTransf::makeDHTransformation(T3, 0.0f, -PI / 2, -UpperArmLength, ElbowYaw);
	KMatTransf::makeDHTransformation(T4, 0.0f, PI / 2, 0.0f, ElbowRoll); //Allagh apo matlab
	Tend = TBaseRArm;
	Tend *= T1;
	Tend *= T2;
	Tend *= T3;
	Tend *= T4;
	Tend *= RotRArm;
	Tend *= TEndRArm;
	Tend *= RotRArmFix;
	EndTransf = Tend;
}

void  NAOKinematics::forwardLeftLeg(kmatTable & EndTransf, float HipYawPitch, float HipRoll, float HipPitch, float KneePitch, float AnklePitch, float AnkleRoll)
{
	KMatTransf::makeDHTransformation(T1, 0.0f, -3 * PI / 4, 0.0f, HipYawPitch - PI / 2);
	KMatTransf::makeDHTransformation(T2, 0.0f, -PI / 2, 0.0f, HipRoll + PI / 4);
	KMatTransf::makeDHTransformation(T3, 0.0f, PI / 2, 0.0f, HipPitch);
	KMatTransf::makeDHTransformation(T4, -ThighLength, 0.0f, 0.0f, KneePitch);
	KMatTransf::makeDHTransformation(T5, -TibiaLength, 0.0f, 0.0f, AnklePitch);
	KMatTransf::makeDHTransformation(T6, 0.0f, -PI / 2, 0.0f, AnkleRoll);
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

void NAOKinematics::forwardRightLeg(kmatTable & EndTransf, float HipYawPitch, float HipRoll, float HipPitch, float KneePitch, float AnklePitch, float AnkleRoll)
{
	KMatTransf::makeDHTransformation(T1, 0.0f, -PI / 4, 0.0f, HipYawPitch - PI / 2);
	KMatTransf::makeDHTransformation(T2, 0.0f, -PI / 2, 0.0f, HipRoll - PI / 4); //allagh
	KMatTransf::makeDHTransformation(T3, 0.0f, PI / 2, 0.0f, HipPitch);
	KMatTransf::makeDHTransformation(T4, -ThighLength, 0.0f, 0.0f, KneePitch);
	KMatTransf::makeDHTransformation(T5, -TibiaLength, 0.0f, 0.0f, AnklePitch); //allagh
	KMatTransf::makeDHTransformation(T6, 0.0f, -PI / 2, 0.0f, AnkleRoll);
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

void NAOKinematics::forwardCamera(kmatTable & EndTransf, float HeadYaw, float HeadPitch, bool topCamera)
{
	KMatTransf::makeDHTransformation(T1, 0.0f, 0.0f, 0.0f, HeadYaw);
	KMatTransf::makeDHTransformation(T2, 0.0f, -PI / 2, 0.0f, HeadPitch - PI / 2);
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

void NAOKinematics::filterForward(kmatTable & Tmatrix, string WhatForward, std::vector<float> joints)
{
	Tmatrix.zero();
	std::vector<float>::iterator iter;
	iter = joints.begin();

	if(!WhatForward.compare("CameraTop"))
	{
		float HP = *iter;
		iter++;
		float HY = *iter;
		forwardCamera(Tmatrix, HP, HY, true);
	}
	else if(!WhatForward.compare("CameraBot"))
	{
		float HP = *iter;
		iter++;
		float HY = *iter;
		forwardCamera(Tmatrix, HP, HY, false);
	}
	else if(!WhatForward.compare("LeftArm") || !WhatForward.compare("LeftHand"))
	{
		float SP = *iter;
		iter++;
		float SR = *iter;
		iter++;
		float EY = *iter;
		iter++;
		float ER = *iter;
		forwardLeftHand(Tmatrix, SP, SR, EY, ER);
	}
	else if(!WhatForward.compare("RightArm") || !WhatForward.compare("RightHand"))
	{
		float SP = *iter;
		iter++;
		float SR = *iter;
		iter++;
		float EY = *iter;
		iter++;
		float ER = *iter;
		forwardRightHand(Tmatrix, SP, SR, EY, ER);
	}
	else if(!WhatForward.compare("LeftLeg"))
	{
		float HYP = *iter;
		iter++;
		float HR = *iter;
		iter++;
		float HP = *iter;
		iter++;
		float KP = *iter;
		iter++;
		float AP = *iter;
		iter++;
		float AR = *iter;
		forwardLeftLeg(Tmatrix, HYP, HR, HP, KP, AP, AR);
	}
	else if(!WhatForward.compare("RightLeg"))
	{
		float HYP = *iter;
		iter++;
		float HR = *iter;
		iter++;
		float HP = *iter;
		iter++;
		float KP = *iter;
		iter++;
		float AP = *iter;
		iter++;
		float AR = *iter;
		forwardRightLeg(Tmatrix, HYP, HR, HP, KP, AP, AR);
	}

	Tmatrix.check();
}

FKvars NAOKinematics::filterForwardFromTo(std::string start, std::string stop, std::vector<float> jointsStart, std::vector<float> jointsEnd)
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

kmatTable NAOKinematics::forwardFromTo(std::string start, std::string stop, std::vector<float> jointsStart, std::vector<float> jointsEnd)
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

FKvars NAOKinematics::calculateCenterOfMass(vector<float> allJoints)
{
	kmatTable endTr1, endTr2, endTr3, endTr4, endTr5, endTr6, temp;
	KMat::GenMatrix<float, 3, 1> lh1, lh2, lh3, lh4, rh1, rh2, rh3, rh4, ll1, ll2, ll3, ll4, ll5, ll6, rl1, rl2, rl3, rl4, rl5, rl6, h1, h2, t;
	float PI = KMatTransf::PI;
	//Left Hand
	KMatTransf::makeTranslation(endTr1, LShoulderPitchX, LShoulderPitchY, LShoulderPitchZ);
	KMatTransf::makeTranslation(endTr2, LShoulderRollX, LShoulderRollY, LShoulderRollZ);
	KMatTransf::makeTranslation(endTr3, LElbowYawX, LElbowYawY, LElbowYawZ);
	KMatTransf::makeTranslation(endTr4, LElbowRollX, LElbowRollY, LElbowRollZ);
	KMatTransf::makeTranslation(base, 0.0f, ShoulderOffsetY + ElbowOffsetY, ShoulderOffsetZ);
	KMatTransf::makeDHTransformation(T1, 0.0f, -PI / 2, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0f, PI / 2, 0.0f, allJoints.front() - PI / 2);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T3, 0.0f, -PI / 2, UpperArmLength, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T4, 0.0f, PI / 2, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= endTr1;
	lh1 = temp.get_translation();
	lh1.scalar_mult(LShoulderPitchMass);
	base *= T2;
	temp = base;
	temp *= endTr2;
	lh2 = temp.get_translation();
	lh2.scalar_mult(LShoulderRollMass);
	base *= T3;
	temp = base;
	temp *= endTr3;
	lh3 = temp.get_translation();
	lh3.scalar_mult(LElbowYawMass);
	base *= T4;
	temp = base;
	temp *= endTr4;
	lh4 = temp.get_translation();
	lh4.scalar_mult(LElbowRollMass);
	lh1 += lh2;
	lh1 += lh3;
	lh1 += lh4;
	//Right Hand
	KMatTransf::makeTranslation(endTr1, RShoulderPitchX, RShoulderPitchY, RShoulderPitchZ);
	KMatTransf::makeTranslation(endTr2, RShoulderRollX, RShoulderRollY, RShoulderRollZ);
	KMatTransf::makeTranslation(endTr3, RElbowYawX, RElbowYawY, RElbowYawZ);
	KMatTransf::makeTranslation(endTr4, RElbowRollX, RElbowRollY, RElbowRollZ);
	KMatTransf::makeTranslation(base, 0.0f, -(ShoulderOffsetY + ElbowOffsetY), allJoints.front());
	KMatTransf::makeDHTransformation(T1, 0.0f, -PI / 2, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0f, PI / 2, 0.0f, allJoints.front() + PI / 2);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T3, 0.0f, -PI / 2, -UpperArmLength, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T4, 0.0f, PI / 2, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= endTr1;
	rh1 = temp.get_translation();
	rh1.scalar_mult(RShoulderPitchMass);
	base *= T2;
	temp = base;
	temp *= endTr2;
	rh2 = temp.get_translation();
	rh2.scalar_mult(RShoulderRollMass);
	base *= T3;
	temp = base;
	temp *= endTr3;
	rh3 = temp.get_translation();
	rh3.scalar_mult(RElbowYawMass);
	base *= T4;
	temp = base;
	temp *= endTr4;
	rh4 = temp.get_translation();
	rh4.scalar_mult(RElbowRollMass);
	rh1 += rh2;
	rh1 += rh3;
	rh1 += rh4;
	//Left Leg
	KMatTransf::makeTranslation(endTr1, LHipYawPitchX, LHipYawPitchY, LHipYawPitchZ);
	KMatTransf::makeTranslation(endTr2, LHipRollX, LHipRollY, LHipRollZ);
	KMatTransf::makeTranslation(endTr3, LHipPitchX, LHipPitchY, LHipPitchZ);
	KMatTransf::makeTranslation(endTr4, LKneePitchX, LKneePitchY, LKneePitchZ);
	KMatTransf::makeTranslation(endTr5, LAnklePitchX, LAnklePitchY, LAnklePitchZ);
	KMatTransf::makeTranslation(endTr6, LAnkleRollX, LAnkleRollY, LAnkleRollZ);
	KMatTransf::makeTranslation(base, 0.0f, HipOffsetY, -HipOffsetZ);
	KMatTransf::makeDHTransformation(T1, 0.0f, -3 * PI / 4, 0.0f, allJoints.front() - PI / 2);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0f, -PI / 2, 0.0f, allJoints.front() + PI / 4);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T3, 0.0f, PI / 2, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T4, -ThighLength, 0.0f, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T5, -TibiaLength, 0.0f, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T6, 0.0f, -PI / 2, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= endTr1;
	ll1 = temp.get_translation();
	ll1.scalar_mult(LHipYawPitchMass);
	base *= T2;
	temp = base;
	temp *= endTr2;
	ll2 = temp.get_translation();
	ll2.scalar_mult(LHipRollMass);
	base *= T3;
	temp = base;
	temp *= endTr3;
	ll3 = temp.get_translation();
	ll3.scalar_mult(LHipPitchMass);
	base *= T4;
	temp = base;
	temp *= endTr4;
	ll4 = temp.get_translation();
	ll4.scalar_mult(LKneePitchMass);
	base *= T5;
	temp = base;
	temp *= endTr5;
	ll5 = temp.get_translation();
	ll5.scalar_mult(LAnklePitchMass);
	base *= T6;
	temp = base;
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
	KMatTransf::makeTranslation(base, 0.0f, -HipOffsetY, -HipOffsetZ);
	KMatTransf::makeDHTransformation(T1, 0.0f, -PI / 4, 0.0f, allJoints.front() - PI / 2);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0f, -PI / 2, 0.0f, allJoints.front() - PI / 4);
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T3, 0.0f, PI / 2, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T4, -ThighLength, 0.0f, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T5, -TibiaLength, 0.0f, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T6, 0.0f, -PI / 2, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= endTr1;
	rl1 = temp.get_translation();
	rl1.scalar_mult(RHipYawPitchMass);
	base *= T2;
	temp = base;
	temp *= endTr2;
	rl2 = temp.get_translation();
	rl2.scalar_mult(RHipRollMass);
	base *= T3;
	temp = base;
	temp *= endTr3;
	rl3 = temp.get_translation();
	rl3.scalar_mult(RHipPitchMass);
	base *= T4;
	temp = base;
	temp *= endTr4;
	rl4 = temp.get_translation();
	rl4.scalar_mult(RKneePitchMass);
	base *= T5;
	temp = base;
	temp *= endTr5;
	rl5 = temp.get_translation();
	rl5.scalar_mult(RAnklePitchMass);
	base *= T6;
	temp = base;
	temp *= endTr6;
	rl6 = temp.get_translation();
	rl6.scalar_mult(RAnkleRollMass);
	rl1 += rl2;
	rl1 += rl3;
	rl1 += rl4;
	rl1 += rl5;
	rl1 += rl6;
	//Head
	KMatTransf::makeTranslation(endTr1, HeadYawX, HeadYawY, HeadYawZ);
	KMatTransf::makeTranslation(endTr2, HeadPitchX, HeadPitchY, HeadPitchZ);
	KMatTransf::makeTranslation(base, 0.0f, 0.0f, NeckOffsetZ);
	KMatTransf::makeDHTransformation(T1, 0.0f, 0.0f, 0.0f, allJoints.front());
	allJoints.erase(allJoints.begin());
	KMatTransf::makeDHTransformation(T2, 0.0f, -PI / 2, 0.0f, allJoints.front() - PI / 2);
	allJoints.erase(allJoints.begin());
	base *= T1;
	temp = base;
	temp *= endTr1;
	h1 = temp.get_translation();
	h1.scalar_mult(HeadYawMass);
	base *= T2;
	temp = base;
	temp *= endTr2;
	h2 = temp.get_translation();
	h2.scalar_mult(HeadPitchMass);
	h1 += h2;
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
	float tmass = 1 / TotalMassH21;
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

vector<vector<float> > NAOKinematics::inverseHead(float px, float py, float pz, float rx, float ry, float rz, bool withAngles, bool topCamera)
{
	std::vector<float> fc, empty;
	std::vector<vector<float> > returnResult;
	FKvars output;
	KMatTransf::makeTransformation(T, px, py, pz, rx, ry, rz);
	float theta1, theta2;

	if(rx != 0.0f && withAngles)
		return returnResult;

	if(withAngles)
	{
		theta1 = rz;
		theta2 = ry;
		//---------------------------Forward validation step--------------------------------------------------------------------------------------
		fc.clear();
		fc.push_back(theta1);
		fc.push_back(theta2);

		if(topCamera)
			output = filterForwardFromTo("Torso", "CameraTop", empty, fc);
		else
			output = filterForwardFromTo("Torso", "CameraBot", empty, fc);

		float x = output.pointX, y = output.pointY, z = output.pointZ, ax = output.angleX, ay = output.angleY, az = output.angleZ;

		if(px - 1 <= x && x <= px + 1 && py - 1 <= y && y <= py + 1 && pz - 1 <= z && z <= pz + 1 && rx - 0.001 <= ax && rx + 0.001 >= ax && ry - 0.001 <= ay && ry + 0.001 >= ay && rz - 0.001 <= az && rz + 0.001 >= az)
		{
			returnResult.push_back(fc);
		}

		//------------------------------------------------------------------------------------------------------------------------------------------
	}
	else
	{
		float up = - pz + NeckOffsetZ;
		float downt2, downt1, psi;

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

		float tempTheta2 = asin(up / downt2); //-psi;
		float posOrNegPI = (tempTheta2 >= 0) ? PI : -PI;

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
			theta1 = acos(px / downt1);

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

			float x = output.pointX, y = output.pointY, z = output.pointZ;

			//Validate only the points
			if(px - 1 <= x && x <= px + 1 && py - 1 <= y && y <= py + 1 && pz - 1 <= z && z <= pz + 1)
			{
				returnResult.push_back(fc);
			}

			//-----------------------------------------------------------------------------------------------------------------------------------------
		}
	}

	return returnResult;
}

vector<vector<float> > NAOKinematics::inverseLeftHand(float px, float py, float pz, float rx, float ry, float rz)
{
	std::vector<float> flh, empty;
	std::vector<vector<float> > returnResult;
	KMatTransf::makeTransformation(Tinit, px, py, pz, rx, ry, rz);
	float startX = 0;
	float startY = ShoulderOffsetY + ElbowOffsetY;
	float startZ = ShoulderOffsetZ;
	float side1 = UpperArmLength;
	float side2 = LowerArmLength + HandOffsetX;
	float value1 = ShoulderOffsetY + ElbowOffsetY; //113
	float value2 = LowerArmLength + HandOffsetX; //113.7
	float value3 = UpperArmLength; //to allo
	//Calculate Theta 4
	float distance = sqrt(pow(startX - px, 2) + pow(startY - py, 2) + pow(startZ - pz, 2));
	float theta4 = PI - acos((pow(side1, 2) + pow(side2, 2) - pow(distance, 2)) / (2 * side1 * side2));
	theta4 = - theta4;

	if(theta4 != theta4 || theta4 > LElbowRollHigh || theta4 < LElbowRollLow)
		return returnResult;

	float cth4 = cos(theta4);
	float sth4 = sin(theta4);
	//Calculate Theta 2
	float equationForTheta2 = sin(rz) * sin(rx) * sin(ry) + cos(rz) * cos(rx);
	float upForTheta2 = py - value1 - (equationForTheta2 * value2 * sth4) / cth4;
	float downForTheta2 = value3 + value2 * cth4 + value2 * pow(sth4, 2) / cth4;
	float theta2temp = acos(upForTheta2 / downForTheta2);

	for(int i = 0; i < 2; i++)
	{
		float theta2 = theta2temp;

		if(i == 0 && (theta2 + PI / 2 > LShoulderRollHigh || theta2 + PI / 2 < LShoulderRollLow))
			continue;
		else if(i == 1 && (-theta2 + PI / 2 > LShoulderRollHigh || -theta2 + PI / 2 < LShoulderRollLow))
			continue;
		else if(i == 0)
			theta2 = theta2 + PI / 2;
		else
			theta2 = -theta2 + PI / 2;

		//Calculate Theta 3
		float equationForTheta3 = sin(rz) * sin(ry) * cos(rx) - cos(rz) * sin(rx);
		float upForTheta3 = equationForTheta3;
		float downForTheta3 = sin(theta2 - PI / 2);
		float theta3temp = asin(upForTheta3 / downForTheta3);
		float posOrNegPI = (theta3temp >= 0) ? PI : -PI;

		for(int j = 0; j < 2; j++)
		{
			float theta3 = theta3temp;

			if(j == 0 && (theta3 > LElbowYawHigh || theta3 < LElbowYawLow))
				continue;
			else if(j == 1 && (posOrNegPI - theta3 > LElbowYawHigh || posOrNegPI - theta3 < LElbowYawLow))
				continue;
			else if(j == 1)
				theta3 = posOrNegPI - theta3;

			//Calculate Theta 1
			float equation1ForTheta1 = cos(rz) * sin(ry) * cos(rx) + sin(rz) * sin(rx);
			float equation2ForTheta1 = cos(ry) * cos(rx);
			float theta1temp;

			if(cos(theta3) == 0)
			{
				theta1temp = acos(equation1ForTheta1 / cos(theta2 - PI / 2));
			}
			else
			{
				float upForTheta1 = equation2ForTheta1 + equation1ForTheta1 * sin(theta3) * cos(theta2 - PI / 2) / cos(theta3);
				float downForTheta1 = cos(theta3) + pow(cos(theta2 - PI / 2), 2) * pow(sin(theta3), 2) / cos(theta3);
				theta1temp = acos(upForTheta1 / downForTheta1);
			}

			for(int l = 0; l < 2; l++)
			{
				float theta1 = theta1temp;

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
				kmatTable test;
				filterForward(test, "LeftHand", flh);

				if(test.almostEqualTo(Tinit))
				{
					//std::cout << "Niki!!\nTheta 1 = " << theta1 << "\nTheta 2 = " << theta2 << "\nTheta 3 = " << theta3 << "\nTheta 4 = " << theta4 << std::endl;
					returnResult.push_back(flh);
				}
				else
				{
					;//std::cout << "Hta!!\nTheta 1 = " << theta1 << "Theta 2 = " << theta2 << "Theta 3 = " << theta3 << "Theta 4 = " << theta4 << std::endl;
				}

				//------------------------------------------------------------------------------------------------------------------------------------------
			}
		}
	}

	return returnResult;
}


vector<vector<float> > NAOKinematics::inverseRightHand(float px, float py, float pz, float rx, float ry, float rz)
{
	std::vector<float> frh, empty;
	std::vector<vector<float> > returnResult;
	//Rotate input to remvoe Rfix
	KMatTransf::makeTransformation(T, px, py, pz, rx, ry, rz);
	Tinit = T;
	T *= RotRArmFixInv;
	px = T(0, 3);
	py = T(1, 3);
	pz = T(2, 3);
	rx = atan2(T(2, 1), T(2, 2));
	ry = atan2(-T(2, 0), sqrt(pow(T(2, 1), 2) + pow(T(2, 2), 2)));
	rz = atan2(T(1, 0), T(0, 0));
	//continue with the rotated input
	float startX = 0;
	float startY = -ShoulderOffsetY - ElbowOffsetY;
	float startZ = ShoulderOffsetZ;
	float side1 = UpperArmLength;
	float side2 = LowerArmLength + HandOffsetX;
	float value1 = ShoulderOffsetY + ElbowOffsetY; //113
	float value2 = LowerArmLength + HandOffsetX; //113.7
	float value3 = UpperArmLength; //to allo
	//Calculate Theta 4
	float distance = sqrt(pow(startX - px, 2) + pow(startY - py, 2) + pow(startZ - pz, 2));
	float theta4 = PI - acos((pow(side1, 2) + pow(side2, 2) - pow(distance, 2)) / (2 * side1 * side2));
	theta4 = theta4;

	if(theta4 != theta4 || theta4 > RElbowRollHigh || theta4 < RElbowRollLow)
		return returnResult;

	float cth4 = cos(theta4);
	float sth4 = sin(theta4);
	//Calculate Theta 2
	float equationForTheta2 = sin(rz) * sin(rx) * sin(ry) + cos(rz) * cos(rx);
	float upForTheta2 = - py - value1 - (equationForTheta2 * value2 * sth4) / cth4;
	float downForTheta2 = value3 + value2 * cth4 + value2 * pow(sth4, 2) / cth4;
	float theta2temp = acos(upForTheta2 / downForTheta2);

	for(int i = 0; i < 2; i++)
	{
		float theta2 = theta2temp;

		if(i == 0 && (theta2 - PI / 2 > RShoulderRollHigh || theta2 - PI / 2 < RShoulderRollLow))
			continue;
		else if(i == 1 && (-theta2 - PI / 2 > RShoulderRollHigh || -theta2 - PI / 2 < RShoulderRollLow))
			continue;
		else if(i == 0)
			theta2 = theta2 - PI / 2;
		else
			theta2 = -theta2 - PI / 2;

		//Calculate Theta 3
		float equationForTheta3 = sin(rz) * sin(ry) * cos(rx) - cos(rz) * sin(rx);
		float upForTheta3 = equationForTheta3;
		float downForTheta3 = sin(theta2 + PI / 2);
		float theta3temp = asin(upForTheta3 / downForTheta3);
		float posOrNegPI = (theta3temp >= 0) ? PI : -PI;

		for(int j = 0; j < 2; j++)
		{
			float theta3 = theta3temp;

			if(j == 0 && (theta3 > RElbowYawHigh || theta3 < RElbowYawLow))
				continue;
			else if(j == 1 && (posOrNegPI - theta3 > RElbowYawHigh || posOrNegPI - theta3 < RElbowYawLow))
				continue;
			else if(j == 1)
				theta3 = posOrNegPI - theta3;

			//Calculate Theta 1
			float equation1ForTheta1 = cos(rz) * sin(ry) * cos(rx) + sin(rz) * sin(rx);
			float equation2ForTheta1 = cos(ry) * cos(rx);
			float theta1temp;

			if(cos(theta3) == 0)
			{
				theta1temp = acos(equation1ForTheta1 / cos(theta2 - PI / 2));
			}
			else
			{
				float upForTheta1 = equation2ForTheta1 + equation1ForTheta1 * sin(theta3) * cos(theta2 + PI / 2) / cos(theta3);
				float downForTheta1 = cos(theta3) + pow(cos(theta2 + PI / 2), 2) * pow(sin(theta3), 2) / cos(theta3);
				theta1temp = acos(upForTheta1 / downForTheta1);
			}

			for(int l = 0; l < 2; l++)
			{
				float theta1 = theta1temp;

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
				kmatTable test;
				filterForward(test, "RightHand", frh);

				if(test.almostEqualTo(Tinit))
				{
					//std::cout << "Niki!!\nTheta 1 = " << theta1 << "\nTheta 2 = " << theta2 << "\nTheta 3 = " << theta3 << "\nTheta 4 = " << theta4 << std::endl;
					returnResult.push_back(frh);
				}
				else
				{
					;//std::cout << "Htaa!!\nTheta 1 = " << theta1 << "Theta 2 = " << theta2 << "Theta 3 = " << theta3 << "Theta 4 = " << theta4 << std::endl;
				}

				//-----------------------------------------------------------------------------------------------------------------------------------------
			}
		}
	}

	frh.clear();
	return returnResult;
}

vector<vector<float> > NAOKinematics::inverseLeftLeg(float px, float py, float pz, float rx, float ry, float rz)
{
	std::vector<float> fll, empty;
	std::vector<vector<float> > returnResult;
	KMatTransf::makeTransformation(T, px, py, pz, rx, ry, rz);
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
	float startX = 0;
	float startY = 0;
	float startZ = 0;
	float side1 = ThighLength;
	float side2 = TibiaLength;
	float pxInvert = T(0, 3);
	float pyInvert = T(1, 3);
	float pzInvert = T(2, 3);
	//Calculate Theta 4
	float distance = sqrt(pow(startX - pxInvert, 2) + pow(startY - pyInvert, 2) + pow(startZ - pzInvert, 2));
	float theta4 = PI - acos((pow(side1, 2) + pow(side2, 2) - pow(distance, 2)) / (2 * side1 * side2));

	if(theta4 != theta4)
		return returnResult;

	float theta6 = atan(pyInvert / pzInvert);

	if(theta6 < LAnkleRollLow || theta6 > LAnkleRollHigh)
		return returnResult;

	KMatTransf::makeDHTransformation(T6i, 0.0f, -PI / 2, 0.0f, theta6);
	T6i *= RotRLeg;

	try
	{
		T6i.fast_invert();
		Tstart *= T6i;
		TtempTheta5 = Tstart;
		TtempTheta5.fast_invert();
	}
	catch(KMat::SingularMatrixInvertionException d)
	{
		return returnResult;
	}

	for(int itter = 0; itter < 2; itter++)
	{
		theta4 = (itter == 0) ? theta4 : -theta4;

		if(theta4 < RKneePitchLow || theta4 > RKneePitchHigh)
			continue;

		KMatTransf::makeDHTransformation(T4i, -ThighLength, 0.0f, 0.0f, theta4);
		float up = TtempTheta5(1, 3) * (TibiaLength + ThighLength * cos(theta4)) + ThighLength * TtempTheta5(0, 3) * sin(theta4);
		float down = pow(ThighLength, 2) * pow(sin(theta4), 2) + pow(TibiaLength + ThighLength * cos(theta4), 2);
		float theta5 = asin(-up / down);
		float posOrNegPIt5 = (theta5 >= 0) ? PI : -PI;

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

			KMatTransf::makeDHTransformation(T5i, -TibiaLength, 0.0f, 0.0f, theta5);
			Ttemp = T4i;
			Ttemp *= T5i;

			try
			{
				Ttemp.fast_invert();
			}
			catch(KMat::SingularMatrixInvertionException d)
			{
				continue;
			}

			Ttemp2 = Tstart;
			Ttemp2 *= Ttemp;
			float temptheta2 = acos(Ttemp2(1, 2));
			float theta2;

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

				float theta3 = asin(Ttemp2(1, 1) / sin(theta2 + PI / 4));
				float posOrNegPIt3 = (theta3 >= 0) ? PI : -PI;

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

					float temptheta1 = acos(Ttemp2(0, 2) / sin(theta2 + PI / 4));

					if(temptheta1 != temptheta1)
						temptheta1 = 0;

					for(int p = 0; p < 2; p++)
					{
						float theta1;

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

vector<vector<float> > NAOKinematics::inverseRightLeg(float px, float py, float pz, float rx, float ry, float rz)
{
	std::vector<float> frl, empty;
	std::vector<vector<float> > returnResult;
	KMatTransf::makeTransformation(T, px, py, pz, rx, ry, rz);
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
	float startX = 0;
	float startY = 0;
	float startZ = 0;
	float side1 = ThighLength;
	float side2 = TibiaLength;
	float pxInvert = T(0, 3);
	float pyInvert = T(1, 3);
	float pzInvert = T(2, 3);
	//Calculate Theta 4
	float distance = sqrt(pow(startX - pxInvert, 2) + pow(startY - pyInvert, 2) + pow(startZ - pzInvert, 2));
	float theta4 = PI - acos((pow(side1, 2) + pow(side2, 2) - pow(distance, 2)) / (2 * side1 * side2));

	if(theta4 != theta4)
		return returnResult;

	float theta6 = atan(pyInvert / pzInvert);

	if(theta6 < RAnkleRollLow || theta6 > RAnkleRollHigh)
		return returnResult;

	KMatTransf::makeDHTransformation(T6i, 0.0f, -PI / 2, 0.0f, theta6);
	T6i *= RotRLeg;

	try
	{
		T6i.fast_invert();
		Tstart *= T6i;
		TtempTheta5 = Tstart;
		TtempTheta5.fast_invert();
	}
	catch(KMat::SingularMatrixInvertionException d)
	{
		return returnResult;
	}

	for(int itter = 0; itter < 2; itter++)
	{
		theta4 = (itter == 0) ? theta4 : -theta4;

		if(theta4 < RKneePitchLow || theta4 > RKneePitchHigh)
			continue;

		KMatTransf::makeDHTransformation(T4i, -ThighLength, 0.0f, 0.0f, theta4);
		float up = TtempTheta5(1, 3) * (TibiaLength + ThighLength * cos(theta4)) + ThighLength * TtempTheta5(0, 3) * sin(theta4);
		float down = pow(ThighLength, 2) * pow(sin(theta4), 2) + pow(TibiaLength + ThighLength * cos(theta4), 2);
		float theta5 = asin(-up / down);
		float posOrNegPIt5 = (theta5 >= 0) ? PI : -PI;

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

			KMatTransf::makeDHTransformation(T5i, -TibiaLength, 0.0f, 0.0f, theta5);
			Ttemp = T4i;
			Ttemp *= T5i;

			try
			{
				Ttemp.fast_invert();
			}
			catch(KMat::SingularMatrixInvertionException d)
			{
				continue;
			}

			Ttemp2 = Tstart;
			Ttemp2 *= Ttemp;
			float temptheta2 = acos(Ttemp2(1, 2));
			float theta2;

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

				float theta3 = asin(Ttemp2(1, 1) / sin(theta2 - PI / 4));
				float posOrNegPIt3 = (theta3 >= 0) ? PI : -PI;

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

					float temptheta1 = acos(Ttemp2(0, 2) / sin(theta2 - PI / 4));

					if(temptheta1 != temptheta1)
						temptheta1 = 0;

					for(int p = 0; p < 2; p++)
					{
						float theta1;

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
#endif
