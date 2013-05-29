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

#include <iostream>
using namespace std;
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

class NAOKinematics
{
public:
	double PI;
	/**
	*@struct FKvars
	*@brief This struct contains all the cartesian points and angles that we extract from the forward kinematics
	*/
	struct FKvars
	{
		double pointX, pointY, pointZ;
		double angleX, angleY, angleZ;
	};
	typedef KMath::KMat::ATMatrix<double, 4> kmatTable;

private:
	typedef KMath::KMat::transformations KMatTransf;
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
	kmatTable TBaseLArmInv, TEndAndRotLArmInv, TLArmOldR, TLArmOldEnd;
	kmatTable TBaseRArmInv, TEndAndRotRArmInv, TRArmOldR, TRArmOldEnd;
	//Predifined only for Center of Mass
	kmatTable RotLHshouldP, RotLHshoulR, RotLHelbowY, RotLHelbowR;
	kmatTable RotRHshouldP, RotRHshoulR, RotRHelbowY, RotRHelbowR, RotRHwristY;
	kmatTable RotLLhipYP, RotLLhipR, RotLLallPitchs;
	kmatTable RotRLhipYP, RotRLhipR, RotRLallPitchs;

public:
	NAOKinematics()
	{
		PI = KMatTransf::PI;
		//Head
		KMatTransf::makeTranslation(TBaseHead, 0.0, 0.0, NeckOffsetZ);
		KMatTransf::makeRotationXYZ(RotHead, PI / 2, PI / 2, 0.0);
		KMatTransf::makeTranslation(TEndHead1, CameraBotomX, 0.0, CameraBotomZ);
		KMatTransf::makeTranslation(TEndHead2, CameraTopX, 0.0, CameraTopZ);
		//Left Arm
		KMatTransf::makeTranslation(TBaseLArm, 0.0, ShoulderOffsetY + ElbowOffsetY, ShoulderOffsetZ);
		KMatTransf::makeRotationXYZ(RotLArm, PI / 2, 0.0, PI / 2);
		KMatTransf::makeTranslation(TEndLArm, HandOffsetX, 0.0, 0.0);
		
		TBaseLArmInv = TBaseLArm;
		TBaseLArmInv.fast_invert();
		
		kmatTable temp1, temp2;
		temp1 = RotLArm;
		temp1.fast_invert();
		temp2 = TEndLArm;
		temp2.fast_invert();
		TEndAndRotLArmInv = temp2;
		TEndAndRotLArmInv *= temp1;
		
		KMatTransf::makeRotationXYZ(TLArmOldR, 0.0, 0.0, PI / 2);
		KMatTransf::makeTranslation(TLArmOldEnd, HandOffsetX + LowerArmLength, 0.0, 0.0);
		//Right Arm
		KMatTransf::makeTranslation(TBaseRArm, 0.0, -(ShoulderOffsetY + ElbowOffsetY), ShoulderOffsetZ);
		KMatTransf::makeRotationXYZ(RotRArm, PI / 2, 0.0, PI / 2);
		KMatTransf::makeTranslation(TEndRArm, -HandOffsetX, 0.0, 0.0);
		KMatTransf::makeRotationXYZ(RotRArmFix, 0.0, 0.0, -PI);
		RotRArmFixInv = RotRArmFix.fast_invert();
		RotRArmFix.fast_invert();
		
		TBaseRArmInv = TBaseRArm;
		TBaseRArmInv.fast_invert();
		
		temp1 = RotRArm;
		temp1.fast_invert();
		temp2 = TEndRArm;
		temp2.fast_invert();
		TEndAndRotRArmInv = temp2;
		TEndAndRotRArmInv *= temp1;
		
		KMatTransf::makeRotationXYZ(TRArmOldR, 0.0, 0.0, PI / 2);
		KMatTransf::makeTranslation(TRArmOldEnd, -(HandOffsetX + LowerArmLength), 0.0, 0.0);
		//Left Leg
		KMatTransf::makeTranslation(TBaseLLeg, 0.0, HipOffsetY, -HipOffsetZ);
		KMatTransf::makeRotationZYX(RotLLeg, PI, -PI / 2, 0.0);
		KMatTransf::makeTranslation(TEndLLeg, 0.0, 0.0, -FootHeight);
		KMatTransf::makeRotationXYZ(RotFixLLeg, PI / 4, 0.0, 0.0);
		TBaseLLegInv = TBaseLLeg.fast_invert();
		TBaseLLeg.fast_invert();
		TEndLLegInv = TEndLLeg.fast_invert();
		TEndLLeg.fast_invert();
		//Right Leg
		KMatTransf::makeTranslation(TBaseBRLeg, 0.0, -HipOffsetY, -HipOffsetZ);
		KMatTransf::makeRotationZYX(RotRLeg, PI, -PI / 2, 0.0);
		KMatTransf::makeTranslation(TEndRLeg, 0.0, 0.0, -FootHeight);
		KMatTransf::makeRotationXYZ(RotFixRLeg, -PI / 4, 0.0, 0.0);
		TBaseRLegInv = TBaseBRLeg.fast_invert();
		TBaseBRLeg.fast_invert();
		TEndRLegInv = TEndRLeg.fast_invert();
		TEndRLeg.fast_invert();

		//For Center of Mass
		//Left hand
		KMatTransf::makeRotationXYZ(RotLHshouldP, PI/2, 0.0, 0.0);
		KMatTransf::makeRotationXYZ(RotLHshoulR, 0.0, 0.0, PI/2);
		KMatTransf::makeRotationXYZ(RotLHelbowY, PI/2, 0.0, PI/2);
		KMatTransf::makeRotationXYZ(RotLHelbowR, 0.0, 0.0, PI/2);
		//Right hand
		KMatTransf::makeRotationXYZ(RotRHshouldP, PI/2, 0.0, 0.0);
		KMatTransf::makeRotationXYZ(RotRHshoulR, 0.0, 0.0, -PI/2);
		KMatTransf::makeRotationXYZ(RotRHelbowY, PI/2, 0.0, -PI/2);
		KMatTransf::makeRotationXYZ(RotRHelbowR, 0.0, 0.0, -PI/2);
		KMatTransf::makeRotationXYZ(RotRHwristY, PI/2, 0.0, -PI/2);
		//Left leg
		KMatTransf::makeRotationXYZ(RotLLhipYP, 0.0, 3*PI/4, PI/2);
		KMatTransf::makeRotationXYZ(RotLLhipR, 0.0, PI/2, 0.0);
		KMatTransf::makeRotationXYZ(RotLLallPitchs, 0.0, PI/2, PI/2);
		//Right leg
		KMatTransf::makeRotationXYZ(RotRLhipYP, 0.0, PI/4, PI/2);
		KMatTransf::makeRotationXYZ(RotRLhipR, 0.0, PI/2, 0.0);
		KMatTransf::makeRotationXYZ(RotRLallPitchs, 0.0, PI/2, PI/2);
	}


	/**
	 * @fn void forwardLeftHand(kmatTable & EndTransf, double ShoulderPitch, double ShoulderRoll, double ElbowYaw, double ElbowRoll)
	 * @brief Forward kinematic for the left hand.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param ShoulderPitch. The value of the Left's arm shoulder pitch joint.
	 * @param ShoulderRoll. The value of the Left's arm shoulder roll joint.
	 * @param ElbowYaw. The value of the Left's arm elbow yaw joint.
	 * @param ElbowRoll. The value of the Left's arm elbow roll joint.
	 * @param EwristYaw. The value of the Left's arm wrist yaw joint.
	 * */
	void forwardLeftHand(kmatTable & EndTransf, double ShoulderPitch, double ShoulderRoll, double ElbowYaw, double ElbowRoll, double EwristYaw);

	/**
	 * @fn void forwardRightHand(kmatTable & EndTransf, double ShoulderPitch, double ShoulderRoll, double ElbowYaw, double ElbowRoll)
	 * @brief Forward kinematic for the right hand.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param ShoulderPitch. The value of the Right's arm shoulder pitch joint.
	 * @param ShoulderRoll. The value of the Right's arm shoulder roll joint.
	 * @param ElbowYaw. The value of the Right's arm elbow yaw joint.
	 * @param ElbowRoll. The value of the Right's arm elbow roll joint.
	 * @param EwristYaw. The value of the Right's arm wrist yaw joint.
	 * */
	void forwardRightHand(kmatTable & EndTransf, double ShoulderPitch, double ShoulderRoll, double ElbowYaw, double ElbowRoll, double EwristYaw);

	/**
	 * @fn void forwardLeftLeg(kmatTable & EndTransf, double HipYawPitch, double HipRoll, double HipPitch, double KneePitch, double AnklePitch, double AnkleRoll)
	 * @brief Forward kinematic for the left leg.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param HipYawPitch. The value of the Left's leg hip yaw pitch joint.
	 * @param HipRoll. The value of the Left's leg hip roll joint.
	 * @param HipPitch. The value of the Left's hip pitch yaw joint.
	 * @param KneePitch. The value of the Left's knee pitch roll joint.
	 * @param AnklePitch. The value of the Left's ankle pitch roll joint.
	 * @param AnkleRoll. The value of the Left's ankle elbow roll joint.
	 * */
	void forwardLeftLeg(kmatTable & EndTransf, double HipYawPitch, double HipRoll, double HipPitch, double KneePitch, double AnklePitch, double AnkleRoll);

	/**
	 * @fn void forwardRightLeg(kmatTable & EndTransf, double HipYawPitch, double HipRoll, double HipPitch, double KneePitch, double AnkleRoll, double AnklePitch)
	 * @brief Forward kinematic for the right leg.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param HipYawPitch. The value of the Right's leg hip yaw pitch joint.
	 * @param HipRoll. The value of the Right's leg hip roll joint.
	 * @param HipPitch. The value of the Right's hip pitch yaw joint.
	 * @param KneePitch. The value of the Right's knee pitch roll joint.
	 * @param AnklePitch. The value of the Right's ankle pitch roll joint.
	 * @param AnkleRoll. The value of the Right's ankle elbow roll joint.
	 * */
	void forwardRightLeg(kmatTable & EndTransf, double HipYawPitch, double HipRoll, double HipPitch, double KneePitch, double AnklePitch, double AnkleRoll);

	/**
	 * @fn void forwardCamera(kmatTable & EndTransf, double HeadYaw, double HeadPitch, bool topCamera)
	 * @brief Forward kinematic for the camera's on the head.
	 * @param EndTransf. The matrix table that we will return the result.
	 * @param HeadYaw. The value of the Head's yaw joint.
	 * @param HeadPitch. The value of the Head's pitch joint.
	 * @param topCamera. This value is true if we want forward kinematics for the top camera, or false if we want for the bottom camera.
	 * */
	void forwardCamera(kmatTable & EndTransf, double HeadYaw, double HeadPitch, bool topCamera);

	/**
	 * @fn void filterForward(kmatTable & Tmatrix, string WhatForward, std::vector<double> joints)
	 * @brief This function take the name of the end effector and one vector with joint and then it call's the apropriate function.
	 * @param Tmatrix. The matrix table that we will return the result.
	 * @param WhatForward. With this string, this function understands whate forward chain we want.
	 * @param joints. One vector with all the joints for the chain.
	 * */
	void filterForward(kmatTable & Tmatrix, std::string WhatForward, std::vector<double> joints);

	/**
	 * @fn FKvars filterForwardFromTo(std::string start, std::string stop, std::vector<double> jointsStart, std::vector<double> jointsEnd)
	 * @brief This function take's the name of the start point for the chain, the name for the end point and returns the cartesian values of the end effector.
	 * @param start. The name of the start point of the chain.
	 * @param stop. The name of the end point of the chain.
	 * @param jointsStart. One vector with all the joints for the chain of the start point.
	 * @param jointsEnd. One vector with all the joints for the chain of the end point.
	 * @returns FKVariables. The struct with the 3 cartesian points and with the 3 cartesian angles.
	 *
	 * @details Format of vector for filtering.
	 * */
	FKvars filterForwardFromTo(std::string start, std::string stop, std::vector<double> jointsStart, std::vector<double> jointsEnd);

	/**
	 * @fn FKvars forwardFromTo(std::string start, std::string stop, std::vector<double> jointsStart, std::vector<double> jointsEnd)
	 * @brief This function take's the name of the start point for the chain, the name for the end point and returns the transformation table.
	 * @param start. The name of the start point of the chain.
	 * @param stop. The name of the end point of the chain.
	 * @param jointsStart. One vector with all the joints for the chain of the start point.
	 * @param jointsEnd. One vector with all the joints for the chain of the end point.
	 * @returns Tamatrix1. The transformation matrix.
	 *
	 * @details Return the whole transformation table
	 * */
	kmatTable forwardFromTo(std::string start, std::string stop, std::vector<double> jointsStart, std::vector<double> jointsEnd);

	/**
	 * @fn FKvars calculateCenterOfMass(vector<double> allJoints)
	 * @brief Calculate the center of mass of the robot
	 * @param allJoints. all the joint of the robot. They must be Head,Larm,Lleg,Rleg,Rarm with that order.
	 * */
	//Makaronada code
	FKvars calculateCenterOfMass(std::vector<double> allJoints);

	/**
	 * vector<vector<double> > inverseHead(double px,double py,double pz, double rx, double ry, double rz, bool withAngles, bool topCamera)
	 * @brief Inverse Kinematics for the head (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation. For the head it's every time 0.0 so don't bother.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @param wihtAngles. If true, the problem will be solved only with angles, else only with the cartesian points.
	 * @param topCamera. If true, the top camera is chosen as end point, else the bottom camera is chosen.
	 * @returns vector<vector<double> >. It returns n vectors of double where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: HeadYaw,HeadPitch.
	 * */
	std::vector<std::vector<double> > inverseHead(double px, double py, double pz, double rx, double ry, double rz, bool withAngles, bool topCamera);
	std::vector<std::vector<double> > inverseHead(kmatTable targetPoint, bool withAngles, bool topCamera);

	/**
	 * vector<vector<double> > inverseLeftHand(double px,double py,double pz, double rx, double ry, double rz)
	 * @brief Inverse Kinematics for the left hand (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @returns vector<vector<double> >. It returns n vectors of double where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll
	 * */
	std::vector<std::vector<double> > inverseLeftHand(double px, double py, double pz, double rx, double ry, double rz);
	std::vector<std::vector<double> > inverseLeftHand(kmatTable targetPoint);

	/**
	 * vector<vector<double> > inverseRightHand(double px,double py,double pz, double rx, double ry, double rz)
	 * @brief Inverse Kinematics for the right hand (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @returns vector<vector<double> >. It returns n vectors of double where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll
	 * */
	std::vector<std::vector<double> > inverseRightHand(double px, double py, double pz, double rx, double ry, double rz);
	std::vector<std::vector<double> > inverseRightHand(kmatTable targetPoint);
	/**
	 * vector<vector<double> > inverseLeftLeg(double px,double py,double pz, double rx, double ry, double rz)
	 * @brief Inverse Kinematics for the left leg (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @returns vector<vector<double> >. It returns n vectors of double where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: LHipYawPitch,LHipRoll,LHipPitch,LKneePitch,LAnklePitch,LAnkleRoll
	 * */
	std::vector<std::vector<double> > inverseLeftLeg(double px, double py, double pz, double rx, double ry, double rz);
	std::vector<std::vector<double> > inverseLeftLeg(kmatTable targetPoint);

	/**
	 * vector<vector<double> > inverseRightLeg(double px,double py,double pz, double rx, double ry, double rz)
	 * @brief Inverse Kinematics for the right leg (DON'T try to understand the code, it's just maths)
	 * @param px. The x cartesian coordinate.
	 * @param py. The y cartesian coordinate.
	 * @param pz. The z cartesian coordinate.
	 * @param ax. The x rotation.
	 * @param ay. The y rotation.
	 * @param az. The z rotation.
	 * @returns vector<vector<double> >. It returns n vectors of double where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: RHipYawPitch,RHipRoll,RHipPitch,RKneePitch,RAnklePitch,RAnkleRoll
	 * */
	std::vector<std::vector<double> > inverseRightLeg(double px, double py, double pz, double rx, double ry, double rz);
	std::vector<std::vector<double> > inverseRightLeg(kmatTable targetPoint);

};

#endif
