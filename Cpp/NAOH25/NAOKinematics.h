#ifndef __NAOKIN_H__
#define __NAOKIN_H__
#include <iostream>
#include <stdexcept>
#include <string.h>
#include <vector>
#include <iomanip>
#include <limits>
#include <math.h>

#include "KMat.hpp"
#include "robotConsts.h"
/**
 * This is the code for the Forward and Inverse Kinematics for nao v3.3 robot.

 * @author Kofinas Nikos aka eldr4d, 2012 kouretes team
 * @author Vosk
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
 * \file NAOKinematics.h
 
 This code uses a "fake" chain on the NAO robot but it equevelant to the real one.
 There are several fixes to trasnform the "real" chain to the "fake" one and they are located at the NAOKinematics.cpp file and at the KinematicsDefines header.
 The change is that we move the wristYaw angle immediately after the ElbowRoll but there is no difference! (see detailed report for the reason))
 The results are THE SAME!
*/

class NAOKinematics
{
public:
	enum Effectors{ EFF_START=0, EFF_CAMERA_BOT=KDeviceLists::CHAINS_SIZE, EFF_CAMERA_TOP, EFFECTOR_SIZE };
	enum Frames {
		FR_BASE=0,
		FR_BASE_T=FR_BASE+KDeviceLists::NUMOFJOINTS,
		FR_END_T=FR_BASE_T+KDeviceLists::CHAINS_SIZE,
		FR_SIZE=FR_END_T+EFFECTOR_SIZE,
	};

	/**
	*@struct FKvars
	*@brief This struct contains all the cartesian points and angles that we extract from the forward kinematics
	*/
	struct FKvars
	{
		KVecDouble3 p;
		KVecDouble3 a;
	};
	typedef KMath::KMat::ATMatrix<double, 4> kmatTable;
	typedef KMath::KMat::GenMatrix<double, 4, 4> kmatJacobianTable;
	typedef float AngleType;
	typedef std::vector<std::vector<AngleType> > AngleContainer ;
private:

	std::vector<kmatTable> T;
	std::vector<kmatJacobianTable> Tjacobian;

	std::vector<AngleType> joints;
	std::vector<KVecDouble3> coms;
	std::vector<float> masses;

	kmatTable  TBaseLLegInv, //LLEG
			   TEndLLegInv,
			   RotFixLLeg,
			   RotRLeg;

	typedef KMath::KMat::transformations KMatTransf;
	/**
	 * @fn void prepareForward(KDeviceLists::ChainNames)
	 * @param ef Which chain or effector
	 * @brief prepare dh matrices
	 * */
	void prepareForward(KDeviceLists::ChainsNames=KDeviceLists::CHAINS_SIZE);
	/**
	 * @fn void prepareDerivatives(KDeviceLists::ChainsNames)
	 * @param ef Which chain or effector
	 * @brief prepare dhderivative matrices
	 * */
	void prepareDerivatives(KDeviceLists::ChainsNames=KDeviceLists::CHAINS_SIZE);

	KVecDouble3 calculateCoMChain(Frames start, Frames BaseFrame, int startdown,float &mass);

public:

	NAOKinematics();
	/**
	 * @fn void setJoints(std::vector<float> joints)
	 * @brief Set Joint values
	 * */
	bool setJoints(std::vector<AngleType> jointsset);
	bool setChain(KDeviceLists::ChainsNames ch,std::vector<AngleType> jointsset);

	/**
	 * @fn kmatTable  getForwardEffector(Effector ef)
	 * @brief Get Forward Kinematics Solution
	 * */
	kmatTable  getForwardEffector(Effectors ef);
	
	/**
	 * @fn kmatTable  forwardFromTo(Effectors start, Effectors stop)
	 * @brief This function take's the name of the start point for the chain, the name for the end point and returns the transformation table.
	 * @param start. The name of the start point of the chain.
	 * @param stop. The name of the end point of the chain.
	 * @returns kmatTable. The transformation matrix.
	 *
	 * @details Return the whole transformation table
	 * */
	kmatTable getForwardFromTo(Effectors start, Effectors stop);

	/**
	 * @fn KVecFloat3 calculateCenterOfMass()
	 * @brief Calculate the center of mass of the robot
	 * */
	KVecDouble3 calculateCenterOfMass();

	/**
	 * vector<vector<float> > inverseHead(FKvars s, bool withAngles, bool topCamera)
	 * @brief Inverse Kinematics for the head (DON'T try to understand the code, it's just maths)
	 * @param wihtAngles. If true, the problem will be solved only with angles, else only with the cartesian points.
	 * @param topCamera. If true, the top camera is chosen as end point, else the bottom camera is chosen.
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: HeadYaw,HeadPitch.
	 * */
	AngleContainer inverseHead(const FKvars s, bool withAngles, bool topCamera);
	AngleContainer inverseHead(kmatTable targetPoint, bool withAngles, bool topCamera);
	AngleContainer jacobianInverseHead(const FKvars s, bool topCamera);
	AngleContainer jacobianInverseHead(kmatTable targetPoint, bool topCamera);

	/**
	 * vector<vector<float> > inverseLeftHand(const FKvars s);
	 * @brief Inverse Kinematics for the left hand (DON'T try to understand the code, it's just maths)
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: LShoulderPitch,LShoulderRoll,LElbowYaw,LElbowRoll
	 * */
	AngleContainer inverseLeftHand(const FKvars s);
	AngleContainer inverseLeftHand(kmatTable targetPoint);
	AngleContainer jacobianInverseLeftHand(const FKvars s);
	AngleContainer jacobianInverseLeftHand(kmatTable targetPoint);

	/**
	 * vector<vector<float> > inverseRightHand(const FKvars s)
	 * @brief Inverse Kinematics for the right hand (DON'T try to understand the code, it's just maths)
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: RShoulderPitch,RShoulderRoll,RElbowYaw,RElbowRoll
	 * */
	AngleContainer inverseRightHand(const FKvars s);
	AngleContainer inverseRightHand(kmatTable targetPoint);
	AngleContainer jacobianInverseRightHand(const FKvars s);
	AngleContainer jacobianInverseRightHand(kmatTable targetPoint);
	
	/**
	 * vector<vector<float> > inverseLeftLeg(const FKvars s)
	 * @brief Inverse Kinematics for the left leg (DON'T try to understand the code, it's just maths)
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: LHipYawPitch,LHipRoll,LHipPitch,LKneePitch,LAnklePitch,LAnkleRoll
	 * */
	AngleContainer inverseLeftLeg(const FKvars s);
	AngleContainer inverseLeftLeg(kmatTable targetPoint);
	AngleContainer jacobianInverseLeftLeg(const FKvars s);
	AngleContainer jacobianInverseLeftLeg(kmatTable targetPoint);
	
	/**
	 * vector<vector<float> > inverseRightLeg(float px,float py,float pz, float rx, float ry, float rz)
	 * @brief Inverse Kinematics for the right leg (DON'T try to understand the code, it's just maths)
	 * @returns vector<vector<float> >. It returns n vectors of float where n is the number of solutions (almost every time it's 0 or 1).
		Each solutions vector contains the angles with this order: RHipYawPitch,RHipRoll,RHipPitch,RKneePitch,RAnklePitch,RAnkleRoll
	 * */
	AngleContainer inverseRightLeg(const FKvars s);
	AngleContainer inverseRightLeg(kmatTable targetPoint);
	AngleContainer jacobianInverseRightLeg(const FKvars s);
	AngleContainer jacobianInverseRightLeg(kmatTable targetPoint);
	
private:
	//Jacobian Inverse Kinematics
	AngleContainer jacobianInverseHead(kmatTable targetPoint, KDeviceLists::ChainsNames ch, bool topCamera);
	AngleContainer jacobianInverseHands(kmatTable targetPoint, KDeviceLists::ChainsNames ch);
	AngleContainer jacobianInverseLegs(kmatTable targetPoint, KDeviceLists::ChainsNames ch);
	
	
	static kmatTable getTransformation(const FKvars s)
	{
		kmatTable T;
		KMatTransf::makeTransformation(T, s.p(0), s.p(1), s.p(2), s.a(0), s.a(1), s.a(2));
		return T;
	}
	
	static void mirrorTransformation(kmatTable & targetPoint)
	{
		//This can be proved by the analytical construction of a rotation matrix from axis-angle rotation
		targetPoint(0,1)=-targetPoint(0,1);
		//targetPoint(0,2)=-targetPoint(0,2);
		targetPoint(1,0)=-targetPoint(1,0);
		targetPoint(1,2)=-targetPoint(1,2);
		//targetPoint(2,0)=-targetPoint(2,0);
		targetPoint(2,1)=-targetPoint(2,1);
		//Mirror Y translation
		targetPoint(1,3)=-targetPoint(1,3);
	}
		
};

#endif
