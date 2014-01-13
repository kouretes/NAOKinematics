/**
 * robot_consts.h
 *
 *  Created on: Dec 9, 2010
 *      Author: trs
 */

#ifndef ROBOT_CONSTS_H_
#define ROBOT_CONSTS_H_
#include <vector>
#include <map>
#include <string>

namespace KDeviceLists
{
	enum ChainHeadNames
	{
	    YAW = 0, PITCH, HEAD_SIZE
	};

	enum ChainArmNames
	{
	    SHOULDER_PITCH = 0, SHOULDER_ROLL, ELBOW_YAW, ELBOW_ROLL, WRIST_YAW, /*HAND,*/ARM_SIZE
	};

	enum ChainLegNames
	{
	    HIP_YAW_PITCH = 0, HIP_ROLL, HIP_PITCH, KNEE_PITCH, ANKLE_PITCH, ANKLE_ROLL, LEG_SIZE
	};

	enum ChainsNames
	{
		CHAIN_HEAD = 0, CHAIN_L_ARM, CHAIN_L_LEG, CHAIN_R_LEG, CHAIN_R_ARM, CHAINS_SIZE
	};

	enum JointNames
	{
	    HEAD = 0, L_ARM = HEAD_SIZE, L_LEG = L_ARM + ARM_SIZE, R_LEG = L_LEG + LEG_SIZE, R_ARM = R_LEG + LEG_SIZE, NUMOFJOINTS = R_ARM + ARM_SIZE

	};
};
#endif /* ROBOT_CONSTS_H_ */
