#ifndef __DKIN_H__
#define __DKIN_H__

//Define for the lengths of the arms foots etc
#define ShoulderOffsetY 	98.0f
#define ElbowOffsetY		15.0f
#define UpperArmLength		105.0f
#define ShoulderOffsetZ		100.0f
#define LowerArmLength		57.75f
#define HandOffsetX			55.95f
#define HipOffsetZ			85.0f
#define HipOffsetY			50.0f
#define ThighLength			100.0f
#define TibiaLength			102.9f
#define FootHeight			45.11f
#define NeckOffsetZ			126.5f
#define CameraBotomX		48.8f
#define CameraBotomZ		23.81f
#define CameraTopX			53.9f
#define CameraTopZ			67.9f

//Head Limits
#define HeadYawHigh			2.0857f
#define HeadYawLow			-2.0857f
#define HeadPitchHigh		0.5149f
#define HeadPitchLow		-0.6720f
//Left Hand limits
#define LShoulderPitchHigh	2.0857f
#define LShoulderPitchLow 	-2.0857f
#define LShoulderRollHigh	1.3265f
#define LShoulderRollLow 	-0.3142f
#define LElbowYawHigh		2.0875f
#define LElbowYawLow 		-2.0875f
#define LElbowRollHigh  	0.00001f//Aldebaran gives this value (-0.0349f) but the hand can go further
#define LElbowRollLow 		-1.5446f
//Right Hand limits
#define RShoulderPitchHigh 	2.0857f
#define RShoulderPitchLow 	-2.0857f
#define RShoulderRollHigh	0.3142f
#define RShoulderRollLow	-1.3265f
#define RElbowYawHigh 		2.0875f
#define RElbowYawLow 		-2.0875f
#define RElbowRollHigh		1.5446f
#define RElbowRollLow		0.00001f//Aldebaran gives this value (0.0349f) but the hand can go further
//Left Leg limits
#define LHipYawPitchHigh	0.7408f
#define LHipYawPitchLow		-1.1453f
#define LHipRollHigh		0.7904f
#define LHipRollLow			-0.3794f
#define LHipPitchHigh		0.4840f
#define LHipPitchLow		-1.7739f
#define LKneePitchHigh		2.1125f
#define LKneePitchLow		-0.0923f
#define LAnklePitchHigh		0.9227f
#define LAnklePitchLow		-1.1895f
#define LAnkleRollHigh		0.7690f
#define LAnkleRollLow		-0.3978f
//Left Right limits
#define RHipYawPitchHigh	0.7408f
#define RHipYawPitchLow		-1.1453f
#define RHipRollHigh		0.4147f
#define RHipRollLow			-0.7383f
#define RHipPitchHigh		0.4856f
#define RHipPitchLow		-1.7723f
#define RKneePitchHigh		2.1201f
#define RKneePitchLow		-0.1030f
#define RAnklePitchHigh		0.9320f
#define RAnklePitchLow		-1.1864f
#define RAnkleRollHigh		0.3886f
#define RAnkleRollLow		-1.1864f

//Masses defines
//Total mass
#define TotalMassH21		4.879f
//Torso
#define TorsoMass			1.03948f
#define TorsoX				-4.15f
#define TorsoY				0.07f
#define TorsoZ				42.58f
//Head
#define HeadYawMass			0.05930f
#define HeadYawX			-0.02f
#define HeadYawY			0.17f
#define HeadYawZ			-25.56f

#define HeadPitchMass		0.52065f
#define HeadPitchX			1.20f
#define HeadPitchY			-0.84f
#define HeadPitchZ			53.53f
//Left Hand
#define LShoulderPitchMass	0.06996f
#define LShoulderPitchX		-1.78f
#define LShoulderPitchY		-24.96f
#define LShoulderPitchZ		0.18f

#define LShoulderRollMass	0.12309f
#define LShoulderRollX		18.85f
#define LShoulderRollY		5.77f
#define LShoulderRollZ		0.65f

#define LElbowYawMass		0.05971f
#define LElbowYawX			-25.60f
#define LElbowYawY			-0.01f
#define LElbowYawZ			-0.19f

#define LElbowRollMass		0.18500f
#define LElbowRollX			65.36f
#define LElbowRollY			0.34f
#define LElbowRollZ			-0.02f
//Right Hand
#define RShoulderPitchMass	0.06996f
#define RShoulderPitchX		-1.78f
#define RShoulderPitchY		24.96f
#define RShoulderPitchZ		0.18f

#define RShoulderRollMass	0.12309f
#define RShoulderRollX		18.85f
#define RShoulderRollY		-5.77f
#define RShoulderRollZ		0.65f

#define RElbowYawMass		0.05971f
#define RElbowYawX			-25.60f
#define RElbowYawY			0.01f
#define RElbowYawZ			-0.19f

#define RElbowRollMass		0.18500f
#define RElbowRollX			65.36f
#define RElbowRollY			-0.34f
#define RElbowRollZ			-0.02f
//LeftLeg Leg
#define LHipYawPitchMass	0.07117f
#define LHipYawPitchX		-7.66f
#define LHipYawPitchY		-12.00f
#define LHipYawPitchZ		27.17f

#define LHipRollMass		0.13530f
#define LHipRollX			-16.49f
#define LHipRollY			0.29f
#define LHipRollZ			-4.75f

#define LHipPitchMass		0.39421
#define LHipPitchX			1.32f
#define LHipPitchY			2.35f
#define LHipPitchZ			-53.52f

#define LKneePitchMass		0.29159
#define LKneePitchX			4.22f
#define LKneePitchY			2.52f
#define LKneePitchZ			-48.68f

#define LAnklePitchMass		0.13892
#define LAnklePitchX		1.42f
#define LAnklePitchY		0.28f
#define LAnklePitchZ		6.38f

#define LAnkleRollMass		0.16175
#define LAnkleRollX			25.40f
#define LAnkleRollY			3.32f
#define LAnkleRollZ			-32.41f
//Right Leg
#define RHipYawPitchMass	0.07117f
#define RHipYawPitchX		-7.66f
#define RHipYawPitchY		12.00f
#define RHipYawPitchZ		27.17f

#define RHipRollMass		0.13530f
#define RHipRollX			-16.49f
#define RHipRollY			-0.29f
#define RHipRollZ			-4.75f

#define RHipPitchMass		0.39421
#define RHipPitchX			1.32f
#define RHipPitchY			-2.35f
#define RHipPitchZ			-53.52f

#define RKneePitchMass		0.29159
#define RKneePitchX			4.22f
#define RKneePitchY			-2.52f
#define RKneePitchZ			-48.68f

#define RAnklePitchMass		0.13892
#define RAnklePitchX		1.42f
#define RAnklePitchY		-0.28f
#define RAnklePitchZ		6.38f

#define RAnkleRollMass		0.16175
#define RAnkleRollX			25.40f
#define RAnkleRollY			-3.32f
#define RAnkleRollZ			-32.41f
#endif
