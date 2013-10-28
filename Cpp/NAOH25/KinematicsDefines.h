#ifndef __DKIN_H__
#define __DKIN_H__

//Define for the lengths of the arms foots etc
#define ShoulderOffsetY 	98.0
#define ElbowOffsetY		15.0
#define UpperArmLength		105.0
#define ShoulderOffsetZ		100.0
#define LowerArmLength		57.75
#define HandOffsetX			55.95
#define HandOffsetZ			12.31
#define HipOffsetZ			85.0
#define HipOffsetY			50.0
#define ThighLength			100.0
#define TibiaLength			102.9
#define FootHeight			45.11
#define NeckOffsetZ			126.5
#define CameraBotomX		48.8
#define CameraBotomZ		23.81
#define CameraTopX			53.9
#define CameraTopZ			67.9

//Head Limits
#define HeadYawHigh			2.0857
#define HeadYawLow			-2.0857
#define HeadPitchHigh		0.5149
#define HeadPitchLow		-0.6720
//Left Hand limits
#define LShoulderPitchHigh	2.0857
#define LShoulderPitchLow 	-2.0857
#define LShoulderRollHigh	1.3265
#define LShoulderRollLow 	-0.3142
#define LElbowYawHigh		2.0875
#define LElbowYawLow 		-2.0875
#define LElbowRollHigh  	0.00001f//Aldebaran gives this value (-0.0349f) but the hand can go further
#define LElbowRollLow 		-1.5446
#define LWristYawHigh		1.8238
#define LWristYawLow		-1.8238
//Right Hand limits
#define RShoulderPitchHigh 	2.0857
#define RShoulderPitchLow 	-2.0857
#define RShoulderRollHigh	0.3142
#define RShoulderRollLow	-1.3265
#define RElbowYawHigh 		2.0875
#define RElbowYawLow 		-2.0875
#define RElbowRollHigh		1.5446
#define RElbowRollLow		0.00001f//Aldebaran gives this value (0.0349f) but the hand can go further
#define RWristYawHigh		1.8238
#define RWristYawLow		-1.8238
//Left Leg limits
#define LHipYawPitchHigh	0.7408
#define LHipYawPitchLow		-1.1453
#define LHipRollHigh		0.7904
#define LHipRollLow			-0.3794
#define LHipPitchHigh		0.4840
#define LHipPitchLow		-1.7739
#define LKneePitchHigh		2.1125
#define LKneePitchLow		-0.0923
#define LAnklePitchHigh		0.9227
#define LAnklePitchLow		-1.1895
#define LAnkleRollHigh		0.7690
#define LAnkleRollLow		-0.3978
//Left Right limits
#define RHipYawPitchHigh	0.7408
#define RHipYawPitchLow		-1.1453
#define RHipRollHigh		0.4147
#define RHipRollLow			-0.7383
#define RHipPitchHigh		0.4856
#define RHipPitchLow		-1.7723
#define RKneePitchHigh		2.1201
#define RKneePitchLow		-0.1030
#define RAnklePitchHigh		0.9320
#define RAnklePitchLow		-1.1864
#define RAnkleRollHigh		0.3886
#define RAnkleRollLow		-1.1864

//Masses defines
//Total mass
#define TotalMassH25		(5.182530+0.345)
//Torso
#define TorsoMass			1.04956
#define TorsoX				-4.13
#define TorsoY				0.09
#define TorsoZ				43.42

//Not provided by aldebaran
#define BatteryMass			0.345
#define BatteryX			-30.00
#define BatteryY			0.00
#define BatteryZ			39.00

//Head
#define HeadYawMass			0.06442
#define HeadYawX			-0.01
#define HeadYawY			0.14
#define HeadYawZ			-27.42

#define HeadPitchMass		0.60533
#define HeadPitchX			-1.12
#define HeadPitchY			0.0
#define HeadPitchZ			52.58

//Right Hand
#define RShoulderPitchMass	0.06996
#define RShoulderPitchX		-1.65
#define RShoulderPitchY		26.63
#define RShoulderPitchZ		0.14

#define RShoulderRollMass	0.15794
#define RShoulderRollX		24.29
#define RShoulderRollY		-9.52
#define RShoulderRollZ		0.32

#define RElbowYawMass		0.06483
#define RElbowYawX			-27.44
#define RElbowYawY			0.00
#define RElbowYawZ			-0.14

#define RElbowRollMass		0.07778
#define RElbowRollX			25.52
#define RElbowRollY			-2.81
#define RElbowRollZ			0.9

#define RWristYawMass		0.18533
#define RWristYawX			LowerArmLength+34.34
#define RWristYawY			-0.88
#define RWristYawZ			3.08

//Right Leg
#define RHipYawPitchMass	0.07118
#define RHipYawPitchX		-7.66
#define RHipYawPitchY		12.00
#define RHipYawPitchZ		27.16

#define RHipRollMass		0.13053
#define RHipRollX			-15.49
#define RHipRollY			-0.29
#define RHipRollZ			-5.16

#define RHipPitchMass		0.38976
#define RHipPitchX			1.39
#define RHipPitchY			-2.25
#define RHipPitchZ			-53.74

#define RKneePitchMass		0.29163
#define RKneePitchX			3.94
#define RKneePitchY			-2.21
#define RKneePitchZ			-49.38

#define RAnklePitchMass		0.13415
#define RAnklePitchX		0.45
#define RAnklePitchY		-0.3
#define RAnklePitchZ		6.84

#define RAnkleRollMass		0.16171
#define RAnkleRollX			25.42
#define RAnkleRollY			-3.32
#define RAnkleRollZ			-32.39
#endif
