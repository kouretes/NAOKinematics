#ifndef __DKIN_H__
#define __DKIN_H__

//Define for the lengths of the arms foots etc
#define ShoulderOffsetY 	98.0
#define ElbowOffsetY		15.0
#define UpperArmLength		105.0
#define ShoulderOffsetZ		100.0
#define LowerArmLength		57.75
#define HandOffsetX			55.95
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
#define TotalMassH21		4.879
//Torso
#define TorsoMass			1.03948
#define TorsoX				-4.15
#define TorsoY				0.07
#define TorsoZ				42.58
//Head
#define HeadYawMass			0.05930
#define HeadYawX			-0.02
#define HeadYawY			0.17
#define HeadYawZ			-25.56

#define HeadPitchMass		0.52065
#define HeadPitchX			1.20
#define HeadPitchY			-0.84
#define HeadPitchZ			53.53
//Left Hand
#define LShoulderPitchMass	0.06996
#define LShoulderPitchX		-1.78
#define LShoulderPitchY		-24.96
#define LShoulderPitchZ		0.18

#define LShoulderRollMass	0.12309
#define LShoulderRollX		18.85
#define LShoulderRollY		5.77
#define LShoulderRollZ		0.65

#define LElbowYawMass		0.05971
#define LElbowYawX			-25.60
#define LElbowYawY			-0.01
#define LElbowYawZ			-0.19

#define LElbowRollMass		0.07761
#define LElbowRollX			25.56
#define LElbowRollY			-2.81
#define LElbowRollZ			0.76

#define LWristYawMass		0.18533
#define LWristYawX			34.34
#define LWristYawY			-0.88
#define LWristYawZ			3.08
//Right Hand
#define RShoulderPitchMass	0.06996
#define RShoulderPitchX		-1.78
#define RShoulderPitchY		24.96
#define RShoulderPitchZ		0.18

#define RShoulderRollMass	0.12309
#define RShoulderRollX		18.85
#define RShoulderRollY		-5.77
#define RShoulderRollZ		0.65

#define RElbowYawMass		0.05971
#define RElbowYawX			-25.60
#define RElbowYawY			0.01
#define RElbowYawZ			-0.19

#define RElbowRollMass		0.07778
#define RElbowRollX			25.52
#define RElbowRollY			-2.81
#define RElbowRollZ			0.9

#define RWristYawMass		0.18533
#define RWristYawX			34.34
#define RWristYawY			-0.88
#define RWristYawZ			3.08
//LeftLeg Leg
#define LHipYawPitchMass	0.07117
#define LHipYawPitchX		-7.66
#define LHipYawPitchY		-12.00
#define LHipYawPitchZ		27.17

#define LHipRollMass		0.13530
#define LHipRollX			-16.49
#define LHipRollY			0.29
#define LHipRollZ			-4.75

#define LHipPitchMass		0.39421
#define LHipPitchX			1.32
#define LHipPitchY			2.35
#define LHipPitchZ			-53.52

#define LKneePitchMass		0.29159
#define LKneePitchX			4.22
#define LKneePitchY			2.52
#define LKneePitchZ			-48.68

#define LAnklePitchMass		0.13892
#define LAnklePitchX		1.42
#define LAnklePitchY		0.28
#define LAnklePitchZ		6.38

#define LAnkleRollMass		0.16175
#define LAnkleRollX			25.40
#define LAnkleRollY			3.32
#define LAnkleRollZ			-32.41
//Right Leg
#define RHipYawPitchMass	0.07117
#define RHipYawPitchX		-7.66
#define RHipYawPitchY		12.00
#define RHipYawPitchZ		27.17

#define RHipRollMass		0.13530
#define RHipRollX			-16.49
#define RHipRollY			-0.29
#define RHipRollZ			-4.75

#define RHipPitchMass		0.39421
#define RHipPitchX			1.32
#define RHipPitchY			-2.35
#define RHipPitchZ			-53.52

#define RKneePitchMass		0.29159
#define RKneePitchX			4.22
#define RKneePitchY			-2.52
#define RKneePitchZ			-48.68

#define RAnklePitchMass		0.13892
#define RAnklePitchX		1.42
#define RAnklePitchY		-0.28
#define RAnklePitchZ		6.38

#define RAnkleRollMass		0.16175
#define RAnkleRollX			25.40
#define RAnkleRollY			-3.32
#define RAnkleRollZ			-32.41
#endif
