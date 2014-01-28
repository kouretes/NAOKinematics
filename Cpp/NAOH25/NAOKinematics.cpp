#include "NAOKinematics.h"
#include "KinematicsDefines.h"


using namespace KMath;
using namespace KMath::KMat;
using namespace KDeviceLists;

double PI=M_PI;
double PI_2=M_PI_2;
NAOKinematics::	NAOKinematics() :T(FR_SIZE),Tjacobian(FR_SIZE),joints(FR_SIZE),coms(FR_SIZE),masses(FR_SIZE)
{
	kmatTable t1,t2;
	for (unsigned i=0;i<FR_SIZE;i++){
		T[i].identity();
		Tjacobian[i].identity();
	}

	//Base and End  Transforms
	//--------HEAD
	//End and Base effectors
	KMatTransf::makeTranslation(T[FR_BASE_T+CHAIN_HEAD], 0.0, 0.0, NeckOffsetZ);
	KMatTransf::makeRotationXYZ(T[FR_END_T+CHAIN_HEAD], PI_2, PI_2, 0.0);
	
	//CoM coordinates
	coms[HEAD+YAW]=KVecDouble3(HeadYawX,HeadYawY,HeadYawZ);
	coms[HEAD+PITCH]=KVecDouble3(HeadPitchX,HeadPitchY,HeadPitchZ);
	//Get to pitch coordinate system
	coms[HEAD+PITCH]=T[FR_END_T+CHAIN_HEAD].transform(coms[HEAD+PITCH]);
	//Masses
	masses[HEAD+YAW]=HeadYawMass;
	masses[HEAD+PITCH]=HeadPitchMass;

	//--------RARM
	//End and Base effectors
	KMatTransf::makeTranslation(T[FR_BASE_T+CHAIN_R_ARM], 0.0, -(ShoulderOffsetY), ShoulderOffsetZ);
	KMatTransf::makeRotationXYZ(T[FR_END_T+CHAIN_R_ARM], -PI_2, 0.0, -PI_2);
	//This is the correct
	//KMatTransf::makeTranslation(t1, HandOffsetX,0.0,HandOffsetZ );
	//THIS is the fix
	KMatTransf::makeTranslation(t1, LowerArmLength+HandOffsetX,0.0,-HandOffsetZ );
	T[FR_END_T+CHAIN_R_ARM]*=t1;
	
	//CoM coordinates
	KMatTransf::makeRotationXYZ(t1, PI_2, 0.0, 0.0);
	coms[R_ARM+SHOULDER_PITCH]=KVecDouble3(RShoulderPitchX,RShoulderPitchY,RShoulderPitchZ);
	coms[R_ARM+SHOULDER_PITCH]=t1.transform(coms[R_ARM+SHOULDER_PITCH]);
	
	KMatTransf::makeRotationXYZ(t1, 0.0, 0.0, -PI_2);
	coms[R_ARM+SHOULDER_ROLL]=KVecDouble3(RShoulderRollX,RShoulderRollY,RShoulderRollZ);
	coms[R_ARM+SHOULDER_ROLL]=t1.transform(coms[R_ARM+SHOULDER_ROLL]);
	
	KMatTransf::makeRotationXYZ(t1, -PI_2, 0.0, -PI_2);
	coms[R_ARM+ELBOW_YAW]=KVecDouble3(RElbowYawX,RElbowYawY,RElbowYawZ);
	coms[R_ARM+ELBOW_YAW]=t1.transform(coms[R_ARM+ELBOW_YAW]);
	
	KMatTransf::makeRotationXYZ(t1, 0.0, 0.0, -PI_2);
	coms[R_ARM+ELBOW_ROLL]=KVecDouble3(RElbowRollX,RElbowRollY,RElbowRollZ);
	coms[R_ARM+ELBOW_ROLL]=t1.transform(coms[R_ARM+ELBOW_ROLL]);
	
	KMatTransf::makeRotationZYX(t1, PI_2, 0.0, PI_2);
	coms[R_ARM+WRIST_YAW]=KVecDouble3(RWristYawX,RWristYawY,RWristYawZ);
	coms[R_ARM+WRIST_YAW]=t1.transform(coms[R_ARM+WRIST_YAW]);
	
	//Masses
	masses[R_ARM+SHOULDER_PITCH]=RShoulderPitchMass;
	masses[R_ARM+SHOULDER_ROLL]=RShoulderRollMass;
	masses[R_ARM+ELBOW_YAW]=RElbowYawMass;
	masses[R_ARM+ELBOW_ROLL]=RElbowRollMass;
	masses[R_ARM+WRIST_YAW]=RWristYawMass;

	//--------LARM
	//End and Base effectors
	KMatTransf::makeTranslation(T[FR_BASE_T+CHAIN_L_ARM], 0.0, (ShoulderOffsetY), ShoulderOffsetZ);
	T[FR_END_T+CHAIN_L_ARM]=T[FR_END_T+CHAIN_R_ARM];

	//CoM coordinates and masses
	for(unsigned i=0;i<ARM_SIZE;i++)
	{
		coms[L_ARM+i]=coms[R_ARM+i];
        masses[L_ARM+i]=masses[R_ARM+i];
	}
	//Now mirror directions that we need
	coms[L_ARM+SHOULDER_PITCH](2)=-coms[L_ARM+SHOULDER_PITCH](2);
	coms[L_ARM+SHOULDER_ROLL](0)=-coms[L_ARM+SHOULDER_ROLL](0);
	coms[L_ARM+ELBOW_YAW](0)=-coms[L_ARM+ELBOW_YAW](0);
	coms[L_ARM+ELBOW_ROLL](0)=-coms[L_ARM+ELBOW_ROLL](0);
	coms[L_ARM+WRIST_YAW](0)=-coms[L_ARM+ELBOW_ROLL](0);

	//-----RLEG
	//End and Base effectors
	KMatTransf::makeTranslation(T[FR_BASE_T+CHAIN_R_LEG], 0.0, -HipOffsetY, -HipOffsetZ);
	KMatTransf::makeRotationZYX(T[FR_END_T+CHAIN_R_LEG],  PI ,  -PI_2, 0.0);
	RotRLeg=T[FR_END_T+CHAIN_R_LEG];
	KMatTransf::makeTranslation(t1, 0.0, 0.0, -FootHeight);
	T[FR_END_T+CHAIN_R_LEG]*=t1;


	//CoM coordinates
	KMatTransf::makeRotationXYZ(t1, -PI_2/2, 0.0, -PI_2);
	coms[R_LEG+HIP_YAW_PITCH]=KVecDouble3(RHipYawPitchX,RHipYawPitchY,RHipYawPitchZ);
	t1.fast_invert();
	coms[R_LEG+HIP_YAW_PITCH]=t1.transform(coms[R_LEG+HIP_YAW_PITCH]);

	KMatTransf::makeRotationXYZ(t1, PI, PI_2, 0.0);
	coms[R_LEG+HIP_ROLL]=KVecDouble3(RHipRollX,RHipRollY,RHipRollZ);
	coms[R_LEG+HIP_ROLL]=t1.transform(coms[R_LEG+HIP_ROLL]);

	KMatTransf::makeRotationXYZ(t1, PI_2, PI_2, 0.0);
	coms[R_LEG+HIP_PITCH]=KVecDouble3(RHipPitchX,RHipPitchY,RHipPitchZ);
	coms[R_LEG+HIP_PITCH]=t1.transform(coms[R_LEG+HIP_PITCH]);

	KMatTransf::makeRotationXYZ(t1, PI_2, PI_2, 0.0);
	coms[R_LEG+KNEE_PITCH]=KVecDouble3(RKneePitchX,RKneePitchY,RKneePitchZ);
	coms[R_LEG+KNEE_PITCH]=t1.transform(coms[R_LEG+KNEE_PITCH]);

	KMatTransf::makeRotationXYZ(t1, PI_2, PI_2, 0.0);
	coms[R_LEG+ANKLE_PITCH]=KVecDouble3(RAnklePitchX,RAnklePitchY,RAnklePitchZ);
	coms[R_LEG+ANKLE_PITCH]=t1.transform(coms[R_LEG+ANKLE_PITCH]);

	KMatTransf::makeRotationXYZ(t1, PI, PI_2, 0.0);
	coms[R_LEG+ANKLE_ROLL]=KVecDouble3(RAnkleRollX,RAnkleRollY,RAnkleRollZ);
	coms[R_LEG+ANKLE_ROLL]=t1.transform(coms[R_LEG+ANKLE_ROLL]);
	
	//Masses
	masses[R_LEG+HIP_YAW_PITCH]=RHipYawPitchMass;
	masses[R_LEG+HIP_ROLL]=RHipRollMass;
	masses[R_LEG+HIP_PITCH]=RHipPitchMass;
	masses[R_LEG+KNEE_PITCH]=RKneePitchMass;
	masses[R_LEG+ANKLE_PITCH]=RAnklePitchMass;
	masses[R_LEG+ANKLE_ROLL]=RAnkleRollMass;

	//------LLEG
	//End and Base effectors
	KMatTransf::makeTranslation(T[FR_BASE_T+CHAIN_L_LEG], 0.0, HipOffsetY, -HipOffsetZ);
	TBaseLLegInv=T[FR_BASE_T+CHAIN_L_LEG];
	TBaseLLegInv.fast_invert();
	KMatTransf::makeRotationZYX(T[FR_END_T+CHAIN_L_LEG], PI ,  -PI_2, 0.0);
	KMatTransf::makeRotationXYZ(RotFixLLeg,  PI_2 / 2, 0.0, 0.0);
	KMatTransf::makeTranslation(t1, 0.0, 0.0, -FootHeight);
	T[FR_END_T+CHAIN_L_LEG]*=t1;
	TEndLLegInv=t1;
	TEndLLegInv.fast_invert();


	//CoM coordinates
	KMatTransf::makeRotationXYZ(t1, -(3*PI) /4, 0.0, -PI_2);
	coms[L_LEG+HIP_YAW_PITCH]=KVecDouble3(RHipYawPitchX,-RHipYawPitchY,RHipYawPitchZ);
	t1.fast_invert();
	coms[L_LEG+HIP_YAW_PITCH]=t1.transform(coms[L_LEG+HIP_YAW_PITCH]);

	KMatTransf::makeRotationXYZ(t1, PI, PI_2,0.0);
	coms[L_LEG+HIP_ROLL]=KVecDouble3(RHipRollX,-RHipRollY,RHipRollZ);
	coms[L_LEG+HIP_ROLL]=t1.transform(coms[L_LEG+HIP_ROLL]);

	KMatTransf::makeRotationXYZ(t1, PI_2, PI_2,0.0);
	coms[L_LEG+HIP_PITCH]=KVecDouble3(RHipPitchX,-RHipPitchY,RHipPitchZ);
	coms[L_LEG+HIP_PITCH]=t1.transform(coms[L_LEG+HIP_PITCH]);

	KMatTransf::makeRotationXYZ(t1, PI_2, PI_2,0.0);
	coms[L_LEG+KNEE_PITCH]=KVecDouble3(RKneePitchX,-RKneePitchY,RKneePitchZ);
	coms[L_LEG+KNEE_PITCH]=t1.transform(coms[L_LEG+KNEE_PITCH]);

	KMatTransf::makeRotationXYZ(t1, PI_2, PI_2,0.0);
	coms[L_LEG+ANKLE_PITCH]=KVecDouble3(RAnklePitchX,-RAnklePitchY,RAnklePitchZ);
	coms[L_LEG+ANKLE_PITCH]=t1.transform(coms[L_LEG+ANKLE_PITCH]);

	KMatTransf::makeRotationXYZ(t1, PI, PI_2,0.0);
	coms[L_LEG+ANKLE_ROLL]=KVecDouble3(RAnkleRollX,-RAnkleRollY,RAnkleRollZ);
	coms[L_LEG+ANKLE_ROLL]=t1.transform(coms[L_LEG+ANKLE_ROLL]);
	
	//Masses
	masses[L_LEG+HIP_YAW_PITCH]=RHipYawPitchMass;
	masses[L_LEG+HIP_ROLL]=RHipRollMass;
	masses[L_LEG+HIP_PITCH]=RHipPitchMass;
	masses[L_LEG+KNEE_PITCH]=RKneePitchMass;
	masses[L_LEG+ANKLE_PITCH]=RAnklePitchMass;
	masses[L_LEG+ANKLE_ROLL]=RAnkleRollMass;

	//End Effectors for head
	KMatTransf::makeTranslation(T[FR_END_T+EFF_CAMERA_BOT], CameraBotomX, 0.0, CameraBotomZ);
	KMatTransf::makeTranslation(T[FR_END_T+EFF_CAMERA_TOP], CameraTopX, 0.0, CameraTopZ);

	prepareForward();
}

bool NAOKinematics::setJoints(std::vector<AngleType> jointsset)
{
	if(jointsset.size() != KDeviceLists::NUMOFJOINTS){
		return false;
	}
	joints=jointsset;
	prepareForward();
	return true;
}

bool NAOKinematics::setChain(KDeviceLists::ChainsNames ch, std::vector<AngleType> jointsset)
{
	unsigned chsize;
	Frames frstart;
	switch(ch)
	{
		case CHAIN_HEAD:
				frstart=(Frames)HEAD;
				chsize=HEAD_SIZE;
				break;
		case CHAIN_L_LEG:
				frstart=(Frames)L_LEG;
				chsize=LEG_SIZE;
				break;
		case CHAIN_R_LEG:
				frstart=(Frames)R_LEG;
				chsize=LEG_SIZE;
				break;
		case CHAIN_L_ARM:
				frstart=(Frames)L_ARM;
				chsize=ARM_SIZE;
				break;
		case CHAIN_R_ARM:
				frstart=(Frames)R_ARM;
				chsize=ARM_SIZE;
				break;
		default :
			return false;
	}

	if(jointsset.size() != chsize){
		return false;
	}

	for(unsigned i=0;i<chsize;i++)
	{
		joints[frstart+i]=jointsset[i];
	}
	prepareForward(ch);
	return true;
}

void NAOKinematics::prepareForward(KDeviceLists::ChainsNames ch)
{
	if(ch==CHAIN_L_ARM || ch==CHAINS_SIZE)
	{
		KMatTransf::makeDHTransformation(T[L_ARM+SHOULDER_PITCH],0.0, -PI_2, 0.0, (double)joints[L_ARM+SHOULDER_PITCH]);
		KMatTransf::makeDHTransformation(T[L_ARM+SHOULDER_ROLL], 0.0, PI_2, 0.0, (double)joints[L_ARM+SHOULDER_ROLL] + PI_2);
		KMatTransf::makeDHTransformation(T[L_ARM+ELBOW_YAW], +ElbowOffsetY, PI_2, UpperArmLength, (double)joints[L_ARM+ELBOW_YAW] );
		KMatTransf::makeDHTransformation(T[L_ARM+ELBOW_ROLL], 0.0, -PI_2, 0.0, (double)joints[L_ARM+ELBOW_ROLL]);
		/*Correct one BUT THE FIX does not change anythging*/
		//KMatTransf::makeDHTransformation(T[L_ARM+WRIST_YAW], 0.0, PI_2, LowerArmLength, (double)joints[L_ARM+WRIST_YAW]);
		/*The fix*/
		KMatTransf::makeDHTransformation(T[L_ARM+WRIST_YAW], 0.0, PI_2, 0.0, (double)joints[L_ARM+WRIST_YAW]);
	}

	if(ch==CHAIN_R_ARM || ch==CHAINS_SIZE)
	{
		KMatTransf::makeDHTransformation(T[R_ARM+SHOULDER_PITCH], 0.0, -PI_2, 0.0, (double)joints[R_ARM+SHOULDER_PITCH]);
		KMatTransf::makeDHTransformation(T[R_ARM+SHOULDER_ROLL], 0.0, PI_2, 0.0, (double)joints[R_ARM+SHOULDER_ROLL] + PI_2);
		KMatTransf::makeDHTransformation(T[R_ARM+ELBOW_YAW], -ElbowOffsetY, PI_2, UpperArmLength, (double) joints[R_ARM+ELBOW_YAW]);
		KMatTransf::makeDHTransformation(T[R_ARM+ELBOW_ROLL], 0.0, -PI_2, 0.0, (double)joints[R_ARM+ELBOW_ROLL]);
		/*Correct one BUT THE FIX does not change anythging*/
		//KMatTransf::makeDHTransformation(T[R_ARM+WRIST_YAW], 0.0, PI_2, LowerArmLength, (double)joints[R_ARM+WRIST_YAW]);
		/*The fix*/
		KMatTransf::makeDHTransformation(T[R_ARM+WRIST_YAW], 0.0, PI_2, 0.0, (double)joints[R_ARM+WRIST_YAW]);
	}

	if(ch==CHAIN_L_LEG || ch==CHAINS_SIZE)
	{
		KMatTransf::makeDHTransformation(T[L_LEG+HIP_YAW_PITCH], 0.0, -3 * PI_2 / 2, 0.0, (double)joints[L_LEG+HIP_YAW_PITCH] - PI_2);
		KMatTransf::makeDHTransformation(T[L_LEG+HIP_ROLL], 0.0, -PI_2, 0.0, (double)joints[L_LEG+HIP_ROLL] + PI_2 / 2);
		KMatTransf::makeDHTransformation(T[L_LEG+HIP_PITCH], 0.0, PI_2, 0.0, (double)joints[L_LEG+HIP_PITCH]);
		KMatTransf::makeDHTransformation(T[L_LEG+KNEE_PITCH], -ThighLength, 0.0, 0.0, (double)joints[L_LEG+KNEE_PITCH]);
		KMatTransf::makeDHTransformation(T[L_LEG+ANKLE_PITCH], -TibiaLength, 0.0, 0.0, (double)joints[L_LEG+ANKLE_PITCH]);
		KMatTransf::makeDHTransformation(T[L_LEG+ANKLE_ROLL], 0.0, -PI_2, 0.0, (double)joints[L_LEG+ANKLE_ROLL]);
	}

	if(ch==CHAIN_R_LEG || ch==CHAINS_SIZE)
	{
		KMatTransf::makeDHTransformation(T[R_LEG+HIP_YAW_PITCH], 0.0, -PI_2 / 2, 0.0,(double) joints[R_LEG+HIP_YAW_PITCH] - PI_2);
		KMatTransf::makeDHTransformation(T[R_LEG+HIP_ROLL], 0.0, -PI_2, 0.0,(double) joints[R_LEG+HIP_ROLL] - PI_2 / 2);
		KMatTransf::makeDHTransformation(T[R_LEG+HIP_PITCH], 0.0, PI_2, 0.0, (double)joints[R_LEG+HIP_PITCH]);
		KMatTransf::makeDHTransformation(T[R_LEG+KNEE_PITCH], -ThighLength, 0.0, 0.0,(double) joints[R_LEG+KNEE_PITCH]);
		KMatTransf::makeDHTransformation(T[R_LEG+ANKLE_PITCH], -TibiaLength, 0.0, 0.0, (double)joints[R_LEG+ANKLE_PITCH]);
		KMatTransf::makeDHTransformation(T[R_LEG+ANKLE_ROLL], 0.0, -PI_2, 0.0,  (double)joints[R_LEG+ANKLE_ROLL]);
	}

	if(ch==CHAIN_HEAD || ch==CHAINS_SIZE)
	{
		KMatTransf::makeDHTransformation(T[HEAD+YAW], 0.0, 0.0, 0.0, (double)joints[HEAD+YAW]);
		KMatTransf::makeDHTransformation(T[HEAD+PITCH], 0.0, -PI_2, 0.0, (double)joints[HEAD+PITCH] - PI_2);
	}
}

NAOKinematics::kmatTable NAOKinematics::getForwardEffector(Effectors ef)
{
	unsigned chsize;
	Frames frstart;
	Frames baseframe=(Frames) ef;

	switch(ef)
	{
		case EFF_CAMERA_BOT:
		case EFF_CAMERA_TOP:
				baseframe=(Frames)CHAIN_HEAD;
		case CHAIN_HEAD:
				frstart=(Frames)HEAD;
				chsize=HEAD_SIZE;
				break;
		case CHAIN_L_LEG:
				frstart=(Frames)L_LEG;
				chsize=LEG_SIZE;
				break;
		case CHAIN_R_LEG:
				frstart=(Frames)R_LEG;
				chsize=LEG_SIZE;
				break;
		case CHAIN_L_ARM:
				frstart=(Frames)L_ARM;
				chsize=ARM_SIZE;
				break;
		case CHAIN_R_ARM:
				frstart=(Frames)R_ARM;
				chsize=ARM_SIZE;
				break;
		default:
				return kmatTable().identity();
	}
	
	kmatTable t;
	t=T[FR_BASE_T+baseframe];
	
	for (unsigned i=0;i<chsize;i++){
		t*=T[FR_BASE+frstart+i];
	}		
	t*=T[FR_END_T+ef];

	return t;
}

NAOKinematics::kmatTable NAOKinematics::getForwardFromTo(Effectors start, Effectors stop)
{
	kmatTable t1= getForwardEffector(start);
	t1.fast_invert();
	t1*=getForwardEffector(stop);
	return t1;
}

KVecDouble3 NAOKinematics::calculateCoMChain(Frames start, Frames BaseFrame, int startdown, float & mass)
{
	KVecDouble3 pp,cp;
	float pm=0;
	pp.zero();
	cp.zero();

	for(int i=startdown-1;i>=0;i--)
	{
		pp=pp*pm+coms[start+i]*masses[start+i];
		pm=pm+masses[start+i];

		if(pm>0){
			pp.scalar_mult(1.0/pm);
		}
		
		pp=T[start+i].transform(pp);
	}
	
	pp=T[BaseFrame].transform(pp);
	mass=pm;
	return pp;
}

KVecDouble3 NAOKinematics::calculateCenterOfMass()
{
	KVecDouble3 p,t,sumc,torso,bat;
	float m,summass;
	summass=0;
    sumc.zero();

	p=calculateCoMChain((Frames)HEAD, (Frames)(FR_BASE_T+CHAIN_HEAD), HEAD_SIZE, m);
	sumc=p;
	summass=m;

	p=calculateCoMChain((Frames)L_ARM,(Frames)(FR_BASE_T+CHAIN_L_ARM), ARM_SIZE, m);
	sumc=sumc*summass+p*m;
	summass+=m;
	sumc.scalar_mult(1./summass);

	p=calculateCoMChain((Frames)R_ARM,(Frames)(FR_BASE_T+CHAIN_R_ARM), ARM_SIZE, m);
	sumc=sumc*summass+p*m;
	summass+=m;
	sumc.scalar_mult(1./summass);

	p=calculateCoMChain((Frames)L_LEG,(Frames)(FR_BASE_T+CHAIN_L_LEG), LEG_SIZE, m);
	sumc=sumc*summass+p*m;
	summass+=m;
	sumc.scalar_mult(1./summass);

	p=calculateCoMChain((Frames)R_LEG,(Frames) (FR_BASE_T+CHAIN_R_LEG),LEG_SIZE,m);
	sumc=sumc*summass+p*m;
	summass+=m;
	sumc.scalar_mult(1./summass);

	p=KVecDouble3(TorsoX,TorsoY,TorsoZ);
	m=TorsoMass;

	sumc=sumc*summass+p*m;
	summass+=m;
	sumc.scalar_mult(1./summass);

	p=KVecDouble3(BatteryX,BatteryY,BatteryZ);
	m=BatteryMass;

	sumc=sumc*summass+p*m;
	summass+=m;
	sumc.scalar_mult(1./summass);

	return sumc;
}

NAOKinematics::AngleContainer NAOKinematics::inverseHead(const FKvars s, bool withAngles, bool topCamera)
{
	return inverseHead(getTransformation(s), withAngles, topCamera);
}

NAOKinematics::AngleContainer NAOKinematics::inverseHead(kmatTable targetPoint, bool withAngles, bool topCamera)
{
	kmatTable T;
	kmatTable outputT;
	NAOKinematics::AngleContainer returnResult;
	T = targetPoint;
	float theta1, theta2;

	if(T(0,3) != 0.0 && withAngles)
		return returnResult;

	if(withAngles)
	{
		theta1 = atan2(T(1,0), T(0,0));
		theta2 = atan2(-T(2,0), sqrt(pow(T(2,1), 2) + pow(T(2,2), 2)));
		std::vector<float> r(HEAD_SIZE);
		r[YAW]=theta1;
		r[PITCH]=theta2;
		returnResult.push_back(r);
	}
	else
	{
		float up = - T(2,3) + NeckOffsetZ;
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

			theta2 = theta2 + PI_2;
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
			joints[HEAD+YAW]=theta1;
			joints[HEAD+PITCH]=theta2;
			prepareForward(CHAIN_HEAD);
			outputT = getForwardEffector(topCamera?EFF_CAMERA_TOP:EFF_CAMERA_BOT);
			KVecDouble3 r=targetPoint.getTranslation();
			r-=outputT.getTranslation();
			//Validate only the translation
			if(r.norm2()<3)
			{
				std::vector<float> r(HEAD_SIZE);
				r[YAW]=theta1;
				r[PITCH]=theta2;
				returnResult.push_back(r);
			}
			//-----------------------------------------------------------------------------------------------------------------------------------------
		}
	}

	return returnResult;
}

NAOKinematics::AngleContainer NAOKinematics::inverseLeftHand(const FKvars s)
{
	return inverseLeftHand(getTransformation(s));
}

NAOKinematics::AngleContainer NAOKinematics::inverseLeftHand(kmatTable targetPoint)
{
	kmatTable Temp,T1,T2,T3,Tinit,Torig;
	std::vector<std::vector<float> > returnResult;
	double theta1, theta2, theta3, theta4, theta5;
	Tinit = targetPoint;
	Tinit.fast_invert();
	Temp = T[FR_END_T+CHAIN_L_ARM];
	Temp*=Tinit;
	Temp*=T[FR_BASE_T+CHAIN_L_ARM];
	try
	{
		Temp.fast_invert();
	}
	catch(KMath::KMat::SingularMatrixInvertionException d)
	{
		return returnResult;
	}
	theta1 = atan2(-Temp(2,3),Temp(0,3));
	if(theta1 > LShoulderPitchHigh || theta1 < LShoulderPitchLow)
		return returnResult;
	
	KMatTransf::makeDHTransformation(T1, 0.0, -PI_2, 0.0,  theta1);
	try
	{
		T1.fast_invert();
	}
	catch(KMath::KMat::SingularMatrixInvertionException d)
	{
		return returnResult;
	}
	T1 *= Temp;
	Temp = T1;
	
	double top, bottom;
	top = Temp(0,3)*ElbowOffsetY/UpperArmLength -Temp(2,3);
	bottom = ElbowOffsetY*ElbowOffsetY/UpperArmLength + UpperArmLength;
	double theta2temp = acos(top/bottom);

	for(int i=0; i<2; i++){
		if(i == 0 && (theta2temp-PI_2 > LShoulderRollHigh || theta2temp-PI_2 < LShoulderRollLow)){
			continue;
		}else if(i == 1 && (-theta2temp-PI_2 > LShoulderRollHigh || -theta2temp-PI_2 < LShoulderRollLow)){
			continue;
		}else if(i == 1){
			theta2temp = -theta2temp;
		}
		theta2 = theta2temp-PI_2;
		KMatTransf::makeDHTransformation(T2, 0.0, PI_2, 0.0, theta2 + PI_2);
		try
		{
			T2.fast_invert();
		}
		catch(KMath::KMat::SingularMatrixInvertionException d)
		{
			return returnResult;
		}
		T2*=Temp;
		theta3 = atan2(-T2(2,2),-T2(0,2));
		KMatTransf::makeDHTransformation(T3, +ElbowOffsetY, PI_2, UpperArmLength, theta3);
		try
		{
			T3.fast_invert();
		}
		catch(KMath::KMat::SingularMatrixInvertionException d)
		{
			return returnResult;
		}
		T3*=T2;
		theta4 = atan2(T3(0,2),T3(2,2));
		theta5 = atan2(T3(1,0),T3(1,1));
		//---------------------------Forward validation step--------------------------------------------------------------------------------------
		joints[L_ARM+SHOULDER_PITCH]=theta1;
		joints[L_ARM+SHOULDER_ROLL]=theta2;
		joints[L_ARM+ELBOW_YAW]=theta3;
		joints[L_ARM+ELBOW_ROLL]=theta4;
		joints[L_ARM+WRIST_YAW]=theta5;
		prepareForward(CHAIN_L_ARM);
		kmatTable test=getForwardEffector((Effectors)CHAIN_L_ARM);
		if(test.almostEqualTo(targetPoint))
		{
			std::vector<float> r(ARM_SIZE);
			r[SHOULDER_PITCH]=theta1;
			r[SHOULDER_ROLL]=theta2;
			r[ELBOW_YAW]=theta3;
			r[ELBOW_ROLL]=theta4;
			r[WRIST_YAW]=theta5;
			//std::cout << "Niki!!\nTheta 1 = " << theta1 << "\nTheta 2 = " << theta2 << "\nTheta 3 = " << theta3 << "\nTheta 4 = " << theta4 << "\nTheta 5 = " << theta5 << std::endl;
			returnResult.push_back(r);
		}
		else
		{
			//std::cout << "Htta!!\nTheta 1 = " << theta1 << "\nTheta 2 = " << theta2 << "\nTheta 3 = " << theta3 << "\nTheta 4 = " << theta4 << "\nTheta 5 = " << theta5 << std::endl;
		}
		//-----------------------------------------------------------------------------------------------------------------------------------------
	}
	return returnResult;
}

std::vector<std::vector<float> > NAOKinematics::inverseRightHand(const FKvars s)
{
	return inverseRightHand(getTransformation(s));
}

std::vector<std::vector<float> > NAOKinematics::inverseRightHand(kmatTable targetPoint)
{
	mirrorTransformation(targetPoint);
	std::vector<std::vector<float> > res=inverseLeftHand(targetPoint);
	for(unsigned i=0;i<res.size();i++)
	{
		(res[i])[SHOULDER_ROLL]=-(res[i])[SHOULDER_ROLL];
		(res[i])[ELBOW_ROLL]=-(res[i])[ELBOW_ROLL];
		(res[i])[ELBOW_YAW]=-(res[i])[ELBOW_YAW];
		(res[i])[WRIST_YAW]=-(res[i])[WRIST_YAW];
	}
	return res;
}

std::vector<std::vector<float> > NAOKinematics::inverseLeftLeg(const FKvars s)
{
	return inverseLeftLeg(getTransformation(s));
}

std::vector<std::vector<float> > NAOKinematics::inverseLeftLeg(kmatTable targetPoint)
{
	std::vector<std::vector<float> > returnResult;
	kmatTable TtempTheta5,T4i,T5i,T6i,Ttemp,Ttemp2;
	kmatTable T = targetPoint;
	kmatTable Tinit = T;
	//Move the start point to the hipyawpitch point
	kmatTable base = TBaseLLegInv;
	base *= T;
	//Move the end point to the anklePitch joint
	base *= TEndLLegInv;
	//Rotate hipyawpitch joint
	kmatTable Rot = RotFixLLeg;
	Rot *= base;
	//Invert the table, because we need the chain from the ankle to the hip
	kmatTable Tstart = Rot;
	try
	{
		Rot.fast_invert();
	}
	catch(KMath::KMat::SingularMatrixInvertionException d)
	{
		return returnResult;
	}

	T = Rot;
	//Build the rotation table
	double side1 = ThighLength;
	double side2 = TibiaLength;
	//Calculate Theta 4
	double distancesqrd = T.getTranslation().norm2();
	double theta4 = PI - acos((pow(side1, 2) + pow(side2, 2) -distancesqrd) / (2 * side1 * side2));

	if(theta4 != theta4)
	{
		return returnResult;
	}


	double theta6 = atan(T(1,3) / T(2,3));

	if(theta6 < LAnkleRollLow || theta6 > LAnkleRollHigh)
		return returnResult;

	KMatTransf::makeDHTransformation(T6i, 0.0, -PI_2, 0.0, theta6);
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
			theta5 = -PI_2;
		else if(theta5 != theta5)
			theta5 = PI_2;

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
			float temptheta2 = acos(Ttemp2(1, 2));
			float theta2;

			for(int l = 0; l < 2; l++)
			{
				if(l == 0 && (temptheta2 - PI_2 / 2 > LHipRollHigh || temptheta2 - PI_2 / 2 < LHipRollLow))
					continue;
				else if(l == 1 && (-temptheta2 - PI_2 / 2 > LHipRollHigh || -temptheta2 - PI_2 / 2 < LHipRollLow))
					continue;
				else if(l == 0)
					theta2 = temptheta2 - PI_2 / 2;
				else if(l == 1)
					theta2 = -temptheta2 - PI_2 / 2;

				float theta3 = asin(Ttemp2(1, 1) / sin(theta2 + PI_2 / 2));
				float posOrNegPIt3 = (theta3 >= 0) ? PI : -PI;

				if(theta3 != theta3 && Ttemp2(1, 1) / sin(theta2 + PI_2 / 2) < 0)
					theta3 = -PI_2;
				else if(theta3 != theta3)
					theta3 = PI_2;

				for(int k = 0; k < 2; k++)
				{
					if(k == 0 && (theta3 > LHipPitchHigh || theta3 < LHipPitchLow))
						continue;
					else if(k == 1 && (posOrNegPIt3 - theta3 > LHipPitchHigh || posOrNegPIt3 - theta3 < LHipPitchLow))
						continue;
					else if(k == 1)
						theta3 = posOrNegPIt3 - theta3;

					float temptheta1 = acos(Ttemp2(0, 2) / sin(theta2 + PI_2 / 2));

					if(temptheta1 != temptheta1)
						temptheta1 = 0;

					for(int p = 0; p < 2; p++)
					{
						float theta1;

						if(p == 0 && (temptheta1 + PI_2 > LHipYawPitchHigh || -temptheta1 + PI_2 < LHipYawPitchLow))
							continue;
						else if(p == 1 && (-temptheta1 + PI_2 > LHipYawPitchHigh || - temptheta1 + PI_2 < LHipYawPitchLow))
							continue;
						else if(p == 0)
							theta1 = temptheta1 + PI_2;
						else if(p == 1)
							theta1 = -temptheta1 + PI_2;

						//--------------------------------------Forward validation step------------------------------------------------------------------------------
						joints[L_LEG+HIP_YAW_PITCH]=theta1;
						joints[L_LEG+HIP_ROLL]=theta2;
						joints[L_LEG+HIP_PITCH]=theta3;
						joints[L_LEG+KNEE_PITCH]=theta4;
						joints[L_LEG+ANKLE_PITCH]=theta5;
						joints[L_LEG+ANKLE_ROLL]=theta6;

						prepareForward(CHAIN_L_LEG);
						kmatTable test=getForwardEffector((Effectors)CHAIN_L_LEG);

						if(test.almostEqualTo(Tinit))
						{
							std::vector<float> r(LEG_SIZE);
							r[HIP_YAW_PITCH]=theta1;
							r[HIP_ROLL]=theta2;
							r[HIP_PITCH]=theta3;
							r[KNEE_PITCH]=theta4;
							r[ANKLE_PITCH]=theta5;
							r[ANKLE_ROLL]=theta6;
							//std::cout << "Niki!!\nTheta 1 = " << theta1 << "\nTheta 2 = " << theta2 << "\nTheta 3 = " << theta3 << "\nTheta 4 = " << theta4  << "\nTheta 5 = " << theta5 << "\nTheta 6 = " << theta6 << std::endl;
							returnResult.push_back(r);
						}
						//---------------------------------------------------------------------------------------------------------------------------------------------
					}
				}
			}
		}
	}
	return returnResult;
}

std::vector<std::vector<float> > NAOKinematics::inverseRightLeg(const FKvars s)
{
	return inverseRightLeg(getTransformation(s));
}

std::vector<std::vector<float> > NAOKinematics::inverseRightLeg(kmatTable targetPoint)
{

	mirrorTransformation(targetPoint);
	std::vector<std::vector<float> > res=inverseLeftLeg(targetPoint);
	for(unsigned i=0;i<res.size();i++)
	{
		(res[i])[HIP_ROLL]=-(res[i])[HIP_ROLL];
		(res[i])[ANKLE_ROLL]=-(res[i])[ANKLE_ROLL];
	}

	return res;
}
