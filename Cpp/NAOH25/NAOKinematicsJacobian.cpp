#include "NAOKinematics.h"
#include "KinematicsDefines.h"

#define TOL 1e-2
#define MAXITER 50
#define CLAMP 10.0
#define alpha 0.8

using namespace KMath;
using namespace KMath::KMat;
using namespace KDeviceLists;


void NAOKinematics::prepareDerivatives(KDeviceLists::ChainsNames ch)
{
	if(ch==CHAIN_L_ARM || ch==CHAINS_SIZE)
	{		
		KMatTransf::makeDHDerivative(Tjacobian[L_ARM+SHOULDER_PITCH], -M_PI_2, (double)joints[L_ARM+SHOULDER_PITCH]);
		KMatTransf::makeDHDerivative(Tjacobian[L_ARM+SHOULDER_ROLL], M_PI_2, (double)joints[L_ARM+SHOULDER_ROLL] + M_PI_2);
		KMatTransf::makeDHDerivative(Tjacobian[L_ARM+ELBOW_YAW], M_PI_2, (double)joints[L_ARM+ELBOW_YAW] );
		KMatTransf::makeDHDerivative(Tjacobian[L_ARM+ELBOW_ROLL], -M_PI_2, (double)joints[L_ARM+ELBOW_ROLL]);
		KMatTransf::makeDHDerivative(Tjacobian[L_ARM+WRIST_YAW], M_PI_2, (double)joints[L_ARM+WRIST_YAW]);
	}

	if(ch==CHAIN_R_ARM || ch==CHAINS_SIZE)
	{
		KMatTransf::makeDHDerivative(Tjacobian[R_ARM+SHOULDER_PITCH], -M_PI_2, (double)joints[R_ARM+SHOULDER_PITCH]);
		KMatTransf::makeDHDerivative(Tjacobian[R_ARM+SHOULDER_ROLL], M_PI_2, (double)joints[R_ARM+SHOULDER_ROLL] + M_PI_2); //Allagh apo matlab
		KMatTransf::makeDHDerivative(Tjacobian[R_ARM+ELBOW_YAW], M_PI_2, (double) joints[R_ARM+ELBOW_YAW]);
		KMatTransf::makeDHDerivative(Tjacobian[R_ARM+ELBOW_ROLL], -M_PI_2, (double)joints[R_ARM+ELBOW_ROLL]); //Allagh apo matlab
		KMatTransf::makeDHDerivative(Tjacobian[R_ARM+WRIST_YAW], M_PI_2, (double)joints[R_ARM+WRIST_YAW]);
	}

	if(ch==CHAIN_L_LEG || ch==CHAINS_SIZE)
	{
		KMatTransf::makeDHDerivative(Tjacobian[L_LEG+HIP_YAW_PITCH], -3 * M_PI_2 / 2, (double)joints[L_LEG+HIP_YAW_PITCH] - M_PI_2);
		KMatTransf::makeDHDerivative(Tjacobian[L_LEG+HIP_ROLL], -M_PI_2, (double)joints[L_LEG+HIP_ROLL] + M_PI_2 / 2);
		KMatTransf::makeDHDerivative(Tjacobian[L_LEG+HIP_PITCH], M_PI_2, (double)joints[L_LEG+HIP_PITCH]);
		KMatTransf::makeDHDerivative(Tjacobian[L_LEG+KNEE_PITCH], 0.0, (double)joints[L_LEG+KNEE_PITCH]);
		KMatTransf::makeDHDerivative(Tjacobian[L_LEG+ANKLE_PITCH], 0.0, (double)joints[L_LEG+ANKLE_PITCH]);
		KMatTransf::makeDHDerivative(Tjacobian[L_LEG+ANKLE_ROLL], -M_PI_2, (double)joints[L_LEG+ANKLE_ROLL]);
	}

	if(ch==CHAIN_R_LEG || ch==CHAINS_SIZE)
	{	
		KMatTransf::makeDHDerivative(Tjacobian[R_LEG+HIP_YAW_PITCH], -M_PI_2 / 2, (double) joints[R_LEG+HIP_YAW_PITCH] - M_PI_2);
		KMatTransf::makeDHDerivative(Tjacobian[R_LEG+HIP_ROLL], -M_PI_2, (double) joints[R_LEG+HIP_ROLL] - M_PI_2 / 2); //allagh
		KMatTransf::makeDHDerivative(Tjacobian[R_LEG+HIP_PITCH], M_PI_2, (double)joints[R_LEG+HIP_PITCH]);
		KMatTransf::makeDHDerivative(Tjacobian[R_LEG+KNEE_PITCH], 0.0, (double) joints[R_LEG+KNEE_PITCH]);
		KMatTransf::makeDHDerivative(Tjacobian[R_LEG+ANKLE_PITCH], 0.0, (double)joints[R_LEG+ANKLE_PITCH]); //allagh
		KMatTransf::makeDHDerivative(Tjacobian[R_LEG+ANKLE_ROLL], -M_PI_2, (double)joints[R_LEG+ANKLE_ROLL]);
	}

	if(ch==CHAIN_HEAD || ch==CHAINS_SIZE)
	{
		KMatTransf::makeDHDerivative(Tjacobian[HEAD+YAW], 0.0, (double)joints[HEAD+YAW]);
		KMatTransf::makeDHDerivative(Tjacobian[HEAD+PITCH], -M_PI_2, (double)joints[HEAD+PITCH] - M_PI_2);
	}
}

std::vector<std::vector<float> > NAOKinematics::jacobianInverseHead(const FKvars s, bool topCamera)
{
	return jacobianInverseHead(getTransformation(s), topCamera);
}

std::vector<std::vector<float> > NAOKinematics::jacobianInverseHead(kmatTable targetPoint, bool topCamera)
{
	return jacobianInverseHead(targetPoint, CHAIN_HEAD, topCamera);
}

NAOKinematics::AngleContainer NAOKinematics::jacobianInverseHead(kmatTable targetPoint, KDeviceLists::ChainsNames ch, bool topCamera){
	KMath::KMat::GenMatrix<double, 16, 1> errorVector;
	KMath::KMat::GenMatrix<double, 2, 1> jacobianResult;
	KMath::KMat::GenMatrix<double, 2, 16> jacobian, jacobianPseudoInv;
	kmatTable TargetJacobian;
	int itterations = -1;
	double quality = 0;
	
	int chsize;
	kmatJacobianTable mask, error;
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			mask(i,j) = 1;
		}
	}
	
	Frames frstart;
	Frames baseframe=(Frames)CHAIN_HEAD;
	frstart=(Frames)HEAD;
	chsize=HEAD_SIZE;
	Effectors ef = EFF_CAMERA_BOT;
	if(topCamera){
		ef = EFF_CAMERA_TOP;
	}

	prepareForward(ch);
	prepareDerivatives(ch);
	std::vector<kmatJacobianTable> derivatives(chsize);
	std::vector<kmatJacobianTable> atToGen(chsize);
	
	do{
		itterations++;
		TargetJacobian=T[FR_BASE_T+baseframe];
	
		for (int i=0; i<chsize; i++){
			TargetJacobian*=T[FR_BASE+frstart+i];
			atToGen[i] = KMatTransf::castToGenMatrix(T[FR_BASE+frstart+i]);
		}
				
		TargetJacobian*=T[FR_END_T+ef];
		
		//Make the derivatives matrices
		//calculate max and error
		for(int j=0; j<chsize; j++){
			derivatives[j] = KMatTransf::castToGenMatrix(T[FR_BASE_T+baseframe]);
			for (int i=0; i<chsize; i++){
				if(j==i){
					derivatives[j]*=Tjacobian[FR_BASE+frstart+i];
				}else{
					derivatives[j]*=atToGen[i];
				}
			}
			derivatives[j]*=KMatTransf::castToGenMatrix(T[FR_END_T+ef]);
		}

		double max = 0.0;
		for(int i=0; i<4; i++){
			for(int j=0; j<4; j++){
				if(i<3){
					error(i,j) = (targetPoint(i,j)-TargetJacobian(i,j))*mask(i,j);
					if(error(i,j) > 0 && error(i,j) > max) {
						max = error(i,j);
					}else if(error(i,j) < 0 && -error(i,j) > max){
						max = -error(i,j);
					}
				}else{
					error(i,j) = 0;
				}
			}
		}
		double value = max > CLAMP ? CLAMP/max : 1;
		for(int j=0; j<4; j++){
			for(int i=0; i<4; i++){
				errorVector(i+j*4,0) = error(i,j)*value;
			}
		}
		
		for(int k=0; k < chsize; k++){
			for(int j=0; j<4; j++){
				for(int i=0; i<4; i++){
					jacobian(k,i+j*4) = derivatives[k](i,j);
				}
			}
		}
		jacobianPseudoInv = jacobian.pseudoInverse();
		jacobianResult = jacobianPseudoInv*errorVector;
		jacobianResult.scalar_mult(alpha);
		
		quality = 0.0;
		for(int i=0; i<3; i++){
			for(int j=0; j<4; j++){
					quality += (targetPoint(i,j)-TargetJacobian(i,j))*mask(i,j);
			}
		}

		for(int i=0; i<chsize; i++){
			joints[frstart+i] = joints[frstart+i] + jacobianResult(i,0);
		}
		
		prepareForward(ch);
		prepareDerivatives(ch);
	}while(fabs(quality) > TOL && itterations <= MAXITER);
	AngleContainer returnResult;
	std::vector<float> r(chsize);
	for(int i=0; i<chsize; i++){
		r[i] = joints[frstart+i];
	}
	returnResult.push_back(r);
	return returnResult;
}

std::vector<std::vector<float> > NAOKinematics::jacobianInverseLeftHand(const FKvars s)
{
	return jacobianInverseLeftHand(getTransformation(s));
}

std::vector<std::vector<float> > NAOKinematics::jacobianInverseLeftHand(kmatTable targetPoint)
{
	return jacobianInverseHands(targetPoint, CHAIN_L_ARM);
}


std::vector<std::vector<float> > NAOKinematics::jacobianInverseRightHand(const FKvars s)
{
	return jacobianInverseRightHand(getTransformation(s));
}

std::vector<std::vector<float> > NAOKinematics::jacobianInverseRightHand(kmatTable targetPoint)
{
	return jacobianInverseHands(targetPoint, CHAIN_R_ARM);
}

NAOKinematics::AngleContainer NAOKinematics::jacobianInverseHands(kmatTable targetPoint, KDeviceLists::ChainsNames ch){
	KMath::KMat::GenMatrix<double, 16, 1> errorVector;
	KMath::KMat::GenMatrix<double, 5, 1> jacobianResult;
	KMath::KMat::GenMatrix<double, 5, 16> jacobian, jacobianPseudoInv;
	kmatTable TargetJacobian;
	int itterations = -1;
	double quality = 0;
	
	int chsize;
	kmatJacobianTable mask, error;
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			mask(i,j) = 1;
		}
	}
	
	Frames frstart;
	Frames baseframe= (Frames) ch;
	if(ch == CHAIN_L_ARM){
		frstart=(Frames)L_ARM;
		chsize=ARM_SIZE;
	}else{
		frstart=(Frames)R_ARM;
		chsize=ARM_SIZE;
	}

	prepareForward(ch);
	prepareDerivatives(ch);
	std::vector<kmatJacobianTable> derivatives(chsize);
	std::vector<kmatJacobianTable> atToGen(chsize);
	
	do{
		itterations++;
		TargetJacobian=T[FR_BASE_T+baseframe];
	
		for (int i=0; i<chsize; i++){
			TargetJacobian*=T[FR_BASE+frstart+i];
			atToGen[i] = KMatTransf::castToGenMatrix(T[FR_BASE+frstart+i]);
		}
				
		TargetJacobian*=T[FR_END_T+ch];
		
		//Make the derivatives matrices
		//calculate max and error
		for(int j=0; j<chsize; j++){
			derivatives[j] = KMatTransf::castToGenMatrix(T[FR_BASE_T+baseframe]);
			for (int i=0; i<chsize; i++){
				if(j==i){
					derivatives[j]*=Tjacobian[FR_BASE+frstart+i];
				}else{
					derivatives[j]*=atToGen[i];
				}
			}
			derivatives[j]*=KMatTransf::castToGenMatrix(T[FR_END_T+ch]);
		}

		double max = 0.0;
		for(int i=0; i<4; i++){
			for(int j=0; j<4; j++){
				if(i<3){
					error(i,j) = (targetPoint(i,j)-TargetJacobian(i,j))*mask(i,j);
					if(error(i,j) > 0 && error(i,j) > max) {
						max = error(i,j);
					}else if(error(i,j) < 0 && -error(i,j) > max){
						max = -error(i,j);
					}
				}else{
					error(i,j) = 0;
				}
			}
		}
		double value = max > CLAMP ? CLAMP/max : 1;
		for(int j=0; j<4; j++){
			for(int i=0; i<4; i++){
				errorVector(i+j*4,0) = error(i,j)*value;
			}
		}
		
		for(int k=0; k < chsize; k++){
			for(int j=0; j<4; j++){
				for(int i=0; i<4; i++){
					jacobian(k,i+j*4) = derivatives[k](i,j);
				}
			}
		}
		jacobian.prettyPrint();
		jacobianPseudoInv = jacobian.pseudoInverse();
		jacobianResult = jacobianPseudoInv*errorVector;
		jacobianResult.scalar_mult(alpha);
		
		quality = 0.0;
		for(int i=0; i<3; i++){
			for(int j=0; j<4; j++){
					quality += (targetPoint(i,j)-TargetJacobian(i,j))*mask(i,j);
			}
		}

		for(int i=0; i<chsize; i++){
			joints[frstart+i] = joints[frstart+i] + jacobianResult(i,0);
		}
		
		prepareForward(ch);
		prepareDerivatives(ch);
	}while(fabs(quality) > TOL && itterations <= MAXITER);
	AngleContainer returnResult;
	std::vector<float> r(chsize);
	for(int i=0; i<chsize; i++){
		r[i] = joints[frstart+i];
	}
	returnResult.push_back(r);
	return returnResult;
}

std::vector<std::vector<float> > NAOKinematics::jacobianInverseLeftLeg(const FKvars s)
{
	return jacobianInverseLeftLeg(getTransformation(s));
}

std::vector<std::vector<float> > NAOKinematics::jacobianInverseLeftLeg(kmatTable targetPoint)
{
	return jacobianInverseLegs(targetPoint, CHAIN_L_LEG);
}


std::vector<std::vector<float> > NAOKinematics::jacobianInverseRightLeg(const FKvars s)
{
	return jacobianInverseRightLeg(getTransformation(s));
}

std::vector<std::vector<float> > NAOKinematics::jacobianInverseRightLeg(kmatTable targetPoint)
{
	return jacobianInverseLegs(targetPoint, CHAIN_R_LEG);
}

NAOKinematics::AngleContainer NAOKinematics::jacobianInverseLegs(kmatTable targetPoint, KDeviceLists::ChainsNames ch){
	KMath::KMat::GenMatrix<double, 16, 1> errorVector;
	KMath::KMat::GenMatrix<double, 6, 1> jacobianResult;
	KMath::KMat::GenMatrix<double, 6, 16> jacobian, jacobianPseudoInv;
	kmatTable TargetJacobian;
	int itterations = -1;
	double quality = 0;
	
	int chsize;
	kmatJacobianTable mask, error;
	for(int i=0; i<4; i++){
		for(int j=0; j<4; j++){
			mask(i,j) = 1;
		}
	}
	
	Frames frstart;
	Frames baseframe= (Frames) ch;
	if(ch == CHAIN_L_LEG){
		frstart=(Frames)L_LEG;
		chsize=LEG_SIZE;
	}else{
		frstart=(Frames)R_LEG;
		chsize=LEG_SIZE;
	}

	prepareForward(ch);
	prepareDerivatives(ch);
	std::vector<kmatJacobianTable> derivatives(chsize);
	std::vector<kmatJacobianTable> atToGen(chsize);
	
	do{
		itterations++;
		TargetJacobian=T[FR_BASE_T+baseframe];
	
		for (int i=0; i<chsize; i++){
			TargetJacobian*=T[FR_BASE+frstart+i];
			atToGen[i] = KMatTransf::castToGenMatrix(T[FR_BASE+frstart+i]);
		}
				
		TargetJacobian*=T[FR_END_T+ch];
		
		//Make the derivatives matrices
		//calculate max and error
		for(int j=0; j<chsize; j++){
			derivatives[j] = KMatTransf::castToGenMatrix(T[FR_BASE_T+baseframe]);
			for (int i=0; i<chsize; i++){
				if(j==i){
					derivatives[j]*=Tjacobian[FR_BASE+frstart+i];
				}else{
					derivatives[j]*=atToGen[i];
				}
			}
			derivatives[j]*=KMatTransf::castToGenMatrix(T[FR_END_T+ch]);
		}

		double max = 0.0;
		for(int i=0; i<4; i++){
			for(int j=0; j<4; j++){
				if(i<3){
					error(i,j) = (targetPoint(i,j)-TargetJacobian(i,j))*mask(i,j);
					if(error(i,j) > 0 && error(i,j) > max) {
						max = error(i,j);
					}else if(error(i,j) < 0 && -error(i,j) > max){
						max = -error(i,j);
					}
				}else{
					error(i,j) = 0;
				}
			}
		}
		double value = max > CLAMP ? CLAMP/max : 1;
		for(int j=0; j<4; j++){
			for(int i=0; i<4; i++){
				errorVector(i+j*4,0) = error(i,j)*value;
			}
		}
		
		for(int k=0; k < chsize; k++){
			for(int j=0; j<4; j++){
				for(int i=0; i<4; i++){
					jacobian(k,i+j*4) = derivatives[k](i,j);
				}
			}
		}
		jacobianPseudoInv = jacobian.pseudoInverse();
		jacobianResult = jacobianPseudoInv*errorVector;
		jacobianResult.scalar_mult(alpha);
		
		quality = 0.0;
		for(int i=0; i<3; i++){
			for(int j=0; j<4; j++){
					quality += (targetPoint(i,j)-TargetJacobian(i,j))*mask(i,j);
			}
		}

		for(int i=0; i<chsize; i++){
			joints[frstart+i] = joints[frstart+i] + jacobianResult(i,0);
		}
		
		prepareForward(ch);
		prepareDerivatives(ch);
	}while(fabs(quality) > TOL && itterations <= MAXITER);
	AngleContainer returnResult;
	std::vector<float> r(chsize);
	for(int i=0; i<chsize; i++){
		r[i] = joints[frstart+i];
	}
	returnResult.push_back(r);
	return returnResult;
}
