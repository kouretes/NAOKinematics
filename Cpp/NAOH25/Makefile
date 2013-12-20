CPP_FLAGS=-Wall -g 

all: NAOKinematics


NAOKinematics: NAOKinematics.o Main.o NAOKinematicsJacobian.o
	g++ Main.o NAOKinematics.o NAOKinematicsJacobian.o -o NAOKinematics $(CPP_FLAGS)


NAOKinematics.o: NAOKinematics.cpp NAOKinematics.h robotConsts.h KMat.hpp
	g++ -c NAOKinematics.cpp  $(CPP_FLAGS)

NAOKinematicsJacobian.o: NAOKinematicsJacobian.cpp NAOKinematics.h robotConsts.h KMat.hpp
	g++ -c NAOKinematicsJacobian.cpp  $(CPP_FLAGS)

Main.o: Main.cpp
	g++ -c Main.cpp $(CPP_FLAGS)
	
clean:
	rm -rf *o NAOKinematics
