all: NAOKinematics

NAOKinematics: NAOKinematics.o Main.o
	g++ Main.o NAOKinematics.o -o NAOKinematics


NAOKinematics.o: NAOKinematics.cpp NAOKinematics.h robotConsts.h KMat.hpp
	g++ -c NAOKinematics.cpp

Main.o: Main.cpp
	g++ -c Main.cpp
	
clean:
	rm -rf *o NAOKinematics
