all: NAOKinematics

NAOKinematics: NAOKinematics.o Main.o
	g++ Main.o NAOKinematics.o -o NAOKinematics

Main.o: Main.cpp
	g++ -c Main.cpp

NAOKinematics.o: NAOKinematics.cpp
	g++ -c NAOKinematics.cpp

clean:
	rm -rf *o NAOKinematics
