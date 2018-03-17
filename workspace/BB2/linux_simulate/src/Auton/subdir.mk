################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Auton/AutoCommand.cpp \
../src/Auton/AutonDeployIntake.cpp \
../src/Auton/AutonDriveStraight.cpp \
../src/Auton/AutonIntake.cpp \
../src/Auton/AutonMoveElevatorToHeight.cpp \
../src/Auton/AutonOuttake.cpp \
../src/Auton/AutonSCurve.cpp \
../src/Auton/AutonStowIntake.cpp \
../src/Auton/AutonTurnLeft.cpp \
../src/Auton/AutonTurnRight.cpp \
../src/Auton/SequentialCommand.cpp 

OBJS += \
./src/Auton/AutoCommand.o \
./src/Auton/AutonDeployIntake.o \
./src/Auton/AutonDriveStraight.o \
./src/Auton/AutonIntake.o \
./src/Auton/AutonMoveElevatorToHeight.o \
./src/Auton/AutonOuttake.o \
./src/Auton/AutonSCurve.o \
./src/Auton/AutonStowIntake.o \
./src/Auton/AutonTurnLeft.o \
./src/Auton/AutonTurnRight.o \
./src/Auton/SequentialCommand.o 

CPP_DEPS += \
./src/Auton/AutoCommand.d \
./src/Auton/AutonDeployIntake.d \
./src/Auton/AutonDriveStraight.d \
./src/Auton/AutonIntake.d \
./src/Auton/AutonMoveElevatorToHeight.d \
./src/Auton/AutonOuttake.d \
./src/Auton/AutonSCurve.d \
./src/Auton/AutonStowIntake.d \
./src/Auton/AutonTurnLeft.d \
./src/Auton/AutonTurnRight.d \
./src/Auton/SequentialCommand.d 


# Each subdirectory must supply rules for building sources it contributes
src/Auton/%.o: ../src/Auton/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++11 -I"\\VMWARE-HOST\Shared Folders\Documents\GitHub\2018FRC\2018FRC\workspace\BB2/src" -IC:\Users\Sanket Nayak/wpilib/simulation/include -I/usr/include -I/usr/include/gazebo-6.5 -I/usr/include/ignition/math2 -I/usr/include/sdformat-3.7 -O0 -Og -g3 -Wall -c -fmessage-length=0 -pthread -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


