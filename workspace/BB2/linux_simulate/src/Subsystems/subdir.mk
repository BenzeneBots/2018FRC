################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/Subsystems/Climber.cpp \
../src/Subsystems/Drive.cpp \
../src/Subsystems/Elevator.cpp \
../src/Subsystems/Intake.cpp 

OBJS += \
./src/Subsystems/Climber.o \
./src/Subsystems/Drive.o \
./src/Subsystems/Elevator.o \
./src/Subsystems/Intake.o 

CPP_DEPS += \
./src/Subsystems/Climber.d \
./src/Subsystems/Drive.d \
./src/Subsystems/Elevator.d \
./src/Subsystems/Intake.d 


# Each subdirectory must supply rules for building sources it contributes
src/Subsystems/%.o: ../src/Subsystems/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -std=c++11 -I"\\VMWARE-HOST\Shared Folders\Documents\GitHub\2018FRC\2018FRC\workspace\BB2/src" -IC:\Users\Sanket Nayak/wpilib/simulation/include -I/usr/include -I/usr/include/gazebo-6.5 -I/usr/include/ignition/math2 -I/usr/include/sdformat-3.7 -O0 -Og -g3 -Wall -c -fmessage-length=0 -pthread -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


