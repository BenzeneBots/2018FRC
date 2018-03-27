/*
 * Wait.cpp
 *
 *  Created on: Mar 27, 2018
 *      Author: Sabita Dhal
 */

#include <Auton/Wait.h>

Wait::Wait(double time){
	timeOut = time;
	waitTimer = new Timer();
}
Wait::~Wait(){

}

void Wait::Initialize(){
	waitTimer->Reset();
	waitTimer->Start();
}
bool Wait::Run(){
	if(waitTimer->Get()>=timeOut){
		return true;
	}else{
		return false;
	}
}
