/*
 * main.cpp
 *
 *  Created on: 9 déc. 2012
 *      Author: j.bolting
 */

#include "../inc/alPeriodicPingNode.h"
#include "../../convenienceLayer/inc/clAssistant.h"
#include "../../convenienceLayer/inc/clTimebase.h"


int main(){

	TimeBase::Init();
	clAssistant::Init();
	alPeriodicPingNode* node = new alPeriodicPingNode();
	node->runLoop();
}


