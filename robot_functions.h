#ifndef ROBOT_FUNCTIONS_H
#define ROBOT_FUNCTIONS_H

#include "vex.h"

void pre_auton();
int intialSearch();
bool searchCube();
void intake(int vel);
void driveStop();
void clockWiseTurn(int degrees);
void counterTurn(int degrees);
void allignWall(int time, int vel);
bool searchWall();
void scoreThenBack();

#endif
