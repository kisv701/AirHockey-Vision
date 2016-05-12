#pragma once
//Function decleration for puck
#include <math.h>
#include <iostream>

using namespace std;

#ifndef PUCK_H
#define PUCK_H

class Puck {
public:
	//Default constructor
	Puck();

	//Overload constructor
	Puck(int, int);
	Puck(int xPosition, int yPosition, double xSpeed, double ySpeed);

	//Destructor
	~Puck();

	//Accesser functions
	double getXSpeed() const;
	double getYSpeed() const;
	int getXPos() const;
	int getYPos() const;

	//Mutation functions
	void setXPos(int);
	void setYPos(int);

	void calcXSpeed(int oldXPos, int newXPos, int sampleTime);
	void calcYSpeed(int oldYPos, int newYPos, int sampleTime);

	int calcEndX(int minX, int maxX, int minY, int maxY, int sampleTime);
	int calcEndY(int minX, int maxX, int minY, int maxY, int sampleTime);

private:
	double xSpeed;
	double ySpeed;
	int xPos;
	int yPos;
};
#endif