/*
A class used to model the properties of the Puck in Air Hockey.
*/
#include "Puck.h"

Puck::Puck() {
	xPos = 200;
	yPos = 200;
	xSpeed = 0.0;
	ySpeed = 0.0;
}

Puck::Puck(int xPosition, int yPosition) {
	xPos = xPosition;
	yPos = yPosition;
	xSpeed = 0.0;
	ySpeed = 0.0;

}

Puck::Puck(int xPosition, int yPosition, double dXSpeed, double dYSpeed) {
	xPos = xPosition;
	yPos = yPosition;
	xSpeed = dXSpeed;
	ySpeed = dYSpeed;

}

Puck::~Puck() {

}

double Puck::getXSpeed() const {
	return xSpeed;
}

double Puck::getYSpeed() const {
	return ySpeed;
}

int Puck::getXPos() const {
	return xPos;
}

int Puck::getYPos() const {
	return yPos;
}

void Puck::setXPos(int newXPos) {
	xPos = newXPos;
}
void Puck::setYPos(int newYPos) {
	yPos = newYPos;
}

/**
 SampleTime i ms.
 XPos pixel
*/
void Puck::calcXSpeed(int oldXPos, int newXPos, int sampleTime) {
	if (oldXPos < newXPos) {
		//Taking the avarage to get a more stable trajectory
		if (xSpeed > 0) {
			double tempSpeed = (newXPos - oldXPos) * 1000 / (sampleTime);
			xSpeed = (tempSpeed + xSpeed) / 2;
		}
		else {
			//Pixlar per sekund
			xSpeed = (newXPos - oldXPos) * 1000 / (sampleTime);
		}
	}
	else if(oldXPos > newXPos){
		//Taking the avarage to get more stable trajectory
		if (xSpeed < 0) {
			double tempSpeed = ((oldXPos - newXPos) * 1000 / (sampleTime)) * -1;
			xSpeed = (tempSpeed + xSpeed) / 2;
		}
		else {
			xSpeed = ((oldXPos - newXPos) * 1000 / (sampleTime)) * -1;
		}
	}
	else {
		xSpeed = 0;
	}

	xSpeed = xSpeed / 1000;
}

/**
SampleTime i ms.
XPos pixel
*/
void Puck::calcYSpeed(int oldYPos, int newYPos, int sampleTime) {
	if (oldYPos < newYPos) {
		if (ySpeed > 0) {
			//If same direction, taka avarage
			double tempSpeed = (newYPos - oldYPos) * 1000 / (sampleTime);
			ySpeed = (tempSpeed + ySpeed) / 2;
		}
		else {
			//Pixlar per tid
			ySpeed = (newYPos - oldYPos) * 1000 / (sampleTime);
		}
	}
	else if (oldYPos > newYPos) {
		//If same direction, taka avarage
		if (ySpeed < 0) {
			double tempSpeed = ((oldYPos - newYPos) * 1000 / (sampleTime))* -1;
			ySpeed = (tempSpeed + ySpeed) / 2;
		}
		else {
			ySpeed = ((oldYPos - newYPos) * 1000 / (sampleTime))* -1;
		}
	}
	else {
		ySpeed = 0;
	}

	ySpeed = ySpeed / 1000;
}

int Puck::calcEndX(int minX, int maxX, int minY, int maxY, int sampleTime) {
	int count = 0;
	double dCurrentX = xPos;
	double dCurrentY = yPos;
	int iCurrentX = xPos;
	int iCurrentY = yPos;
	while (iCurrentX < maxX && iCurrentX > minX && iCurrentY < maxY && iCurrentY > minY) {
		dCurrentX = dCurrentX + xSpeed * count;
		dCurrentY = dCurrentY + ySpeed * count;
		iCurrentX = dCurrentX;
		iCurrentY = dCurrentY;
		count++;
	}
	if (iCurrentX < minX) {
		return minX;
	}
	else if (iCurrentX > maxX) {
		return maxX;
	}

	return iCurrentX;
}

int Puck::calcEndY(int minX, int maxX, int minY, int maxY, int sampleTime) {
	int count = 0;
	double dCurrentX = xPos;
	double dCurrentY = yPos;
	int iCurrentX = xPos;
	int iCurrentY = yPos;
	while (iCurrentX < maxX && iCurrentX > minX && iCurrentY < maxY && iCurrentY > minY) {
		dCurrentX = dCurrentX + xSpeed * count;
		dCurrentY = dCurrentY + ySpeed * count;
		iCurrentX = dCurrentX;
		iCurrentY = dCurrentY;
		count++;
	}

	if (iCurrentY < minY) {
		return minY;
	}
	else if (iCurrentY > maxY) {
		return maxY;
	}

	return iCurrentY;
}