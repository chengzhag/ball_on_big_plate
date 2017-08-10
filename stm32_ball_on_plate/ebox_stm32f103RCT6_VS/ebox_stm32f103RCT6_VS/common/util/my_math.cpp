#include "my_math.h"

TicToc::TicToc() :
	ticTime(0)
{

}

void TicToc::tic()
{
	ticTime = millis();
}

unsigned long TicToc::toc()
{
	return millis() - ticTime;
}

FpsCounter::FpsCounter()
{

}

void FpsCounter::begin()
{
	tic();
}

float FpsCounter::getFps()
{
	fps = 1000.0f / toc();
	tic();
	return fps;
}

float FpsCounter::getOldFps()
{
	return fps;
}
