#include <franka/robot.h>

int main(int argc, char** argv)
{
	franka::Robot robot("192.168.1.2");
	return 0;
}