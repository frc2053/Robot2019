#include "DriveControllerOutput.h"

DriveControllerOutput::DriveControllerOutput() {

}

DriveControllerOutput::~DriveControllerOutput() {
}

void DriveControllerOutput::PIDWrite(double output) {
	val = output;
}

double DriveControllerOutput::GetOutput() {
	return val;
}
