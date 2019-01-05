#pragma once

#include <atomic>
#include <frc/PIDOutput.h>

class DriveControllerOutput : public frc::PIDOutput {
public:
	DriveControllerOutput();
	virtual ~DriveControllerOutput();
	void PIDWrite(double output);
	double GetOutput();

private:
	std::atomic<double> val;

};
