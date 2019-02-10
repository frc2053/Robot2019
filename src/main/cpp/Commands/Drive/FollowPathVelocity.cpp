#include "Commands/Drive/FollowPathVelocity.h"
#include "Robot.h"

FollowPathVelocity::FollowPathVelocity(std::string pathName) {
  Requires(Robot::drivebaseSubsystem.get());
  isDone = false;
  PathLoader flLoader = PathLoader();
  PathLoader frLoader = PathLoader();
  PathLoader blLoader = PathLoader();
  PathLoader brLoader = PathLoader();

  flVelocityList = flLoader.LoadVelocityPath("/home/lvuser/flvelocity" + pathName);
  frVelocityList = flLoader.LoadVelocityPath("/home/lvuser/frvelocity" + pathName);
  blVelocityList = flLoader.LoadVelocityPath("/home/lvuser/blvelocity" + pathName);
  brVelocityList = flLoader.LoadVelocityPath("/home/lvuser/brvelocity" + pathName);
  int tick = 0;
}

void FollowPathVelocity::Initialize() {

}

void FollowPathVelocity::Execute() {
  Robot::drivebaseSubsystem->SetWheelVelocities(flVelocityList[tick], frVelocityList[tick], blVelocityList[tick], brVelocityList[tick]);
  tick++;
  if(tick > flVelocityList.size()) {
    isDone = true;
  }
}

bool FollowPathVelocity::IsFinished() { 
  return isDone; 
}  

void FollowPathVelocity::End() {
  Robot::drivebaseSubsystem->SetWheelVelocities(0, 0, 0, 0);
}

void FollowPathVelocity::Interrupted() {

}
