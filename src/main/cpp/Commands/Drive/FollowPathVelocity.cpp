#include "Commands/Drive/FollowPathVelocity.h"
#include "Robot.h"

FollowPathVelocity::FollowPathVelocity(std::string pathName) {
  Requires(Robot::drivebaseSubsystem.get());
  isDone = false;
  Path2D nullPath;
  PathLoader flLoader = PathLoader(nullPath);
  PathLoader frLoader = PathLoader(nullPath);
  PathLoader blLoader = PathLoader(nullPath);
  PathLoader brLoader = PathLoader(nullPath);

  flVelocityList = flLoader.LoadVelocityPath("/home/lvuser/deploy/flvelocity" + pathName);
  frVelocityList = flLoader.LoadVelocityPath("/home/lvuser/deploy/frvelocity" + pathName);
  blVelocityList = flLoader.LoadVelocityPath("/home/lvuser/deploy/blvelocity" + pathName);
  brVelocityList = flLoader.LoadVelocityPath("/home/lvuser/deploy/brvelocity" + pathName);
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
