#include "autonomous_actor/AutonomousActorPlugin.hh"

#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(AutoActorPlugin)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
AutoActorPlugin::AutoActorPlugin()
{
}

/////////////////////////////////////////////////
void AutoActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&AutoActorPlugin::OnUpdate, this, std::placeholders::_1)));

  // Added by brucechanjianle
  // Read in multiple targets
  if (_sdf->HasElement("targets"))
  {
    // Obtain targets with element pointer
    sdf::ElementPtr local_targets = _sdf->GetElement("targets")->GetElement("target");

    // Extract target
    while(local_targets)
    {
      this->targets.push_back(local_targets->Get<ignition::math::Vector3d>());
      local_targets = local_targets->GetNextElement("target");
    }

    // Set index to zero
    this->idx = 0;
  }

  this->Reset();

  // Added by ljsv
  if (_sdf->HasElement("limit_x_inf"))
    this->limit_x_inf = _sdf->Get<double>("limit_x_inf");
  else
    this->limit_x_inf = -2.0;

  if (_sdf->HasElement("limit_y_inf"))
    this->limit_y_inf = _sdf->Get<double>("limit_y_inf");
  else
    this->limit_y_inf = -12.0;

  if (_sdf->HasElement("limit_x_sup"))
    this->limit_x_sup = _sdf->Get<double>("limit_x_sup");
  else
    this->limit_x_sup = 6.0;

  if (_sdf->HasElement("limit_y_sup"))
    this->limit_y_sup = _sdf->Get<double>("limit_y_sup");
  else
    this->limit_y_sup = -5.0;  

  if (_sdf->HasElement("velocidad"))
    this->velocity = _sdf->Get<double>("velocidad");
  else
    this->velocity = 5; 


  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement( "model");
    }
  }

  // Added by brucechanjianle
  // Read in target tolerance
  if (_sdf->HasElement("target_tolerance"))
    this->tolerance = _sdf->Get<double>("target_tolerance");
  else
    this->tolerance = 1.5;

}

/////////////////////////////////////////////////
void AutoActorPlugin::Reset()
{
  //this->velocity = 0.8;
  this->lastUpdate = 0;

  this->target = this->targets.at(this->idx);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}

/////////////////////////////////////////////////
void AutoActorPlugin::ChooseNewTarget()
{
  // Added by brucechanjianle
  // Increase index number in sequence
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  { 
    double min_x, max_x, min_y, max_y; 
  
    min_x = std::min(this->limit_x_inf, this->limit_x_sup);
    max_x = std::max(this->limit_x_inf, this->limit_x_sup);
    
    min_y = std::min(this->limit_y_inf, this->limit_y_sup);
    max_y = std::max(this->limit_y_inf, this->limit_y_sup);

    newTarget.X(ignition::math::Rand::DblUniform(min_x, max_x));
    newTarget.Y(ignition::math::Rand::DblUniform(min_y, max_y));
    /*
    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
    */
  }

  // Set next target
  this->target = newTarget;
  
}

/////////////////////////////////////////////////
void AutoActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{ 
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void AutoActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  double distance = pos.Length();
  
  // Added by brucechanjianle
  #ifdef DEBUG_
    // For debug
    // gzdbg << distance << std::endl;
  #endif

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < this->tolerance)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  double min_x, max_x, min_y, max_y; 

  min_x = std::min(this->limit_x_inf, this->limit_x_sup);
  max_x = std::max(this->limit_x_inf, this->limit_x_sup);
  
  min_y = std::min(this->limit_y_inf, this->limit_y_sup);
  max_y = std::max(this->limit_y_inf, this->limit_y_sup);
  
  pose.Pos().X(std::max(min_x, std::min(max_x, pose.Pos().X())));
  pose.Pos().Y(std::max(min_y, std::min(max_y, pose.Pos().Y())));
  pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}
