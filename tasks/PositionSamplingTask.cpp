/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PositionSamplingTask.hpp"

using namespace joint_trajectory_follower;

PositionSamplingTask::PositionSamplingTask(std::string const& name)
    : PositionSamplingTaskBase(name)
{
}

PositionSamplingTask::PositionSamplingTask(std::string const& name, RTT::ExecutionEngine* engine)
    : PositionSamplingTaskBase(name, engine)
{
}

PositionSamplingTask::~PositionSamplingTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PositionSamplingTask.hpp for more detailed
// documentation about them.

bool PositionSamplingTask::configureHook()
{
    if (! PositionSamplingTaskBase::configureHook())
        return false;
    return true;
}
bool PositionSamplingTask::startHook()
{
    if (! PositionSamplingTaskBase::startHook())
        return false;
    return true;
}
void PositionSamplingTask::updateHook()
{
    PositionSamplingTaskBase::updateHook();
}
void PositionSamplingTask::errorHook()
{
    PositionSamplingTaskBase::errorHook();
}
void PositionSamplingTask::stopHook()
{
    PositionSamplingTaskBase::stopHook();
}
void PositionSamplingTask::cleanupHook()
{
    PositionSamplingTaskBase::cleanupHook();
}
