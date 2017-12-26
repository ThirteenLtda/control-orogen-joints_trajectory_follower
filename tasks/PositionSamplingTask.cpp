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

    mBreakUpdateHook = false;
    mPeriod = _period.get();
    return true;
}
bool PositionSamplingTask::updateTrajectory()
{
    RTT::FlowStatus state = _trajectory.read(mTrajectory, false);
    if (state == RTT::NoData)
        return false;
    else if (state == RTT::OldData)
        return true;
    if (!mTrajectory.isValid())
    {
        exception(INVALID_TRAJECTORY);
        throw std::runtime_error("received invalid trajectory");
    }
    else if (!mTrajectory.times.empty() && !mTrajectory.times[0].isNull())
    {
        exception(TRAJECTORY_START_TIME_NON_NULL);
        throw std::runtime_error("received trajectory with a non-null start time");
    }
   
    mCurrentStep = 0;
    mTrajectorySize = 0;
    if (!mTrajectory.elements.empty())
        mTrajectorySize = mTrajectory.elements[0].size();
    return true;
}

base::Time PositionSamplingTask::getTimeAtStep(uint64_t step) const
{
    if (mTrajectory.times.empty())
        return mPeriod * step;
    else
        return mTrajectory.times[step];
}

void PositionSamplingTask::getPositionCmdAtStep(uint64_t step, base::samples::Joints& result)
{
    base::Time time = getTimeAtStep(step);
    mTrajectory.getJointsAtTimeStep(step, result);
    for(size_t i=0; i<result.elements.size(); i++)
    {
        result.elements[i] = base::JointState::Position(result.elements[i].position);
    }
    result.time = time;
}

void PositionSamplingTask::updateHook()
{
    base::samples::Joints before, result;
    base::Time currentTime;
    base::Time t0 = base::Time::now();
    base::Time t1;
    while(!mBreakUpdateHook)
    {
        PositionSamplingTaskBase::updateHook();

        if (!updateTrajectory())
        {
            usleep(10000);
            continue;
        }

        base::Time now = base::Time::now();
        uint64_t step = updateCurrentStep(currentTime);

        if (step == 0)
        {
            getPositionCmdAtStep(step, result);
            result.time = result.time + t0;
            _joints_cmd.write(result);
        }
        else if (step == mTrajectorySize)
        {
            getPositionCmdAtStep(step - 1, result);
            if (result.time < now)
                result.time = now;
            _joints_cmd.write(result);
        }
        else
        {
            getPositionCmdAtStep(step, result);
            getPositionCmdAtStep(step - 1, before);
            double r = (currentTime - before.time).toSeconds() / (result.time - before.time).toSeconds();
            for (size_t i = 0; i < before.elements.size(); ++i)
                result.elements[i].position = result.elements[i].position * r + before.elements[i].position * (1 - r);
            result.time = t0 + currentTime;
            _joints_cmd.write(result);
        }

        base::Time nextTick = (currentTime + t0 + mPeriod);
        if (nextTick > now)
        {
            uint64_t deltaT = (nextTick - now).toMicroseconds();
            usleep(deltaT);
        }

        currentTime = currentTime + mPeriod;
    }
}
uint64_t PositionSamplingTask::updateCurrentStep(base::Time t)
{
    if (mCurrentStep == mTrajectorySize)
        return mCurrentStep;

    if (mTrajectory.times.empty())
    {
        return ++mCurrentStep;
    }
    else
    {
        while((mCurrentStep < mTrajectorySize) && (mTrajectory.times[mCurrentStep] < t))
            ++mCurrentStep;
        return mCurrentStep;
    }
}
bool PositionSamplingTask::breakUpdateHook()
{
    mBreakUpdateHook = true;
    return true;
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
