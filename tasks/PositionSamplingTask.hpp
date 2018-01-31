/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef JOINT_TRAJECTORY_FOLLOWER_POSITIONSAMPLINGTASK_TASK_HPP
#define JOINT_TRAJECTORY_FOLLOWER_POSITIONSAMPLINGTASK_TASK_HPP

#include "joint_trajectory_follower/PositionSamplingTaskBase.hpp"

namespace joint_trajectory_follower{

    /*! \class PositionSamplingTask
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Periodically emits a joint position based on a JointTrajectory

Does simple linear interpolation between two samples in the trajectory
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','joint_trajectory_follower::PositionSamplingTask')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class PositionSamplingTask : public PositionSamplingTaskBase
    {
	friend class PositionSamplingTaskBase;
    protected:
        bool mBreakUpdateHook;
        base::Time mPeriod;
        uint64_t mTrajectorySize;
        uint64_t mCurrentStep;
        uint64_t updateCurrentStep(base::Time t);
        base::JointsTrajectory mTrajectory;
        void getPositionCmdAtStep(uint64_t step, base::samples::Joints& result);
        base::Time getTimeAtStep(uint64_t step) const;

        /** Update the trajectory from the port, if needed
         *
         * @return true if we have a trajectory to execute, false otherwise
         */
        bool updateTrajectory();

    public:
        /** TaskContext constructor for PositionSamplingTask
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        PositionSamplingTask(std::string const& name = "joint_trajectory_follower::PositionSamplingTask");

        /** TaskContext constructor for PositionSamplingTask
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices.
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task.
         *
         */
        PositionSamplingTask(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of PositionSamplingTask
         */
	~PositionSamplingTask();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        bool breakUpdateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif
