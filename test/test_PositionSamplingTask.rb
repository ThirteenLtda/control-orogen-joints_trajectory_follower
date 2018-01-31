require 'orocos/test/component'
require 'minitest/autorun'
require 'minitest/spec'

describe 'PositionSamplingTask' do
    include Orocos::Test::Component

    start 'task', 'joint_trajectory_follower::PositionSamplingTask' => 'task'
    writer 'task', 'trajectory'
    reader 'task', 'joints_cmd', type: :buffer, size: 1000

    before do
        task.period = Time.at(0.1)
        task.configure
    end

    it "rejects a trajectory that is not valid" do
        traj = Types.base.JointsTrajectory.new(
            names: [],
            times: [Time.at(0)],
            elements: [])

        task.start
        task_trajectory.write(traj)
        assert_state_change(task) { |s| s == :INVALID_TRAJECTORY }
    end

    it "rejects a trajectory whose time array does not start with a null time" do
        traj = Types.base.JointsTrajectory.new(
            names: [],
            times: [Time.now],
            elements: [[Types.base.JointState.new]])

        task.start
        task_trajectory.write(traj)
        assert_state_change(task) { |s| s == :TRAJECTORY_START_TIME_NON_NULL }
    end

    it "rejects a trajectory with no joint names" do
        traj = Types.base.JointsTrajectory.new(
            names: [],
            times: [Time.at(0)],
            elements: [[Types.base.JointState.new]])

        task.start
        task_trajectory.write(traj)
        assert_state_change(task) { |s| s == :INVALID_JOINT_NAMES }
    end


    def setup_and_read_samples(times: [], elements: [], sample_count: 10)
        traj = Types.base.JointsTrajectory.new(
            names: [],
            times: times,
            elements: elements)

        task.start
        task_trajectory.write(traj)

        samples = []
        while samples.size != sample_count
            samples << assert_has_one_new_sample(task_joints_cmd)
        end
        samples
    end

    def assert_matches_period(samples)
        times = samples.map { |s| s.time.to_f }
        times.each_with_index.each_cons(2).map do |(before, before_i), (after, after_i)|
            assert_in_delta (after - before), 0.1, 1e-2, "failed at i=[#{before_i}, #{after_i}]: #{times}"
        end
    end

    it "sends one sample per cycle when there are no times specified" do
        samples = setup_and_read_samples(
            elements: [[Types.base.JointState.new(position: 0), Types.base.JointState.new(position: 1)]],
            sample_count: 10)

        assert_matches_period samples

        positions = samples.map { |s| s.elements[0].position }
        assert_equal([0] + [1] * 9, positions)
    end

    it "sub-samples the trajectory" do
        samples = setup_and_read_samples(
            times: [Time.at(0), Time.at(1)],
            elements: [[Types.base.JointState.new(position: 0), Types.base.JointState.new(position: 1)]],
            sample_count: 15)

        assert_matches_period samples
        positions = samples.map { |s| s.elements[0].position }
        expected  = (0...11).map { |i| 0.1 * i }
        [positions[0, 11], expected].transpose.each do |p, e|
            assert_in_delta p, e, 1e-3
        end
        positions[11..-1].each do |p|
            assert_in_delta p, 1, 1e-3
        end
    end

    it "interpolates the position" do
    end
end
