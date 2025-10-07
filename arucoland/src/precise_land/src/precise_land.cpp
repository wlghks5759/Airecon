#include "precise_land.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <optional>

// ======================= 1. 새로운 상태 추가 =======================
// 기존 State enum에 WaitForTarget을 추가합니다.
// (이 코드는 보통 헤더 파일인 precise_land.hpp에 있어야 하지만,
// 설명을 위해 여기에 명시합니다. .hpp 파일의 enum을 수정해주세요.)
enum class State {
	Idle,
	WaitForTarget,  // <--- ADDED: 마커를 기다리는 새로운 상태
	Search,
	Approach,
	Descend,
	Finished
};
// =================================================================

static const std::string kModeName = "ARUCO_PRECISION_LAND";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

Precise_Land::Precise_Land(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{
	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("target_pose",
			   rclcpp::QoS(1).best_effort(), std::bind(&Precise_Land::targetPoseCallback, this, std::placeholders::_1));

    _node.declare_parameter<std::string>("land_detected_topic", "/fmu/out/vehicle_land_detected");
    std::string land_detected_topic;
    _node.get_parameter("land_detected_topic", land_detected_topic);
    RCLCPP_INFO(_node.get_logger(), "Subscribing to land detected topic: %s", land_detected_topic.c_str());
	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>(land_detected_topic,
			   rclcpp::QoS(1).best_effort(), std::bind(&Precise_Land::vehicleLandDetectedCallback, this, std::placeholders::_1));

  	_node.declare_parameter<float>("descent_vel", 1.0);
	_node.declare_parameter<float>("vel_p_gain", 1.5);
	_node.declare_parameter<float>("vel_i_gain", 0.0);
	_node.declare_parameter<float>("max_velocity", 3.0);
	_node.declare_parameter<float>("target_timeout", 3.0);
	_node.declare_parameter<float>("delta_position", 0.25);
	_node.declare_parameter<float>("delta_velocity", 0.25);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);
}

void Precise_Land::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void Precise_Land::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    ArucoTag received_tag;
    received_tag.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    received_tag.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    received_tag.timestamp = _node.now();
    _tag = getTagWorld(received_tag);
}

Precise_Land::ArucoTag Precise_Land::getTagWorld(const ArucoTag& tag)
{
	Eigen::Matrix3d R;
	R << 0, -1, 0, 1, 0, 0, 0, 0, 1;
	Eigen::Quaterniond quat_NED(R);
	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());
	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, 0) * quat_NED;
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
	Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;
    ArucoTag world_tag;
    world_tag.position = tag_world_transform.translation();
    world_tag.orientation = Eigen::Quaterniond(tag_world_transform.rotation());
    world_tag.timestamp = tag.timestamp;
	return world_tag;
}

// ======================= 2. onActivate() 함수 수정 =======================
void Precise_Land::onActivate()
{
	_tag = ArucoTag(); // 이전에 감지된 태그 정보 초기화

    // --- MODIFIED ---
    // 활성화 시점의 현재 위치를 호버링 목표 지점으로 저장합니다.
    _hover_position = _vehicle_local_position->positionNed();
	RCLCPP_INFO(_node.get_logger(), "ARUCO_PRECISION_LAND activated. Hovering and waiting for target...");

    // --- REMOVED ---
	// 더 이상 나선형 탐색을 하지 않으므로 웨이포인트 생성 함수를 호출하지 않습니다.
	// generateSearchWaypoints();

    // --- MODIFIED ---
    // 초기 상태를 Search가 아닌 WaitForTarget으로 변경합니다.
	switchToState(State::WaitForTarget);
}
// ======================================================================

void Precise_Land::onDeactivate()
{
	// No-op
}

// ======================= 3. updateSetpoint() 상태 머신 수정 =======================
void Precise_Land::updateSetpoint(float dt_s)
{
	bool target_lost = checkTargetTimeout();
	if (target_lost && !_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target lost: State %s", stateName(_state).c_str());
	} else if (!target_lost && _target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target acquired");
	}
	_target_lost_prev = target_lost;

	// State machine
	switch (_state) {
	case State::Idle: {
		break;
	}

    // --- ADDED ---
    // 새로운 WaitForTarget 상태의 로직을 추가합니다.
	case State::WaitForTarget: {
		// 마커가 감지되었는지 확인합니다.
		if (!std::isnan(_tag.position.x())) {
			RCLCPP_INFO(_node.get_logger(),"Target detected! Switching to Approach."
			" Tag Position (World NED): x=%.2f, y=%.2f, z=%.2f",
			_tag.position.x(), _tag.position.y(), _tag.position.z());

			// 접근 고도를 현재 드론의 고도로 설정합니다.
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}

		// 마커가 없으면, 활성화 시점의 위치에서 계속 호버링합니다.
		_trajectory_setpoint->updatePosition(_hover_position);
		break;
	}

	case State::Search: {
        // 이 상태는 더 이상 사용되지 않지만, 만약을 위해 남겨둡니다.
        // (코드를 완전히 삭제해도 무방합니다)
		if (!std::isnan(_tag.position.x())) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}
		auto waypoint_position = _search_waypoints[_search_waypoint_index];
		_trajectory_setpoint->updatePosition(waypoint_position);
		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;
			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}
		break;
	}

	case State::Approach: {
		if (target_lost) {
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}
		auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);
		_trajectory_setpoint->updatePosition(target_position);
		if (positionReached(target_position)) {
			switchToState(State::Descend);
		}
		break;
	}

	case State::Descend: {
		if (target_lost) {
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}
		Eigen::Vector2f vel = calculateVelocitySetpointXY();
		_trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt, px4_ros2::quaternionToYaw(_tag.orientation));
		if (_land_detected) {
			switchToState(State::Finished);
		}
		break;
	}

	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}
// ======================================================================

Eigen::Vector2f Precise_Land::calculateVelocitySetpointXY()
{
	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;
	float delta_pos_x = _vehicle_local_position->positionNed().x() - _tag.position.x();
	float delta_pos_y = _vehicle_local_position->positionNed().y() - _tag.position.y();
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);
	float Xp = delta_pos_x * p_gain;
	float Xi = _vel_x_integral * i_gain;
	float Yp = delta_pos_y * p_gain;
	float Yi = _vel_y_integral * i_gain;
	float vx = -1.f * (Xp + Xi);
	float vy = -1.f * (Yp + Yi);
	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);
	return Eigen::Vector2f(vx, vy);
}

bool Precise_Land::checkTargetTimeout()
{
	if (!_tag.valid()) { return false; }
	if (_node.now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}
	return false;
}

void Precise_Land::generateSearchWaypoints()
{
    // 이 함수는 더 이상 호출되지 않지만, 코드는 그대로 둡니다.
    // ... (내용은 이전과 동일) ...
}

bool Precise_Land::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();
	const auto delta_pos = target - position;
	return (delta_pos.norm() < _param_delta_position) && (velocity.norm() < _param_delta_velocity);
}

// ======================= 4. stateName() 함수 수정 =======================
std::string Precise_Land::stateName(State state)
{
	switch (state) {
	case State::Idle:
		return "Idle";
    // --- ADDED ---
    // 새로운 상태의 이름을 로그에 출력하기 위해 추가합니다.
	case State::WaitForTarget:
		return "WaitForTarget";
	case State::Search:
		return "Search";
	case State::Approach:
		return "Approach";
	case State::Descend:
		return "Descend";
	case State::Finished:
		return "Finished";
	default:
		return "Unknown";
	}
}
// ======================================================================

void Precise_Land::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<Precise_Land>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}