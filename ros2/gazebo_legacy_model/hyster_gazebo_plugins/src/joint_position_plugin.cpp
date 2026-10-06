// Drives a set of joints to position targets using the physics engine's
// velocity motor, so each joint moves at a bounded speed and holds its load
// when stopped, like a hydraulic cylinder.
//
// <plugin name="implement_control" filename="libhyster_joint_position.so">
//   <gain>5.0</gain>
//   <controlled_joint name="carriage_joint">
//     <max_velocity>0.3</max_velocity>
//     <max_force>300000</max_force>
//   </controlled_joint>
//   <gripper>
//     <joint>clamp_1_joint</joint>
//     <joint>clamp_2_joint</joint>
//   </gripper>
// </plugin>
//
// Targets arrive on `joint_targets` (sensor_msgs/JointState, name + position).
//
// The optional <gripper> names two controlled joints that close on a load.
// Friction grasps are unreliable in ODE (light loads get squeezed out, the
// arms drift), so once both arms press sideways on the same object it is
// attached to their parent link with a fixed joint, and released when the
// arms are commanded to open again.

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace hyster_gazebo_plugins
{

class JointPositionPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;
    ros_node_ = gazebo_ros::Node::Get(_sdf);
    gain_ = _sdf->Get<double>("gain", 5.0).first;

    for (auto elem = _sdf->GetElement("controlled_joint"); elem;
      elem = elem->GetNextElement("controlled_joint"))
    {
      auto name = elem->Get<std::string>("name");
      auto joint = _model->GetJoint(name);
      if (!joint) {
        RCLCPP_ERROR(ros_node_->get_logger(), "Joint [%s] not found, ignoring.", name.c_str());
        continue;
      }
      Axis axis;
      axis.joint = joint;
      axis.max_velocity = elem->Get<double>("max_velocity", 0.2).first;
      axis.max_force = elem->Get<double>("max_force", 1e5).first;
      axis.target = joint->Position(0);
      axis.hold = axis.target;
      axes_.push_back(axis);
      RCLCPP_INFO(ros_node_->get_logger(), "Controlling joint [%s]", name.c_str());
    }

    if (_sdf->HasElement("gripper")) {
      for (auto elem = _sdf->GetElement("gripper")->GetElement("joint"); elem;
        elem = elem->GetNextElement("joint"))
      {
        auto name = elem->Get<std::string>();
        for (size_t i = 0; i < axes_.size(); ++i) {
          if (axes_[i].joint->GetName() == name) {
            grip_axes_.push_back(i);
          }
        }
      }
      if (grip_axes_.size() == 2) {
        // Contacts are only kept when someone listens for them
        _model->GetWorld()->Physics()->GetContactManager()->SetNeverDropContacts(true);
        RCLCPP_INFO(ros_node_->get_logger(), "Gripper enabled");
      } else {
        RCLCPP_ERROR(ros_node_->get_logger(), "<gripper> needs two controlled joints, ignoring.");
        grip_axes_.clear();
      }
    }

    sub_ = ros_node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_targets", rclcpp::QoS(10),
      std::bind(&JointPositionPlugin::OnTargets, this, std::placeholders::_1));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&JointPositionPlugin::OnUpdate, this));
  }

private:
  struct Axis
  {
    gazebo::physics::JointPtr joint;
    double max_velocity;
    double max_force;
    double target;
    // Position held while a load is attached, in place of `target`
    double hold;
  };

  void OnTargets(const sensor_msgs::msg::JointState::SharedPtr _msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < _msg->name.size() && i < _msg->position.size(); ++i) {
      for (auto & axis : axes_) {
        if (axis.joint->GetName() == _msg->name[i]) {
          axis.target = std::max(
            axis.joint->LowerLimit(0), std::min(axis.joint->UpperLimit(0), _msg->position[i]));
        }
      }
    }
  }

  void OnUpdate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!grip_axes_.empty()) {
      UpdateGripper();
    }
    for (size_t i = 0; i < axes_.size(); ++i) {
      auto & axis = axes_[i];
      bool holding = held_link_ && (i == grip_axes_[0] || i == grip_axes_[1]);
      double error = (holding ? axis.hold : axis.target) - axis.joint->Position(0);
      double vel = std::max(-axis.max_velocity, std::min(axis.max_velocity, gain_ * error));
      axis.joint->SetParam("fmax", 0, axis.max_force);
      axis.joint->SetParam("vel", 0, vel);
    }
  }

  /// Object pressed sideways by the arm of a gripper axis, if any.
  gazebo::physics::LinkPtr PressedBy(const Axis & _axis)
  {
    auto arm = _axis.joint->GetChild();
    auto direction = _axis.joint->GlobalAxis(0);
    auto manager = model_->GetWorld()->Physics()->GetContactManager();
    const auto & contacts = manager->GetContacts();
    for (unsigned int i = 0; i < manager->GetContactCount(); ++i) {
      auto contact = contacts[i];
      auto link1 = contact->collision1->GetLink();
      auto link2 = contact->collision2->GetLink();
      auto other = link1 == arm ? link2 : (link2 == arm ? link1 : nullptr);
      if (!other || other->IsStatic() || other->GetModel() == model_) {
        continue;
      }
      for (int k = 0; k < contact->count; ++k) {
        if (std::abs(contact->normals[k].Dot(direction)) > 0.7) {
          return other;
        }
      }
    }
    return nullptr;
  }

  void UpdateGripper()
  {
    auto & a = axes_[grip_axes_[0]];
    auto & b = axes_[grip_axes_[1]];

    if (held_link_) {
      if (a.target < a.hold - kReleaseMargin || b.target < b.hold - kReleaseMargin) {
        RCLCPP_INFO(
          ros_node_->get_logger(), "Released [%s]", held_link_->GetScopedName().c_str());
        grasp_joint_->Detach();
        grasp_joint_.reset();
        held_link_.reset();
      }
      return;
    }

    // Only grab while the arms are being told to close further
    bool closing = a.target > a.joint->Position(0) + kCloseMargin ||
      b.target > b.joint->Position(0) + kCloseMargin;
    auto pressed = closing ? PressedBy(a) : nullptr;
    if (!pressed || pressed != PressedBy(b)) {
      pressed_steps_ = 0;
      return;
    }
    if (++pressed_steps_ < kAttachSteps) {
      return;
    }
    pressed_steps_ = 0;

    auto palm = a.joint->GetParent();
    grasp_joint_ = model_->GetWorld()->Physics()->CreateJoint("fixed", model_);
    grasp_joint_->Attach(palm, pressed);
    grasp_joint_->Load(palm, pressed, ignition::math::Pose3d());
    grasp_joint_->SetModel(model_);
    grasp_joint_->Init();
    held_link_ = pressed;
    a.hold = a.joint->Position(0);
    b.hold = b.joint->Position(0);
    RCLCPP_INFO(ros_node_->get_logger(), "Grabbed [%s]", pressed->GetScopedName().c_str());
  }

  static constexpr double kReleaseMargin = 0.01;
  static constexpr double kCloseMargin = 0.002;
  static constexpr int kAttachSteps = 20;

  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  gazebo::event::ConnectionPtr update_connection_;
  std::vector<Axis> axes_;
  std::vector<size_t> grip_axes_;
  gazebo::physics::JointPtr grasp_joint_;
  gazebo::physics::LinkPtr held_link_;
  int pressed_steps_{0};
  std::mutex mutex_;
  double gain_{5.0};
};

GZ_REGISTER_MODEL_PLUGIN(JointPositionPlugin)

}  // namespace hyster_gazebo_plugins
