#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{

using Vector3 = std::array<double, 3>;
using Vector6 = std::array<double, 6>;
using Matrix6 = std::array<std::array<double, 6>, 6>;

Vector3 cross(const Vector3 & a, const Vector3 & b)
{
  return {
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0]};
}

double norm(const Vector3 & v)
{
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

Matrix6 invert(Matrix6 matrix)
{
  Matrix6 inverse{};
  for (std::size_t i = 0; i < 6; ++i) {
    inverse[i][i] = 1.0;
  }

  for (std::size_t col = 0; col < 6; ++col) {
    std::size_t pivot = col;
    for (std::size_t row = col + 1; row < 6; ++row) {
      if (std::abs(matrix[row][col]) > std::abs(matrix[pivot][col])) {
        pivot = row;
      }
    }

    if (std::abs(matrix[pivot][col]) < 1e-9) {
      throw std::runtime_error("thruster allocation matrix is singular");
    }

    std::swap(matrix[col], matrix[pivot]);
    std::swap(inverse[col], inverse[pivot]);

    const double pivot_value = matrix[col][col];
    for (std::size_t j = 0; j < 6; ++j) {
      matrix[col][j] /= pivot_value;
      inverse[col][j] /= pivot_value;
    }

    for (std::size_t row = 0; row < 6; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = matrix[row][col];
      for (std::size_t j = 0; j < 6; ++j) {
        matrix[row][j] -= factor * matrix[col][j];
        inverse[row][j] -= factor * inverse[col][j];
      }
    }
  }

  return inverse;
}

Matrix6 transpose(const Matrix6 & matrix)
{
  Matrix6 output{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 6; ++col) {
      output[row][col] = matrix[col][row];
    }
  }
  return output;
}

Matrix6 multiply(const Matrix6 & a, const Matrix6 & b)
{
  Matrix6 output{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 6; ++col) {
      for (std::size_t k = 0; k < 6; ++k) {
        output[row][col] += a[row][k] * b[k][col];
      }
    }
  }
  return output;
}

Vector6 multiply(const Matrix6 & matrix, const Vector6 & vector)
{
  Vector6 result{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 6; ++col) {
      result[row] += matrix[row][col] * vector[col];
    }
  }
  return result;
}

std::array<Vector3, 6> to_vector3_array(const std::vector<double> & values)
{
  if (values.size() != 18) {
    throw std::runtime_error("expected 18 values for six Vector3 entries");
  }

  std::array<Vector3, 6> output{};
  for (std::size_t i = 0; i < 6; ++i) {
    output[i] = {values[3 * i], values[3 * i + 1], values[3 * i + 2]};
  }
  return output;
}

}  // namespace

class ThrusterAllocator : public rclcpp::Node
{
public:
  ThrusterAllocator()
  : Node("thruster_allocator")
  {
    input_wrench_topic_ = declare_parameter<std::string>("input_wrench_topic", "cmd_wrench");
    const auto topic_prefix = declare_parameter<std::string>("thruster_topic_prefix", "cmd_thruster");
    max_thrust_ = declare_parameter<double>("max_thrust", 40.0);
    min_thrust_ = declare_parameter<double>("min_thrust", -40.0);
    deadzone_ = std::abs(declare_parameter<double>("deadzone", 0.0));
    lowpass_alpha_ = clamp(declare_parameter<double>("lowpass_alpha", 1.0), 0.0, 1.0);
    damping_lambda_ = std::abs(declare_parameter<double>("damping_lambda", 0.1));
    const auto publish_rate_hz = declare_parameter<double>("publish_rate_hz", 30.0);

    const auto positions_flat =
      declare_parameter<std::vector<double>>("thruster_positions", std::vector<double>{});
    const auto directions_flat =
      declare_parameter<std::vector<double>>("thruster_directions", std::vector<double>{});

    const auto positions = to_vector3_array(positions_flat);
    const auto directions = to_vector3_array(directions_flat);
    allocation_matrix_ = build_allocation_matrix(positions, directions);
    pseudo_inverse_allocation_matrix_ =
      compute_damped_pseudo_inverse(allocation_matrix_, damping_lambda_);

    for (std::size_t i = 0; i < 6; ++i) {
      publishers_[i] = create_publisher<std_msgs::msg::Float64>(
        topic_prefix + std::to_string(i + 1), 10);
    }

    subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      input_wrench_topic_, 10,
      std::bind(&ThrusterAllocator::on_wrench, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&ThrusterAllocator::publish_commands, this));

    RCLCPP_INFO(
      get_logger(), "thruster allocator ready: input=%s, output=%s1..6",
      input_wrench_topic_.c_str(), topic_prefix.c_str());
  }

private:
  Matrix6 build_allocation_matrix(
    const std::array<Vector3, 6> & positions,
    const std::array<Vector3, 6> & directions)
  {
    Matrix6 matrix{};

    for (std::size_t col = 0; col < 6; ++col) {
      Vector3 force = directions[col];
      const auto force_norm = norm(force);
      if (force_norm < 1e-9) {
        throw std::runtime_error("thruster direction cannot be zero");
      }
      for (auto & value : force) {
        value /= force_norm;
      }

      const auto torque = cross(positions[col], force);
      matrix[0][col] = force[0];
      matrix[1][col] = force[1];
      matrix[2][col] = force[2];
      matrix[3][col] = torque[0];
      matrix[4][col] = torque[1];
      matrix[5][col] = torque[2];
    }

    return matrix;
  }

  Matrix6 compute_damped_pseudo_inverse(const Matrix6 & allocation_matrix, const double damping)
  {
    // Damped least-squares pseudo-inverse:
    //   A+ = A^T (A A^T + lambda^2 I)^-1
    // This remains usable when the vehicle is underactuated or near singular.
    const auto allocation_transpose = transpose(allocation_matrix);
    auto normal_matrix = multiply(allocation_matrix, allocation_transpose);
    for (std::size_t i = 0; i < 6; ++i) {
      normal_matrix[i][i] += damping * damping;
    }
    return multiply(allocation_transpose, invert(normal_matrix));
  }

  void on_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    const Vector6 wrench = {
      msg->wrench.force.x,
      msg->wrench.force.y,
      msg->wrench.force.z,
      msg->wrench.torque.x,
      msg->wrench.torque.y,
      msg->wrench.torque.z};

    auto command = multiply(pseudo_inverse_allocation_matrix_, wrench);
    for (auto & value : command) {
      value = shape_command(value);
    }
    target_command_ = command;
  }

  double shape_command(double value) const
  {
    if (std::abs(value) < deadzone_) {
      return 0.0;
    }
    return clamp(value, min_thrust_, max_thrust_);
  }

  void publish_commands()
  {
    for (std::size_t i = 0; i < 6; ++i) {
      filtered_command_[i] += lowpass_alpha_ * (target_command_[i] - filtered_command_[i]);

      std_msgs::msg::Float64 msg;
      msg.data = filtered_command_[i];
      publishers_[i]->publish(msg);
    }
  }

  std::string input_wrench_topic_;
  double max_thrust_{40.0};
  double min_thrust_{-40.0};
  double deadzone_{0.0};
  double lowpass_alpha_{1.0};
  double damping_lambda_{0.1};
  Matrix6 allocation_matrix_{};
  Matrix6 pseudo_inverse_allocation_matrix_{};
  Vector6 target_command_{};
  Vector6 filtered_command_{};

  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 6> publishers_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ThrusterAllocator>());
  rclcpp::shutdown();
  return 0;
}
