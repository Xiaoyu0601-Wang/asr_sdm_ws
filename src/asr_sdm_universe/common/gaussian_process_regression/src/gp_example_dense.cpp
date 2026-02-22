#include "gaussian_process_regression/gp.h"
#include "gaussian_process_regression/gp_utils.h"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

using namespace libgp;

class GpExampleNode : public rclcpp::Node
{
public:
  GpExampleNode() : Node("gp_example_dense")
  {
    this->declare_parameter("n_train", 4000);
    this->declare_parameter("n_test", 1000);

    int n = this->get_parameter("n_train").as_int();
    int m = this->get_parameter("n_test").as_int();

    RCLCPP_INFO(this->get_logger(), "Running GP regression: %d train, %d test samples", n, m);

    GaussianProcess gp(2, "CovSum ( CovSEiso, CovNoise)");

    Eigen::VectorXd params(gp.covf().get_param_dim());
    params << 0.0, 0.0, -2.0;
    gp.covf().set_loghyper(params);

    for (int i = 0; i < n; ++i) {
      double x[] = {drand48() * 4 - 2, drand48() * 4 - 2};
      double y = Utils::hill(x[0], x[1]) + Utils::randn() * 0.1;
      gp.add_pattern(x, y);
    }

    double tss = 0;
    for (int i = 0; i < m; ++i) {
      double x[] = {drand48() * 4 - 2, drand48() * 4 - 2};
      double f = gp.f(x);
      double y = Utils::hill(x[0], x[1]);
      double error = f - y;
      tss += error * error;
    }

    RCLCPP_INFO(this->get_logger(), "MSE = %f", tss / m);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpExampleNode>();
  rclcpp::shutdown();
  return 0;
}
