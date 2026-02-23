#include "gaussian_process_regression/gp.h"
#include "gaussian_process_regression/gp_utils.h"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <matplot/matplot.h>

using namespace libgp;

class GpExampleNode : public rclcpp::Node
{
public:
  GpExampleNode() : Node("gp_example_dense")
  {
    this->declare_parameter("n_train", 2000);
    this->declare_parameter("n_test", 500);

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
    std::vector<double> x_vals, y_true_vals, y_pred_vals;

    for (int i = 0; i < m; ++i) {
      double x[] = {drand48() * 4 - 2, drand48() * 4 - 2};
      double f = gp.f(x);
      double y = Utils::hill(x[0], x[1]);
      double error = f - y;
      tss += error * error;

      x_vals.push_back(x[0]);
      y_true_vals.push_back(y);
      y_pred_vals.push_back(f);
    }

    RCLCPP_INFO(this->get_logger(), "MSE = %f", tss / m);

    auto sorted_idx = matplot::iota(0u, x_vals.size() - 1);
    std::sort(sorted_idx.begin(), sorted_idx.end(), [&](size_t a, size_t b) {
      return x_vals[a] < x_vals[b];
    });

    std::vector<double> x_sorted, y_true_sorted, y_pred_sorted;
    for (auto i : sorted_idx) {
      x_sorted.push_back(x_vals[i]);
      y_true_sorted.push_back(y_true_vals[i]);
      y_pred_sorted.push_back(y_pred_vals[i]);
    }

    auto fig = matplot::figure(true);
    matplot::hold(matplot::on);
    matplot::plot(x_sorted, y_true_sorted)->display_name("Ground truth");
    matplot::plot(x_sorted, y_pred_sorted)->display_name("GP prediction");
    matplot::hold(matplot::off);
    matplot::xlabel("x");
    matplot::ylabel("y");
    matplot::title("GP Regression: Truth vs Prediction");
    matplot::legend();
    matplot::show();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpExampleNode>();
  rclcpp::shutdown();
  return 0;
}
