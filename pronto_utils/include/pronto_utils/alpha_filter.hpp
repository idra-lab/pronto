#ifndef __ALPHA_FILTER_HPP__
#define __ALPHA_FILTER_HPP__

#include <Eigen/Dense>

namespace pronto_utils {

template <class T>
class AlphaFilter {
 public:
  AlphaFilter(double alpha_ = 0.0);

  ~AlphaFilter() {}

  void processSample(const T& x, T& x_filtered);
  /**
   * @brief processSample
   * @param x [in/out] calculation in place of the new value
   */
  void processSample(T &x);

  T getFilteredPrev() {
      return  x_filtered_prev_;
  }

  bool isInit(){
      return init_;
  }

 private:
  double alpha_;
  bool init_ = false;
  T x_filtered_prev_;
};

// Tell other translation units: "don't instantiate these here"
extern template class AlphaFilter<double>;
extern template class AlphaFilter<Eigen::Vector3d>;
extern template class AlphaFilter<Eigen::VectorXd>;

}  // namespace pronto_utils

#endif
