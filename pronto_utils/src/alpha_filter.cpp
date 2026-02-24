#include "pronto_utils/alpha_filter.hpp"
#include <iostream>
using namespace Eigen;
using namespace std;

namespace pronto_utils {

template <class T>
AlphaFilter<T>::AlphaFilter(double alpha_) : alpha_(alpha_), init_(false){
}

template <class T>
void AlphaFilter<T>::processSample(const T& x, T& x_filtered) {
  if (!init_) {
    x_filtered = x;
    x_filtered_prev_ = x_filtered;
    init_ = true;
    return;
  }

  x_filtered = alpha_ * x_filtered_prev_ + (1.0 - alpha_) * x;
  x_filtered_prev_ = x_filtered;
}

template <class T>
void AlphaFilter<T>::processSample(T& x){

    if (!init_) {
        T x_filtered = x;
        x_filtered_prev_ = x_filtered;
        init_ = true;
        return;
    }

    T x_filtered = alpha_ * x_filtered_prev_ + (1.0 - alpha_) * x;
    std::cerr << "x_filtered = " << alpha_ << " * " << x_filtered_prev_ << " + (1.0 - " << alpha_ << ") * " << x << " = " << x_filtered << endl;
    x_filtered_prev_ = x_filtered;
    x = x_filtered;
}

template class AlphaFilter<double>;
template class AlphaFilter<Eigen::Vector3d>;
template class AlphaFilter<Eigen::VectorXd>;

}  // namespace pronto_utils
