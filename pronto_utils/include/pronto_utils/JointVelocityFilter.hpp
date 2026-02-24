#ifndef JOINTVELOCITYFILTER_HPP
#define JOINTVELOCITYFILTER_HPP
#include <pronto_utils/alpha_filter.hpp>
#include <vector>
#include <iostream>

namespace pronto_utils {


class JointVelocityFilter {
public:
    JointVelocityFilter(const double dt, std::vector<double> alphas) : dt_(dt), alphas_(alphas){
        af_.resize(alphas_.size());

        size_t i = 0;
        for(auto& a : af_){
            a = AlphaFilter<double>(alphas_[i++]);
        }

    }

    JointVelocityFilter(const double dt, double alpha, uint32_t N) :
        JointVelocityFilter(dt,  std::vector<double>(N, alpha)) {
    }

    void processSample(std::vector<double>& pos, std::vector<double>& vel, double dt);
    void processSample(std::vector<double>& pos, std::vector<double>& vel);

private:
    std::vector<AlphaFilter<double>> af_;
    double dt_;
    std::vector<double> alphas_;

};

}

#endif // JOINTVELOCITYFILTER_HPP
