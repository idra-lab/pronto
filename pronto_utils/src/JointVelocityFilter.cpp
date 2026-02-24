
#include <pronto_utils/JointVelocityFilter.hpp>

namespace pronto_utils {

void JointVelocityFilter::processSample(std::vector<double>& pos, std::vector<double>& vel, double dt){
    size_t i = 0; // assuming pos and vel have the same size here!
    for (auto& p : pos) {
        const auto p_old = af_[i].getFilteredPrev();
        //std::cerr << " p = " << p << std::endl;
        //std::cerr << " p_old = " << p_old << std::endl;
        af_[i].processSample(p);
        //std::cerr << " p_f = " << p << std::endl;
        if(!af_[i].isInit()){
            continue;
        }
        vel[i++] = (p - p_old) / dt;
        //std::cerr << "swapped_msg.velocity[" << i - 1 << "] = " << p  - p_old << " / " << dt << " = " << swapped_msg.velocity[i - 1]
        //          << std::endl;
    }
}

void JointVelocityFilter::processSample(std::vector<double> &pos, std::vector<double> &vel){
    processSample(pos, vel, dt_);
}
}
