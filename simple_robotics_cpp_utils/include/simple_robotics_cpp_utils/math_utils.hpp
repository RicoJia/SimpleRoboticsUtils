// 
/**
* Notes: 
* the inline keyword:
    - is a hint for compiler to replace function calls with the code directly. 
        It's just a hint, compilers may not do it. But it will make sure there's only
        one copy of this function
    - So, it can avoid ODR violation when linked against other source files, inline 
* <cmath> has M_PI
* #pragma once preprocessor directive, though not in standard c++ standard, is portable
*/
#pragma once
#include <cmath>
#include <random>

namespace SimpleRoboticsCppUtils {

    /**
     * @brief Draw from 1D normal distribution with specified mean and variance
     * 
     * @param mean mean of normal distribution
     * @param variance variance of normal distribution
     * @return randomly generated number 
     */
    inline double draw_from_pdf_normal(const double& mean, const double& variance){
        // random seed 
        std::random_device rd;
        // Mersenne twister PRNG, seeded with rd
        std::mt19937 gen(rd());
        std::normal_distribution<double> d(mean, std::sqrt(variance));
        return d(gen);
    }

}