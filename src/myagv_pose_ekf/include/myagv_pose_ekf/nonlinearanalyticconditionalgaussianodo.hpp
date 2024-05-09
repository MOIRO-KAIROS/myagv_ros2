#ifndef __NONLINEARANALYTICCONDITIONALGAUSSIANODO_HPP__
#define __NONLINEARANALYTICCONDITIONALGAUSSIANODO_HPP__

#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>

namespace BFL
{
    class NonLinearAnalyticConditionalGaussianOdo : public AnalyticConditionalGaussianAdditiveNoise
    {
        public:
            NonLinearAnalyticConditionalGaussianOdo(const Gaussian& additiveNoise);
            virtual ~NonLinearAnalyticConditionalGaussianOdo() {}

            virtual MatrixWrapper::ColumnVector   ExpectedValueGet() const;
            virtual MatrixWrapper::Matrix         dfGet(unsigned int i) const;

        private:
            mutable MatrixWrapper::Matrix df;
    };
}

#endif // __NONLINEARANALYTICCONDITIONALGAUSSIANODO_HPP__