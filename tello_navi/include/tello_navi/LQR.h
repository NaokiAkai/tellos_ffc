#ifndef __LQR_H__
#define __LQR_H__

#include <Eigen/Dense>

class LQR {
private:
    int dimX_, dimU_;
    Eigen::MatrixXd F_;

public:
    LQR(void) {}

    LQR(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R):
        dimX_(A.rows()),
        dimU_(R.rows())
    {
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dimX_, dimX_);

        if (!solveRiccatiArimotoPotter(A, B, Q, R, P)) {
            fprintf(stderr, "Could not solve Riccati equation.\n");
            exit(1);
        }

        F_ = -R.inverse() * B.transpose() * P;
    }

    void printFeedbackGain(void) {
        std::cout << F_ << std::endl << std::endl;
    }

    Eigen::MatrixXd getControlInput(Eigen::VectorXd output) {
        return F_ * output;
    }

private:
    bool solveRiccatiArimotoPotter(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R, Eigen::MatrixXd &P) {
        Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dimX_, 2 * dimX_);
        Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

        Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);
        Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dimX_, dimX_);
        int j = 0;
        for (int i = 0; i < 2 * dimX_; ++i) {
            if (Eigs.eigenvalues()[i].real() < 0.) {
                eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dimX_, 1);
                ++j;
            }
        }

        Eigen::MatrixXcd vs1, vs2;
        vs1 = eigvec.block(0, 0, dimX_, dimX_);
        vs2 = eigvec.block(dimX_, 0, dimX_, dimX_);
        P = (vs2 * vs1.inverse()).real();
        return true;
    }
}; // class LQR

#endif // __LQR_H__