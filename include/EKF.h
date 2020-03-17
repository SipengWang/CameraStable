#ifndef EKF_H
#define EKF_H


#include "common_include.h"
#include "index.h"

using namespace std;
using namespace Eigen;

class EKF
{
public:
    EKF();
    ~EKF();

    Vector3f filter(Vector3f Z);
    Vector3f filter(vector<Vector3f>& trajectory, int type);
    Vector3f filter(Vector3f Z, bool extended);
    Vector3f constrain(Vector3f Xk_);

private:

    Vector3f X{0, 0, 0}, Xk{0, 0, 0}, Xk_{0, 0, 0};
    Matrix3f P = Vector3f(1, 1, 1).asDiagonal(), Pk= Vector3f(1, 1, 1).asDiagonal(),
             Pk_= Vector3f(1, 1, 1).asDiagonal(), K= Vector3f(1, 1, 1).asDiagonal(),
             A = Vector3f(1, 1, 1).asDiagonal(), H = Vector3f(1, 1, 1).asDiagonal();
    Matrix3f Q = Vector3f(KALMANQ, KALMANQ, KALMANQ).asDiagonal(),
             R = Vector3f(KALMANR, KALMANR, KALMANR).asDiagonal();
    Matrix3f I = Vector3f(1, 1, 1).asDiagonal(), temp;

    Matrix<float, 5, 1> Xe, Xek, Xek_;
    Matrix<float, 5, 5> Pe, Pek, Pek_, Ae, Qe, Ie;
    Matrix<float, 5, 3> Ke;
    Matrix<float, 3, 5> He;
    Matrix<float, 3, 3> Re;

};

EKF::EKF()
{
    Xe << 0, 0, 0, 0, 0;
    Xek = Xe; Xek_ = Xe;

    for (int i = 0; i < 5; i++)
        for(int j = 0; j < 5; j++)
        {
            if (i == j)
                Pe(i, j) = 1;
            else
                Pe(i, j) = 0;
        }
    Pek = Pe; Pek_ = Pe; Ae = Pe; Qe = Pe * KALMANQ; Ie = Pe;
    Ae(0, 1) = 1; Ae(2, 3) = 1;
    He << 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 0, 1;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            if (i == j)
                Re(i, j) = KALMANR;
            else
                Re(i, j) = 0;
        }
}
EKF::~EKF() {}

Vector3f EKF::filter(Vector3f Z)
{
//    Xk_ = X;
//    Pk_ = A * P * A.transpose() + Q;

//    temp = (H * Pk_ * H.transpose() + R).inverse();
//    if (isnanf(temp.determinant()))
//    {
//        temp = I;
//    }
//    K = Pk_ * H.transpose() * temp;
//    Xk = Xk_ + K * (Z - H * Xk_);
//    Pk = (I - K * H) * Pk_;

//    Xk = constrain(Xk);
//    X = Xk;
//    P = Pk;

//    return Xk;

    Xk_ = X;
    Pk_ = P + Q;

    temp = (Pk_ + R).inverse();
    if (isnanf(temp.determinant()))
    {
        temp = I;
    }
    K = Pk_ * temp;
    Xk = Xk_ + K * (Z - Xk_);
    Pk = (I - K) * Pk_;

    X = Xk;
    P = Pk;

    return Xk;


}

Vector3f EKF::filter(vector<Vector3f> &trajectory, int type)
{
    Vector3f Z = trajectory.back();
    if (type == FILTER_SMOOTH)
    {
        Vector3f output{0, 0, 0};
        int i;
        for (i = 0; i < min(SMOOTH_RADIUS, int(trajectory.size())); i++)
        {
            output += trajectory[trajectory.size() - 1 - i];
        }
        output /= i;
        return output;
    }

    if (type == FILTER_EKF)
    {
        Xk_ = X;
        Pk_ = P + Q;

        temp = (Pk_ + R).inverse();
        if (isnanf(temp.determinant()))
        {
            temp = I;
        }
        K = Pk_ * temp;
        Xk = Xk_ + K * (Z - Xk_);
        Pk = (I - K) * Pk_;

        X = Xk;
        P = Pk;
        return Xk;
    }

    if (type == FILTER_EKF_EXTEND)
    {
        Xek_ = Ae * Xe;
        Pek_ = Ae * Pe * Ae.transpose() + Qe;

        temp = (He * Pek_ * He.transpose() + Re).inverse();
        if (isnanf(temp.determinant()))
        {
            temp = I;
        }
        Ke = Pek_ * He.transpose() * temp;
        Xek = Xek_ + Ke * (Z - He * Xek_);
        Pek = (Ie - Ke * He) * Pek_;

    //    Xek = constrain(Xek);
        Xe = Xek;
        Pe = Pek;

        return Vector3f(Xek(0, 0), Xek(2, 0), Xek(4, 0));
    }
}

Vector3f EKF::filter(Vector3f Z, bool extended)
{
    Xek_ = Ae * Xe;
    Pek_ = Ae * Pe * Ae.transpose() + Qe;

    temp = (He * Pek_ * He.transpose() + Re).inverse();
    if (isnanf(temp.determinant()))
    {
        temp = I;
    }
    Ke = Pek_ * He.transpose() * temp;
    Xek = Xek_ + Ke * (Z - He * Xek_);
    Pek = (Ie - Ke * He) * Pek_;

//    Xek = constrain(Xek);
    Xe = Xek;
    Pe = Pek;

    return Vector3f(Xek(0, 0), Xek(2, 0), Xek(4, 0));
}

Vector3f EKF::constrain(Vector3f Xk_)
{
    if (fabs(Xk_[0]) > 40)
        Xk_[0] /= 2;
    if (fabs(Xk_[1]) > 40)
        Xk_[1] /= 2;

    return Xk_;
}
#endif
