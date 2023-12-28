/**
 * @file rs_curve.cpp
 * @author Yongmeng He
 * @brief 
 * @version 0.1
 * @date 2023-12-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "rs_curve/rs_curve.hpp"

namespace rs_curve
{
point_type RSCurve::GetDiffVecPoint(
        const point_type& pos1, const point_type& pos2) {
    
    return std::make_pair(std::make_pair(pos2.first.first - pos1.first.first, 
            pos2.first.second - pos1.first.second), pos2.second - pos1.second);
}

double RSCurve::GetDistance(const point_type& pos1, const point_type& pos2) {
    
    return sqrt(pow(pos2.first.first - pos1.first.first, 2) + 
            pow(pos2.first.second - pos1.first.second, 2));
}

void RSCurve::Polar(const double& x, const double& y, double& r, double& theta) {
    r = std::sqrt(x * x + y * y);
    theta = std::atan2(y, x);
}

double RSCurve::Mod2Pi(const double& theta) {
    double v = fmod(theta, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } 
    else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

void RSCurve::TauOmega(double u, double v, double xi, double eta, 
        double phi, double &tau, double &omega) {
    
    double delta = Mod2Pi(u - v);
    double A = sin(u) - sin(delta);
    double B = cos(u) - cos(delta) - 1.0;
    double t1 = atan2(eta * A - xi * B, xi * A + eta * B);
    double t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;
    tau = (t2 < 0.0) ? Mod2Pi(t1 + M_PI) : Mod2Pi(t1);
    omega = Mod2Pi(tau - u + v - phi);
}


bool RSCurve::LpSpLp(const double& x, const double& y, const double& phi,
        double& t, double& u, double& v, const std::string& type) {
    
    Polar(x - sin(phi), y - 1. + cos(phi), u, t);

    if (t >= 0.0) {
        v = Mod2Pi(phi - t);
        if (v >= 0) {
            return true;
            // ROS_INFO_STREAM("t, u, v:" << t << ", " << u << ", " << v);
        }
    }

    ROS_INFO_STREAM("fail to find a LpSpLp curve, type is:" << type);
    return false;
}

bool RSCurve::LpSpRp(const double& x, const double& y, const double& phi,
        double& t, double& u, double& v, const std::string& type) {
    
    Polar(x + sin(phi), y - 1. - cos(phi), u, t);
    u = pow(u, 2);

    if (u < 4.0) {
        ROS_INFO_STREAM("fail to find a LpSpRp curve, type is:" << type);
        return false;
    }

    u = sqrt(u - 4.0);
    t = Mod2Pi(t + atan2(2.0, u));
    v = Mod2Pi(t - phi);
    // ROS_INFO_STREAM("t, u, v:" << t << ", " << u << ", " << v);
    return true;
}

bool RSCurve::LpRmL(const double& x, const double& y, const double& phi,
        double& t, double& u, double& v, const std::string& type) {
    
    Polar(x - sin(phi), y - 1. + cos(phi), u, t);

    // ROS_INFO_STREAM("t, u, v:" << t << ", " << u << ", " << v);

    if (u > 4.0) {
        ROS_INFO_STREAM("fail to find a LpRmL curve, type is:" << type);
        return false;
    }

    u = -2.0 * asin(0.25 * u);
    t = Mod2Pi(t + 0.5 * u + M_PI);
    v = Mod2Pi(phi - t + u);

    return true;
}

bool RSCurve::LpRupLumRm(const double& x, const double& y, const double& phi,
        double& t, double& u, double& v, const std::string& type) {
    
    double xi = x + sin(phi);
    double eta = y - 1. - cos(phi);
    double rho = 0.25 * (2.0 + sqrt(xi * xi + eta * eta));

    if (rho > 1.0) {
        ROS_INFO_STREAM("fail to find a LpRupLumRm curve, type is:" << type);
        return false;
    }

    u = acos(rho);
    TauOmega(u, -u, xi, eta, phi, t, v);

    return true;
}

bool RSCurve::LpRupLumRp(const double& x, const double& y, const double& phi,
        double& t, double& u, double& v, const std::string& type) {
    
    double xi = x + sin(phi);
    double eta = y - 1. - cos(phi);
    double rho = (20.0 - xi * xi - eta * eta) / 16.0;

    if (rho >= 0.0 && rho <= 1.0) {
        u = -acos(rho);
        if (u >= -M_PI_2) {
            TauOmega(u, u, xi, eta, phi, t, v);
            return true;
        }
    }

    ROS_INFO_STREAM("fail to find a LpRupLumRp curve, type is:" << type);
    return false;
}

bool RSCurve::LpRmSmLm(const double& x, const double& y, const double& phi,
        double& t, double& u, double& v, const std::string& type) {

    double rho, theta;
    Polar(x - sin(phi), y - 1. + cos(phi), rho, theta);

    if (rho < 2.0) {
        ROS_INFO_STREAM("fail to find a LpRmSmLm curve, type is:" << type);
        return false;
    }

    double r = sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = Mod2Pi(theta + atan2(r, -2.0));
    v = Mod2Pi(phi - M_PI_2 - t);

    return true;
}

bool RSCurve::LpRmSmRm(const double& x, const double& y, const double& phi,
        double& t, double& u, double& v, const std::string& type) {

    double rho, theta;
    Polar(-(y - 1. - cos(phi)), x + sin(phi), rho, theta);

    if (rho < 2.0) {
        ROS_INFO_STREAM("fail to find a LpRmSmRm curve, type is:" << type);
        return false;
    }

    t = theta;
    u = 2.0 - rho;
    v = Mod2Pi(t + M_PI_2 - phi);

    return true;
}

bool RSCurve::LpRmSLmRp(const double& x, const double& y, const double& phi,
        double& t, double& u, double& v, const std::string& type) {

    double xi = x + sin(phi);
    double eta = y - 1. - cos(phi);
    double rho, theta;
    Polar(x + sin(phi), y - 1. - cos(phi), rho, theta);

    if (rho >= 2.0) {
        u = 4.0 - sqrt(rho * rho - 4.0);

        if (u <= 0.0) {
            t = Mod2Pi(atan2(
                    (4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            v = Mod2Pi(t - phi);
            return true;
        }
    }

    ROS_INFO_STREAM("fail to find a LpRmSLmRp curve, type is:" << type);
    return false;
}

curve_type RSCurve::CreateCurve(
        const RSCurveType* type, const std::string& name, 
        const std::array<double, 5>& angle_array, const double& length) {
    curve_type rs_curve;
    rs_curve.first.first = type;
    rs_curve.first.second = name;
    rs_curve.second.first = angle_array;
    rs_curve.second.second = length;

    return rs_curve;
}

curve_type RSCurve::CreateNoneCurve(
        const RSCurveType* type, const std::string& name) {
    curve_type none_curve;
    none_curve.first.first = type;
    none_curve.first.second = name;
    none_curve.second.first = {0, 0, 0};
    none_curve.second.second = INFINITY;

    return none_curve;
}

std::vector<curve_type> RSCurve::CSC(
        const double& x, const double& y, const double& phi) {
    
    double t, u, v;
    double length;

    curve_type csc_curve;
    std::vector<curve_type> csc_result;

    if (LpSpLp(x, y, phi, t, u, v, "left+, stright+, left+") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        csc_result.push_back(CreateCurve(
                &rs_curve_type_[14][0], "LpSpLp", {t, u, v}, length));
    } 
    else {
        csc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[14][0], "LpSpLp"));
    }

    if (LpSpLp(-x, y, -phi, t, u, v, "left-, stright-, left-") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        csc_result.push_back(CreateCurve(
                &rs_curve_type_[14][0], "LnSnLn", {-t, -u, -v}, length));
    }
    else {
        csc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[14][0], "LnSnLn"));
    }

    if (LpSpLp(x, -y, -phi, t, u, v, "right+, stright+, right+") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        csc_result.push_back(CreateCurve(
                &rs_curve_type_[15][0], "RpSpRp", {t, u, v}, length));
    }
    else {
        csc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[15][0], "RpSpRp"));
    }

    if (LpSpLp(-x, -y, phi, t, u, v, "right-, stright-, right-") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        csc_result.push_back(CreateCurve(
                &rs_curve_type_[15][0], "RnSnRn", {-t, -u, -v}, length));
    }
    else {
        csc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[15][0], "RnSnRn"));
    }
   
    if (LpSpRp(x, y, phi, t, u, v, "left+, stright+, right+") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        csc_result.push_back(CreateCurve(
                &rs_curve_type_[12][0], "LpSpRp", {t, u, v}, length));
    }
    else {
        csc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[12][0], "LpSpRp"));
    }

    if (LpSpRp(-x, y, -phi, t, u, v, "left-, stright-, right-") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        csc_result.push_back(CreateCurve(
                &rs_curve_type_[12][0], "LnSnRn", {-t, -u, -v}, length));
    }
    else {
        csc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[12][0], "LnSnRn"));
    }

    if (LpSpRp(x, -y, -phi, t, u, v, "right+, stright+, left+") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        csc_result.push_back(CreateCurve(
                &rs_curve_type_[13][0], "RpSpLp", {t, u, v}, length));
    }
    else {
        csc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[13][0], "RpSpLp"));
    }

    if (LpSpRp(-x, -y, phi, t, u, v, "right-, stright-, left-") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        csc_result.push_back(CreateCurve(
                &rs_curve_type_[13][0], "RnSnLn", {-t, -u, -v}, length));
    }
    else {
        csc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[13][0], "RnSnLn"));
    }

    return csc_result;
}

std::vector<curve_type> RSCurve::CCC(
        const double& x, const double& y, const double& phi) {
    
    double t, u, v;
    double length;

    curve_type ccc_curve;
    std::vector<curve_type> ccc_result;

    if (LpRmL(x, y, phi, t, u, v, "left+, right-, left+") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        ccc_result.push_back(CreateCurve(
                &rs_curve_type_[0][0], "LpRnLp", {t, u, v}, length));
    }
    else {
        ccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[0][0], "LpRnLp"));
    }

    if (LpRmL(-x, y, -phi, t, u, v, "left-, right+, left-") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        ccc_result.push_back(CreateCurve(
                &rs_curve_type_[0][0], "LnRpLn", {-t, -u, -v}, length));
    }
    else {
        ccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[0][0], "LnRpLn"));
    }

    if (LpRmL(x, -y, -phi, t, u, v, "right+, left-, right+") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        ccc_result.push_back(CreateCurve(
                &rs_curve_type_[1][0], "RpLnRp", {t, u, v}, length));
    }
    else {
        ccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[1][0], "RpLnRp"));
    }
    
    if (LpRmL(-x, -y, phi, t, u, v, "right-, left+, right-") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        ccc_result.push_back(CreateCurve(
                &rs_curve_type_[1][0], "RnLpRn", {-t, -u, -v}, length));
    }
    else {
        ccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[1][0], "RnLpRn"));
    }
    
    double x_new = x * cos(phi) + y * sin(phi);
    double y_new = -1 * (-x * sin(phi) + y * cos(phi));
    if (LpRmL(x_new, y_new, phi, t, u, v, "left+, right-, left-") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        ccc_result.push_back(CreateCurve(
                &rs_curve_type_[0][0], "LpRnLn", {v, u, t}, length));
    }
    else {
        ccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[0][0], "LpRnLn"));
    }
    
    if (LpRmL(-x_new, y_new, -phi, t, u, v, "left-, right+, left+") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        ccc_result.push_back(CreateCurve(
                &rs_curve_type_[0][0], "LnRpLp", {-v, -u, -t}, length));
    }
    else {
        ccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[0][0], "LnRpLp"));
    }
    
    if (LpRmL(x_new, -y_new, -phi, t, u, v, "right+, left-, right-") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        ccc_result.push_back(CreateCurve(
                &rs_curve_type_[1][0], "RpLnRn", {v, u, t}, length));
    }
    else {
        ccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[1][0], "RpLnRn"));
    }
    
    if (LpRmL(-x_new, -y_new, phi, t, u, v, "right-, left+, right+") && 
            !std::isinf(length = fabs(t) + fabs(u) + fabs(v))) {
        ccc_result.push_back(CreateCurve(
                &rs_curve_type_[1][0], "RnLpRp", {-v, -u, -t}, length));
    }
    else {
        ccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[1][0], "RnLpRp"));
    }
    
    return ccc_result;
}

std::vector<curve_type> RSCurve::CCCC(
        const double& x, const double& y, const double& phi) {
    
    double t, u, v;
    double length;

    curve_type cccc_curve;
    std::vector<curve_type> cccc_result;

    if (LpRupLumRm(x, y, phi, t, u, v, "left+, right+, left-, right-") && 
            !std::isinf(length = fabs(t) + 2 * fabs(u) + fabs(v))) {
        cccc_result.push_back(CreateCurve(
                &rs_curve_type_[14][0], "LnSnLn", {-t, -u, -v}, length));
    }
    else {
        cccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[14][0], "LnSnLn"));
    }
    
    if (LpRupLumRm(-x, y, -phi, t, u, v, "left-, right-, left+, right+") && 
            !std::isinf(length = fabs(t) + 2 * fabs(u) + fabs(v))) {
        cccc_result.push_back(CreateCurve(
                &rs_curve_type_[14][0], "LnSnLn", {-t, -u, -v}, length));
    }
    else {
        cccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[14][0], "LnSnLn"));
    }

    if (LpRupLumRm(x, -y, -phi, t, u, v, "right+, left+, right-, left-") && 
            !std::isinf(length = fabs(t) + 2 * fabs(u) + fabs(v))) {
        cccc_result.push_back(CreateCurve(
                &rs_curve_type_[3][0], "RpLpRnLn", {t, u, -u, v}, length));
    }
    else {
        cccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[3][0], "RpLpRnLn"));
    }
    
    if (LpRupLumRm(-x, -y, phi, t, u, v, "right-, left-, right+, left+") && 
            !std::isinf(length = fabs(t) + 2 * fabs(u) + fabs(v))) {
        cccc_result.push_back(CreateCurve(
                &rs_curve_type_[3][0], "RnLnRpLp", {-t, -u, u, -v}, length));
    }
    else {
        cccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[3][0], "RnLnRpLp"));
    }
    
    if (LpRupLumRp(x, y, phi, t, u, v, "left+, right-, left-, right+") && 
            !std::isinf(length = fabs(t) + 2 * fabs(u) + fabs(v))) {
        cccc_result.push_back(CreateCurve(
                &rs_curve_type_[2][0], "LpRnLnRp", {t, u, u, v}, length));
    } 
    else {
        cccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[2][0], "LpRnLnRp"));
    }
    
    if (LpRupLumRp(-x, y, -phi, t, u, v, "left-, right+, left+, right-") && 
            !std::isinf(length = fabs(t) + 2 * fabs(u) + fabs(v))) {
        cccc_result.push_back(CreateCurve(
                &rs_curve_type_[2][0], "LnRpLpRn", {-t, -u, -u, -v}, length));
    }       
    else {
        cccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[2][0], "LnRpLpRn"));
    }
    
    if (LpRupLumRp(x, -y, -phi, t, u, v, "right+, left-, right-, left+") && 
            !std::isinf(length = fabs(t) + 2 * fabs(u) + fabs(v))) {
        cccc_result.push_back(CreateCurve(
                &rs_curve_type_[3][0], "RpLnRnLp", {t, u, u, v}, length));
    }    
    else {
        cccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[3][0], "RpLnRnLp"));
    }
    
    if (LpRupLumRp(-x, -y, phi, t, u, v, "right-, left+, right+, left-") && 
            !std::isinf(length = fabs(t) + 2 * fabs(u) + fabs(v))) {
        cccc_result.push_back(CreateCurve(
                &rs_curve_type_[3][0], "RnLpRpLn", {-t, -u, -u, -v}, length));
    }    
    else {
        cccc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[3][0], "RnLpRpLn"));
    }
    
    return cccc_result;
}

std::vector<curve_type> RSCurve::CCSC(
        const double& x, const double& y, const double& phi) {
    
    double t, u, v;
    double length;

    curve_type ccsc_curve;
    std::vector<curve_type> ccsc_result;

    if (LpRmSmLm(x, y, phi, t, u, v, "left+, right-, stright-, left-") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[4][0], "LpRnSnLn", {t, -M_PI_2, u, v}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[4][0], "LpRnSnLn"));
    }
    
    if (LpRmSmLm(-x, y, -phi, t, u, v, "left-, right+, stright+, left+") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[4][0], "LnRpSpLp", {-t, M_PI_2, -u, -v}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[4][0], "LnRpSpLp"));
    }

    if (LpRmSmLm(x, -y, -phi, t, u, v, "right+, left-, stright-, right-") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[5][0], "RpLnSnRn", {t, -M_PI_2, u, v}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[5][0], "RpLnSnRn"));
    }
    
    if (LpRmSmLm(-x, -y, phi, t, u, v, "right-, left+, stright+, right+") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[5][0], "RnLpSpRp", {-t, M_PI_2, -u, -v}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[5][0], "RnLpSpRp"));
    }
    
    if (LpRmSmRm(x, y, phi, t, u, v, "left+, right-, stright-, right-") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[8][0], "LpRnSnRn", {t, -M_PI_2, u, v}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[8][0], "LpRnSnRn"));
    }
    
    if (LpRmSmRm(-x, y, -phi, t, u, v, "left-, right+, stright+, right+") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[8][0], "LnRpSpRp", {-t, M_PI_2, -u, -v}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[8][0], "LnRpSpRp"));
    }
    
    if (LpRmSmRm(x, -y, -phi, t, u, v, "right+, left-, stright-, left-") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[9][0], "RpLnSnLn", {t, -M_PI_2, u, v}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[9][0], "RpLnSnLn"));
    }
    
    if (LpRmSmRm(-x, -y, phi, t, u, v, "right-, left+, stright+, left+") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[9][0], "RnLpSpLp", {-t, M_PI_2, -u, -v}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[9][0], "RnLpSpLp"));
    }
    
    double x_new = x * cos(phi) + y * sin(phi);
    double y_new = x * sin(phi) - y * cos(phi);
    if (LpRmSmLm(x_new, y_new, phi, t, u, v, "left+, stright-, right-, left-") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[6][0], "LpSnRnLn", {v, u, -M_PI_2, t}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[6][0], "LpSnRnLn"));
    }
    
    if (LpRmSmLm(-x_new, y_new, -phi, t, u, v, "left-, stright+, right+, left+") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[6][0], "LnSpRpLp", {-v, -u, M_PI_2, -t}, length));
    }    
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[6][0], "LnSpRpLp"));
    }
    
    if (LpRmSmLm(x_new, -y_new, -phi, t, u, v, "right+, stright-, left-, right-") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[7][0], "RpSnLnRn", {v, u, -M_PI_2, t}, length));
    }   
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[7][0], "RpSnLnRn"));
    }
    
    if (LpRmSmLm(-x_new, -y_new, phi, t, u, v, "right-, stright+, left+, right+") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[7][0], "RnSpLpRp", {-v, -u, M_PI_2, -t}, length));
    }   
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[7][0], "RnSpLpRp"));
    }
    
    if (LpRmSmRm(x_new, y_new, phi, t, u, v, "right+, stright-, left-, right-") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[10][0], "RpSnLnRn", {v, u, -M_PI_2, t}, length));
    }
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[10][0], "RpSnLnRn"));
    }
    
    if (LpRmSmRm(-x_new, y_new, -phi, t, u, v, "right-, stright+, left+, right+") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[10][0], "RnSpLpRp", {-v, -u, M_PI_2, -t}, length));
    }    
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[10][0], "RnSpLpRp"));
    }
    
    if (LpRmSmRm(x_new, -y_new, -phi, t, u, v, "left+, stright-, left-, right-") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[11][0], "LpSnLnRn", {v, u, -M_PI_2, t}, length));
    }    
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[11][0], "LpSnLnRn"));
    }
    
    if (LpRmSmRm(-x_new, -y_new, phi, t, u, v, "left-, stright+, left+, right+") && 
            !std::isinf(length = fabs(t) + M_PI_2+ fabs(u) + fabs(v))) {
        ccsc_result.push_back(CreateCurve(
                &rs_curve_type_[11][0], "LnSpLpRp", {-v, -u, M_PI_2, -t}, length));
    }    
    else {
        ccsc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[11][0], "LnSpLpRp"));
    }
    
    return ccsc_result;
}

std::vector<curve_type> RSCurve::CCSCC(
        const double& x, const double& y, const double& phi) {
    
    double t, u, v;
    double length;

    curve_type ccscc_curve;
    std::vector<curve_type> ccscc_result;
    if (LpRmSLmRp(x, y, phi, t, u, v, "left+, right-, stright-, left-, right+") && 
            !std::isinf(length = fabs(t) + 2 * M_PI_2 + fabs(u) + fabs(v))) {
        ccscc_result.push_back(CreateCurve(&rs_curve_type_[16][0], 
                "LpRnSnLnRp", {t, -M_PI_2, u, -M_PI_2, v}, length));
    }
    else {
        ccscc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[16][0], "LpRnSnLnRp"));
    }
    
    if (LpRmSLmRp(-x, y, -phi, t, u, v, "left-, right+, stright+, left+, right-") && 
            !std::isinf(length = fabs(t) + 2 * M_PI_2 + fabs(u) + fabs(v))) {
        ccscc_result.push_back(CreateCurve(&rs_curve_type_[16][0], 
                "LnRpSpLpRn", {-t, M_PI_2, -u, M_PI_2, -v}, length));
    }
    else {
        ccscc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[16][0], "LnRpSpLpRn"));
    }

    if (LpRmSLmRp(x, -y, -phi, t, u, v, "right+, left-, stright-, right-, left+") && 
            !std::isinf(length = fabs(t) + 2 * M_PI_2 + fabs(u) + fabs(v))) {
        ccscc_result.push_back(CreateCurve(&rs_curve_type_[17][0], 
                "RpLnSnRnLp", {t, -M_PI_2, u, -M_PI_2, v}, length));
    }    
    else {
        ccscc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[17][0], "RpLnSnRnLp"));
    }
    
    if (LpRmSLmRp(-x, -y, phi, t, u, v, "right-, left+, stright+, right+, left-") && 
            !std::isinf(length = fabs(t) + 2 * M_PI_2 + fabs(u) + fabs(v))) {
        ccscc_result.push_back(CreateCurve(&rs_curve_type_[17][0], 
                "RnLpSpRpLn", {-t, M_PI_2, -u, M_PI_2, -v}, length));
    }    
    else {
        ccscc_result.push_back(
                CreateNoneCurve(&rs_curve_type_[17][0], "RnLpSpRpLn"));
    }
    
    return ccscc_result;
}

std::vector<curve_type> RSCurve::GetRSCurve(const point_type& input_pos) {
    double x = input_pos.first.first;
    double y = input_pos.first.second;
    double phi = input_pos.second;

    auto csc_result = CSC(x, y, phi);
    auto ccc_result = CCC(x, y, phi);
    auto cccc_result = CCCC(x, y, phi);
    auto ccsc_result = CCSC(x, y, phi);
    auto ccscc_result = CCSCC(x, y, phi);

    std::vector<curve_type> result;
    result.insert(result.end(), csc_result.begin(), csc_result.end());
    result.insert(result.end(), ccc_result.begin(), ccc_result.end());
    result.insert(result.end(), cccc_result.begin(), cccc_result.end());
    result.insert(result.end(), ccsc_result.begin(), ccsc_result.end());
    result.insert(result.end(), ccscc_result.begin(), ccscc_result.end());

    return result;
}

std::vector<curve_type> RSCurve::GetRSCurve(
        const point_type& start_pos, const point_type& end_pos) {
    
    point_type diff_pos = GetDiffVecPoint(start_pos, end_pos); 
    double translated_x = diff_pos.first.first * cos(start_pos.second) + 
            diff_pos.first.second * sin(start_pos.second);
    double translated_y = -diff_pos.first.first * sin(start_pos.second) + 
            diff_pos.first.second * cos(start_pos.second);
    double phi = diff_pos.second;

    return GetRSCurve(std::make_pair(std::make_pair(
            translated_x / radius_, translated_y / radius_), phi));
}

std::vector<point_type> RSCurve::GetRSPoint(const curve_type& rs_curve) {

    std::vector<point_type> result;

    double curve_length = rs_curve.second.second * radius_;
    double step_size = 0.01;
    auto slice_num = static_cast<unsigned int> (curve_length / step_size);

    for (unsigned int i = 0; i <= slice_num; i++) {
        double seg = i * step_size;
        double v;

        point_type temp_point = 
                std::make_pair(std::make_pair(0, 0), start_pos_.second);
        for (unsigned int j = 0; j < 5u; j++) {
            if (rs_curve.second.first[j] < 0.0) {
                v = std::max(-seg, rs_curve.second.first[j]);
                seg += v;
            } else {
                v = std::min(seg, rs_curve.second.first[j]);
                seg -= v;    
            }

            double phi = temp_point.second;
            switch (rs_curve.first.first[j]) {
                case L:
                    temp_point.first.first += sin(phi + v) - sin(phi);
                    temp_point.first.second += -cos(phi + v) + cos(phi);
                    temp_point.second = phi + v;
                    break;
                case R:
                    temp_point.first.first += -sin(phi - v) + sin(phi);
                    temp_point.first.second += cos(phi - v) - cos(phi);
                    temp_point.second = phi - v;
                    break;
                case S:
                    temp_point.first.first += v * cos(phi);
                    temp_point.first.second += v * sin(phi);
                    temp_point.second = phi;
                    break;
                case N:
                    break;
            }
        }

        result.push_back(std::make_pair(std::make_pair(
                temp_point.first.first + start_pos_.first.first, 
                temp_point.first.second + start_pos_.first.second), 
                temp_point.second));
    }

    return result;
}
}