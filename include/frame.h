#ifndef _CQ_FRAME_H_
#define _CQ_FRAME_H_

#ifdef __linux__
#include <unistd.h>
#include <dirent.h>
#endif
#ifdef _WIN32
#include <direct.h>
#include <io.h>
#endif

#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include "ceres/ceres.h"
#include "ceres/rotation.h"

using std::vector;
using std::string;
using Eigen::Vector3d;
using Eigen::Vector2d;
using cv::Point2d;
using cv::Point3d;
using cv::Mat;

namespace vslam{

//int SolvePoly(double* coe, double y, double &root);


class  Noncopyable
{
protected:
    Noncopyable() {}
    ~Noncopyable() {}
private:
    Noncopyable(const Noncopyable &);
    Noncopyable& operator =(const Noncopyable &);
}; //end of class Noncopyable


class AbstractCamera {
public:
        AbstractCamera() {}
        //AbstractCamera(int width, int height) : width_(width), height_(height) {};
	
        virtual ~AbstractCamera() {}

        virtual Vector3d Cam2World(const double &uw, const double &uh) const =0;
        virtual Vector3d Cam2World(const Vector2d &pixel) const =0;
        virtual Vector3d Cam2World(const Point2d &pixel) const =0;


        virtual Vector2d World2Cam(const double &x, const double &y, const double &z) const =0;
        virtual Vector2d World2Cam(const Vector3d &xyz) const =0;

        template <typename T>
        void World2Cam(const T p_w[3], T p_c[2]) const {std::cout << "W2C from AbstractCamera Error!"<<std::endl;}

        template <typename T>
        void Cam2World(const T p_c[2], T p_w[3]) const {std::cout << "C2W from AbstractCamera Error!"<<std::endl;}

	inline int width() const { return width_; }
	inline int height() const { return height_; }

        virtual bool SetCamParaOrDie(string filename) =0;

        virtual double* para_ptr() =0;

	int width_;
	int height_;
};

//class PinholeCamera : public AbstractCamera {
//public:
//    PinholeCamera() {};

//    PinholeCamera(double width, double height,
//				  double fx, double fy, double cx, double cy,
//				  double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);
//    ~PinholeCamera() {};

//    virtual bool SetCamParaOrDie(string filename);
//    virtual Vector3d Cam2World(const double &uw, const double &uh) const;
//    virtual Vector3d Cam2World(const Vector2d &pixel) const;
//    virtual Vector3d Cam2World(const Point2d &pixel) const;
//    virtual Vector2d World2Cam(const double &x, const double &y, const double &z) const;
//    virtual Vector2d World2Cam(const Vector3d &xyz) const;

//    inline double fx() const { return fx_; };
//    inline double fy() const { return fy_; };
//    inline double cx() const { return cx_; };
//    inline double cy() const { return cy_; };

//private:
//    double fx_, fy_, cx_, cy_, d_[5];
//    bool is_distorted;
//    Mat K_cv_, D_cv_;
//    Mat map_[2];
//};


struct PolyError {
   PolyError(double* coe, double y): coe_(coe), y_(y) {}

    template <typename T>
    bool operator()(const T* const x, T* residual) const {
        //residual[0] = T(coe_[0]) - x[0];
        residual[0] = T(coe_[0]) * x[0] + T(coe_[1]) * pow(x[0], 3) + T(coe_[2]) * pow(x[0], 5) + T(coe_[3]) * pow(x[0], 7) -  T(y_);
        return true;
    }

    double  y_;
    double* coe_;
};

template <typename T>
int SolvePoly(T* coe, T y, T &root) {

    ceres::Problem problem;

    ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<PolyError, 1, 1>(new PolyError(coe, y));
    problem.AddResidualBlock(cost_function, NULL, &root);
    //problem.SetParameterLowerBound(&x, 0, 0.0);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //std::cout << summary.BriefReport() << "\n";
    //std::cout << " root : " << 0 << " -> " << root << "\n";
    return 0;
}

template<int N_ORDER>
class GenericFisheyeCamera : public AbstractCamera {
public:
    GenericFisheyeCamera() : AbstractCamera(), coe_num_(2 + N_ORDER), poly_order_(N_ORDER) {}
    ~GenericFisheyeCamera() {}

    bool     SetCamParaOrDie(string filename) {}
    double*  para_ptr() {return para_;}

    Vector3d Cam2World(const double &uw, const double &uh) const {}
    Vector3d Cam2World(const Vector2d &pixel) const {}
    Vector3d Cam2World(const Point2d &pixel) const {}

    Vector2d World2Cam(const double &x, const double &y, const double &z) const {}
    Vector2d World2Cam(const Vector3d &xyz) const {}

    template <typename T>
    void World2Cam(const T p_w[3], T p_c[2]) const {
        T len = sqrt(p_w[0]*p_w[0] + p_w[1]*p_w[1] + p_w[2]*p_w[2]);
        T theta = acos(p_w[2] / len);

        //para_[2, 3, 4, 5] are the 4-order coefficent for fisheye
        T radius = T(0.0);
        for (int i = 0; i < N_ORDER; i++) {
            //radius += T(para_[2 + i]) * pow(theta, 2 * i + 1);
            radius += T(para_[2 + i]) * pow(theta, i + 1);
        }

        //para_[0, 1] are the principal point
        T r_xy = sqrt(p_w[0]*p_w[0] + p_w[1]*p_w[1]);

        p_c[0] = radius * p_w[0] / r_xy + T(para_[0]);
        p_c[1] = radius * p_w[1] / r_xy + T(para_[1]);

    }

    template <typename T>
    void World2Cam(const T* para, const T p_w[3], T p_c[2]) const {
        T len = sqrt(p_w[0]*p_w[0] + p_w[1]*p_w[1] + p_w[2]*p_w[2]);
        T theta = acos(p_w[2] / len);

        //para_[2, 3, 4, 5] are the 4-order coefficent for fisheye
        T radius = T(0.0);
        for (int i = 0; i < N_ORDER; i++) {
            //radius += para[2 + i] * pow(theta, 2 * i + 1);
            radius += para[2 + i] * pow(theta, i + 1);
        }

        //para_[0, 1] are the principal point
        T r_xy = sqrt(p_w[0]*p_w[0] + p_w[1]*p_w[1]);

        p_c[0] = radius * p_w[0] / r_xy + para[0];
        p_c[1] = radius * p_w[1] / r_xy + para[1];

    }

    void Cam2World(const double p_c[2], double p_w[3]) const {

        double px_c = p_c[0] - para_[0];
        double py_c = p_c[1] - para_[1];
        double radius = sqrt(px_c*px_c + py_c*py_c);
        double theta = 0.0;

        double coe[N_ORDER];
        for (int i = 0; i < N_ORDER; ++i) {
            coe[i] = para_[2 + i];
        }
//        std::cout << coe[0] << " " << coe[1] << " " << coe[2] << std::endl;
//        std::cout << "radius: " << radius<< std::endl;

//        coe[0] = 268.2;
//        coe[1] = 0;
//        coe[2] = 0;
//        coe[3] = 0;
//        radius = 510;
        SolvePoly(coe, radius, theta);

        p_w[0] = px_c / radius * sin(theta);
        p_w[1] = py_c / radius * cos(theta);
        p_w[2] = cos(theta);
    }

    const int   coe_num_;
    const int   poly_order_;
    double      para_[2 + N_ORDER];
};

class StereoPinholeCamera : public AbstractCamera {
public:
    StereoPinholeCamera() {}
    ~StereoPinholeCamera() {}

};

template <class CameraType>
class AbstractFrame {
    typedef std::shared_ptr<AbstractFrame>  FramePtr;
public:
    AbstractFrame() {}
    AbstractFrame(CameraType* cam_ptr, const Mat& img, double timestamp) {
        cam_ptr_ = cam_ptr;
        img_ = img;
        timestamp_ =timestamp;
        id_ = frame_counter_;
        frame_counter_++;

    }
    ~AbstractFrame() {}

    void InitFrame(const Mat& img);
public:
    static int      frame_counter_;
    int             id_;

    double          timestamp_;
    CameraType*     cam_ptr_;
    Mat             img_;

};

//class FeatureFrame : public AbstractFrame {
//public:
//    FeatureFrame(AbstractCamera* cam_ptr, const Mat& img, double timestamp) {}
//private:

//};

//class DirectFrame : public AbstractFrame {
//public:
//    DirectFrame(AbstractCamera* cam_ptr, const Mat& img, double timestamp) {}
//private:

//};

template <class CameraType>
class CalibrationBoardFrame : public AbstractFrame<CameraType> {

    enum PatternType{CHESSBOARD=0, CIRCLE, ASYMMTRICCIRCLE, WHITECIRCLE};

public:
    CalibrationBoardFrame(CameraType* cam_ptr, const Mat& img, double timestamp): AbstractFrame<CameraType>(cam_ptr, img, timestamp) {}
    CalibrationBoardFrame(CameraType* cam_ptr, const Mat& img, double timestamp, int w, int h, int len): AbstractFrame<CameraType>(cam_ptr, img, timestamp) {
        pattern_width_ = w;
        pattern_height_= h;

        omega_t_[0] = 0;
        omega_t_[1] = 0;
        omega_t_[2] = 0.1;
        omega_t_[3] = 0.5;
        omega_t_[4] = 0.5;
        omega_t_[5] = 0.5;

        reprojection_error_ = 0;
    }

    ~CalibrationBoardFrame() {}

    bool DetectCorners(cv::Mat img, cv::Size board_size, int pattern_type, vector<cv::Point2f>& corners) {
        bool found = false;
        Mat tmp;
        tmp = img;
        switch(pattern_type) {
        case 0:
            found = findChessboardCorners(tmp, board_size, corners);
            break;
        case 1:
            found = findCirclesGrid(tmp, board_size, corners, cv::CALIB_CB_SYMMETRIC_GRID);
            break;
        case 2:
            found = findCirclesGrid(tmp, board_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
            break;
        case 3:
            found = findCirclesGrid(255 - tmp, board_size, corners, cv::CALIB_CB_SYMMETRIC_GRID);
            break;
        case 4:
            found = findCirclesGrid(255 - tmp, board_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
            break;
        }
        return found;
    }

    static void ComputePatternPoints3D(int pattern_width, int pattern_height, double len) {
        pattern_point3d_ = new double*[pattern_width * pattern_height];
        for (int j = 0; j < pattern_height; j++) {
            for (int k = 0; k < pattern_width; k++) {
                int idx = j * pattern_width + k;
                pattern_point3d_[idx] = new double [3];
                pattern_point3d_[idx][0] = k *len;
                pattern_point3d_[idx][1] = j *len;
                pattern_point3d_[idx][2] = 0;

                //std::cout<< idx<<" " <<pattern_point3d_[idx][0] <<" " <<pattern_point3d_[idx][1] <<" " <<pattern_point3d_[idx][2] <<std::endl;
            }
        }
    }

    static void Delete(int pattern_width, int pattern_height) {
        for (int j = 0; j < pattern_height; j++) {
            for (int k = 0; k < pattern_width; k++) {
                int idx = j * pattern_width + k;
                //std::cout<< idx<<" " <<pattern_point3d_[idx][0] <<" " <<pattern_point3d_[idx][1] <<" " <<pattern_point3d_[idx][2] <<std::endl;
                delete [] pattern_point3d_[idx];
            }
        }
        delete [] pattern_point3d_;
    }

    double              omega_t_[6];

    int                 pattern_width_;
    int                 pattern_height_;
    vector<cv::Point2f> features;
    double              reprojection_error_;
    string              img_name_;
    static double**     pattern_point3d_;

    Vector3d            t_;
    Vector3d            omega_;
private:

};


} //end of namespace vslam

#endif
