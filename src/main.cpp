#include <thread>
#include <iostream>
#include "frame.h"
#include "epipolar_error.h"
#include "generic_fisheye_reprojection_error.h"
#include <opencv2/highgui/highgui.hpp>

using std::vector;
using ceres::fisheye::GenericFisheyeReprojectionError;
using vslam::CalibrationBoardFrame;
using vslam::GenericFisheyeCamera;
using vslam::AbstractFrame;
using vslam::AbstractCamera;

template <class CameraType>
double** CalibrationBoardFrame<CameraType>::pattern_point3d_ = NULL;

template <class CameraType>
int AbstractFrame<CameraType>::frame_counter_ = 0;

extern int QtDisplayThread();

namespace ceres {

template <class CameraType>
class CameraCalibration {
    enum PatternType{CHESSBOARD=0, CIRCLE, ASYMMTRICCIRCLE, WHITECIRCLE};
public:

    CameraCalibration() {
        cam_ptr_ = new CameraType();
        pattern_type_ = CHESSBOARD;
        grid_len_ = 0.25;
        board_size_.width = 10;
        board_size_.height = 14;
        CalibrationBoardFrame<CameraType>::ComputePatternPoints3D(board_size_.width, board_size_.height, grid_len_);
    }

    ~CameraCalibration() {
        int sz = frame_ptrs_.size();
        std::cout << "Active frames: " << sz            <<std::endl;
        for (int i = 0; i < sz; ++i) {
            delete frame_ptrs_[i];
        }
        CalibrationBoardFrame<CameraType>::Delete(board_size_.width, board_size_.height);
        delete cam_ptr_;
    }

    vector<string> GetFilesFromDir(string dir_name) {
        vector<string> files;
#ifdef __linux__
        DIR *dir;
        struct dirent *ptr;

        if ((dir=opendir(dir_name.c_str())) == NULL) {
            perror("Open dir error...");
            exit(1);
        }

        while ((ptr=readdir(dir)) != NULL) {
            if(strcmp(ptr->d_name,".") == 0 || strcmp(ptr->d_name,"..") == 0)    //current dir OR parrent dir
                continue;
            else if(ptr->d_type == 8)    //file
                files.push_back(ptr->d_name);
            else if(ptr->d_type == 10)   //link file
                continue;
            else if(ptr->d_type == 4)    //dir
                continue;
        }
        closedir(dir);
#endif
        sort(files.begin(), files.end());
        return files;
    }

    void GetImagesFromDir(string dir_name) {
        file_dir_ = dir_name;
        vector<string> image_names = GetFilesFromDir(dir_name);
        int sz = image_names.size();

        for (int i = 0 ; i < sz; ++i) {
            Mat img = cv::imread(dir_name + image_names[i], 0);
            std::cout << dir_name + image_names[i] /*<<std::endl*/;
            if (img.empty()) continue;
//            cv::imshow("img", img);
//            cv::waitKey(0);

            CalibrationBoardFrame<CameraType>* frame_ptr = new CalibrationBoardFrame<CameraType>(cam_ptr_, img, 0, board_size_.width, board_size_.height, grid_len_);
            frame_ptr->img_name_ = image_names[i];
            if(i == 0) {
                frame_ptr->cam_ptr_->para_ptr()[0] = img.cols / 2.0;
                frame_ptr->cam_ptr_->para_ptr()[1] = img.rows / 2.0;
                frame_ptr->cam_ptr_->para_ptr()[2] = img.rows / 2.0;
            }

            bool ok = frame_ptr->DetectCorners(frame_ptr->img_, board_size_, pattern_type_, frame_ptr->features);
            if (ok) {
                //cv::drawChessboardCorners(frame_ptr->img_, board_size, frame_ptr->features, 1);
                //cv::circle(frame_ptr->img_, frame_ptr->features[j++], 3, cv::Scalar(0), 4);
                //cv::imshow("img", frame_ptr->img_);
                //cv::waitKey(1);
                AddFrames(frame_ptr);
                std::cout << " ok!" <<std::endl;
            } else {
                std::cout << " not ok" <<std::endl;

            }
        }
    }

    void AddFrames(CalibrationBoardFrame<CameraType> *frame_ptr) {
        int sz = frame_ptr->features.size();
        double p_c[2];
        double p_w[3];

//        p_c[0] = 500;
//        p_c[1] = 500;
//        frame_ptr->cam_ptr_->Cam2World(p_c, p_w);
//        std::cout << p_w[0] << " " << p_w[1] << " " << p_w[2] << std::endl;
        for (int i = 0 ; i < sz; ++i) {
            p_c[0] = frame_ptr->features[i].x;
            p_c[1] = frame_ptr->features[i].y;
//            double px_c = p_c[0] - cam_ptr_->para_ptr()[0];
//            double py_c = p_c[1] - cam_ptr_->para_ptr()[1];
//            double radius = sqrt(px_c*px_c + py_c*py_c);

            //frame_ptr->cam_ptr_->Cam2World(p_c, p_w);
        }
        frame_ptrs_.push_back(frame_ptr);
    }

    void Calibrate(vector<CalibrationBoardFrame<CameraType>*> &frame_ptrs) {

        ceres::Problem problem;
        int sz = frame_ptrs.size();

        for (int i = 0; i < sz; i++) {
            for (int j = 0; j < frame_ptrs[i]->pattern_height_; j++) {
                for (int k = 0; k < frame_ptrs[i]->pattern_width_; k++) {

                    int idx = j*frame_ptrs[i]->pattern_width_ + k;

                    double observed_x = frame_ptrs[i]->features[idx].x;
                    double observed_y = frame_ptrs[i]->features[idx].y;

                    ceres::CostFunction* cost_function = GenericFisheyeReprojectionError<CameraType>::Create(observed_x, observed_y);
                    problem.AddResidualBlock(cost_function,
                                             NULL,
                                             frame_ptrs[i]->cam_ptr_->para_ptr(),
                                             frame_ptrs[i]->omega_t_,
                                             CalibrationBoardFrame<CameraType>::pattern_point3d_[idx]);

                    problem.SetParameterBlockConstant(CalibrationBoardFrame<CameraType>::pattern_point3d_[idx]);
                    //problem.SetParameterLowerBound(&x, 0, 0.0);
                }
            }
        }

        ceres::Solver::Options options;
        options.use_nonmonotonic_steps = true;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.use_inner_iterations = true;
        options.max_num_iterations = 1000;
        options.minimizer_progress_to_stdout = true;
//      ceres::Solver::Options options;
//      options.linear_solver_type = ceres::DENSE_QR;;
//      options.max_num_iterations = 1000;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << "Final report:\n" << summary.FullReport();

        std::cout << "coef: [ " ;
        for (int j = 0 ; j < cam_ptr_->poly_order_; ++j) {
            std::cout<< cam_ptr_->para_ptr()[2+j] << " " ;
        }
        std::cout << " ]" << std::endl;

        std::cout << "center(h&w): [ " << cam_ptr_->para_ptr()[1] << " " << cam_ptr_->para_ptr()[0] << " ]" <<std::endl;
        //std::cout << "point: " << CalibrationBoardFrame<CameraType>::pattern_point3d_[3][0] << " " << CalibrationBoardFrame<CameraType>::pattern_point3d_[3][1] << " " << CalibrationBoardFrame<CameraType>::pattern_point3d_[3][2] <<std::endl;
    }

    void Reproject(vector<CalibrationBoardFrame<CameraType>*> &frame_ptrs) {
        int sz = frame_ptrs.size();
        double error = 0;

        for (int i = 0; i < sz; i++) {

            cv::cvtColor(frame_ptrs[i]->img_, frame_ptrs[i]->img_, cv::COLOR_GRAY2BGR);
            for (int j = 0; j < frame_ptrs[i]->pattern_height_; j++) {
                for (int k = 0; k < frame_ptrs[i]->pattern_width_; k++) {

                    int idx = j*frame_ptrs[i]->pattern_width_ + k;

                    double observed_x = frame_ptrs[i]->features[idx].x;
                    double observed_y = frame_ptrs[i]->features[idx].y;

                    double p[3];
                    ceres::AngleAxisRotatePoint(frame_ptrs[i]->omega_t_, CalibrationBoardFrame<CameraType>::pattern_point3d_[idx], p);
                    // omega_t[3,4,5] are the translation.
                    p[0] += frame_ptrs[i]->omega_t_[3];
                    p[1] += frame_ptrs[i]->omega_t_[4];
                    p[2] += frame_ptrs[i]->omega_t_[5];

                    double pixel[2];
                    frame_ptrs[i]->cam_ptr_->World2Cam(p, pixel);
                    cv::circle(frame_ptrs[i]->img_, cv::Point(observed_x, observed_y), 3, cv::Scalar(0, 255, 0), 1);
                    cv::circle(frame_ptrs[i]->img_, cv::Point(pixel[0], pixel[1]), 3, cv::Scalar(0, 0, 255), 1);

                    double err = (observed_x - pixel[0])*(observed_x - pixel[0]) + (observed_y - pixel[1])*(observed_y - pixel[1]);
                    frame_ptrs[i]->reprojection_error_ += err;
                }
            }

            frame_ptrs[i]->reprojection_error_ /= (frame_ptrs[i]->pattern_width_ * frame_ptrs[i]->pattern_height_);
            error += frame_ptrs[i]->reprojection_error_;

            char text[100];
            sprintf(text, "err_%1.3f_", frame_ptrs[i]->reprojection_error_);
            string err_str(text);

            //std::cout << frame_ptrs[i]->img_name_ + " err: " <<  frame_ptrs[i]->reprojection_error_ <<std::endl;

            cv::putText(frame_ptrs[i]->img_, err_str, cv::Point(10, 30), 3, 0.8, cv::Scalar(0, 0, 255), 1, 8, false);
            cv::imwrite(file_dir_ + "error/"+ err_str+frame_ptrs[i]->img_name_, frame_ptrs[i]->img_);

            cv::imshow("img", frame_ptrs[i]->img_);
            cv::waitKey(1);
        }
        error /= sz;
        std::cout <<"total err: " <<  error  <<std::endl;
    }

    double grid_len_;
    int pattern_type_;
    cv::Size board_size_;
    vector<CalibrationBoardFrame<CameraType>*> frame_ptrs_;
    CameraType* cam_ptr_;

    string file_dir_;
};
} //ceres_calibration



string dir_name = "/home/qichen/git_ws/camera_calibration/data/";
int MainThread() {
    ceres::CameraCalibration<GenericFisheyeCamera<3> > test;
    test.GetImagesFromDir(dir_name);
    test.Calibrate(test.frame_ptrs_);
    test.Reproject(test.frame_ptrs_);
	return 0;
}

int  main() {
	std::thread t1(MainThread);
    //std::thread t2(QtDisplayThread);

	t1.join();
    //t2.join();
	return 1;
}
