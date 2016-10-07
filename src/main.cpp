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

double** CalibrationBoardFrame::pattern_point3d_ = NULL;
int AbstractFrame::frame_counter_ = 0;

extern int QtDisplayThread();

namespace ceres {
class CameraCalibration {
    enum PatternType{CHESSBOARD=0, CIRCLE, ASYMMTRICCIRCLE, WHITECIRCLE};
public:

    CameraCalibration() {
        cam_ptr_ = new GenericFisheyeCamera();
        CalibrationBoardFrame::ComputePatternPoints3D(11, 9, 0.25);

    }

    ~CameraCalibration() {

        int sz = frame_ptrs_.size();
        std::cout << "Active frames: " << sz            <<std::endl;
        for (int i = 0; i < sz; ++i) {
            delete frame_ptrs_[i];
        }
        CalibrationBoardFrame::Delete(11, 9);
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
         vector<string> names = GetFilesFromDir(dir_name);
         int sz = names.size();
         for (int i = 0 ; i < sz; ++i) {
            Mat img = cv::imread(dir_name + names[i], 0);
            std::cout << dir_name + names[i] /*<<std::endl*/;
            if (img.empty()) continue;
//            cv::imshow("img", img);
//            cv::waitKey(10);
            CalibrationBoardFrame* frame_ptr = new CalibrationBoardFrame(cam_ptr_, img, 0, 11, 9, 0.25);

            static int j = 0;
            cv::Size board_size(11, 9);
            if ( frame_ptr->DetectCorners(frame_ptr->img_, board_size, WHITECIRCLE, frame_ptr->features) ) {
                cv::drawChessboardCorners(frame_ptr->img_, board_size, frame_ptr->features, 1);
                cv::circle(frame_ptr->img_, frame_ptr->features[j++], 3, cv::Scalar(0), 4);
                cv::imshow("img", frame_ptr->img_);
                cv::waitKey(1);
                 AddFrames(frame_ptr);
                 std::cout << " ok!" <<std::endl;
            } else {
                std::cout << " not ok" <<std::endl;

            }
         }


    }


    void AddFrames(CalibrationBoardFrame *frame_ptr) {
        frame_ptrs_.push_back(frame_ptr);
    }

    void Calibrate(vector<CalibrationBoardFrame*> &frame_ptrs) {

        //ceres::Problem::Options problem_options;
        ceres::Problem problem;
        int sz = frame_ptrs.size();

        for (int i = 0; i < sz; i++) {
            for (int j = 0; j < frame_ptrs[i]->pattern_height_; j++) {
                for (int k = 0; k < frame_ptrs[i]->pattern_width_; k++) {

                    int idx = j*frame_ptrs[i]->pattern_width_ + k;

                    double observed_x = frame_ptrs[i]->features[idx].x;
                    double observed_y = frame_ptrs[i]->features[idx].y;

                    ceres::CostFunction* cost_function = GenericFisheyeReprojectionError::Create(observed_x, observed_y);
                    problem.AddResidualBlock(cost_function,
                                             NULL /* squared loss */,
                                             frame_ptrs[i]->cam_ptr_->para_ptr(),
                                             frame_ptrs[i]->omega_t_,
                                             CalibrationBoardFrame::pattern_point3d_[idx]);

                    problem.SetParameterBlockConstant(CalibrationBoardFrame::pattern_point3d_[idx]);
                }
            }
        }

        // Configure the solver.
        ceres::Solver::Options options;
        options.use_nonmonotonic_steps = true;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.use_inner_iterations = true;
        options.max_num_iterations = 1000;
        options.minimizer_progress_to_stdout = true;

//        ceres::Solver::Options options;
//        options.linear_solver_type = ceres::DENSE_QR;;
//        options.max_num_iterations = 1000;
          // Solve!
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        std::cout << "Final report:\n" << summary.FullReport();

        std::cout << "center: " << cam_ptr_->para_ptr()[0] << " " << cam_ptr_->para_ptr()[1] <<std::endl;
        std::cout << "coef: " << cam_ptr_->para_ptr()[2] << " " << cam_ptr_->para_ptr()[3] <<" " <<cam_ptr_->para_ptr()[4] <<" " <<cam_ptr_->para_ptr()[5] <<std::endl;
        std::cout << "point: " << CalibrationBoardFrame::pattern_point3d_[3][0] << " " << CalibrationBoardFrame::pattern_point3d_[3][1] << " " << CalibrationBoardFrame::pattern_point3d_[3][2] <<std::endl;
    }

    vector<CalibrationBoardFrame*> frame_ptrs_;
    AbstractCamera* cam_ptr_;
};
} //ceres_calibration





string dir_name = "./data/";
int MainThread() {
    //std::cout << "hello" << std::endl;
    ceres::CameraCalibration test;
    test.GetImagesFromDir(dir_name);
    test.Calibrate(test.frame_ptrs_);
	return 0;

}

int  main() {
	std::thread t1(MainThread);
    //std::thread t2(QtDisplayThread);

	t1.join();
    //t2.join();
	return 1;
}
