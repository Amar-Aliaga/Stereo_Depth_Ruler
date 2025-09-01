// #include <opencv2/opencv.hpp>
// #include <opencv2/ximgproc.hpp>
#include "stereo_calibrator.hpp"
#include "stereo_rectifier.hpp"
#include "stereo_disparity.hpp"
#include "utils.hpp"
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

// void change_filename();
// void save_frames();

// class StereoCalibrator {
// private:
//     const cv::Size boardSize {8, 6};          
//     static constexpr float squareSize {20.0f};          
//     static constexpr const char *left_frames_dir  {"/home/amar-aliaga/Desktop/left_frames/" };
//     static constexpr const char *right_frames_dir {"/home/amar-aliaga/Desktop/right_frames/"};
    
//     cv::Mat cameraMatrixLeft, distCoeffsLeft;
//     cv::Mat cameraMatrixRight, distCoeffsRight;
//     cv::Mat R, T, E, F;        
//     cv::Mat R1, R2, P1, P2, Q;  
//     cv::Size imageSize;

// public:
//     StereoCalibrator() = default;

//     bool calibrate() {
//         std::vector<std::vector<cv::Point2f>> imagePointsLeft, imagePointsRight;
//         std::vector<std::vector<cv::Point3f>> objectPoints;
        
//         std::vector<cv::Point3f> obj;
//         for (int i = 0; i < boardSize.height; i++) {
//             for (int j = 0; j < boardSize.width; j++) {
//                 obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
//             }
//         }

//         std::vector<std::string> left_files, right_files;
//         //cv::Size imageSize;
//         int successfulPairs = 0;
//         const int flags = cv::CALIB_CB_ADAPTIVE_THRESH + 
//                          cv::CALIB_CB_NORMALIZE_IMAGE + 
//                          cv::CALIB_CB_FAST_CHECK;

//         for(const auto& entry : std::filesystem::directory_iterator(left_frames_dir)) {
//             left_files.push_back(entry.path().string());
//         }

//         for(const auto& entry : std::filesystem::directory_iterator(right_frames_dir)) {
//                 right_files.push_back(entry.path().string());
//         }

//         std::sort( left_files.begin(),  left_files.end());
//         std::sort(right_files.begin(), right_files.end());

        
//         size_t max_pairs = std::min(left_files.size(), right_files.size());

//         for (int i = 0; i < max_pairs; i++) { 
//             cv::Mat leftImg  = cv::imread(left_files[i], cv::IMREAD_GRAYSCALE);
//             cv::Mat rightImg = cv::imread(right_files[i], cv::IMREAD_GRAYSCALE);

//             if (leftImg.empty() || rightImg.empty()) {
//                 std::cout << "Skipping pair " << i << " - images not found" << std::endl;
//                 continue;
//             }

//             if (imageSize.width == 0) {
//                 imageSize = leftImg.size();
//             }

//             std::vector<cv::Point2f> cornersLeft, cornersRight;
//             bool foundLeft  = cv::findChessboardCorners(leftImg, boardSize, cornersLeft, flags);
//             bool foundRight = cv::findChessboardCorners(rightImg, boardSize, cornersRight, flags);

//             std::cout << "Pair " << i << ": left=" << foundLeft << ", right=" << foundRight << std::endl;

//             if (foundLeft && foundRight) {
//                 cv::cornerSubPix(leftImg, cornersLeft, cv::Size(11, 11), cv::Size(-1, -1),
//                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
//                 cv::cornerSubPix(rightImg, cornersRight, cv::Size(11, 11), cv::Size(-1, -1),
//                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));

//                 imagePointsLeft.push_back(cornersLeft);
//                 imagePointsRight.push_back(cornersRight);
//                 objectPoints.push_back(obj);
//                 successfulPairs++;

//                 cv::Mat displayLeft, displayRight;
//                 cv::cvtColor(leftImg,  displayLeft, cv::COLOR_GRAY2BGR);
//                 cv::cvtColor(rightImg, displayRight, cv::COLOR_GRAY2BGR);
                
//                 cv::drawChessboardCorners(displayLeft, boardSize, cornersLeft, foundLeft);
//                 cv::drawChessboardCorners(displayRight, boardSize, cornersRight, foundRight);
                
//                 cv::imshow("Left Camera", displayLeft);
//                 cv::imshow("Right Camera", displayRight);
//                 cv::waitKey(100); 
//             }
//         }

//         std::cout << "Found " << successfulPairs << " valid stereo pairs" << std::endl;

//         if (successfulPairs < 20) {
//             std::cerr << "Error: Need at least 20 stereo pairs for calibration" << std::endl;
//             return false;
//         }

//         std::vector<cv::Mat> rvecsLeft, tvecsLeft;
//         std::vector<cv::Mat> rvecsRight, tvecsRight;

//         double rmsLeft = cv::calibrateCamera(objectPoints, imagePointsLeft, imageSize,
//             cameraMatrixLeft, distCoeffsLeft, rvecsLeft, tvecsLeft);
//         double rmsRight = cv::calibrateCamera(objectPoints, imagePointsRight, imageSize,
//             cameraMatrixRight, distCoeffsRight, rvecsRight, tvecsRight);

//         std::cout << "Left camera RMS error: " << rmsLeft << std::endl;
//         std::cout << "Right camera RMS error: " << rmsRight << std::endl;

//         double rmsStereo = cv::stereoCalibrate(
//             objectPoints, imagePointsLeft, imagePointsRight,
//             cameraMatrixLeft, distCoeffsLeft,
//             cameraMatrixRight, distCoeffsRight,
//             imageSize, R, T, E, F,
//             cv::CALIB_FIX_INTRINSIC, 
//             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5)
//         );

//         std::cout << "Stereo calibration RMS error: " << rmsStereo << std::endl;

//         cv::stereoRectify(
//             cameraMatrixLeft, distCoeffsLeft,
//             cameraMatrixRight, distCoeffsRight,
//             imageSize, R, T, R1, R2, P1, P2, Q,
//             cv::CALIB_ZERO_DISPARITY, 0, imageSize
//         );

//         // cv::Mat mapL1, mapR1, mapL2, mapR2;
//         // cv::initUndistortRectifyMap( cameraMatrixLeft,  distCoeffsLeft, R1, P1, imageSize, CV_32FC1, mapL1, mapL2);
//         // cv::initUndistortRectifyMap(cameraMatrixRight, distCoeffsRight, R2, P2, imageSize, CV_32FC1, mapR1, mapR2);
//         // std::cout << "Remapping tables created." << std::endl;

//         // for (int i = 0; i < max_pairs; i++) { 
//         //     cv::Mat leftImg = cv::imread(left_files[i], cv::IMREAD_GRAYSCALE);
//         //     cv::Mat rightImg = cv::imread(right_files[i], cv::IMREAD_GRAYSCALE);

//         //     if (leftImg.empty() || rightImg.empty()) {
//         //         std::cout << "Skipping pair " << i << " - images not found" << std::endl;
//         //         continue;
//         //     }
//         //     cv::Mat rectifiedLeftImage, rectifiedRightImage;
//         //     cv::remap(leftImg,  rectifiedLeftImage, mapL1, mapL2, cv::INTER_LINEAR);
//         //     cv::remap(rightImg, rectifiedRightImage, mapR1, mapR2, cv::INTER_LINEAR);
//         //     std::cout << "Images rectified successfully!" << std::endl;

//         //     cv::imshow("Rectified Left", rectifiedLeftImage);
//         //     cv::imshow("Rectified Right", rectifiedRightImage);
//         //     cv::waitKey(0);
//         // }

//         return true;
//     }


//     void saveCalibration(const std::string& filename) {
//         cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        
//         if (fs.isOpened()) {
//             fs << "imageWidth" << imageSize.width;
//             fs << "imageHeight" << imageSize.height;
//             fs << "cameraMatrixLeft" << cameraMatrixLeft;
//             fs << "distCoeffsLeft" << distCoeffsLeft;
//             fs << "cameraMatrixRight" << cameraMatrixRight;
//             fs << "distCoeffsRight" << distCoeffsRight;
//             fs << "R" << R;
//             fs << "T" << T;
//             fs << "E" << E;
//             fs << "F" << F;
//             fs << "R1" << R1;
//             fs << "R2" << R2;
//             fs << "P1" << P1;
//             fs << "P2" << P2;
//             fs << "Q" << Q;
            
//             fs.release();
//             std::cout << "Calibration saved to " << filename << std::endl;
//         } else {
//             std::cerr << "Failed to open file for writing: " << filename << std::endl;
//         }
//     }


//     void printCalibrationResults() {
//         std::cout << "\n=== CALIBRATION RESULTS ===" << std::endl;
//         std::cout << "Left Camera Matrix:\n" << cameraMatrixLeft << std::endl;
//         std::cout << "Left Distortion Coefficients:\n" << distCoeffsLeft << std::endl;
//         std::cout << "Right Camera Matrix:\n" << cameraMatrixRight << std::endl;
//         std::cout << "Right Distortion Coefficients:\n" << distCoeffsRight << std::endl;
//         std::cout << "Rotation Matrix:\n" << R << std::endl;
//         std::cout << "Translation Vector:\n" << T << std::endl;
//         std::cout << "Essential Matrix:\n" << E << std::endl;
//         std::cout << "Fundamental Matrix:\n" << F << std::endl;
//     }


//     const int getBoardSize_Width()    const noexcept { return boardSize.width;   }
//     const int getBoardSize_Height()   const noexcept { return boardSize.height;  }
//     const int getSquareSize()         const noexcept { return squareSize;        }

//     const cv::Mat &getCameraMatrixLeft()   const noexcept { return cameraMatrixLeft;  }
//     const cv::Mat &getDistCoeffsLeft()     const noexcept { return distCoeffsLeft;    }
//     const cv::Mat &getCameraMatrixRight()  const noexcept { return cameraMatrixRight; }
//     const cv::Mat &getDistCoeffsRight()    const noexcept { return distCoeffsRight;   }

//     const cv::Mat &getRotation()           const noexcept { return R;  }
//     const cv::Mat &getTranslation()        const noexcept { return T;  }
//     const cv::Mat &getRectificationLeft()  const noexcept { return R1; }
//     const cv::Mat &getRectificationRight() const noexcept { return R2; }
//     const cv::Mat &getProjectionLeft()     const noexcept { return P1; }
//     const cv::Mat &getProjectionRight()    const noexcept { return P2; }
//     const cv::Mat &getDisparityToDepth()   const noexcept { return Q;  }
// };


// class StereoRectifier {
// private:
//     cv::Mat cameraMatrixLeft, distCoeffsLeft;
//     cv::Mat cameraMatrixRight, distCoeffsRight;        
//     cv::Mat R1, R2, P1, P2, Q; 

//     cv::Mat mapL1, mapL2;
//     cv::Mat mapR1, mapR2;
//     cv::Size imageSize;

// public:

//     cv::Mat scaleCameraMatrix(const cv::Mat& cameraMatrix, double scaleX, double scaleY) {
//         cv::Mat scaled = cameraMatrix.clone();
//         scaled.at<double>(0, 0) *= scaleX; 
//         scaled.at<double>(0, 2) *= scaleX; 
//         scaled.at<double>(1, 1) *= scaleY; 
//         scaled.at<double>(1, 2) *= scaleY; 
//         return scaled;
//     }


//     bool loadCalibration(const std::string& filename) {
//         cv::FileStorage fs(filename, cv::FileStorage::READ);

//         if(!fs.isOpened()) {
//             std::cerr << "Failed to open calibration file: " << filename << std::endl;
//             return false;
//         }

//         int calibWidth, calibHeight;
//         fs["imageWidth"] >> calibWidth;
//         fs["imageHeight"] >> calibHeight;

//         fs["cameraMatrixLeft"] >> cameraMatrixLeft;
//         fs["distCoeffsLeft"] >> distCoeffsLeft;
//         fs["cameraMatrixRight"] >> cameraMatrixRight;
//         fs["distCoeffsRight"] >> distCoeffsRight;
//         fs["R1"] >> R1;
//         fs["R2"] >> R2;
//         fs["P1"] >> P1;
//         fs["P2"] >> P2;
//         fs["Q"] >> Q;

//         fs.release();

//         cv::initUndistortRectifyMap( cameraMatrixLeft,  distCoeffsLeft, R1, P1, imageSize, CV_32FC1, mapL1, mapL2);
//         cv::initUndistortRectifyMap(cameraMatrixRight, distCoeffsRight, R2, P2, imageSize, CV_32FC1, mapR1, mapR2);
//         std::cout << "Remapping tables created." << std::endl;

//         return true;
//     }


//     void rectify(const cv::Mat &left_src, const cv::Mat &right_src, cv::Mat &left_dst, cv::Mat &right_dst) {

//         cv::Mat leftResized, rightResized;
//         if (left_src.size() != imageSize) {
//             std::cerr << "Warning: Resizing input images to match calibration resolution." << std::endl;
//             cv::resize(left_src, leftResized, imageSize);
//             cv::resize(right_src, rightResized, imageSize);
//         } else {
//             leftResized = left_src;
//             rightResized = right_src;
//         }

//         cv::remap( leftResized,  left_dst, mapL1, mapL2, cv::INTER_LINEAR);
//         cv::remap(rightResized, right_dst, mapR1, mapR2, cv::INTER_LINEAR);
//         std::cout << "Images rectified successfully!" << std::endl;
//     }


//     void drawEpipolarLines(cv::Mat& rectifiedLeft, cv::Mat& rectifiedRight) {
//         for (int y = 0; y < rectifiedLeft.rows; y += 30) {
//             cv::line(rectifiedLeft, cv::Point(0, y), cv::Point(rectifiedLeft.cols, y), 
//                     cv::Scalar(0, 255, 0), 1);
//             cv::line(rectifiedRight, cv::Point(0, y), cv::Point(rectifiedRight.cols, y), 
//                     cv::Scalar(0, 255, 0), 1);
//         }
//     }

//     const cv::Size  &getImageSize() const noexcept { return imageSize; }
//     const cv::Mat   &getQ() const noexcept { return Q; };
// };


// class StereoDisparity {
// private:
//     cv::Ptr<cv::StereoSGBM> matcher;
//     cv::Mat Q;
// public:
//     StereoDisparity(const cv::Mat &Q_matrix) : Q(Q_matrix) {
//         matcher = cv::StereoSGBM::create(
//             0, 64, 4, 8*4*4, 32*4*4, 1, 23, 10, 100, 1, cv::StereoSGBM::MODE_SGBM_3WAY
//         );

//     }

//     cv::Mat computeDisparity(const cv::Mat& left, const cv::Mat& right) {
//         cv::Mat leftGray, rightGray;
//         if (left.channels() > 1)
//             cv::cvtColor(left, leftGray, cv::COLOR_BGR2GRAY);
//         else
//             leftGray = left;

//         if (right.channels() > 1)
//             cv::cvtColor(right, rightGray, cv::COLOR_BGR2GRAY);
//         else
//             rightGray = right;

//         cv::Ptr<cv::StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(matcher);

//         cv::Mat disp_left, disp_right;
//         matcher->compute(leftGray, rightGray, disp_left);
//         right_matcher->compute(rightGray, leftGray, disp_right);

//         cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(matcher);
//         wls_filter->setLambda(9300.0);
//         wls_filter->setSigmaColor(1.9);

//         cv::Mat filtered_disp;
//         wls_filter->filter(disp_left, leftGray, filtered_disp, disp_right);

//         return filtered_disp;
//     }

//     cv::Mat computeDepth(const cv::Mat& disparity) {
//         cv::Mat depth;
//         cv::reprojectImageTo3D(disparity, depth, Q);
//         return depth;
//     }

//     const cv::Ptr<cv::StereoSGBM> get_matcher() const { return matcher; }

// };




//    // save_frames();

//     // StereoRectifier rectifier;
//     // if (!rectifier.loadCalibration(outputFile)) {
//     //     std::cerr << "Failed to load calibration!" << std::endl;
//     //     return -1;
//     // }

//     // StereoDisparity disparityComputer(rectifier.getQ());

//     // cv::VideoCapture cap(2);
//     // if(!cap.isOpened()) {
//     //     std::cerr << "Error: Could not open camera." << std::endl;
//     //     return -1;
//     // }


//     // while(true) {
//     //     cv::Mat frame;
//     //     cap >> frame;

//     //     if(frame.empty()) {
//     //        std::cout << "No frame has been found." << std::endl; 
//     //     }

//     //     cv::Mat left_image  = frame(cv::Rect(0, 0, frame.cols/2, frame.rows));
//     //     cv::Mat right_image = frame(cv::Rect(frame.cols/2, 0, frame.cols/2, frame.rows));

//     //     cv::Mat left_res, right_res;
//     //     rectifier.rectify(left_image, right_image, left_res, right_res);
//     //     cv::Mat disparity = disparityComputer.computeDisparity(left_res, right_res);
//     //     rectifier.drawEpipolarLines(left_res, right_res);
//     //     cv::imshow("left", left_res);
//     //     cv::imshow("right", right_res);
//     //     cv::waitKey(0);

//     //     disparity.setTo(0, disparity <= 0);

//     //     cv::Mat xyz = disparityComputer.computeDepth(disparity);

//     //     std::vector<cv::Mat> channels(3);
//     //     cv::split(xyz, channels); 
//     //     cv::Mat depthZ = channels[2].clone();
//     //     depthZ.setTo(0, disparity <= 0);

//     //     double minVal, maxVal = 2000;
//     //     cv::minMaxLoc(depthZ, &minVal, &maxVal);
//     //     cv::Mat disp8;
//     //     if (maxVal > minVal)
//     //         depthZ.convertTo(disp8, CV_8U, 255.0/(maxVal-minVal));
//     //     else
//     //         depthZ.convertTo(disp8, CV_8U);

//     //     std::cout << "Disparity min: " << minVal << ", max: " << maxVal << std::endl;

//     //     cv::Mat yo = disparityComputer.computeDepth(disparity);


//     //     char c = cv::waitKey(10);
//     //     if(c == 27) break;
//     //     else {
//     //         cv::imshow("Disparity", disp8);
//     //     }

        
    
//     return 0;
// }




// void save_frames() {
//     cv::VideoCapture cap(2);

//     if(!cap.isOpened()) {
//         std::cerr << "Error: Could not open camera." << std::endl;
//         return;
//     }

//     std::string left_directory  {"/home/amar-aliaga/Desktop/left_frames/" };
//     std::string right_directory {"/home/amar-aliaga/Desktop/right_frames/"};

//     int frame_count = 1;

//     while(true) {
//         cv::Mat frame;
//         cap >> frame;

//         if(frame.empty()) {
//             std::cerr << "Error: Blank frame." << std::endl;
//             break;
//         }

//         cv::Mat left_image = frame(cv::Rect(0, 0, frame.cols/2, frame.rows));
//         cv::Mat right_image = frame(cv::Rect(frame.cols/2, 0, frame.cols/2, frame.rows));

//         std::vector<cv::Point2f> left_corners, right_corners;
//         bool found_left_board  = cv::findChessboardCornersSB( left_image,  cv::Size(8,6),  left_corners);
//         bool found_right_board = cv::findChessboardCornersSB(right_image,  cv::Size(8,6), right_corners);

//         cv::drawChessboardCorners(left_image, cv::Size(8,6), left_corners, found_left_board);
//         cv::drawChessboardCorners(right_image, cv::Size(8,6), right_corners, found_right_board);

//         cv::imshow("Left Camera", left_image);
//         cv::imshow("Right Camera", right_image);

//         char key = cv::waitKey(10);

//         if(key == 27) break;
//         else if(key == ' ') {
//             std::string left_filename  = left_directory  + "left_"  + std::to_string(frame_count) + ".jpg";
//             std::string right_filename = right_directory + "right_" + std::to_string(frame_count) + ".jpg";

//             // std::vector<cv::Point2f> left_corners, right_corners;
//             // bool found_left_board  = cv::findChessboardCornersSB( left_image,  cv::Size(8,6),  left_corners);
//             // bool found_right_board = cv::findChessboardCornersSB(right_image,  cv::Size(8,6), right_corners);

//             if(found_left_board && found_right_board) {
//                 cv::imwrite(left_filename,   left_image);
//                 cv::imwrite(right_filename, right_image);
//                 std::cout << "Saved: " << frame_count << std::endl;
//                 frame_count++;
//             } else {
//                 std::cout << "Take the picture again." << std::endl;
//             }
//         }
//     }

//     cap.release();
//     cv::destroyAllWindows();

// }

// void change_filename() {
//     std::string folder_path = "/home/amar-aliaga/Desktop/calibration_frames/";

//     int rcount = 1;
//     int lcount = 1;
//     int count_img = 1;
//     for(auto &i : std::filesystem::directory_iterator(folder_path)) {
//         if(i.is_regular_file()) {
//             std::string lfilename, rfilename;
//             if(count_img <= 27) {
//                 lfilename =  "leftImg_" + std::to_string(lcount++) + ".jpg";
//                 std::filesystem::path new_lpath = i.path().parent_path() / lfilename;
//                 std::filesystem::rename(i.path(), new_lpath);
//             } else {
//                 rfilename = "rightImg_" + std::to_string(rcount++) + ".jpg";
//                 std::filesystem::path new_rpath = i.path().parent_path() / rfilename;
//                 std::filesystem::rename(i.path(), new_rpath);
//             }
//             count_img++;
//         }
//     }
//     std::cout << "File names update" << std::endl;
// }








// // %YAML:1.0
// // ---
// // imageWidth: 1280
// // imageHeight: 720
// // cameraMatrixLeft: !!opencv-matrix
// //    rows: 3
// //    cols: 3
// //    dt: d
// //    data: [ 699.40367387964193, 0., 668.59270505751181, 0.,
// //        699.42794136236591, 350.66970696733705, 0., 0., 1. ]
// // distCoeffsLeft: !!opencv-matrix
// //    rows: 1
// //    cols: 5
// //    dt: d
// //    data: [ -0.16518109876295864, 0.0106608276339365,
// //        -0.00011791513999614162, -0.00038503854526506095,
// //        0.010787666213626297 ]
// // cameraMatrixRight: !!opencv-matrix
// //    rows: 3
// //    cols: 3
// //    dt: d
// //    data: [ 698.31531224838807, 0., 642.8501204689494, 0.,
// //        699.30774297199764, 348.12078189501125, 0., 0., 1. ]
// // distCoeffsRight: !!opencv-matrix
// //    rows: 1
// //    cols: 5
// //    dt: d
// //    data: [ -0.16442291396023168, -0.0037101653834304927,
// //        -0.00034814885545897194, -0.00084372696836470303,
// //        0.024946490343938348 ]
// // R: !!opencv-matrix
// //    rows: 3
// //    cols: 3
// //    dt: d
// //    data: [ 0.99985338005296465, -0.0005995316746109714,
// //        0.01711312240454619, 0.00052486130094491499, 0.99999032469404159,
// //        0.0043674980160234292, -0.017115575283250431,
// //        -0.0043578756380071642, 0.99984402083657387 ]
// // T: !!opencv-matrix
// //    rows: 3
// //    cols: 1
// //    dt: d
// //    data: [ -126.51971085377559, 0.022050118973052181,
// //        -0.58911780293431915 ]
// // E: !!opencv-matrix
// //    rows: 3
// //    cols: 3
// //    dt: d
// //    data: [ -6.8195334829986498e-05, 0.58901601136304249,
// //        0.024619650449461038, -2.7544890624461282, -0.55100397087444286,
// //        126.48989477004739, -0.08845218601966337, -126.51847351711855,
// //        -0.55295193252674346 ]
// // F: !!opencv-matrix
// //    rows: 3
// //    cols: 3
// //    dt: d
// //    data: [ 1.1605652114766974e-10, -1.0023673547121128e-06,
// //        0.00032211842259002947, 4.6810056202922901e-06,
// //        9.3634904178652298e-07, -0.15380056425011526,
// //        -0.0015245124449148096, 0.15066906358354978, 1. ]
// // R1: !!opencv-matrix
// //    rows: 3
// //    cols: 3
// //    dt: d
// //    data: [ 0.99976273950752792, -0.00079409516921821147,
// //        0.021767730824894076, 0.00074660251128828663, 0.99999732360683335,
// //        0.0021898319251565782, -0.021769411500841211,
// //        -0.0021730605220566745, 0.99976065662281122 ]
// // R2: !!opencv-matrix
// //    rows: 3
// //    cols: 3
// //    dt: d
// //    data: [ 0.9999891442748956, -0.00017428019281927356,
// //        0.0046562816470131158, 0.00018443783621972996,
// //        0.99999760427612383, -0.0021811521491022583,
// //        -0.0046558903602308844, 0.0021819872656259186,
// //        0.99998678072088854 ]
// // P1: !!opencv-matrix
// //    rows: 3
// //    cols: 4
// //    dt: d
// //    data: [ 669.51909151610789, 0., 638.6422119140625, 0., 0.,
// //        669.51909151610789, 346.5215950012207, 0., 0., 0., 1., 0. ]
// // P2: !!opencv-matrix
// //    rows: 3
// //    cols: 4
// //    dt: d
// //    data: [ 669.51909151610789, 0., 638.6422119140625,
// //        -84708.281439517843, 0., 669.51909151610789, 346.5215950012207,
// //        0., 0., 0., 1., 0. ]
// // Q: !!opencv-matrix
// //    rows: 4
// //    cols: 4
// //    dt: d
// //    data: [ 1., 0., 0., -638.6422119140625, 0., 1., 0.,
// //        -346.5215950012207, 0., 0., 0., 669.51909151610789, 0., 0.,
// //        0.0079038209740348465, 0. ]

void test() {
        const std::string &outputFile {"config/stereo.yaml"};

   // cv::Mat image1 = "";

    StereoCalibrator calibrator;
    StereoRectifier   rectifier;
    //calibrator.run_calibration();
    rectifier.loadCalibration(outputFile);
    cv::Mat right, left;
    cv::VideoCapture cap("/home/amar-aliaga/Desktop/my_video/output.mp4");
    if(!cap.isOpened()) {
        std::cerr << "Camera not found!" << std::endl;
        return;
    }
    while(true) {
        cv::Mat frame;
       // cv::Mat frame = cv::imread("/home/amar-aliaga/rama_img.jpg", cv::IMREAD_COLOR);
       cap >> frame;
        if(frame.empty()) {
           std::cerr << "Frame not found" << std::endl;
            return;
       }
        cv::Mat left_res = frame(cv::Rect(0, 0, frame.cols/2, frame.rows));
        cv::Mat right_res = frame(cv::Rect(frame.cols/2, 0, frame.cols/2, frame.rows));
        cv::Mat left_rec, right_rec;
        rectifier.rectify(left_res, right_res, left_rec, right_rec);
        StereoDisparity disparity (rectifier.getQ());
        
        cv::Mat disp = disparity.computeDisparity(left_rec, right_rec);
//        cv::Mat dispNormalized;
//         if (!disp.empty()) {
//     double minVal, maxVal;
//     cv::minMaxLoc(disp, &minVal, &maxVal);
//     if (maxVal > minVal) {
        
//         disp.convertTo(dispNormalized, CV_8U, 255.0 / (maxVal - minVal), -255.0 * minVal / (maxVal - minVal));
//     }
// }
// cv::Mat dispColor;
//cv::applyColorMap(dispNormalized, dispColor, cv::COLORMAP_JET);
if(cv::waitKey(1) == 27) break;
        else {
        cv::imshow("Disparity", disp);
        //cv::imwrite("/home/amar-aliaga/disp_map.jpg", dispNormalized);
        }

   }


    cap.release();
}

int main() {

    //image_desparity("/home/amar-aliaga/rama_img.jpg");

    
    test();













    //rectifier.rectify()
    //StereoDisparity disparity (rectifier.getQ());

    //disparity.computeDisparity();

    //calibrator.run_calibration();

    //rectifier.run_rectification();

    
    // cv::VideoCapture cap(2);
    // if (!cap.isOpened()) {
    //     std::cerr << "Error: Could not open camera." << std::endl;
    //     return -1;
    // }

    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // // Set up VideoWriter
    // std::string video_filename = "/home/amar-aliaga/Desktop/my_video/output.mp4";
    // int fourcc = cv::VideoWriter::fourcc('M', 'P', '4', 'V');  // MP4 codec
    // double fps = 30.0;
    // cv::VideoWriter writer(video_filename, fourcc, fps, cv::Size(2560, 720));

    // if (!writer.isOpened()) {
    //     std::cerr << "Failed to open video writer." << std::endl;
    //     return -1;
    // }

    // std::cout << "Recording video. Press ESC to stop." << std::endl;
    // while (true) {
    //     cv::Mat frame;
    //     cap >> frame;
    //     if (frame.empty()) {
    //         std::cout << "No frame found." << std::endl;
    //         continue;
    //     }

    //     writer.write(frame);  // Write frame to video
    //     cv::imshow("Recording", frame);

    //     if (cv::waitKey(1) == 27) break;  // ESC to stop
    // }

    // writer.release();
    // std::cout << "Video saved: " << video_filename << std::endl;

    // if (!rectifier.loadCalibration(outputFile)) {
    //     std::cerr << "Failed to load calibration!" << std::endl;
    //     return -1;
    // }

    // StereoDisparity disparityComputer(rectifier.getQ());

    // cv::VideoCapture cap(2);
    // if(!cap.isOpened()) {
    //     std::cerr << "Error: Could not open camera." << std::endl;
    //     return -1;
    // }

    // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // while(true) {
    //     cv::Mat frame;
    //     cap >> frame;

    //     if(frame.empty()) {
    //        std::cout << "No frame has been found." << std::endl; 
    //     }

    //     cv::Mat left_image  = frame(cv::Rect(0, 0, frame.cols/2, frame.rows));
    //     cv::Mat right_image = frame(cv::Rect(frame.cols/2, 0, frame.cols/2, frame.rows));

    //     cv::Mat left_res, right_res;
    //     rectifier.rectify(left_image, right_image, left_res, right_res);
    //     //rectifier.drawEpipolarLines(left_res, right_res);

    //     cv::Mat raw_disp = disparityComputer.computeDisparity(left_res, right_res);
    //     if (raw_disp.empty()) {
    //         std::cerr << "computeDisparity returned empty image, skipping frame\n";
    //         continue;
    //     }

    //     // build valid mask (disparity > 0)
    //     cv::Mat validMask;
    //     if (raw_disp.type() == CV_16S || raw_disp.type() == CV_32S || raw_disp.type() == CV_16U) {
    //         validMask = raw_disp > 0;
    //     } else {
    //         raw_disp.convertTo(raw_disp, CV_32F);
    //         validMask = raw_disp > 0.0f;
    //     }

    //     // convert disparity to float in pixels.
    //     // StereoSGBM usually outputs CV_16S with 16*disparity.
    //     cv::Mat dispFloat;
    //     if (raw_disp.type() == CV_16S) {
    //         raw_disp.convertTo(dispFloat, CV_32F, 1.0/16.0); // scale down if matcher used 16x
    //     } else if (raw_disp.type() == CV_32F) {
    //         dispFloat = raw_disp;
    //     } else {
    //         raw_disp.convertTo(dispFloat, CV_32F);
    //     }

    //     // compute min/max only on valid pixels
    //     double minD = 0, maxD = 0;
    //     cv::minMaxLoc(dispFloat, &minD, &maxD, nullptr, nullptr, validMask);
    //     if (maxD <= minD) {
    //         std::cerr << "invalid disparity range: min=" << minD << " max=" << maxD << "\n";
    //         continue;
    //     }

    //     // normalize to 8-bit for display (closer = brighter)
    //     cv::Mat disp8;
    //     dispFloat.convertTo(disp8, CV_8U, 255.0/(maxD - minD), -255.0*minD/(maxD - minD));
    //     // mask out invalid pixels
    //     disp8.setTo(0, ~validMask);

    //     // optional: apply color map for readability
    //     // cv::Mat dispColor;
    //     // cv::applyColorMap(disp8, dispColor, cv::COLORMAP_JET);
        
    //     cv::Mat cloud = disparityComputer.computeDepth(dispFloat); // CV_32FC3
    //     cv::Mat depthZ;
    //     if (!cloud.empty()) {
    //         std::vector<cv::Mat> ch(3);
    //         cv::split(cloud, ch);
    //         depthZ = ch[2]; // Z channel (depth)
    //         // Mask invalid pixels (where disparity <= 0)
    //         depthZ.setTo(std::numeric_limits<float>::quiet_NaN(), ~validMask);
    //     }

    //     // Visualize depth: compute min/max on valid pixels and normalize (closer = brighter)
    //     cv::Mat depth8;
    //     if (!depthZ.empty()) {
    //         double minZ = 0.0, maxZ = 0.0;
    //         cv::minMaxLoc(depthZ, &minZ, &maxZ, nullptr, nullptr, validMask);
    //         if (maxZ > minZ && std::isfinite(minZ) && std::isfinite(maxZ)) {
    //             // invert so small Z (close) -> large value (bright)
    //             cv::Mat scaled = (maxZ - depthZ) * (255.0 / (maxZ - minZ));
    //             scaled.convertTo(depth8, CV_8U);
    //             depth8.setTo(0, ~validMask);
    //         } else {
    //             depth8 = cv::Mat::zeros(disp8.size(), CV_8U);
    //         }
    //     } else {
    //         depth8 = cv::Mat::zeros(disp8.size(), CV_8U);
    //     }

    


    //     char c = cv::waitKey(10);
    //     if(c == 27) break;
    //     else {
    //         cv::imshow("Rectified Left", left_res);
    //         cv::imshow("Rectified Right", right_res);
    //         cv::imshow("Disparity (jet)", disp8);
    //         cv::imshow("Depth (jet)", depth8);
    //         std::cout << cap.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    //         std::cout << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    //     }
    // }

    return 0;
}