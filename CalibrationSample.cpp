#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/omnidir.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

std::vector<std::string> fileList(std::string dir, std::string ext = "")
{
    //対象のディレクトリ直下のファイルのリスト取得
    std::vector<std::string> flist;
    for ( auto ent : fs::directory_iterator(dir) ) {
        if(!fs::is_directory(ent)){
            std::string path = ent.path().string();
            if(ext != ""){
                auto pos = path.rfind(ext);
                if(pos == std::string::npos)
                    continue;
                if(pos + ext.length() != path.length())
                    continue;
            }
            flist.push_back(path);
        }
    }
    std::sort(flist.begin(), flist.end());
    return flist;
}

int main(){

    std::string imgDir = "./img";   //チェスボード画像を格納しているディレクトリ
    cv::Size patternSize(7,10);     //チェスボードの交点数
    cv::Size imgSize;

    std::vector<std::vector<cv::Point2f>> imgPoints;    //画像上のチェスボードの交点の座標
    std::vector<std::vector<cv::Point3f>> objPoints;    //3次元空間でのチェスボードの交点の座標

    //チェスボード検出
    std::vector<std::string> imgList = fileList(imgDir, ".jpg");
    for(std::string imgPath : imgList){
        std::cout << imgPath << " ...";
        cv::Mat img = cv::imread(imgPath);
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(img, patternSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_FILTER_QUADS);
        if(!found){
            std::cout << " not found" << std::endl;
        }
        imgPoints.push_back(corners);
        std::cout << " found" << std::endl;
        imgSize = img.size();
    }
    
    //チェスボードの三次元座標
    for(int i = 0; i < imgPoints.size(); i ++){
        std::vector<cv::Point3f> obj;
        for(int c = 0; c < patternSize.height; c ++){
            for(int r = 0; r < patternSize.width; r ++){
                float x = r*24.0; //mm
                float y = c*24.0; //mm
                float z = 0.0;
                obj.push_back(cv::Point3f(x,y,z));
            }
        }
        objPoints.push_back(obj);
    }
    
    //キャリブレーション
    std::cout << "cv::calibrateCamera" << std::endl;
    cv::Mat persK, persD, persR, persT;
    std::string persFile = "./perspectiveCalibrate.xml";
    double persRMS = cv::calibrateCamera(objPoints, imgPoints, imgSize, persK, persD, persR, persT);
    cv::FileStorage persxml(persFile, cv::FileStorage::WRITE);
    cv::write(persxml, "RMS", persRMS);
    cv::write(persxml, "K", persK);
    cv::write(persxml, "D", persD);
    persxml.release();
    
    std::cout << "cv::fisheye::calibrate" << std::endl;
    cv::Mat fishK, fishD, fishR, fishT;
    std::string fishFile = "./fisheyeCalibrate.xml";
    int fisheyeFlag = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW;
    double fishRMS = cv::fisheye::calibrate(objPoints, imgPoints, imgSize, fishK, fishD, fishR, fishT, fisheyeFlag);
    cv::FileStorage fishxml(fishFile, cv::FileStorage::WRITE);
    cv::write(fishxml, "RMS", fishRMS);
    cv::write(fishxml, "K", fishK);
    cv::write(fishxml, "D", fishD);
    fishxml.release();
    
    std::cout << "cv::omnidir::calibrate" << std::endl;
    cv::Mat omniK, omniXi, omniD, omniR, omniT, idx;
    std::string omniFile = "./omnidirectionalCalibrate.xml";
    cv::TermCriteria critia(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);
    double omniRMS = cv::omnidir::calibrate(objPoints, imgPoints, imgSize, omniK, omniXi, omniD, omniR, omniT, 0, critia, idx);
    cv::FileStorage omnixml(omniFile, cv::FileStorage::WRITE);
    cv::write(omnixml, "RMS", omniRMS);
    cv::write(omnixml, "K", omniK);
    cv::write(omnixml, "Xi", omniXi);
    cv::write(omnixml, "D", omniD);
    omnixml.release();
    
    //Undistort
    std::cout << "Undistort" << std::endl;
    cv::Mat distorted = cv::imread("./photo.jpg");
    cv::Mat undistorted, fishUndistorted, omniUndistorted;
    
    cv::undistort(distorted, undistorted, persK, persD);
    cv::fisheye::undistortImage(distorted, fishUndistorted, fishK, fishD);
    cv::omnidir::undistortImage(distorted, omniUndistorted, omniK, omniD, omniXi, cv::omnidir::RECTIFY_PERSPECTIVE);
    
    cv::imwrite("./undistort.jpg", undistorted);
    cv::imwrite("./undistort_fisheye.jpg", fishUndistorted);
    cv::imwrite("./undistort_omnidir.jpg", omniUndistorted);

    
    return 0;
}
