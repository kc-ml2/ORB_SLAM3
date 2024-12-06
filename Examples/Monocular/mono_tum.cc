/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <Tool.h>
#include "Settings.h"
#include "Text.h"

using namespace tool;

std::vector<std::string> vstrImageFilenames;
std::vector<TextFrame> textFrameArray;
std::mutex mTextFrameMutex;
std::mutex mTextMutex; // 뮤텍스 추가


void LoadImages(const string &strFile, std::vector<string> &vstrImageFilenames,
                std::vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    float imageScale = SLAM.GetImageScale();

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    double t_resize = 0.f;
    double t_track = 0.f;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // tool 싱글톤 인스턴스에 데이터 로드
        std::string imagePath = string(argv[3]) + "/" + vstrImageFilenames[ni];
        // cout << "imagePath:  " << imagePath << endl;
        
        // Declare temporary variables for detections and mean information
        std::vector<vector<Eigen::Matrix<double,2,1>>> vTextDete;
        std::vector<TextInfo> vTextMean;

        tool::LoadTexts(imagePath, vTextDete, vTextMean);
        assert(vTextDete.size()==vTextMean.size());

        // 임시 변수에 복사하여 접근
        std::vector<std::vector<Vec2>> localTextDete;
        std::vector<TextInfo> localTextMean;
        double localTframe;
        {
            std::lock_guard<std::mutex> lock(mTextMutex);
            localTextDete = vTextDete;
            localTextMean = vTextMean;
            localTframe = vTimestamps[ni];
        }
        std::cout << "image fileName: " << std::fixed << std::setprecision(6) << localTframe << std::endl;

        // TextFrame 배열에 추가
        TextFrame tf;
        tf.frame_name = std::to_string(localTframe);
        tf.text_dete = localTextDete;
        tf.text_mean = localTextMean;

        // 멤버 변수에 접근할 때는 뮤텍스 잠금 필요 (멀티스레드 환경일 경우)
        {
            std::lock_guard<std::mutex> lock(mTextFrameMutex);
            textFrameArray.push_back(tf); // push_back 사용
        }

        // textFrameArray 내부의 모든 값을 출력
        // {
        //     std::lock_guard<std::mutex> lock(mTextFrameMutex);
        //     std::cout << "=== TextFrameArray 내용 ===" << std::endl;
        //     std::cout << "size: " << textFrameArray.size() << std::endl;
            
        //     std::cout << "============================" << std::endl;
        // }
        
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];
        // std::cout << std::fixed << std::setprecision(6) << vTimestamps[ni] << std::endl;

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular_2(im,tframe,ni,textFrameArray);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    {
        // std::lock_guard<std::mutex> lock(mTextFrameMutex);
        // std::cout << "=== TextFrameArray 내용 ===" << std::endl;
        // std::cout << "size: " << textFrameArray.size() << std::endl;
        // for (size_t i = 0; i < textFrameArray.size(); ++i) {
        //     const TextFrame& currentFrame = textFrameArray[i];
        //     std::cout << "TextFrame " << i << ":" << std::endl;
        //     std::cout << "  Frame Name: " << currentFrame.frame_name << std::endl;
            
        //     // text_dete 출력
        //     std::cout << "  TextDete:" << std::endl;
        //     for (size_t j = 0; j < currentFrame.text_dete.size(); ++j) {
        //         std::cout << "    Detection " << j << ":" << std::endl;
        //         for (size_t k = 0; k < currentFrame.text_dete[j].size(); ++k) {
        //             std::cout << "      Point " << k << ": (" 
        //                       << currentFrame.text_dete[j][k].transpose() << ")" << std::endl;
        //         }
        //     }
            
        //     // text_mean 출력
        //     std::cout << "  TextMean:" << std::endl;
        //     for (size_t j = 0; j < currentFrame.text_mean.size(); ++j) {
        //         std::cout << "    TextInfo " << j << ":" << std::endl;
        //         std::cout << "      Mean: " << currentFrame.text_mean[j].mean << std::endl;
        //         std::cout << "      Score: " << currentFrame.text_mean[j].score << std::endl;
        //     }
        // }


        // std::cout << "============================" << std::endl;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, std::vector<string> &vstrImageFilenames, std::vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
