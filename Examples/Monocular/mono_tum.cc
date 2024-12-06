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

// ProminentSignMap 선언
std::vector<ProminentSignMap> ProminentSignMapList;
std::mutex mProminentSignMutex;

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

    // Levenshtein 거리 임계값 정의
    const int LEVENSHTEIN_THRESHOLD = 3;

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
        tf.frame_name = localTframe;
        tf.text_dete = localTextDete;
        tf.text_mean = localTextMean;

        // 멤버 변수에 접근할 때는 뮤텍스 잠금 필요 (멀티스레드 환경일 경우)
        {
            std::lock_guard<std::mutex> lock(mTextFrameMutex);
            textFrameArray.push_back(tf); // push_back 사용
        }

        // ** ProminentSignMap 업데이트 시작 **
        {
            std::lock_guard<std::mutex> lock(mProminentSignMutex);

            for(const auto& textInfo : localTextMean)
            {
                const std::string& detectedWord = textInfo.mean;
                int minDistance = INT32_MAX;
                size_t bestMatchIndex = ProminentSignMapList.size(); // 초기값은 리스트 크기 (매칭 없음)

                // 기존 canonical_word와의 거리 계산
                for(size_t i = 0; i < ProminentSignMapList.size(); ++i)
                {
                    int distance = static_cast<int>(tool::LevenshteinDist(detectedWord, ProminentSignMapList[i].canonical_word));
                    if(distance < minDistance)
                    {
                        minDistance = distance;
                        bestMatchIndex = i;
                    }

                    // 정확한 매칭이 있으면 조기 종료
                    if(distance == 0)
                        break;
                }

                if(minDistance > LEVENSHTEIN_THRESHOLD || bestMatchIndex == ProminentSignMapList.size())
                {
                    // 유사한 단어가 없을 경우 새로운 ProminentSignMap 항목 생성
                    ProminentSignMap newSign;
                    newSign.canonical_word = detectedWord;
                    newSign.detections.push_back(tf); // 현재 TextFrame과 연관
                    ProminentSignMapList.push_back(newSign);
                }
                else
                {
                    // 유사한 단어가 존재할 경우 기존 항목에 현재 TextFrame 추가
                    ProminentSignMapList[bestMatchIndex].detections.push_back(tf);
                }
            }
        }
        
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
        SLAM.TrackMonocular_2(im,tframe,ni,textFrameArray,ProminentSignMapList);

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

    // Stop all threads
    SLAM.Shutdown();

    {
    std::lock_guard<std::mutex> lock(mProminentSignMutex);
    std::ofstream outFile("ProminentSignMapList.txt");
    if(outFile.is_open())
    {
        for(const auto& sign : ProminentSignMapList)
        {
            outFile << "Canonical Word: " << sign.canonical_word << "\n";
            outFile << "Detections:\n";
            for(const auto& detection : sign.detections)
            {
                outFile << "  Frame Name: " << detection.frame_name 
                        << ", Score: " << detection.text_mean[0].score << "\n"; // 필요에 따라 조정
            }
            outFile << "-------------------------\n";
        }
        outFile.close();
        std::cout << "ProminentSignMapList가 ProminentSignMapList.txt에 저장되었습니다." << std::endl;
    }
    else
    {
        std::cerr << "ProminentSignMapList.txt 파일을 열 수 없습니다." << std::endl;
    }
}
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
