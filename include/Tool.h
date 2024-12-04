#ifndef TOOL_H
#define TOOL_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <cassert>
#include <Settings.h>

namespace tool {

void LoadTexts(const std::string &Path, std::vector<std::vector<Eigen::Matrix<double,2,1>>> &vDetec, std::vector<TextInfo> &vMean);

class tool
{
public:
    void GetDeteInfo(const std::vector<Eigen::Matrix<double,2,1>> &vDetecRaw, std::vector<std::vector<Eigen::Matrix<double,2,1>>> &vDetec);
    bool Readfulltxt(const std::string &Path, char* &m_binaryStr, int &m_length);
    size_t get_utf8_char_len(const char & byte);
};

} 

#endif