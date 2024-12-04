#include <Tool.h>
#include <sstream>
#include <string>
#include <cstring>
#include <cmath>
#include <cfloat>
#include <iostream>

using namespace std;

namespace tool {

void GetDeteInfo(const std::vector<Eigen::Matrix<double,2,1>> &vDetecRaw, std::vector<std::vector<Eigen::Matrix<double,2,1>>> &vDetec);
bool Readfulltxt(const std::string &Path, char* &m_binaryStr, int &m_length);
size_t get_utf8_char_len(const char & byte);

void LoadTexts(const string &Path, std::vector<std::vector<Eigen::Matrix<double,2,1>>> &vDetec, std::vector<TextInfo> &vMean)
{
    bool DOUBLE = false;

    string path1, path2;
    string path_text = Path;

    path_text = path_text.replace(path_text.find("images"), 6, "text");
    path_text = path_text.replace(path_text.find(".png"), 4, "");
    path1 = path_text+"_dete.txt";
    path2 = path_text+"_mean.txt";

    ifstream infile;
    infile.open(path1.c_str());
    assert(infile.is_open);
//     cout << "path1: " << path1 << endl;
//     cout << "path2: " << path2 << endl;

    vector<Eigen::Matrix<double,2,1>> vDetecRaw;
    std::string s;
    int idx = -1;
    while(getline(infile,s))
    {
        idx ++;
        char *char_s = (char *)s.c_str();
        const char *split = ",";
        char *p = strtok(char_s, split);

        std::vector<double> nums;
        double a;
        while(p != NULL)
        {
            std::sscanf(p, "%lf", &a);
            nums.push_back(a);
            p=std::strtok(NULL, split);
        }

        assert(nums.size()==8);

        for(int i = 0; i < nums.size(); )
        {
            Eigen::Matrix<double,2,1> data_tmp;
            if(DOUBLE)
                data_tmp = Eigen::Matrix<double,2,1>(nums[i], nums[i+1]);
            else
                data_tmp = Eigen::Matrix<double,2,1>(std::round(nums[i]), std::round(nums[i+1]));

            vDetecRaw.push_back(data_tmp);
            i = i+2;
        }

    }
    infile.close();
    GetDeteInfo(vDetecRaw, vDetec);

    // recognition information
    ifstream f;
    f.open(path2.c_str());
    int UseRead = 1;

    // read 1: get 'mean,'
    if(UseRead==0){
        while(!f.eof())
        {
            string s;
            getline(f,s);
            if(!s.empty())
            {
                stringstream ss;
                ss << s;
                string MeanTmp;
                ss >> MeanTmp;
                struct TextInfo TextMean = {MeanTmp, 1.0};
                vMean.push_back(TextMean);
            }
        }
    }
    // read 2: get 'mean' and mean_score
    vector<string> vMeanRaw;    // text mean
    vector<int> vLangRaw;          // text language
    vector<double> vScoreRaw;      // text recognition score
    double MAX_SCORE = DBL_MAX;
    if(UseRead==1){

        char* m_binaryStr;
        int m_length;
        // cout << path2 << endl;
        Readfulltxt(path2, m_binaryStr, m_length);
        size_t m_index = 0;

        string OneChar;
        vector<string> OneTextMean;
        double sum_len = 0;
        int idx_line = 0;

        while(m_index < m_length){
            // 1. get single character
            size_t utf8_char_len = get_utf8_char_len(m_binaryStr[m_index]);
            size_t next_idx = m_index + utf8_char_len;
            OneChar = string(m_binaryStr + m_index, next_idx - m_index);

            OneTextMean.push_back(OneChar);
            sum_len += utf8_char_len;

            // 2. encouter , get word -----------
            if(OneChar==","){
                string TextMean;
                for(size_t ichar=0; ichar<OneTextMean.size()-1; ichar++){
                    TextMean = TextMean+OneTextMean[ichar];
                }
                vMeanRaw.push_back(TextMean);

                // 3. which language. english -- 0; Chinese -- 1; Chinese+english -- 2
                int lang;
                if( (sum_len-1)==(OneTextMean.size()-1) )
                    lang = (int)0;
                else if( (sum_len-1)==3*(OneTextMean.size()-1) )
                    lang = (int)1;
                else if( (sum_len-1)<3*(OneTextMean.size()-1) && sum_len>(OneTextMean.size()-1))
                    lang = (int)2;
                vLangRaw.push_back(lang);

                TextMean.clear();
                OneTextMean.clear();        // clear text in OneTextMean
                sum_len = 0;
            }
            // 2 -------------------------------

            // 4. end of the line, get recognition score
            if(OneChar=="\n"){
                string TextScore;
                for(size_t ichar=0; ichar<OneTextMean.size()-1; ichar++){
                    TextScore = TextScore+OneTextMean[ichar];
                }
                stringstream ss;
                ss << TextScore;
                double score;
                ss >> score;
                vScoreRaw.push_back(score);

                struct TextInfo TextMean = {vMeanRaw[idx_line], vScoreRaw[idx_line]};
                vMean.push_back(TextMean);

                TextScore.clear();
                OneTextMean.clear();
                sum_len = 0;
                idx_line++;

            }

            m_index = next_idx;

        }

    }


    f.close();

    if(vDetec.size()!=vMean.size()){
        std::cout<<"recognition input error."<<std::endl;
    }
    assert(vDetec.size()==vMean.size());

}


void GetDeteInfo(const std::vector<Eigen::Matrix<double,2,1>> &vDetecRaw, std::vector<std::vector<Eigen::Matrix<double,2,1>>> &vDetec)
{
    for(size_t i0 = 0; i0<vDetecRaw.size(); ){
        std::vector<Eigen::Matrix<double,2,1>> DetecCell;
        DetecCell.push_back(vDetecRaw[i0]);
        DetecCell.push_back(vDetecRaw[i0+1]);
        DetecCell.push_back(vDetecRaw[i0+2]);
        DetecCell.push_back(vDetecRaw[i0+3]);
        vDetec.push_back(DetecCell);
        DetecCell.clear();
        i0 += 4;
    }
}

bool Readfulltxt(const string &Path, char* &m_binaryStr, int &m_length)
{
    std::ifstream infile;
    infile.open(Path.c_str(), ios::binary);
    if(!infile){
        std::cout<<Path<<" Load text error."<<std::endl;
        return false;
    }

    std::filebuf *pbuf=infile.rdbuf();
    m_length = (int)pbuf->pubseekoff(0,ios::end,ios::in);
    pbuf->pubseekpos(0,ios::in);

    m_binaryStr = new char[m_length+1];
    pbuf->sgetn(m_binaryStr,m_length);
    infile.close();

    return true;
}

size_t get_utf8_char_len(const char & byte)
{

    size_t len = 0;
    unsigned char mask = 0x80;
    while( byte & mask )
    {
        len++;
        if( len > 6 )
        {
            return 0;
        }
        mask >>= 1;
    }
    if( 0 == len)
    {
        return 1;
    }
    return len;
}

} // namespace tool