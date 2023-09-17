#ifndef TOOLS_H
#define TOOLS_H
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <curl/curl.h>
#include "global.h"
#include <uuid/uuid.h>
using namespace std;

#define GUID_LEN 64

static std::string push_address_result;

static map<string, cv::Scalar> colorMap = {
    {"redTrafficLight", cv::Scalar(0, 0, 255)},
    {"greenTrafficLight", cv::Scalar(0, 255, 0)},
    {"yellowTrafficLight", cv::Scalar(0, 255, 255)},
    {"person", cv::Scalar(0, 0, 255)},
    {"car", cv::Scalar(223, 135, 41)},
    {"truck", cv::Scalar(112, 40, 246)},
    {"bus", cv::Scalar(96, 132, 20)},
    {"motorcycle", cv::Scalar(215, 61, 108)},
    {"bicycle", cv::Scalar(147, 130, 14)},
    {"tricycle", cv::Scalar(203, 103, 29)},
    {"agriculturalVehicle", cv::Scalar(230, 73, 151)},
    {"trafficLight", cv::Scalar(19, 90, 207)},
    {"roadSign", cv::Scalar(203, 82, 202)},
    {"text", cv::Scalar(202, 48, 58)},
};

class Tools {
public:
    static int getNetworkStatus()
    {
        string strRe = getCmdResult("ping www.baidu.com -w 1");
        if(strRe.find("received") != string::npos && strRe.find(", 0 received") == string::npos)
         {
            return 1;
         }
         else
         {
            return 0;
         }
    }

    static string getCmdResult(const string &strCmd) // 这个是获取命令执行的结果， 类似于system, 之前我已经说过了
    {
        char buf[10240] = {0};
        FILE *pf = NULL;
        if( (pf = popen(strCmd.c_str(), "r")) == NULL )
        {
        return "";
        }
        string strResult;
        while(fgets(buf, sizeof buf, pf))
        {
        strResult += buf;
        }
        pclose(pf);
        unsigned int iSize = strResult.size();
        if(iSize > 0 && strResult[iSize - 1] == '\n') // linux
        {
        strResult = strResult.substr(0, iSize - 1);
        }
        return strResult;
    }

    //0->success -1->fail
    static int getPushAddressInfo(QString& address)
    {
        CURL *curl;
        CURLcode result_code;
        curl = curl_easy_init();
        if (curl) {
           std::string url = g_platform_address.toStdString() + g_serial_number.toStdString();
           curl_easy_setopt(curl, CURLOPT_URL, url.data());
           curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
           curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
           curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5);
           curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10);
           std::string address_result;
           curl_easy_setopt(curl, CURLOPT_WRITEDATA, &address_result);
           curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, pushAddressCallback);
           struct curl_slist *headers = NULL;
           headers = curl_slist_append(headers, "User-Agent:Mozilla/5.0 (Windows NT 10.0; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/80.0.3987.87 Safari/537.36 SE 2.X MetaSr 1.0");
           curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
           result_code = curl_easy_perform(curl);
           if (result_code != CURLE_OK || address_result == "") {
               fprintf(stderr, "getPushAddressInfo: curl_easy_perform() failed: %s\n",
                       curl_easy_strerror(result_code));
               qWarning() << "getPushAddressInfo: curl_easy_perform() failed";
               return -1;
           }
           address = QString::fromStdString(address_result);
           curl_slist_free_all(headers);
           curl_easy_cleanup(curl);
           curl_global_cleanup();
        } else {
           fprintf(stderr, "getPushAddressInfo: curl_easy_init() failed.");
           qWarning() << "getPushAddressInfo: curl_easy_perform() failed";
           return -1;
        }
        return 0;
    }

    static size_t pushAddressCallback(void *ptr, size_t size, size_t nmemb, void *stream)
    {
        // 获取到的body存放在ptr中，先将其转换为string格式
        std::string s((char *) ptr, size * nmemb);
        std::string* address_result = static_cast<std::string*>(stream);
        QJsonParseError json_error;
        QJsonDocument json_doc = QJsonDocument::fromJson(s.c_str(), &json_error);
        if (json_error.error != QJsonParseError::NoError){
            qDebug() << "json parse error!";
            *address_result = "";
        }
        QJsonObject json_object = json_doc.object();
        if (json_object.contains("code") && json_object["code"].toInt() == 1000 && json_object.contains("data") && json_object["data"].toObject().contains("streamAddr")){
            *address_result = json_object["data"].toObject()["streamAddr"].toString().toStdString();
        }
        else{
            *address_result = "";
        }
        return size * nmemb;
    }

    //base64编码
    static std::string base64Encode(const unsigned char* Data, int DataByte)
    {
       //编码表
       const char EncodeTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
       //返回值
       std::string strEncode;
       unsigned char Tmp[4] = { 0 };
       int LineLength = 0;
       for (int i = 0; i < (int)(DataByte / 3); i++)
       {
           Tmp[1] = *Data++;
           Tmp[2] = *Data++;
           Tmp[3] = *Data++;
           strEncode += EncodeTable[Tmp[1] >> 2];
           strEncode += EncodeTable[((Tmp[1] << 4) | (Tmp[2] >> 4)) & 0x3F];
           strEncode += EncodeTable[((Tmp[2] << 2) | (Tmp[3] >> 6)) & 0x3F];
           strEncode += EncodeTable[Tmp[3] & 0x3F];
           if (LineLength += 4, LineLength == 76) { strEncode += "\r\n"; LineLength = 0; }
       }
       //对剩余数据进行编码
       int Mod = DataByte % 3;
       if (Mod == 1)
       {
           Tmp[1] = *Data++;
           strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
           strEncode += EncodeTable[((Tmp[1] & 0x03) << 4)];
           strEncode += "==";
       }
       else if (Mod == 2)
       {
           Tmp[1] = *Data++;
           Tmp[2] = *Data++;
           strEncode += EncodeTable[(Tmp[1] & 0xFC) >> 2];
           strEncode += EncodeTable[((Tmp[1] & 0x03) << 4) | ((Tmp[2] & 0xF0) >> 4)];
           strEncode += EncodeTable[((Tmp[2] & 0x0F) << 2)];
           strEncode += "=";
       }

       return strEncode;
    }

    static bool drawBoxes(cv::Mat& src, cv::Rect rect, string className, double confidence, int id)
    {
       cv::Scalar boxColor = cv::Scalar(0, 0, 255);
       cv::Scalar rectColor = cv::Scalar(0, 0, 255);
       cv::Scalar labelColor = cv::Scalar(0, 0, 0);
       //draw box
       cv::rectangle(src, rect, boxColor, 2, 1, 0);
       //draw label
       std::string label = cv::format("%.2f", confidence);
       label = to_string(id) + "-" + className + ":" + label;  //边框上的类别标签与置信度
       //label = className +":" + label;                       //边框上的类别标签与置信度
       int baseLine;
       cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
       int x0;
       if(rect.x < labelSize.height){
           x0 = labelSize.height;
       }
       else{
           x0 = rect.x;
       }
       cv::rectangle(src, cv::Point(x0,rect.y-round(1.5*labelSize.height)), cv::Point(x0+round(1.5*labelSize.width),(rect.y+baseLine)), rectColor, cv::FILLED);
       cv::putText(src, label, cv::Point(x0, rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.75, labelColor, 1);

       return true;
    }

    static bool wayPointCompare(WayPointInfo a, WayPointInfo b)
    {
       if(a.index <= b.index){
           return true;
       }
       return false;
    }

    //将mat类型数据转换为base64
    static std::string getBase64(cv::Mat dst, string code)
    {
        std::vector<uchar> img_encode;
        std::vector<int> vecCompression_params;
        vecCompression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        vecCompression_params.push_back(90);
        if(code == "jpg"){
          cv::imencode(".jpg", dst, img_encode, vecCompression_params);
        }
        else if(code == "png"){
          cv::imencode(".png", dst, img_encode, vecCompression_params);
        }
        else{
          cv::imencode(".jpg", dst, img_encode, vecCompression_params);       //默认采用jpg
        }
        string result = base64Encode(img_encode.data(), img_encode.size());
        return result;
    }

    //将mat类型数据转换为base64
    static std::string getBase64(cv::Mat dst)
    {
      std::vector<uchar> img_encode;
      std::vector<int> vecCompression_params;
      vecCompression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      vecCompression_params.push_back(90);
      cv::imencode(".jpg", dst, img_encode, vecCompression_params);       //默认采用jpg
      string result = base64Encode(img_encode.data(), img_encode.size());
      return result;
    }

    static std::string generateUUID()
    {
        char buf[GUID_LEN] = { 0 };
        uuid_t uu;
        uuid_generate( uu );
        int32_t index = 0;
        for (int32_t i = 0; i < 16; i++)
        {
            int32_t len = i < 15 ?
                sprintf(buf + index, "%02X-", uu[i]) :
                sprintf(buf + index, "%02X", uu[i]);
            if(len < 0 )
                return std::move(std::string(""));
            index += len;
        }
        return std::move(std::string(buf));
    }

    static void checkNetworkConnect(){
        qDebug() << "网络状态检查中...";
        string netWorkQualityCommand = "ping " + g_platform_ip.toStdString() + " -c 1 |grep -E 'time='| cut -f4 -d= | cut -f1 -dm | sed 's/ //g'";
        //获取网络延迟
        while(true){
            try{
                if(Tools::getCmdResult(netWorkQualityCommand) == ""){
                    qWarning("网络异常...");
                }
                else{
                    break;
                }
            }
            catch(...){
                    qWarning("网络异常......");
            }
            g_usleep(500000);
        }
        qDebug() << "网络正常";
    }
};
#endif // TOOLS_H
