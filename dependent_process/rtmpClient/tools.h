#ifndef TOOLS_H
#define TOOLS_H
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <curl/curl.h>
#include <uuid/uuid.h>
using namespace std;

class Tools {
public:
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
};
#endif // TOOLS_H
