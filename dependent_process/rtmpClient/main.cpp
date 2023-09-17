#include <QDebug>
#include "rtmpstream.h"
#include "tools.h"

int main(int argc, char *argv[])
{
    qDebug() << "程序启动中...";
    string outPut;
    string rtmpAddress;
    //参数校验
    if(argc == 2 && string(argv[1]) == "display"){
        outPut = argv[1];
    }
    else if(argc == 3 && string(argv[1]) == "rtmp"){
        outPut = argv[1];
        rtmpAddress = argv[2];
    }
    else{
        qDebug() << "参数不合法";
        return 0;
    }
    RtmpStream* rtmpStream_t = new RtmpStream(outPut, rtmpAddress);
    rtmpStream_t->initPipeline();

    return 0;
}
