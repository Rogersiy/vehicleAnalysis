#include "global.h"
#include "qsettings.h"
#include "qfile.h"

LoopQueue<msg*> loopqueue(30);
LoopQueue<imgmsg*> loopqueueImage(30);
pthread_mutex_t myMutex = PTHREAD_MUTEX_INITIALIZER;  // 初始化互斥锁
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;  // 初始化条件变量
StreamShmMsg *g_shared_stream = NULL;
SemMsg *g_shared_sem = NULL;
DroneInfo *g_shared_drone;
WayPointMissionInfo g_wp_mission_info;
WayPointMissionExecuteInfo g_wp_mission_exe_info;
bool g_analysis = false;
int g_data_integrity_abnormal_count = 0;
int g_cache_size = 512;

QString g_serial_number = "NULL";
QString g_network_operator = "NULL";
QString g_mqtt_address = "NULL";
QString g_platform_address = "NULL";
QString g_platform_ip = "NULL";
QString g_platform_data_push_address = "NULL";
QString g_rtmp_client_output = "display";
QString g_rtmp_client_process_path = "NULL";
QString g_rtmp_client_address_source = "local";
QString g_rtmp_client_address = "NULL";
QString g_analysis_enable = "false";
QString g_analysis_algorithm_type = "personCar";
QString g_analysis_client_output = "display";
QString g_analysis_client_process_path = "NULL";
QString g_analysis_client_address_source = "local";
QString g_analysis_client_address = "NULL";


void App::readConfig()
{
    if (!checkConfig()) {
        return;
    }

    QSettings set(QCoreApplication::applicationDirPath() + "/config/config.ini", QSettings::IniFormat);
    set.beginGroup("BaseConfig");
    g_serial_number = set.value("serialNumber", g_serial_number).toString();
    g_network_operator = set.value("networkOperator", g_network_operator).toString();
    g_mqtt_address = set.value("mqttAddress", g_mqtt_address).toString();
    g_platform_address = set.value("platformAddress", g_platform_address).toString();
    g_platform_ip = set.value("platformIp", g_platform_ip).toString();
    g_platform_data_push_address = set.value("platformDataPushAddress", g_platform_data_push_address).toString();
    g_rtmp_client_output = set.value("rtmpClientOutput", g_rtmp_client_output).toString();
    g_rtmp_client_process_path = set.value("rtmpClientProcessPath", g_rtmp_client_process_path).toString();
    g_rtmp_client_address_source = set.value("rtmpClientAddressSource", g_rtmp_client_address_source).toString();
    g_rtmp_client_address = set.value("rtmpClientAddress", g_rtmp_client_address).toString();
    g_analysis_enable = set.value("analysisEnable", g_analysis_enable).toString();
    g_analysis_algorithm_type = set.value("analysisAlgorithmType", g_analysis_algorithm_type).toString();
    g_analysis_client_output = set.value("analysisClientOutput", g_analysis_client_output).toString();
    g_analysis_client_process_path = set.value("analysisClientProcessPath", g_analysis_client_process_path).toString();
    g_analysis_client_address_source = set.value("analysisClientAddressSource", g_analysis_client_address_source).toString();
    g_analysis_client_address = set.value("analysisClientAddress", g_analysis_client_address).toString();

    set.endGroup();

    //judge
    if(g_serial_number == "NULL"){
        qDebug() << "序列号格式错误!";
        exit(-1);
    }

    if(g_rtmp_client_output == "rtmp"){
        if(g_rtmp_client_address_source == "local"){
            if(g_rtmp_client_address == "NULL"){
                qDebug() << "本地配置RTMP推流地址格式错误!";
                exit(-1);
            }
        }
        else if(g_rtmp_client_address_source == "platform"){
            if(g_platform_address == "NULL"){
                qDebug() << "从平台获取RTMP推流地址,平台地址错误!";
                exit(-1);
            }
        }
        else{
            qDebug() << "RTMP推流地址格式错误!";
            exit(-1);
        }
    }

    if(g_mqtt_address == "NULL"){
        qDebug() << "mqtt address 格式错误!";
        exit(-1);
    }

    qDebug() << "读取配置文件成功!";
}

void App::writeConfig()
{
    QSettings set(QCoreApplication::applicationDirPath() + "/config/config.ini", QSettings::IniFormat);
    set.beginGroup("BaseConfig");
    set.setValue("serialNumber", g_serial_number);
    set.setValue("networkOperator", g_network_operator);
    set.setValue("mqttAddress", g_mqtt_address);
    set.setValue("platformAddress", g_platform_address);
    set.setValue("platformIp", g_platform_ip);
    set.setValue("platformDataPushAddress", g_platform_data_push_address);
    set.setValue("rtmpClientOutput", g_rtmp_client_output);
    set.setValue("rtmpClientProcessPath", g_rtmp_client_process_path);
    set.setValue("rtmpClientAddressSource", g_rtmp_client_address_source);
    set.setValue("rtmpClientAddress", g_rtmp_client_address);
    set.setValue("analysisEnable", g_analysis_enable);
    set.setValue("analysisAlgorithmType", g_analysis_algorithm_type);
    set.setValue("analysisClientOutput", g_analysis_client_output);
    set.setValue("analysisClientProcessPath", g_analysis_client_process_path);
    set.setValue("analysisClientAddressSource", g_analysis_client_address_source);
    set.setValue("analysisClientAddress", g_analysis_client_address);

    set.endGroup();
    qDebug() << "写入配置文件成功!";
}

bool App::checkConfig()
{
    //如果配置文件大小为0,则以初始值继续运行,并生成配置文件
    QFile file(QCoreApplication::applicationDirPath() + "/config/config.ini");
    if (file.size() == 0) {
        qWarning() << "配置文件不存在!";
        writeConfig();
        return false;
    }

    //如果配置文件不完整,则以初始值继续运行,并生成配置文件
    if (file.open(QFile::ReadOnly)) {
        bool ok = true;
        while (!file.atEnd()) {
            QString line = file.readLine();
            line = line.replace("\r", "");
            line = line.replace("\n", "");
            QStringList list = line.split("=");

            if (list.count() == 2) {
                if (list.at(1) == "") {
                    ok = false;
                    break;
                }
            }
        }

        if (!ok) {
            writeConfig();
            return false;
        }
    } else {
        writeConfig();
        return false;
    }

    return true;
}
