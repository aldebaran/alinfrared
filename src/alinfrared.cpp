/**
 * @author Raphael Leber
 * Copyright (c) Aldebaran Robotics 2010, 2011 All Rights Reserved.
 */

#include <alcommon/alproxy.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <pthread.h>
#include <signal.h>
#include <cmath>
#include <iostream>
#include <string.h>
#include <sstream>
#include <alproxies/almemoryproxy.h>

#include <qi/log.hpp>
#include <qi/os.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>

#include <fstream>
//#include <pstreams/pstream.h>
#include <dirent.h>

#include "alinfrared.h"
#include <lirc/lirc_client.h>
#include "paths.h"

#if defined (__linux__)
#include <sys/prctl.h>
#endif

/**************************** Variables in ALMemory for IR received ***************************/

static const std::string& ALMEMORY_S_Remote     = "Device/SubDeviceList/IR/LIRC/Remote/Remote/Sensor/Value/";        //string  - Data
static const std::string& ALMEMORY_S_Key        = "Device/SubDeviceList/IR/LIRC/Remote/Key/Sensor/Value/";           //string  - Data
static const std::string& ALMEMORY_S_LircCode   = "Device/SubDeviceList/IR/LIRC/Remote/LircCode/Sensor/Value/";      //string  - Data
static const std::string& ALMEMORY_S_IrSide     = "Device/SubDeviceList/IR/LIRC/Remote/IrSide/Sensor/Value/";        //int     - Data
static const std::string& ALMEMORY_S_Repeat     = "Device/SubDeviceList/IR/LIRC/Remote/Repeat/Sensor/Value/";        //int     - Data
static const std::string& ALMEMORY_Remote_Event = "InfraRedRemoteKeyReceived";                                       //array   - Event

static const std::string& ALMEMORY_S_IP         = "Device/SubDeviceList/IR/LIRC/Data/IP/Sensor/Value/";              //string  - Data
static const std::string& ALMEMORY_IP_Event     = "InfraRedIpAdressReceived";                                        //string  - Event

static const std::string& ALMEMORY_S_uInt8      = "Device/SubDeviceList/IR/LIRC/Data/uInt8/Byte/Sensor/Value/";      //int     - Data
static const std::string& ALMEMORY_uInt8_Event  = "InfraRedOneByteReceived";                                         //int     - Event

static const std::string& ALMEMORY_S_uInt32_1   = "Device/SubDeviceList/IR/LIRC/Data/uInt32/Byte1/Sensor/Value/";    //int     - Data
static const std::string& ALMEMORY_S_uInt32_2   = "Device/SubDeviceList/IR/LIRC/Data/uInt32/Byte2/Sensor/Value/";    //int     - Data
static const std::string& ALMEMORY_S_uInt32_3   = "Device/SubDeviceList/IR/LIRC/Data/uInt32/Byte3/Sensor/Value/";    //int     - Data
static const std::string& ALMEMORY_S_uInt32_4   = "Device/SubDeviceList/IR/LIRC/Data/uInt32/Byte4/Sensor/Value/";    //int     - Data
static const std::string& ALMEMORY_uInt32_Event = "InfraRedFourBytesReceived";                                       //array   - Event

static const std::string& ALMEMORY_IrSide_Event = "InfraRedSide";                                                    //int     - Event


/*************************** Variables in ALMemory for IR to send *****************************/

static const std::string& ALMEMORY_A_Remote   ="Device/SubDeviceList/IR/LIRC/Remote/Remote/Actuator/Value/";      //string  - Data
static const std::string& ALMEMORY_A_Key      ="Device/SubDeviceList/IR/LIRC/Remote/Key/Actuator/Value/";         //string  - Data
static const std::string& ALMEMORY_A_Repeat   ="Device/SubDeviceList/IR/LIRC/Remote/Repeat/Actuator/Value/";      //int     - Data

static const std::string& ALMEMORY_A_IP       ="Device/SubDeviceList/IR/LIRC/Data/IP/Actuator/Value/";            //int     - Data
static const std::string& ALMEMORY_A_uInt8    ="Device/SubDeviceList/IR/LIRC/Data/uInt8/Byte/Actuator/Value/";    //int     - Data
static const std::string& ALMEMORY_A_uInt32_1 ="Device/SubDeviceList/IR/LIRC/Data/uInt32/Byte1/Actuator/Value/";  //int     - Data
static const std::string& ALMEMORY_A_uInt32_2 ="Device/SubDeviceList/IR/LIRC/Data/uInt32/Byte2/Actuator/Value/";  //int     - Data
static const std::string& ALMEMORY_A_uInt32_3 ="Device/SubDeviceList/IR/LIRC/Data/uInt32/Byte3/Actuator/Value/";  //int     - Data
static const std::string& ALMEMORY_A_uInt32_4 ="Device/SubDeviceList/IR/LIRC/Data/uInt32/Byte4/Actuator/Value/";  //int     - Data


using namespace std;
using namespace AL;

static string int2str (int n)
{
  stringstream ss;
  ss << n;
  return ss.str();
}

static string long2str (long int n)
{
  stringstream ss;
  ss << n;
  return ss.str();
}

static int str2int (const string &str)
{
  stringstream ss(str);
  int n;
  ss >> n;
  return n;
}

static int strhex2int (const string &str)
{
  stringstream ss(str);
  int n;
  ss >> std::hex >> n;
  return n;
}

static bool fileExist (const string& file)
{
  bool isOpen;
  ifstream f;
  f.open(file.c_str());
  isOpen = f.is_open();
  f.close();
  return isOpen;
}

/**********************************************************************************************/
/************************* THREAD to receive from remote control ******************************/
/**********************************************************************************************/

void * rmctrlThread(void * ptr)
{
  ((ALInfrared*)ptr)->remoteControlThread();
  return NULL;
}

void ALInfrared::remoteControlThread()
{
#if defined (__linux__)
  // thread name
  prctl(PR_SET_NAME, "remoteControlThread", 0, 0, 0);
#endif

  std::string IP_adress;
  std::string sn_cmp;
  int uInt8_1;
  int uInt32_1;
  int uInt32_2;
  int uInt32_3;
  int uInt32_4;

  char *code;
  int repeat_code_num;

  char IR_data_state = NONE;


  while(!fIsExiting)
  {
    for (int i=0; i<20;  i++)
    {
      usleep(10000);
      if (fIsExiting)
      {
        return;
      }
    }

    while(ready_to_get)
    {
      qiLogInfo("hardware.alinfrared") << "remoteControlThread(): " << "Ready to get remote controls keys." << std::endl;

      usleep(50000);

      lirc_readconfig(NULL,&config1,NULL);

      if(ready_to_get)
      {
        while((lirc_nextcode(&code)==0) && ready_to_get)
        {
          if(code!=NULL)
          {
            const std::string s = code;
            unsigned int found;
            unsigned int prevfound = 0;
            unsigned char index_found = 0;
            string RemoteValues[5];

            found=s.find_first_of(" ");
            while ((found != string::npos) && (index_found < 5))
            {
              RemoteValues[index_found++].assign(s, prevfound, (found - prevfound));
              prevfound = found + 1;
              found = s.find_first_of(" ", found + 1);
            }
            RemoteValues[index_found].assign(s,prevfound,1);


            /***********************************************/
            /** Check if an other Nao try to communicate ***/
            /***********************************************/
            if(!RemoteValues[LIRC_REMOTE].compare("nao2nao"))
            {

              /***********************************************/
              /******** State Machine to get an IP  **********/
              /***********************************************/
              if(!RemoteValues[LIRC_KEY].compare("val_IP"))
              {
                IR_data_state = IP;
              }
              else if(IR_data_state == IP)
              {
                IR_data_state=IP1;
                IP_adress = RemoteValues[LIRC_KEY].substr(4);
              }
              else if(IR_data_state == IP1)
              {
                IR_data_state=IP2;
                IP_adress += "." + RemoteValues[LIRC_KEY].substr(4);
              }
              else if(IR_data_state == IP2)
              {
                IR_data_state=IP3;
                IP_adress += "." + RemoteValues[LIRC_KEY].substr(4);
              }
              else if(IR_data_state == IP3)
              {
                IR_data_state=IP4;
                IP_adress += "." + RemoteValues[LIRC_KEY].substr(4);
                fSTM->insertData(ALMEMORY_S_IP, IP_adress);
                fSTM->raiseEvent(ALMEMORY_IP_Event, IP_adress);
              }

              /***********************************************/
              /**** State Machine to get one byte of data ****/
              /***********************************************/
              if(!RemoteValues[LIRC_KEY].compare("val_UINT8"))
              {
                IR_data_state = UINT8;
              }
              else if(IR_data_state == UINT8)
              {
                IR_data_state = UINT8_1;
                uInt8_1 = str2int(RemoteValues[LIRC_KEY].substr(4));
                fSTM->insertData(ALMEMORY_S_uInt8, uInt8_1);
                fSTM->raiseEvent(ALMEMORY_uInt8_Event, uInt8_1);
              }

              /***********************************************/
              /***  State Machine to get 4 bytes of data  ****/
              /***********************************************/
              if(!RemoteValues[LIRC_KEY].compare("val_UINT32"))
              {
                IR_data_state = UINT32;
              }
              else if(IR_data_state == UINT32)
              {
                IR_data_state = UINT32_1;
                uInt32_1 = str2int(RemoteValues[LIRC_KEY].substr(4));
              }
              else if(IR_data_state == UINT32_1)
              {
                IR_data_state = UINT32_2;
                uInt32_2 = str2int(RemoteValues[LIRC_KEY].substr(4));
              }
              else if(IR_data_state == UINT32_2)
              {
                IR_data_state = UINT32_3;
                uInt32_3 = str2int(RemoteValues[LIRC_KEY].substr(4));
              }
              else if(IR_data_state == UINT32_3)
              {
                IR_data_state = UINT32_4;
                uInt32_4 = str2int(RemoteValues[LIRC_KEY].substr(4));
                fSTM->insertData(ALMEMORY_S_uInt32_1, uInt32_1);
                fSTM->insertData(ALMEMORY_S_uInt32_2, uInt32_2);
                fSTM->insertData(ALMEMORY_S_uInt32_3, uInt32_3);
                fSTM->insertData(ALMEMORY_S_uInt32_4, uInt32_4);

                AL::ALValue uInt32Event;
                uInt32Event.arrayPush(uInt32_1);
                uInt32Event.arrayPush(uInt32_2);
                uInt32Event.arrayPush(uInt32_3);
                uInt32Event.arrayPush(uInt32_4);
                fSTM->raiseEvent(ALMEMORY_uInt32_Event, uInt32Event);
              }


            }
            /***********************************************/
            /** If it's not Nao, it's a remote control *****/
            /***********************************************/
            else
            {
              IR_data_state = NONE;

              repeat_code_num = strhex2int(RemoteValues[LIRC_REPEAT]);

              //Avoid multi-detect of one key until keyRepeatThreshold
              if((abs(RemoteValues[LIRC_HEX].compare(sn_cmp))) || (repeat_code_num == 0) || (repeat_code_num >= keyRepeatThreshold))
              {
                sn_cmp = RemoteValues[LIRC_HEX];

                AL::ALValue RemoteEvent;
                RemoteEvent.arrayPush(RemoteValues[LIRC_HEX]);
                RemoteEvent.arrayPush(strhex2int(RemoteValues[LIRC_REPEAT]));
                RemoteEvent.arrayPush(RemoteValues[LIRC_KEY]);
                RemoteEvent.arrayPush(RemoteValues[LIRC_REMOTE]);
                RemoteEvent.arrayPush(str2int(RemoteValues[LIRC_SIDE]));
                fSTM->raiseEvent(ALMEMORY_Remote_Event, RemoteEvent);

                fSTM->insertData(ALMEMORY_S_LircCode, RemoteValues[LIRC_HEX]);
                fSTM->insertData(ALMEMORY_S_Repeat, RemoteValues[LIRC_REPEAT]);
                fSTM->insertData(ALMEMORY_S_Key, RemoteValues[LIRC_KEY]);
                fSTM->insertData(ALMEMORY_S_Remote, RemoteValues[LIRC_REMOTE]);
                fSTM->insertData(ALMEMORY_S_IrSide, RemoteValues[LIRC_SIDE]);

                qiLogInfo("hardware.alinfrared") << "remoteControlThread(): " << "Got a key: " << s << std::endl;
              }
            }

            fSTM->raiseEvent(ALMEMORY_IrSide_Event, str2int(RemoteValues[LIRC_SIDE]));

            free(code);
          }
        }
        lirc_freeconfig(config1);
      }

      if(lirc_lircd_rcv!=-1)
      {
        lirc_deinit();
        lirc_lircd_rcv=-1;
      }
    }
  }

}

/**********************************************************************************************/
/********************** RECEIVE IR ************************************************************/
/**********************************************************************************************/

void ALInfrared::initReception(const int& pRepeatThreshold)
{
  char proglirc[] = "naoqi";

  if(pRepeatThreshold > -1)
    keyRepeatThreshold = pRepeatThreshold;

  if(lirc_lircd_rcv == -1)
    lirc_lircd_rcv = lirc_init(proglirc, 1);

  if(ready_to_get == FALSE)
    ready_to_get = TRUE;
}


void ALInfrared::deinitReception(void)
{
  if(lirc_lircd_rcv != -1)
  {
    lirc_deinit();
    lirc_lircd_rcv = -1;
  }

  if(ready_to_get == TRUE)
    ready_to_get = FALSE;
}


/**********************************************************************************************/
/********************** SEND IR ***************************************************************/
/**********************************************************************************************/

void ALInfrared::sendRemoteKey(const std::string& pRemote, const std::string& pKey)
{
  ALInfrared::sendRemoteKeyWithTime(pRemote, pKey, 0);
}


void ALInfrared::sendRemoteKeyWithTime(const std::string& pRemote, const std::string& pKey, const int& timeMs)
{
  int sendSuccess;

  sendSuccess = send(pRemote, pKey, timeMs);

  if(sendSuccess == -1)
    throw ALERROR( getName(), "sendRemoteKey", std::string( "Error: The remote '") + pRemote
                                                + "' or/and the key '" + pKey
                                                + "' is/are not available. "
                                                + "Please check names in Nao web page.");

  fSTM->insertData(ALMEMORY_A_Remote, pRemote);
  fSTM->insertData(ALMEMORY_A_Key, pKey);
  qiLogInfo("hardware.alinfrared") << "sendRemoteKey(): " << "Remote = " << pRemote <<" / Key = " << pKey << std::endl;
}

void ALInfrared::sendIpAddress(const std::string& pIP)
{
  const std::string& prefix = "val_";
  const std::string& header = "IP";

  string IPValues[5];
  unsigned int found=0;
  unsigned int prevfound=0;
  unsigned int index_found=0;

  (void)send(nao2nao, (prefix + header), 0);

  found=pIP.find_first_of(".\0");
  while ((found!=string::npos) && (index_found<4))
  {
    IPValues[index_found].assign(pIP, prevfound, found-prevfound);
    usleep(400000);
    (void)send(nao2nao, (prefix + IPValues[index_found++]), 0);
    prevfound = found+1;
    found=pIP.find_first_of(".\0", found+1);
  }

  usleep(400000);
  IPValues[index_found].assign(pIP, prevfound, found-prevfound);
  (void)send(nao2nao, (prefix + IPValues[index_found]), 0);

  fSTM->insertData(ALMEMORY_A_IP, pIP);
  qiLogInfo("hardware.alinfrared") << "sendIpAddress(): " << "IP address = " << pIP  << std::endl;

}


void ALInfrared::send8(const int& pOctet)
{
  const std::string& prefix = "val_";
  const std::string& header = "UINT8";

  if((pOctet<0) || (pOctet>255))
    throw ALERROR( getName(), "send8", std::string( "Error: The input should be a number between 0 and 255."));
  else
  {
    (void)send(nao2nao, (prefix + header), 0);
    usleep(400000);
    (void)send(nao2nao, (prefix + int2str(pOctet&0xFF)), 0);

    fSTM->insertData(ALMEMORY_A_uInt8, pOctet);
    qiLogInfo("hardware.alinfrared") << "send8(): " << "uInt8 value = " << pOctet  << std::endl;
  }
}


void ALInfrared::send32(const std::string& pData_IR)
{
  const std::string& prefix = "val_";
  const std::string& header = "UINT32";

  unsigned long long n;
  stringstream ss(pData_IR);
  ss >> n;

  qiLogDebug("hardware.alinfrared") << "send32(): " << "************ pData_IR = " << pData_IR  << std::endl;
  qiLogDebug("hardware.alinfrared") << "send32(): " << "************ n = " << n  << std::endl;

  if((n<0) || (n>4294967295u))
    throw ALERROR( getName(), "send32", std::string( "Error: The input should be a number between 0 and 4294967295 (2147483647 with choregraphe)."));
  else
  {
    unsigned char byte1, byte2, byte3, byte4;
    byte4 = (n>>24) & 0xFF;
    byte3 = (n>>16) & 0xFF;
    byte2 = (n>>8) & 0xFF;
    byte1 = (n) & 0xFF;

    (void)send(nao2nao, (prefix + header), 0);
    usleep(400000);

    (void)send(nao2nao, (prefix + int2str(byte1)), 0);
    usleep(400000);
    (void)send(nao2nao, (prefix + int2str(byte2)), 0);
    usleep(400000);
    (void)send(nao2nao, (prefix + int2str(byte3)), 0);
    usleep(400000);
    (void)send(nao2nao, (prefix + int2str(byte4)), 0);

    fSTM->insertData(ALMEMORY_A_uInt32_4, (int)(byte4));
    fSTM->insertData(ALMEMORY_A_uInt32_3, (int)(byte3));
    fSTM->insertData(ALMEMORY_A_uInt32_2, (int)(byte2));
    fSTM->insertData(ALMEMORY_A_uInt32_1, (int)(byte1));

    qiLogInfo("hardware.alinfrared") << "send32(): " << "uInt32 value = " << pData_IR << std::endl;
    qiLogInfo("hardware.alinfrared") << "send32(): " << "uInt32 octets = "  << (int)byte4 << " "
                                                                  << (int)byte3 << " "
                                                                  << (int)byte2 << " "
                                                                  << (int)byte1 << " "  << std::endl;
  }
}


void ALInfrared::send32(const int& pOctet1, const int& pOctet2, const int& pOctet3, const int& pOctet4)
{
  const std::string& prefix = "val_";
  const std::string& header = "UINT32";

  if(((pOctet4<0) || (pOctet4>255)) && ((pOctet3<0) || (pOctet3>255))
                                    && ((pOctet2<0) || (pOctet2>255))
                                    && ((pOctet1<0) || (pOctet1>255)))
  {
    throw ALERROR( getName(), "send32", std::string( "Error: Every bytes have to be between 0 and 255."));
  }
  else
  {
    (void)send(nao2nao.c_str(), (prefix + header).c_str(),0 );

    usleep(400000);
    (void)send(nao2nao, (prefix + int2str(pOctet1 & 0xFF)), 0);
    usleep(400000);
    (void)send(nao2nao, (prefix + int2str(pOctet2 & 0xFF)), 0);
    usleep(400000);
    (void)send(nao2nao, (prefix + int2str(pOctet3 & 0xFF)), 0);
    usleep(400000);
    (void)send(nao2nao, (prefix + int2str(pOctet4 & 0xFF)), 0);

    fSTM->insertData(ALMEMORY_A_uInt32_4, pOctet4);
    fSTM->insertData(ALMEMORY_A_uInt32_3, pOctet3);
    fSTM->insertData(ALMEMORY_A_uInt32_2, pOctet2);
    fSTM->insertData(ALMEMORY_A_uInt32_1, pOctet1);

    qiLogInfo("hardware.alinfrared") << "send32(): " << "uInt32 octets = " << pOctet4 << " "
                                                                 << pOctet3 << " "
                                                                 << pOctet2 << " "
                                                                 << pOctet1 << " "  << std::endl;
  }

}

int ALInfrared::send(const std::string& remote, const std::string& key, int timeMs)
{
  int ret;

  ret = lirc_send_key_with_time(lircd1_sock.c_str(), remote.c_str(), key.c_str(), timeMs);

  return ret;
}

void ALInfrared::confUpdateRemoteConfig(void)
{
  std::ifstream f, f1;
  char line[8];

  f.open(LIRCD_PID);
  f.get(line, 7);
  f.close();

  if(kill((pid_t)atoi(line),SIGHUP)==0) // Send HUP signal to the first lirc daemon and check if successful
    qiLogDebug("hardware.alinfrared") << "confUpdateRemoteConfig(): " << "lircd SIGHUP successful" << std::endl;
  else
    qiLogError("hardware.alinfrared") << "confUpdateRemoteConfig(): " << "lircd SIGHUP error" << std::endl;


  f.open(LIRCD_SEND_PID);
  f.get(line, 7);
  f.close();

  if(kill((pid_t)atoi(line),SIGHUP)==0) // Send HUP signal to the second lirc daemon and check if successful
    qiLogDebug("hardware.alinfrared") << "confUpdateRemoteConfig(): " << "lircd1 SIGHUP successful" << std::endl;
  else
    qiLogError("hardware.alinfrared") << "confUpdateRemoteConfig(): " << "lircd1 SIGHUP error" << std::endl;
}


void ALInfrared::confRemoteRecordSave(void)
{
  std::ifstream fref,fi;
  std::fstream fo,frefc;
  char rname[30];
  char pc[70];

  qiLogDebug("hardware.alinfrared") << "confRemoteRecordSave(): " << "Try to update lircd.conf" << std::endl;

  fref.open(REMOTETOSET);
  if(!fref.is_open()) // if the file do not exist, create it
  {
    frefc.open(REMOTETOSET,ios::out | ios::in | ios::trunc);
    frefc << flush;
    frefc.close();
    fref.open(REMOTETOSET);
  }

  fo.open(LIRCDCONF, ios::trunc | ios::out);

  fi.open(NAO2NAOCONF);
  if(fi.good()) fo << fi.rdbuf();
  fi.close();

  while(fref.good())
  {
    fref.getline(rname,29);
    strcpy(pc, REMOTEFOLDER);
    strcat(pc, rname);
    qiLogDebug("hardware.alinfrared") << "confRemoteRecordSave(): " << pc << std::endl;
    fi.open(pc);
    if(fi.good()) fo << fi.rdbuf();
    fi.close();
  }

  fo.close();
  fref.close();

  confUpdateRemoteConfig();
}


/**********************************************************************************************/
/************************** OTHER FUNCTIONS ***************************************************/
/**********************************************************************************************/

ALInfrared::ALInfrared(boost::shared_ptr<AL::ALBroker> broker, const std::string& name ): AL::ALModule(broker, name )
{
  fIsExiting = false;
  rmctrlThreadId = 0;
  pipeThreadId = 0;
  lirc_lircd_rcv = -1;
  ready_to_get = FALSE;
  keyRepeatThreshold = 10;
  MsgCounter = 0;
  irrecord_aborted = false;
  lircd1_sock = LIRCD_SEND_SOCK;
  nao2nao = NAO2NAO;

  setModuleDescription( "This module works with Linux Infrared Remote Control (LIRC) "
                        "in order to emit/receive IR remotes keys "
                        "or to emit/receive IR information to/from an other Nao.");

  functionName("initReception", "ALInfrared", "Init IR reception (connect as a client to the LIRC daemon).");
  addParam( "RepeatThreshold", "Give the keep-pressing threshold after which the repetition of a key is taken into consideration." );
  BIND_METHOD( ALInfrared::initReception  );

  //Add in futur release (To call in "onStop" in Choregraphe)
  //functionName("deinitReception", "ALInfrared", "Stop IR reception (disconnect client to the LIRC daemon).");
  //BIND_METHOD( ALInfrared::deinitReception  );

  functionName("sendRemoteKey", "ALInfrared", "Simulate a remote control (Nao as a remote control).");
  addParam( "Remote", "IR remote control name.");
  addParam( "Key", "IR remote control key name.");
  BIND_METHOD(ALInfrared::sendRemoteKey);



  functionName("sendRemoteKeyWithTime", "ALInfrared", "Simulate a remote control (Nao as a remote control).");
  addParam( "Remote", "IR remote control name.");
  addParam( "Key", "IR remote control key name.");
  addParam( "pTimeMs", "The time in ms when the remote key must be send. 0 deals like sendRemoteKey");
  BIND_METHOD(ALInfrared::sendRemoteKeyWithTime);


  functionName("sendIpAddress", "ALInfrared", "Send an IP by IR.");
  addParam( "IP", "IP adress to send through IR.");
  BIND_METHOD(ALInfrared::sendIpAddress);

  functionName("send8", "ALInfrared", "Send 1 octet by IR.");
  addParam( "Octet", "octet to send through IR.");
  BIND_METHOD(ALInfrared::send8);

  functionName("send32", "ALInfrared", "Send 4 octets by IR.");
  addParam( "Data_IR", "4 octets to send through IR.");
  completeAndCheck<ALInfrared, const std::string&, void> (&ALInfrared::send32, getCurrentMethodDescription());
  bindMethodOverload(createFunctor<ALInfrared, std::string, void>(this, &ALInfrared::send32));


  functionName("send32", "ALInfrared", "Send 4 octets by IR.");
  addParam( "Octet1", "Octet 1 to send through IR.");
  addParam( "Octet2", "Octet 2 to send through IR.");
  addParam( "Octet3", "Octet 3 to send through IR.");
  addParam( "Octet4", "Octet 4 to send through IR.");
  completeAndCheck<ALInfrared, const int&, const int&, const int&, const int&, void> (&ALInfrared::send32, getCurrentMethodDescription());
  bindMethodOverload(createFunctor<ALInfrared, int, int, int, int, void> (this, &ALInfrared::send32));

  try {
    fSTM = getParentBroker()->getMemoryProxy();
  } catch(ALError& e) {
    qiLogError("hardware.alinfrared") << "IR could not connect to Memory. Error : " << e.what() << endl;
  }
  fSTM->declareEvent(ALMEMORY_Remote_Event);
  fSTM->declareEvent(ALMEMORY_uInt8_Event);
  fSTM->declareEvent(ALMEMORY_IP_Event);
  fSTM->declareEvent(ALMEMORY_uInt32_Event);

  fSTM->declareEvent(ALMEMORY_IrSide_Event);

  pthread_create(&rmctrlThreadId, NULL, rmctrlThread, (void *)this);

}

ALInfrared::~ALInfrared()
{
  fIsExiting = true;
  usleep(100000); // Worst case

  // A try block
  try {
    if(lirc_lircd_rcv!=-1) lirc_deinit();
  }
  catch (ALError& e) {
    qiLogError("hardware.alinfrared") << "Exception while disposing Lirc: " << e.what() << std::endl;
  }
}


