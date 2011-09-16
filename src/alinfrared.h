/**
 * @author Raphael Leber
 * Copyright (c) Aldebaran Robotics 2010 All Rights Reserved.
 */

#ifndef INFRARED_ALINFRARED_H
#define INFRARED_ALINFRARED_H

#define NP1    "/tmp/irrecord_np1"
#define NP2    "/tmp/irrecord_np2"

#define TRUE 1
#define FALSE 0

#define MAX_BUF_SIZE  500

typedef enum IR_DATA_STATE {NONE=0, IP=10, IP1=11, IP2, IP3, IP4, UINT8=20, UINT8_1=21, UINT32=30, UINT32_1=31, UINT32_2=32, UINT32_3=33, UINT32_4=34} IR_DATA_STATE;

typedef enum LIRCD_PART_OF_MSG {LIRC_HEX=0, LIRC_REPEAT, LIRC_KEY, LIRC_REMOTE, LIRC_SIDE} LIRCD_PART_OF_MSG;

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>


namespace AL
{
  class ALBroker;
}

class ALInfrared : public AL::ALModule
{
protected:
  boost::shared_ptr<AL::ALMemoryProxy> fSTM;

  struct lirc_config *config1;

  std::string lircd1_sock;
  std::string nao2nao;

  int send(const std::string& remote, const std::string& key, int timeMs);


public:

  /**
   * Default Constructor.
   */
  ALInfrared(boost::shared_ptr<AL::ALBroker> pBroker, const std::string& pName );

  /**
   * Destructor.
   */
  virtual ~ALInfrared();


  /**
   * Function initReception
   * is called to initialyse receiving IR information from a remote control or a Nao
   * @param pRepeatThreshold : number which set the repetition threshold of buttons
   */
  void initReception(const int& pRepeatThreshold);

  /**
   * Function deinitReception
   * is called to stop receiving IR information
   */
  void deinitReception(void);

  /**
   * Function sendRemoteKey is used as an IR remote control.
   * @param pRemote : string containing the remote control name
   * @param pKey : string containing the button name
   */
  void sendRemoteKey(const std::string& pRemote, const std::string& pKey);

  /**
   * Function sendRemoteKeyWithTime is used as an IR remote control.
   * @param pRemote : string containing the remote control name
   * @param pKey : string containing the button name
   * @param pTimeMs : the time to send the reomte key in ms. 0 deals like sendRemoteKey.
   */
  void sendRemoteKeyWithTime(const std::string& pRemote, const std::string& pKey, const int& pTimeMs);


  /**
   * Function sendIpAddress is called to send an IP address
   * @param pIP : string containing an IP adress (eg "127.0.15.129")
   */
  void sendIpAddress(const std::string& pIP);

  /**
   * Function send8 is called to send 1 octet
   * @param pOctet : Integer containing an octet
   */
  void send8(const int& pOctet);

  /**
   * Function send32 is called to send 4 octets
   * @param pData_IR : string containing a 4 octet value (value in a string because there is no unsigned types)
   */
  void send32(const std::string& pData_IR);

  /**
   * Function send32 is called to send 4 octets
   * @param pOctet1 : 1st octet of the 32 bits value
   * @param pOctet2 : 2nd octet of the 32 bits value
   * @param pOctet3 : 3rd octet of the 32 bits value
   * @param pOctet4 : 4th octet of the 32 bits value
   */
  void send32(const int& pOctet1, const int& pOctet2, const int& pOctet3, const int& pOctet4);

  /**
   * Function confRemoteRecordStart is called when user start creating a new remote
   * @param pRm_name : string containing the name of the remote control to record
   */
  void confRemoteRecordStart(const std::string& pRm_name);

  /**
   * Function confRemoteRecordNext() is called when the user click on NEXT
   * @return string wich is the last message to display
   */
  std::string confRemoteRecordNext();

  /**
   * Function confRemoteRecordAddKey is called when user validate a new key name
   * @param pKeyname : string containing the name of the key to record or nothing if there is no more keys to record
   * @return string wich is the last message to display
   */
  std::string confRemoteRecordAddKey(const std::string& pKeyname);

  /**
   * Function confRemoteRecordGetStatus() is called during polling in order to update further information
   * @return string wich is the last message to display
   */
  std::string confRemoteRecordGetStatus();

  /**
   * Function confRemoteRecordCancel() is called when user want to cancel the record process
   * @return string wich is the last message to display
   */
  std::string confRemoteRecordCancel();

  /**
   * Function confRemoteRecordSave
   * Read in the remotetoset file to know which lirc configuration should be implemented in the lircd.conf file
   * Create the new lircd.conf file
   * Call the confUpdateRemoteConfig function
   */
  void confRemoteRecordSave(void);

  /**
   * Function confUpdateRemoteConfig
   * Check the PID of the 2 lirc daemon and send them a SIGHUP signal
   */
  void confUpdateRemoteConfig(void);

  /**
   * Function pipeIrrecordCommunicationThread()
   * Call the thread which manage remote recording
   */
  void pipeIrrecordCommunicationThread();

  /**
   * Function remoteControlThread()
   * Call the thread which manage remote reception
   */
  void remoteControlThread();

private:

  pthread_t rmctrlThreadId;
  pthread_t pipeThreadId;

  int lirc_lircd_rcv;

  char ready_to_get;

  int wrfd;

  int keyRepeatThreshold;

  std::string gMsg[50];
  std::string gMsgKey[50];
  int MsgCounter;

  bool irrecord_aborted;

  bool fIsExiting;

};


class ALInfraredTools : protected AL::ALModule
{
public:

  /**
   * int2str
   * @param n is a number to convert in string
   * @return a string with the converted value
   */
  std::string int2str (int n);

  /**
   * long2str
   * @param n is a number to convert in string
   * @return a string with the converted value
   */
  std::string long2str (long int n);

  /**
   * str2int
   * @param str is a string containing a number
   * @return an int with the converted value
   */
  int str2int (const std::string &str);

  /**
   * strhex2int
   * @param str is a string containing an hexa number
   * @return an int with the converted value
   */
  int strhex2int (const std::string &str);


  /**
   * fileExist
   * @param file is a string containing file name with path
   * @return true if file exists, false if not
   */
  bool fileExist(const std::string& file);
};
#endif  // INFRARED_ALINFRARED_H

