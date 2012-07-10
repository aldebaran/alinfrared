/*
 *
 * Copyright (C) 2012 Aldebaran Robotics
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 *
 * This module manages several LIRC-related files.
 *
 * Remotes configuration files:
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * You can find every remote configuration files in "/home/nao/remotes/config/".
 *
 * Remotes configuration activated:
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * To activate or deactivate a remote, you can add or remove its name in the
 * file "/home/nao/remotes/remotetoset".
 *
 * LIRC configuration file:
 * ^^^^^^^^^^^^^^^^^^^^^^^^
 * The LIRC configuration file "/home/nao/remotes/lircd.conf" contains the
 * configuration of every remotes activated.
 * This file will be updated and loaded by the LIRC daemon every time you click
 * on Save in the NAO web page (Settings), or every time you call the function
 * :ref:`alinfrared-api::confRemoteRecordSave`.
 *
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
#include <alproxies/almemoryproxy.h>

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


#endif  // INFRARED_ALINFRARED_H

