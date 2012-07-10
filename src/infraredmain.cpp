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

#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

#include "alinfrared.h"


#if defined(INFRARED_IS_REMOTE_OFF) && defined(_WIN32)
    #define ALCALL __declspec(dllexport)
#else
    #define ALCALL
#endif

extern "C"
{
ALCALL int _createModule( boost::shared_ptr<AL::ALBroker> pBroker )
{
  // init broker with the main broker inctance
  // from the parent executable
  AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
  AL::ALBrokerManager::getInstance()->addBroker(pBroker);
  AL::ALModule::createModule<ALInfrared>( pBroker, "ALInfrared" );
  return 0;
}

ALCALL int _closeModule(  )
{
  return 0;
}
}

#ifdef INFRARED_IS_REMOTE_ON

int main( int argc, char *argv[] )
{
  // pointer to createModule
  TMainType sig;
  sig = &_createModule;
  // call main
  ALTools::mainFunction("Infrared",argc, argv,sig);
}
#endif

