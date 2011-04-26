/**
 *
 * \section Author
 * @author Raphael Leber
 */

#include <signal.h>
#include <alcore/alptr.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

#include "alinfrared.h"



#ifdef INFRARED_IS_REMOTE_OFF

#ifdef _WIN32
    #define ALCALL __declspec(dllexport)
#else
    #define ALCALL
#endif
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

