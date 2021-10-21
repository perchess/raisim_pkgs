#include "dynlibclass.h"
#include <iostream>

using namespace std;

extern "C" LibHandler* create_object()
{
  return new LibHandler;
}

extern "C" void destroy_object( LibHandler* object )
{
  delete object;
}

LibHandler::LibHandler()
{
  x = 20;
}

void LibHandler::DoSomething()
{
//  cout<<x<<endl;
}
