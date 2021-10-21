#pragma once

class LibHandler
{
public:
  LibHandler();
  virtual ~LibHandler();

  /* use virtual otherwise linker will try to perform static linkage */
  virtual void DoSomething();

private:
  int x;
};
