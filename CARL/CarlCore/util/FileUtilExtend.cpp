#include "FileUtilExtend.h"


std::string cFileUtilExtend::GetCurrentTimeStamp()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);

  return std::string(buffer);
}
