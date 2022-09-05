#include "Level.h"

namespace SlimeVR
{
  namespace Logging
  {
    const char *levelToString(Level level)
    {
      switch (level)
      {
      case TRACE:
        return "追踪";
      case DEBUG:
        return "调试";
      case INFO:
        return "信息";
      case WARN:
        return "警告";
      case ERROR:
        return "错误";
      case FATAL:
        return "致命";
      default:
        return "未知";
      }
    }
  }
}
