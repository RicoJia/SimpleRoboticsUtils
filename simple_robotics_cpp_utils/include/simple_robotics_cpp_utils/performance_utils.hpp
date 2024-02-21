#pragma once
#include <sys/resource.h>

long get_memory_usage() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_maxrss; // in kb
}