#ifndef QEMSIMPLIFIERUTILS_CONFIG_H_
#define QEMSIMPLIFIERUTILS_CONFIG_H_

// This gets set by CMake via add_definitions(-DQEM_USE_METAL)
#if defined(QEM_USE_METAL)
  #define QEM_BACKEND_METAL
#elif defined(QEM_USE_OPENCL)
  #define QEM_BACKEND_OPENCL
#else
  #define QEM_BACKEND_CPU
#endif

#endif // QEMSIMPLIFIERUTILS_CONFIG_H_