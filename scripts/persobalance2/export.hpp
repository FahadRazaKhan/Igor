#ifndef EXPORT_HPP
#define EXPORT_HPP

#ifdef WIN32
#define EXPORT __declspec( dllexport )
#else
#define EXPORT
#endif

#endif
