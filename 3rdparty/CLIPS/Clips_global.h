#ifndef CLIPS_GLOBAL_H
#define CLIPS_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(CLIPS_LIBRARY)
#  define CLIPS_EXPORT Q_DECL_EXPORT
#else
#  define CLIPS_EXPORT Q_DECL_IMPORT
#endif

#endif // CLIPS_GLOBAL_H
