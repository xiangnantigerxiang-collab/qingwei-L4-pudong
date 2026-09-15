#ifndef HDMAP_EXPORT_H
#define HDMAP_EXPORT_H

#if defined(__GNUC__)
#define HDMAP_API __attribute__((visibility("default")))
#else
#define HDMAP_API
#endif

#endif
