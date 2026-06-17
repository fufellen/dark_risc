#ifndef LWIPDEMO_ARCH_STRING_H
#define LWIPDEMO_ARCH_STRING_H

char *memcpy(void *dptr, const void *sptr, int len);
char *memcmp(const void *dptr, const void *sptr, int len);
char *memset(void *dptr, int c, int len);
int strlen(const char *s1);

#endif
