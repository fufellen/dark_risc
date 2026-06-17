#include <stddef.h>

void *memmove(void *dst, const void *src, size_t len)
{
    unsigned char *d = (unsigned char *)dst;
    const unsigned char *s = (const unsigned char *)src;

    if ((d == s) || (len == 0)) {
        return dst;
    }

    if (d < s) {
        for (size_t i = 0; i < len; i++) {
            d[i] = s[i];
        }
    } else {
        while (len) {
            len--;
            d[len] = s[len];
        }
    }

    return dst;
}
