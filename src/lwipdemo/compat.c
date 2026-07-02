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

#ifdef LIDARSIM_DIAG_BEACON
/* Minimal snprintf stub for mem.c MEM_OVERFLOW_CHECK message formatting.
 * darklibc has no snprintf; the formatted text is unused (assert hook passes
 * only __LINE__), so copying the format string truncated is enough. */
int snprintf(char *out, size_t size, const char *fmt, ...)
{
    size_t i = 0;
    if (!out || size == 0) {
        return 0;
    }
    while (fmt && fmt[i] && (i + 1u) < size) {
        out[i] = fmt[i];
        i++;
    }
    out[i] = 0;
    return (int)i;
}
#endif
