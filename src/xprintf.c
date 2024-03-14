#include "xprintf.h"

#include <string.h>
#include "ets.h"


#define CVTBUFSIZE 80


#define ZEROPAD 1               // Pad with zero
#define SIGN    2               // Unsigned/signed long
#define PLUS    4               // Show plus
#define SPACE   8               // Space if plus
#define LEFT    16              // Left justified
#define SPECIAL 32              // 0x
#define LARGE   64              // Use 'ABCDEF' instead of 'abcdef'

#define is_digit(c) ((c) >= '0' && (c) <= '9')

static char *lower_digits = "0123456789abcdefghijklmnopqrstuvwxyz";
static char *upper_digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";


static int skip_atoi(const char **s)
{
	int i = 0;
	while (is_digit(**s)) i = i*10 + *((*s)++) - '0';
	return i;
}

static char *number(char *str, long num, int base, int size, int precision, int type)
{
	char c, sign, tmp[66];
	char *dig = lower_digits;
	int i;

	if (type & LARGE)  dig = upper_digits;
	if (type & LEFT) type &= ~ZEROPAD;
	if (base < 2 || base > 36) return NULL;

	c = (type & ZEROPAD) ? '0' : ' ';
	sign = 0;
	if (type & SIGN) {
		if (num < 0) {
			sign = '-';
			num = -num;
			size--;
			}
		else if (type & PLUS) {
			sign = '+';
			size--;
			}
		else if (type & SPACE) {
			sign = ' ';
			size--;
			}
		}

	if (type & SPECIAL) {
		if (base == 16) size -= 2;
		else if (base == 8) size--;
		}

	i = 0;

	if (num == 0) tmp[i++] = '0';
	else {
		while (num != 0) {
			tmp[i++] = dig[((unsigned long) num) % (unsigned) base];
			num = ((unsigned long) num) / (unsigned) base;
			}
		}

	if (i > precision) precision = i;
	size -= precision;
	if (!(type & (ZEROPAD | LEFT))) while (size-- > 0) *str++ = ' ';
	if (sign) *str++ = sign;

	if (type & SPECIAL) {
		if (base == 8) *str++ = '0';
		else if (base == 16) {
			*str++ = '0';
			*str++ = lower_digits[33];
			}
		}

	if (!(type & LEFT)) while (size-- > 0) *str++ = c;
	while (i < precision--) *str++ = '0';
	while (i-- > 0) *str++ = tmp[i];
	while (size-- > 0) *str++ = ' ';

	return str;
}

static char *eaddr(char *str, unsigned char *addr, int size, int precision, int type)
{
	char tmp[24];
	char *dig = lower_digits;
	int i, len;

	if (type & LARGE)  dig = upper_digits;
	len = 0;
	for (i = 0; i < 6; i++) {
		if (i != 0) tmp[len++] = ':';
		tmp[len++] = dig[addr[i] >> 4];
		tmp[len++] = dig[addr[i] & 0x0F];
		}

	if (!(type & LEFT)) while (len < size--) *str++ = ' ';
	for (i = 0; i < len; ++i) *str++ = tmp[i];
	while (len < size--) *str++ = ' ';
	return str;
}

static char *iaddr(char *str, unsigned char *addr, int size, int precision, int type)
{
	char tmp[24];
	int i, n, len;

	len = 0;
	for (i = 0; i < 4; i++) {
		if (i != 0) tmp[len++] = '.';
		n = addr[i];

		if (n == 0) tmp[len++] = lower_digits[0];
		else {
			if (n >= 100) {
			tmp[len++] = lower_digits[n / 100];
			n = n % 100;
			tmp[len++] = lower_digits[n / 10];
			n = n % 10;
			}
			else if (n >= 10) {
				tmp[len++] = lower_digits[n / 10];
				n = n % 10;
				}
			tmp[len++] = lower_digits[n];
			}
		}

	if (!(type & LEFT)) while (len < size--) *str++ = ' ';
	for (i = 0; i < len; ++i) *str++ = tmp[i];
	while (len < size--) *str++ = ' ';
	return str;
}

int xvsprintf(char *buf, const char *fmt, va_list args)
{
	int len;
	unsigned long num;
	int i, base;
	char *str;
	char *s;

	int flags;            // Flags to number()

	int field_width;      // Width of output field
	int precision;        // Min. # of digits for integers; max number of chars for from string
	int qualifier;        // 'h', 'l', or 'L' for integer fields

	for (str = buf; *fmt; fmt++) {
		if (*fmt != '%') {
			*str++ = *fmt;
			continue;
			}

    // Process flags
		flags = 0;
repeat:
		fmt++; // This also skips first '%'
		switch (*fmt) {
			case '-': flags |= LEFT; goto repeat;
			case '+': flags |= PLUS; goto repeat;
			case ' ': flags |= SPACE; goto repeat;
			case '#': flags |= SPECIAL; goto repeat;
			case '0': flags |= ZEROPAD; goto repeat;
			}

    // Get field width
		field_width = -1;
		if (is_digit(*fmt)) {
			field_width = skip_atoi(&fmt);
			}
		else if (*fmt == '*') {
			fmt++;
			field_width = va_arg(args, int);
			if (field_width < 0) {
				field_width = -field_width;
				flags |= LEFT;
				}
			}

    // Get the precision
		precision = -1;
		if (*fmt == '.') {
			++fmt;
			if (is_digit(*fmt)) precision = skip_atoi(&fmt);
			else if (*fmt == '*') {
				++fmt;
				precision = va_arg(args, int);
				}
			if (precision < 0) precision = 0;
			}

    // Get the conversion qualifier
		qualifier = -1;
		if (*fmt == 'h' || *fmt == 'l' || *fmt == 'L') {
			qualifier = *fmt;
			fmt++;
			}

    // Default base
    base = 10;

    switch (*fmt)
    {
      case 'c':
        if (!(flags & LEFT)) while (--field_width > 0) *str++ = ' ';
        *str++ = (unsigned char) va_arg(args, int);
        while (--field_width > 0) *str++ = ' ';
        continue;

      case 's':
        s = va_arg(args, char *);
        if (s == NULL) s = "<NULL>";
        len = strnlen(s, precision);
        if (!(flags & LEFT)) while (len < field_width--) *str++ = ' ';
        for (i = 0; i < len; ++i) *str++ = *s++;
        while (len < field_width--) *str++ = ' ';
        continue;

      case 'p':
        if (field_width == -1)
        {
          field_width = 2 * sizeof(void *);
          flags |= ZEROPAD;
        }
        str = number(str, (unsigned long) va_arg(args, void *), 16, field_width, precision, flags);
        continue;

      case 'n':
        if (qualifier == 'l')
        {
          long *ip = va_arg(args, long *);
          *ip = (str - buf);
        }
        else
        {
          int *ip = va_arg(args, int *);
          *ip = (str - buf);
        }
        continue;

      case 'A':
        flags |= LARGE;

      case 'a':
        if (qualifier == 'l')
          str = eaddr(str, va_arg(args, unsigned char *), field_width, precision, flags);
        else
          str = iaddr(str, va_arg(args, unsigned char *), field_width, precision, flags);
        continue;

      // Integer number formats - set up the flags and "break"
      case 'o':
        base = 8;
        break;

      case 'X':
        flags |= LARGE;

      case 'x':
        base = 16;
        break;

      case 'd':
      case 'i':
        flags |= SIGN;

      case 'u':
        break;

      default:
        if (*fmt != '%') *str++ = '%';
        if (*fmt) *str++ = *fmt;
        else --fmt;
        continue;
    }

    if (qualifier == 'l')
      num = va_arg(args, unsigned long);
    else if (qualifier == 'h')
    {
      if (flags & SIGN)
        num = va_arg(args, int);
      else
        num = va_arg(args, unsigned int);
    }
    else if (flags & SIGN)
      num = va_arg(args, int);
    else
      num = va_arg(args, unsigned int);

    str = number(str, num, base, field_width, precision, flags);
  }

  *str = '\0';
  return str - buf;
}

int xsprintf(char *buf, const char *fmt, ...)
{
	va_list args;
	int n;

	va_start(args, fmt);
	n = xvsprintf(buf, fmt, args);

	return n;
}
