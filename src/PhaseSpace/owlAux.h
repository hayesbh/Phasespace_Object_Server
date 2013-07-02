#include "owl/owl.h"

void owl_print_error(const char *s, int n)
{
  if(n > 0)
    switch(n) {
    case 0: printf("%s: No Error\n", s); break;
    case 1: printf("%s: Invalid Value\n", s); break;
    case 2: printf("%s: Invalid Enum\n", s); break;
    case 3: printf("%s: Incalid Operation\n", s); break;
    default: printf("%s: 0x%x\n", s, n); break;
    }
  else printf("%s: %d\n", s, n);
}
