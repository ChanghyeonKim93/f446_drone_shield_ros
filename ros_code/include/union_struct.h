#ifndef _UNION_STRUCT_H_
#define _UNION_STRUCT_H_
#include <iostream>

typedef union SHORT_UNION_{
    int16_t short_;
    unsigned char bytes_[2];
} SHORT_UNION;

typedef union USHORT_UNION_{
    uint16_t ushort_;
    unsigned char bytes_[2];
} USHORT_UNION;

typedef union UINT_UNION_{
    uint32_t uint_;
    unsigned char bytes_[4];
} UINT_UNION;

typedef union FLOAT_UNION_{
    float float_;
    unsigned char bytes_[4];
} FLOAT_UNION;

#endif