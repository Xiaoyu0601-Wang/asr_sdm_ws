#ifndef FASTER_CORNER_UTILITIES_H
#define FASTER_CORNER_UTILITIES_H

namespace fast {

struct Greater {
    static inline bool eval(unsigned char p, unsigned char t) { return p > t; }
    static inline bool eval(unsigned char p, int cb, int c_b) { (void)c_b; return p > cb; }
    static inline unsigned char prep_t(unsigned char p, int barrier) { return (unsigned char)(p + barrier > 255 ? 255 : p + barrier); }
};

struct Less {
    static inline bool eval(unsigned char p, unsigned char t) { return p < t; }
    static inline bool eval(unsigned char p, int cb, int c_b) { (void)cb; return p < c_b; }
    static inline unsigned char prep_t(unsigned char p, int barrier) { return (unsigned char)(p - barrier < 0 ? 0 : p - barrier); }
};

} // namespace fast

#endif
