#pragma once

#include <fast/faster_corner_utilities.h>

namespace fast {

template <class C>
inline bool is_corner_11(const unsigned char* p, const int w, const int barrier)
{
  const int cb = *p + barrier;
  const int c_b = *p - barrier;

  const int pixel[16] = {
    0 + w * 3,
    1 + w * 3,
    2 + w * 2,
    3 + w * 1,
    3 + w * 0,
    3 + w * -1,
    2 + w * -2,
    1 + w * -3,
    0 + w * -3,
    -1 + w * -3,
    -2 + w * -2,
    -3 + w * -1,
    -3 + w * 0,
    -3 + w * 1,
    -2 + w * 2,
    -1 + w * 3,
  };

  if (C::eval(*(p + pixel[0]), cb, c_b))
    if (C::eval(*(p + pixel[8]), cb, c_b))
      if (C::eval(*(p + pixel[3]), cb, c_b))
        if (C::eval(*(p + pixel[6]), cb, c_b))
          if (C::eval(*(p + pixel[2]), cb, c_b))
            if (C::eval(*(p + 3), cb, c_b))
              if (C::eval(*(p + pixel[15]), cb, c_b))
                if (C::eval(*(p + pixel[7]), cb, c_b))
                  if (C::eval(*(p + pixel[5]), cb, c_b))
                    if (C::eval(*(p + pixel[1]), cb, c_b))
                      if (C::eval(*(p + pixel[9]), cb, c_b))
                        return true;
                      else if (C::eval(*(p + pixel[14]), cb, c_b))
                        return true;
                      else
                        return false;
                    else if (C::eval(*(p + pixel[1]), cb, c_b))
                      return false;
                    else if (C::eval(*(p + pixel[9]), cb, c_b))
                      if (C::eval(*(p + pixel[10]), cb, c_b))
                        if (C::eval(*(p + pixel[11]), cb, c_b))
                          if (C::eval(*(p + -3), cb, c_b))
                            return true;
                          else
                            return false;
                        else
                          return false;
                      else
                        return false;
                    else
                      return false;
                  else if (C::eval(*(p + pixel[5]), cb, c_b))
                    return false;
                  else if (C::eval(*(p + pixel[10]), cb, c_b))
                    if (C::eval(*(p + pixel[11]), cb, c_b))
                      if (C::eval(*(p + -3), cb, c_b))
                        if (C::eval(*(p + pixel[13]), cb, c_b))
                          if (C::eval(*(p + pixel[14]), cb, c_b))
                            if (C::eval(*(p + pixel[9]), cb, c_b))
                              return true;
                            else if (C::eval(*(p + pixel[1]), cb, c_b))
                              return true;
                            else
                              return false;
                          else
                            return false;
                        else
                          return false;
                      else
                        return false;
                    else
                      return false;
                  else
                    return false;
                else if (C::eval(*(p + pixel[7]), cb, c_b))
                  return false;
                else if (C::eval(*(p + pixel[1]), cb, c_b))
                  if (C::eval(*(p + -3), cb, c_b))
                    if (C::eval(*(p + pixel[13]), cb, c_b))
                      if (C::eval(*(p + pixel[14]), cb, c_b))
                        if (C::eval(*(p + pixel[5]), cb, c_b))
                          return true;
                        else if (C::eval(*(p + pixel[10]), cb, c_b))
                          if (C::eval(*(p + pixel[11]), cb, c_b))
                            return true;
                          else
                            return false;
                        else
                          return false;
                      else
                        return false;
                    else
                      return false;
                  else
                    return false;
                else if (C::eval(*(p + -3), cb, c_b))
                  if (C::eval(*(p + pixel[14]), cb, c_b))
                    if (C::eval(*(p + pixel[13]), cb, c_b))
                      if (C::eval(*(p + pixel[5]), cb, c_b))
                        if (C::eval(*(p + pixel[1]), cb, c_b))
                          return true;
                        else
                          return false;
                      else if (C::eval(*(p + pixel[5]), cb, c_b))
                        return false;
                      else if (C::eval(*(p + pixel[1]), cb, c_b))
                        if (C::eval(*(p + pixel[10]), cb, c_b))
                          if (C::eval(*(p + pixel[11]), cb, c_b))
                            return true;
                          else
                            return false;
                        else
                          return false;
                      else
                        return false;
                    else
                      return false;
                  else
                    return false;
                else
                  return false;
              else if (C::eval(*(p + pixel[15]), cb, c_b))
                return false;
              else if (C::eval(*(p + pixel[5]), cb, c_b))
                if (C::eval(*(p + pixel[7]), cb, c_b))
                  if (C::eval(*(p + pixel[9]), cb, c_b))
                    if (C::eval(*(p + pixel[10]), cb, c_b))
                      if (C::eval(*(p + pixel[1]), cb, c_b))
                        return true;
                      else if (C::eval(*(p + pixel[11]), cb, c_b))
                        if (C::eval(*(p + -3), cb, c_b))
                          return true;
                        else
                          return false;
                      else
                        return false;
                    else
                      return false;
                  else
                    return false;
                else
                  return false;
              else
                return false;
            else
              return false;
          else
            return false;
        else
          return false;
      else
        return false;
    else
      return false;
  return false;
}

} // namespace fast
