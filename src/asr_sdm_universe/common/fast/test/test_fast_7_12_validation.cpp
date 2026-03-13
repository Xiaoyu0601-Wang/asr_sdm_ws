#include <fast/fast.h>

#include <algorithm>
#include <iostream>
#include <random>
#include <set>
#include <vector>

namespace {

using fast::fast_xy;
using fast::fast_byte;

struct XYLess {
  bool operator()(const fast_xy& a, const fast_xy& b) const {
    if (a.y != b.y) return a.y < b.y;
    return a.x < b.x;
  }
};

inline bool is_positive(int val, int center, int barrier) { return val > center + barrier; }
inline bool is_negative(int val, int center, int barrier) { return val < center - barrier; }

bool is_corner_segment(const std::vector<fast_byte>& img, int w, int x, int y, int barrier, int n) {
  static const int dx[16] = {0,1,2,3,3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1};
  static const int dy[16] = {3,3,2,1,0,-1,-2,-3,-3,-3,-2,-1,0,1,2,3};
  const int c = img[y*w+x];

  int cons=0, first=0;
  for(int i=0;i<16;++i){
    const int v = img[(y+dy[i])*w + (x+dx[i])];
    if(is_positive(v,c,barrier)){ ++cons; if(cons==n) return true; }
    else { if(cons==i) first=i; cons=0; }
  }
  if(first+cons>=n) return true;

  cons=0; first=0;
  for(int i=0;i<16;++i){
    const int v = img[(y+dy[i])*w + (x+dx[i])];
    if(is_negative(v,c,barrier)){ ++cons; if(cons==n) return true; }
    else { if(cons==i) first=i; cons=0; }
  }
  return first+cons>=n;
}

std::set<fast_xy, XYLess> segment_detect(const std::vector<fast_byte>& img, int w, int h, int b, int n) {
  std::set<fast_xy, XYLess> out;
  for (int y = 3; y < h - 3; ++y)
    for (int x = 3; x < w - 3; ++x)
      if (is_corner_segment(img, w, x, y, b, n)) out.insert(fast_xy((short)x, (short)y));
  return out;
}

std::set<fast_xy, XYLess> fast_detect(const std::vector<fast_byte>& img, int w, int h, int b, int n) {
  std::vector<fast_xy> c;
  switch(n){
    case 7: fast::fast_corner_detect_7(img.data(), w, h, w, (short)b, c); break;
    case 8: fast::fast_corner_detect_8(img.data(), w, h, w, (short)b, c); break;
    case 9: fast::fast_corner_detect_9(img.data(), w, h, w, (short)b, c); break;
    case 10: fast::fast_corner_detect_10(img.data(), w, h, w, (short)b, c); break;
    case 11: fast::fast_corner_detect_11(img.data(), w, h, w, (short)b, c); break;
    default: fast::fast_corner_detect_12(img.data(), w, h, w, (short)b, c); break;
  }
  return std::set<fast_xy, XYLess>(c.begin(), c.end());
}

bool validate_type(int n, int rounds){
  std::mt19937 rng(100+n);
  std::uniform_int_distribution<int> size_dist(16, 96), pix(0,255), thr(0,255);
  for(int r=0;r<rounds;++r){
    int w=size_dist(rng), h=size_dist(rng), b=thr(rng);
    std::vector<fast_byte> img((size_t)w*h);
    for(auto& p:img) p=(fast_byte)pix(rng);
    auto ref=segment_detect(img,w,h,b,n);
    auto got=fast_detect(img,w,h,b,n);
    bool same = ref.size()==got.size() && std::equal(ref.begin(), ref.end(), got.begin(), [](const fast_xy&a,const fast_xy&b){return a.x==b.x&&a.y==b.y;});
    if(!same){
      std::cerr << "FAST"<<n<<" mismatch round="<<r<<" size="<<w<<"x"<<h<<" barrier="<<b<<" ref="<<ref.size()<<" got="<<got.size()<<std::endl;
      return false;
    }
  }
  return true;
}

} // namespace

int main(){
  bool ok=true;
  for(int n: {7,8,9,10,11,12}) ok = validate_type(n,120) && ok;
  if(!ok){ std::cerr << "FAST-7..12 validation failed." << std::endl; return 1; }
  std::cout << "FAST-7..12 validation passed." << std::endl;
  return 0;
}
