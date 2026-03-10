#include <iostream>
#include <array>
#include <vector>
#include <algorithm>
#include <cstdint>

typedef int32_t s32;
typedef int16_t s16;
typedef int64_t s64;
typedef uint32_t u32;

std::array<s16, 64> scale_table = {
    0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82,
    0x7D8A, 0x6A6D, 0x471C, 0x18F8, (short)0xE707, (short)0xB8E3, (short)0x9592, (short)0x8275,
    0x7641, 0x30FB, (short)0xCF04, (short)0x89BE, (short)0x89BE, (short)0xCF04, 0x30FB, 0x7641,
    0x6A6D, (short)0xE707, (short)0x8275, (short)0xB8E3, 0x471C, 0x7D8A, 0x18F8, (short)0x9592,
    0x5A82, (short)0xA57D, (short)0xA57D, 0x5A82, 0x5A82, (short)0xA57D, (short)0xA57D, 0x5A82,
    0x471C, (short)0x8275, 0x18F8, 0x6A6D, (short)0x9592, (short)0xE707, 0x7D8A, (short)0xB8E3,
    0x30FB, (short)0x89BE, 0x7641, (short)0xCF04, (short)0xCF04, 0x7641, (short)0x89BE, 0x30FB,
    0x18F8, (short)0xB8E3, 0x6A6D, (short)0x8275, 0x7D8A, (short)0x9592, 0x471C, (short)0xE707
};

s32 SignExtendN(s32 val) {
    if (val & (1 << 8)) return val - (1 << 9);
    return val;
}

void IDCT_Old(s16* blk) {
  std::array<s64, 64> temp_buffer;
  for (u32 x = 0; x < 8; x++) {
    for (u32 y = 0; y < 8; y++) {
      s64 sum = 0;
      for (u32 u = 0; u < 8; u++)
        sum += s32(blk[u * 8 + x]) * s32(scale_table[y * 8 + u]);
      temp_buffer[x + y * 8] = sum;
    }
  }
  for (u32 x = 0; x < 8; x++) {
    for (u32 y = 0; y < 8; y++) {
      s64 sum = 0;
      for (u32 u = 0; u < 8; u++)
        sum += s64(temp_buffer[u + y * 8]) * s32(scale_table[x * 8 + u]);
      blk[x + y * 8] = static_cast<s16>(std::clamp<s32>(SignExtendN((sum >> 32) + ((sum >> 31) & 1)), -128, 127));
    }
  }
}

void my_idct(const std::array<int, 64>& coeffs, std::array<int, 64>& pixels) {
  auto idct_pass = [&](const std::array<int, 64> &src, std::array<int, 64> &dst, int shift) {
    for (int x = 0; x < 8; ++x) {
      for (int y = 0; y < 8; ++y) {
        s64 sum = 0;
        for (int z = 0; z < 8; ++z) sum += (s64)src[y + z * 8] * scale_table[x + z * 8];
        dst[x + y * 8] = static_cast<int>((sum + (1LL << (shift - 1))) >> shift);
      }
    }
  };
  std::array<int, 64> temp{};
  idct_pass(coeffs, temp, 15);
  idct_pass(temp, pixels, 16);
}

int main() {
    std::array<s16, 64> ref_blk = {};
    ref_blk[0] = 100; ref_blk[1] = 50; ref_blk[8] = -30;
    
    std::array<int, 64> my_blk = {};
    for (int i=0;i<64;i++) my_blk[i] = ref_blk[i];
    
    IDCT_Old(ref_blk.data());
    
    std::array<int, 64> my_pix = {};
    my_idct(my_blk, my_pix);
    
    bool match = true;
    for (int i = 0; i < 64; ++i) {
        if (ref_blk[i] != my_pix[i]) {
            std::cout << "Mismatch at " << i << ": ref=" << ref_blk[i] << " my=" << my_pix[i] << "\n";
            match = false;
        }
    }
    if (match) std::cout << "Match!\n";
    return 0;
}
