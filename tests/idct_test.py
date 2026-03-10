import ctypes

scale_table = [
    0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82, 0x5A82,
    0x7D8A, 0x6A6D, 0x471C, 0x18F8, -6393, -18205, -27246, -32139,
    0x7641, 0x30FB, -12540, -30274, -30274, -12540, 0x30FB, 0x7641,
    0x6A6D, -6393, -32139, -18205, 0x471C, 0x7D8A, 0x18F8, -27246,
    0x5A82, -23171, -23171, 0x5A82, 0x5A82, -23171, -23171, 0x5A82,
    0x471C, -32139, 0x18F8, 0x6A6D, -27246, -6393, 0x7D8A, -18205,
    0x30FB, -30274, 0x7641, -12540, -12540, 0x7641, -30274, 0x30FB,
    0x18F8, -18205, 0x6A6D, -32139, 0x7D8A, -27246, 0x471C, -6393
]

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def sign_extend_9(val):
    if val & (1 << 8):
        return val - (1 << 9)
    return val

def idct_old(blk):
    temp = [0] * 64
    for x in range(8):
        for y in range(8):
            s = 0
            for u in range(8):
                s += blk[u * 8 + x] * scale_table[y * 8 + u]
            temp[x + y * 8] = s

    for x in range(8):
        for y in range(8):
            s = 0
            for u in range(8):
                s += temp[u + y * 8] * scale_table[x * 8 + u]
            
            s = (s >> 32) + ((s >> 31) & 1)
            blk[x + y * 8] = clamp(sign_extend_9(s), -128, 127)

def my_idct(blk):
    temp = [0] * 64
    for x in range(8):
        for y in range(8):
            s = 0
            for u in range(8):
                s += blk[u * 8 + x] * scale_table[y * 8 + u]
            temp[x + y * 8] = s

    dst = [0] * 64
    for x in range(8):
        for y in range(8):
            s = 0
            for u in range(8):
                s += temp[u + y * 8] * scale_table[x * 8 + u]
            val = (s >> 32) + ((s >> 31) & 1)
            dst[x + y * 8] = clamp(sign_extend_9(val), -128, 127)
    return dst

ref = [0]*64
ref[0] = 100
ref[1] = 50
ref[8] = -30

my_blk = list(ref)

idct_old(ref)
my_res = my_idct(my_blk)

match = True
for i in range(64):
    if ref[i] != my_res[i]:
        print(f"Mismatch at {i}: ref={ref[i]} my={my_res[i]}")
        match = False

if match:
    print("Match!")
else:
    print("Failed")
