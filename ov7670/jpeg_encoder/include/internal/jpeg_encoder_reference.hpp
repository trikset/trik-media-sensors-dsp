/*
  Copyright (c) 2008, Adobe Systems Incorporated
  All rights reserved.

  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are
  met:

  * Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer.
  
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the 
    documentation and/or other materials provided with the distribution.
  
  * Neither the name of Adobe Systems Incorporated nor the names of its 
    contributors may be used to endorse or promote products derived from 
    this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
JPEG encoder ported to C++ and optimized by Jake and Dmitry, www.trikset.com, 03/2015

Basic GUI blocking jpeg encoder
*/

#ifndef JPEG_ENCODER_HPP_
#define JPEG_ENCODER_HPP_

#ifndef __cplusplus
#error C++-only header
#endif


/*
  Copyright (c) 2008, Adobe Systems Incorporated
  All rights reserved.

  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions are
  met:

  * Redistributions of source code must retain the above copyright notice, 
    this list of conditions and the following disclaimer.
  
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the 
    documentation and/or other materials provided with the distribution.
  
  * Neither the name of Adobe Systems Incorporated nor the names of its 
    contributors may be used to endorse or promote products derived from 
    this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
JPEG encoder ported to C++ and optimized by Jake and Dmitry, www.trikset.com, 03/2015

Basic GUI blocking jpeg encoder
*/
#include <map>
//#define double float

/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace cv /* **** **** **** **** **** */ {

template<typename T, size_t S>
struct array {
  T data[S];

  array() { }

  array(const T(& _data)[S]) {
    init(_data);
  }

  void init(const T(& _data)[S]) {
    std::copy(_data, _data + S, data);
  }

   T& restrict operator[](int i) restrict {
    return data[i];
  }

  const T& restrict operator[](int i) const restrict {
    return data[i];
  }
};

typedef array<int,64> Arr64;
typedef array<double,64> Arr64d;

typedef uint8_t PixIn;
typedef uint8_t PixOut;

struct BitString{
  BitString():len(),val() {}

  int16_t len;
  uint16_t val;
};

static int _ZigZag[64] = {
     0, 1, 5, 6,14,15,27,28,
     2, 4, 7,13,16,26,29,42,
     3, 8,12,17,25,30,41,43,
     9,11,18,24,31,40,44,53,
    10,19,23,32,39,45,52,54,
    20,22,33,38,46,51,55,60,
    21,34,37,47,50,56,59,61,
    35,36,48,49,57,58,62,63};

static int _YQT[64] = {
      16, 11, 10, 16, 24, 40, 51, 61,
      12, 12, 14, 19, 26, 58, 60, 55,
      14, 13, 16, 24, 40, 57, 69, 56,
      14, 17, 22, 29, 51, 87, 80, 62,
      18, 22, 37, 56, 68,109,103, 77,
      24, 35, 55, 64, 81,104,113, 92,
      49, 64, 78, 87,103,121,120,101,
      72, 92, 95, 98,112,100,103, 99};

static int _UVQT[64] = {
      17, 18, 24, 47, 99, 99, 99, 99,
      18, 21, 26, 66, 99, 99, 99, 99,
      24, 26, 56, 99, 99, 99, 99, 99,
      47, 66, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99,
      99, 99, 99, 99, 99, 99, 99, 99};

static int _std_dc_luminance_nrcodes[17] = {0,0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0};
static int _std_dc_luminance_values[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static int _std_ac_luminance_nrcodes[17] = {0,0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d};
static int _std_ac_luminance_values[162] = {
  0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,
  0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
  0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,
  0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,
  0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,
  0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,
  0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,
  0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
  0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,
  0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
  0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,
  0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
  0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,
  0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,
  0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,
  0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,
  0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,
  0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,
  0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,
  0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
  0xf9,0xfa};

static int _std_dc_chrominance_nrcodes[17] = {0,0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
static int _std_dc_chrominance_values[12]  = {0,1,2,3,4,5,6,7,8,9,10,11};
static int _std_ac_chrominance_nrcodes[17] = {0,0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77};
static int _std_ac_chrominance_values[162] = {
  0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,
  0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
  0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,
  0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,
  0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,
  0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,
  0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,
  0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,
  0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,
  0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,
  0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,
  0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,
  0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,
  0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,
  0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,
  0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,
  0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,
  0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,
  0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,
  0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,
  0xf9,0xfa};

static double _aasf[8] = {
      1.0, 1.387039845, 1.306562965, 1.175875602,
      1.0, 0.785694958, 0.541196100, 0.275899379
    };

/**
 * Class that converts BitmapData into a valid JPEG
 */
class JPGEncoder
{
  private:
  //#pragma DATA_ALIGN (64);
  Arr64 YTable;
  Arr64 UVTable;
  Arr64d fdtbl_Y, fdtbl_UV;

  bool ifBlackAndWhite;

  // Static table initialization

  static Arr64 ZigZag;
  static array<int, 17> std_dc_luminance_nrcodes;
  static array<int, 12> std_dc_luminance_values;
  static array<int, 17> std_ac_luminance_nrcodes;
  static array<int, 162> std_ac_luminance_values;

  static array<int, 17> std_dc_chrominance_nrcodes;
  static array<int, 12> std_dc_chrominance_values;
  static array<int, 17> std_ac_chrominance_nrcodes;
  static array<int, 162> std_ac_chrominance_values;

  static int bytenew;
  static int bytepos;

  static array<BitString, 65535> bitcode;
  static array<int, 65535> category;

  void initQuantTables(int sf)
  {
    int i;
    int t;
    Arr64 YQT(_YQT);
    for (i = 0; i < 64; i++) {
      t = floor((YQT[i]*sf+50)/100);
      if (t < 1) {
        t = 1;
      } else if (t > 255) {
        t = 255;
      }
      YTable[ZigZag[i]] = t;
    }
    Arr64 UVQT(_UVQT);
    for (i = 0; i < 64; i++) {
      t = floor((UVQT[i]*sf+50)/100);
      if (t < 1) {
        t = 1;
      } else if (t > 255) {
        t = 255;
      }
      UVTable[ZigZag[i]] = t;
    }
    array<double, 8> aasf(_aasf);
    i = 0;
    for (int row = 0; row < 8; row++)
    {
      for (int col = 0; col < 8; col++)
      {
        fdtbl_Y[i]  = (1.0 / (YTable [ZigZag[i]] * aasf[row] * aasf[col] * 8.0));
        fdtbl_UV[i] = (1.0 / (UVTable[ZigZag[i]] * aasf[row] * aasf[col] * 8.0));
        i++;
      }
    }
  }

  //std::map<int, BitString> YDC_HT, UVDC_HT, YAC_HT, UVAC_HT;
  array<BitString, 12> YDC_HT, UVDC_HT;
  array<BitString, 256> YAC_HT, UVAC_HT;

  template<std::size_t Size0, std::size_t Size1, std::size_t Size2>
  void computeHuffmanTbl(array<BitString, Size0> & HT, array<int, Size1> const& nrcodes, array<int, Size2> const& std_table)
  {
    int codevalue = 0;
    int pos_in_table = 0;

//    std::map<int, BitString> HT;
    for (int k=1; k<=16; k++) {
      for (int j=1; j<=nrcodes[k]; j++) {
        /*
        HT[std_table[pos_in_table]] = new BitString();
        */
        HT[std_table[pos_in_table]].val = codevalue;
        HT[std_table[pos_in_table]].len = k;

        //HT[std_table[pos_in_table]] = {k, codevalue};
        pos_in_table++;
        codevalue++;
      }
      codevalue*=2;
    }
//    return HT;
  }

  void initHuffmanTbl()
  {
    computeHuffmanTbl(YDC_HT, std_dc_luminance_nrcodes,std_dc_luminance_values); //17 12
    computeHuffmanTbl(UVDC_HT, std_dc_chrominance_nrcodes,std_dc_chrominance_values); //17 12
    computeHuffmanTbl(YAC_HT, std_ac_luminance_nrcodes,std_ac_luminance_values);  //17 162
    computeHuffmanTbl(UVAC_HT, std_ac_chrominance_nrcodes,std_ac_chrominance_values); //17 162
  }

  void initCategoryNumber()
  {
    int nrlower = 1;
    int nrupper = 2;
    int nr;
    for (uint8_t cat=1; cat<=15; cat++) {
      //Positive numbers
      for (nr=nrlower; nr<nrupper; nr++) {
        category[32767+nr] = cat;
        /*
        bitcode[32767+nr] = new BitString();
        */
        bitcode[32767+nr].len = cat;
        bitcode[32767+nr].val = nr;

        //bitcode[32767+nr] = {cat, nr};
      }
      //Negative numbers
      //#pragma MUST_ITERATE(1, ,1)
      for (nr=-(nrupper-1); nr<=-nrlower; nr++) {
        category[32767+nr] = cat;
        /*
        bitcode[32767+nr] = new BitString();
        */
        bitcode[32767+nr].len = cat;
        bitcode[32767+nr].val = nrupper-1+nr;

        //bitcode[32767+nr] = {cat, nrupper-1+nr};
      }
      nrlower <<= 1;
      nrupper <<= 1;
    }
  }

  // IO functions

  struct byteout {
    PixOut* ptr;
    int cnt;
  } byteout;

  void writeBits(BitString  const& bs)
  {
    uint32_t value  = bs.val;

    bytepos -= bs.len;
    value <<= (16 + bytepos);
    bytenew |= value >> 16;

    if(bytepos <= 0) {
      writeByte(bytenew);
      if (bytenew == 0xFF) {
        writeByte(0);
      }
      bytepos = -bytepos;

      if(bytepos >= 8) {
        uint8_t tmp = (value >> 8) & 0xFF;
        writeByte(tmp);
        if (tmp == 0xFF) {
          writeByte(0);
        }
        bytepos -= 8;
      }else {
        value >>= 8;
      }

      bytepos = 8 - bytepos;
      bytenew = value & 0xFF;
    }
  }

  void writeByte(int value)
  {
    byteout.ptr[byteout.cnt++] = value;
  }

  void writeWord(uint16_t value)
  {
    writeByte((value>>8)&0xFF);
    writeByte((value   )&0xFF);
  }

  // DCT & quantization core

  Arr64 fDCTQuant(Arr64 data, Arr64d const& fdtbl)
  {
    int tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7;
    int tmp10, tmp11, tmp12, tmp13;
    double z1, z2, z3, z4, z5, z11, z13;
    int i;
    /* Pass 1: process rows. */
    int dataOff=0;
    #pragma MUST_ITERATE(8,8,8)
    for (i=0; i<8; i++) {
      tmp0 = data[dataOff+0] + data[dataOff+7];
      tmp7 = data[dataOff+0] - data[dataOff+7];
      tmp1 = data[dataOff+1] + data[dataOff+6];
      tmp6 = data[dataOff+1] - data[dataOff+6];
      tmp2 = data[dataOff+2] + data[dataOff+5];
      tmp5 = data[dataOff+2] - data[dataOff+5];
      tmp3 = data[dataOff+3] + data[dataOff+4];
      tmp4 = data[dataOff+3] - data[dataOff+4];

      /* Even part */
      tmp10 = tmp0 + tmp3;    /* phase 2 */
      tmp13 = tmp0 - tmp3;
      tmp11 = tmp1 + tmp2;
      tmp12 = tmp1 - tmp2;

      data[dataOff+0] = tmp10 + tmp11; /* phase 3 */
      data[dataOff+4] = tmp10 - tmp11;

      z1 = (tmp12 + tmp13) * 0.707106781; /* c4 */
      data[dataOff+2] = tmp13 + z1; /* phase 5 */
      data[dataOff+6] = tmp13 - z1;

      /* Odd part */
      tmp10 = tmp4 + tmp5; /* phase 2 */
      tmp11 = tmp5 + tmp6;
      tmp12 = tmp6 + tmp7;

      /* The rotator is modified from fig 4-8 to avoid extra negations. */
      z5 = (tmp10 - tmp12) * 0.382683433; /* c6 */
      z2 = 0.541196100 * tmp10 + z5; /* c2-c6 */
      z4 = 1.306562965 * tmp12 + z5; /* c2+c6 */
      z3 = tmp11 * 0.707106781; /* c4 */

      z11 = tmp7 + z3;  /* phase 5 */
      z13 = tmp7 - z3;

      data[dataOff+5] = z13 + z2;     /* phase 6 */
      data[dataOff+3] = z13 - z2;
      data[dataOff+1] = z11 + z4;
      data[dataOff+7] = z11 - z4;

      dataOff += 8; /* advance pointer to next row */
    }

    /* Pass 2: process columns. */
    dataOff = 0;
    #pragma MUST_ITERATE(8,8,8)
    for (i=0; i<8; i++) {
      tmp0 = data[dataOff+ 0] + data[dataOff+56];
      tmp7 = data[dataOff+ 0] - data[dataOff+56];
      tmp1 = data[dataOff+ 8] + data[dataOff+48];
      tmp6 = data[dataOff+ 8] - data[dataOff+48];
      tmp2 = data[dataOff+16] + data[dataOff+40];
      tmp5 = data[dataOff+16] - data[dataOff+40];
      tmp3 = data[dataOff+24] + data[dataOff+32];
      tmp4 = data[dataOff+24] - data[dataOff+32];

      /* Even part */
      tmp10 = tmp0 + tmp3;    /* phase 2 */
      tmp13 = tmp0 - tmp3;
      tmp11 = tmp1 + tmp2;
      tmp12 = tmp1 - tmp2;

      data[dataOff+ 0] = tmp10 + tmp11; /* phase 3 */
      data[dataOff+32] = tmp10 - tmp11;

      z1 = (tmp12 + tmp13) * 0.707106781; /* c4 */
      data[dataOff+16] = tmp13 + z1; /* phase 5 */
      data[dataOff+48] = tmp13 - z1;

      /* Odd part */
      tmp10 = tmp4 + tmp5; /* phase 2 */
      tmp11 = tmp5 + tmp6;
      tmp12 = tmp6 + tmp7;

      /* The rotator is modified from fig 4-8 to avoid extra negations. */
      z5 = (tmp10 - tmp12) * 0.382683433; /* c6 */
      z2 = 0.541196100 * tmp10 + z5; /* c2-c6 */
      z4 = 1.306562965 * tmp12 + z5; /* c2+c6 */
      z3 = tmp11 * 0.707106781; /* c4 */

      z11 = tmp7 + z3;  /* phase 5 */
      z13 = tmp7 - z3;

      data[dataOff+40] = z13 + z2; /* phase 6 */
      data[dataOff+24] = z13 - z2;
      data[dataOff+ 8] = z11 + z4;
      data[dataOff+56] = z11 - z4;

      dataOff++; /* advance pointer to next column */
    }

    // Quantize/descale the coefficients
    #pragma MUST_ITERATE(64,64,64)
    for (i=0; i<64; i++) {
      // Apply the quantization and scaling factor & Round to nearest integer
      data[i] = double2int(data[i]*fdtbl[i]); //round()

    }

    return data;
  }

  // Chunk writing

  void writeAPP0()
  {
    writeWord(0xFFE0); // marker
    writeWord(16); // length
    writeByte(0x4A); // J
    writeByte(0x46); // F
    writeByte(0x49); // I
    writeByte(0x46); // F
    writeByte(0); // = "JFIF",'\0'
    writeByte(1); // versionhi
    writeByte(1); // versionlo
    writeByte(0); // xyunits
    writeWord(1); // xdensity
    writeWord(1); // ydensity
    writeByte(0); // thumbnwidth
    writeByte(0); // thumbnheight
  }

  void writeSOF0(int width, int height)
  {
    writeWord(0xFFC0); // marker
    if (ifBlackAndWhite) writeWord(11);
        else writeWord(17);   // length, truecolor YUV JPG

    writeByte(8);    // precision
    writeWord(height);
    writeWord(width);
    if (ifBlackAndWhite) writeByte(1);
        else writeByte(3);    // nrofcomponents

    writeByte(1);    // IdY
    writeByte(0x11); // HVY
    writeByte(0);    // QTY
    if (!ifBlackAndWhite) {
        writeByte(2);    // IdU
        writeByte(0x11); // HVU
        writeByte(1);    // QTU
        writeByte(3);    // IdV
        writeByte(0x11); // HVV
        writeByte(1);    // QTV
    }
  }

  void writeDQT()
  {
    writeWord(0xFFDB); // marker
    if (ifBlackAndWhite) writeWord(67);
        else writeWord(132);    // length
    writeByte(0);
    int i;
    for (i=0; i<64; i++) {
      writeByte(YTable[i]);
    }
    if (!ifBlackAndWhite) {
        writeByte(1);
        for (i=0; i<64; i++) {
          writeByte(UVTable[i]);
        }
    }
  }

  void writeDHT()
  {
    writeWord(0xFFC4); // marker
    if (ifBlackAndWhite) writeWord(0xD2);
        else writeWord(0x01A2); // length
    int i;

    writeByte(0); // HTYDCinfo
    #pragma MUST_ITERATE(32, 32, 32)
    for (i=0; i<16; i++) {
      writeByte(std_dc_luminance_nrcodes[i+1]);
    }
    #pragma MUST_ITERATE(12, 12, 12)
    for (i=0; i<=11; i++) {
      writeByte(std_dc_luminance_values[i]);
    }

    writeByte(0x10); // HTYACinfo
    #pragma MUST_ITERATE(16, 16, 16)
    for (i=0; i<16; i++) {
      writeByte(std_ac_luminance_nrcodes[i+1]);
    }
    for (i=0; i<=161; i++) {
      writeByte(std_ac_luminance_values[i]);
    }
    if (!ifBlackAndWhite) {
        writeByte(1); // HTUDCinfo
        #pragma MUST_ITERATE(16, 16, 16)
        for (i=0; i<16; i++) {
          writeByte(std_dc_chrominance_nrcodes[i+1]);
        }
        #pragma MUST_ITERATE(12, 12, 12)
        for (i=0; i<=11; i++) {
          writeByte(std_dc_chrominance_values[i]);
        }

        writeByte(0x11); // HTUACinfo
        #pragma MUST_ITERATE(16, 16, 16)
        for (i=0; i<16; i++) {
          writeByte(std_ac_chrominance_nrcodes[i+1]);
        }
        for (i=0; i<=161; i++) {
          writeByte(std_ac_chrominance_values[i]);
        }
    }
  }

  void writeSOS()
  {
    writeWord(0xFFDA); // marker
    if (ifBlackAndWhite) {
        writeWord(8); // length
        writeByte(1); // nrofcomponents
    }
    else {
        writeWord(12); // length
        writeByte(3); // nrofcomponents
    }
    writeByte(1); // IdY
    writeByte(0); // HTY
    if (!ifBlackAndWhite) {
        writeByte(2); // IdU
        writeByte(0x11); // HTU
        writeByte(3); // IdV
        writeByte(0x11); // HTV
    }
    writeByte(0); // Ss
    writeByte(0x3f); // Se
    writeByte(0); // Bf
  }

  // Core processing
  Arr64 DU;

  template<std::size_t Size0, std::size_t Size1>
  int processDU(Arr64 const& CDU, Arr64d const& fdtbl, int DC,
                array<BitString, Size0> const& HTDC,
                array<BitString, Size1> const& HTAC)
  restrict
  {
    BitString EOB = HTAC[0x00];//.find(0x00)->second;
    BitString M16zeroes = HTAC[0xF0];//.find(0xF0)->second;
    int i;

    Arr64 DU_DCT = fDCTQuant(CDU, fdtbl);
    //ZigZag reorder

    #pragma UNROLL(2)
    #pragma MUST_ITERATE(64, ,64)
    for (i=0;i<64;i++) {
      DU[ZigZag[i]]=DU_DCT[i];
    }
    int Diff = DU[0] - DC;
    DC = DU[0];

    //Encode DC
    if (Diff==0) {
      writeBits(HTDC[0x00]);// .find(0)->second); // Diff might be 0
    } else {
      writeBits(HTDC[category[32767+Diff]]);//.find(category[32767+Diff])->second);
      writeBits(bitcode[32767+Diff]);
    }

    //Encode ACs
    int end0pos = 63;
    //for (; (end0pos!=0)&&(DU[end0pos]==0); end0pos--) {}
    for (; (end0pos>0)&&(DU[end0pos]==0); end0pos--) {}
    //end0pos = first BitString in reverse order !=0
    if ( end0pos == 0) {
      writeBits(EOB);
      return DC;
    }

//    while ( i <= end0pos ) {
    for (i = 1; i <= end0pos; i++ ) {
      int startpos = i;
      for (; (DU[i]==0) && (i<=end0pos); i++) { }
      int nrzeroes = i-startpos;
      if ( nrzeroes >= 16 ) {
        for (int nrmarker=1; nrmarker <= nrzeroes/16; nrmarker++) {
          writeBits(M16zeroes);
        }
        nrzeroes = static_cast<int>(nrzeroes&0xF);
      }

      writeBits(HTAC[nrzeroes*16+category[32767+DU[i]]]);//.find(nrzeroes*16+category[32767+DU[i]])->second);
      writeBits(bitcode[32767+DU[i]]);
    }
    if ( end0pos != 63 ) {
      writeBits(EOB);
    }

    return DC;
  }

  Arr64 YDU, UDU, VDU;

  //don't used for now
  void RGB2YUV(PixIn* img, int xpos, int ypos, int ll)
  {
    int pos=0;
    for (int y=0; y<8; y++) {
      for (int x=0; x<8; x++) {
        uint32_t P = img[(ypos+y)*ll + xpos+x];
        int R = static_cast<int>((P>>16)&0xFF);
        int G = static_cast<int>((P>> 8)&0xFF);
        int B = static_cast<int>((P    )&0xFF);
        YDU[pos]=((( 0.29900)*R+( 0.58700)*G+( 0.11400)*B))-128;
        UDU[pos]=(((-0.16874)*R+(-0.33126)*G+( 0.50000)*B));
        VDU[pos]=((( 0.50000)*R+(-0.41869)*G+(-0.08131)*B));
        pos++;
      }
    }
  }

  //YCbCr 422p - img format
  void getBlock(PixIn* img, int xpos, int ypos, int width, int height) 
  {
    const uint16_t *UV = reinterpret_cast<const uint16_t*>(img + width*height*sizeof(uint8_t));
    
    int pos=0;
    #pragma MUST_ITERATE(8, ,8)
    for (int y=0; y<8; y++) {
      const int y_shift = (ypos+y)*width;
      #pragma MUST_ITERATE(8, ,8)
      for (int x=0; x<8; x++) {
        const int xy_shift = y_shift + (xpos + x);
    		const uint16_t uv = UV[xy_shift/2];
        YDU[pos]=img[xy_shift]-128;
        UDU[pos]=static_cast<PixIn>(uv>>8)-128;
        VDU[pos]=static_cast<PixIn>(uv)-128;
        pos++;
      }
    }
  }

  // http://stackoverflow.com/questions/17035464/a-fast-method-to-round-a-double-to-a-32-bit-int-explained
  inline int double2int(double d)
  {
    d += 6755399441055744.0;
    return reinterpret_cast<int&>(d);
  }

  public:
  /**
   * Constructor for JPEGEncoder class
   *
   * @param quality The quality level between 1 and 100 that detrmines the
   * level of compression used in the generated JPEG
   * @langversion ActionScript 3.0
   * @playerversion Flash 9.0
   * @tiptext
   */
  void init(int quality, bool ifBnW)
  {
    ifBlackAndWhite = ifBnW;

    if (quality <= 0) {
      quality = 1;
    }
    if (quality > 100) {
      quality = 100;
    }
    int sf = 0;
    if (quality < 50) {
      sf = static_cast<int>(5000 / quality);
    } else {
      sf = static_cast<int>(200 - quality*2);
    }
    // Create tables
    initHuffmanTbl();
    initCategoryNumber();
    initQuantTables(sf);
  }

  /**
   * Created a JPEG image from the specified BitmapData
   *
   * @param image The BitmapData that will be converted into the JPEG format.
   * @return a ByteArray representing the JPEG encoded image data.
   * @langversion ActionScript 3.0
   * @playerversion Flash 9.0
   * @tiptext
   */
  int encode(PixIn* image, int width, int height, PixOut* imageout)
  {
    // Initialize bit writer
    byteout.ptr = imageout;
    byteout.cnt = 0;

    bytenew=0;
    bytepos=8;

    // Add JPEG headers
    writeWord(0xFFD8); // SOI
    writeAPP0();
    writeDQT();
    writeSOF0(width, height);
    writeDHT();
    writeSOS();

     // Encode 8x8 macroblocks
    int DCY=0;
    int DCU=0;
    int DCV=0;
    bytenew=0;
    bytepos=8;

    for (int ypos=0; ypos<height; ypos+=8) {
      for (int xpos=0; xpos<width; xpos+=8) {
        //RGB2YUV(image, xpos, ypos, width);
        getBlock(image, xpos, ypos, width, height);
        DCY = processDU(YDU, fdtbl_Y, DCY, YDC_HT, YAC_HT);
        if (!ifBlackAndWhite) {
            DCU = processDU(UDU, fdtbl_UV, DCU, UVDC_HT, UVAC_HT);
            DCV = processDU(VDU, fdtbl_UV, DCV, UVDC_HT, UVAC_HT);
        }
      }
    }

    // Do the bit alignment of the EOI marker
    if ( bytepos >= 0 ) {
      BitString fillbits;

      fillbits.len = bytepos;
      fillbits.val = (1<<(bytepos))-1;

      //fillbits = {bytepos+1, (1<<(bytepos+1))-1};
      writeBits(fillbits);
    }

    writeWord(0xFFD9); //EOI

    return byteout.cnt;
  }
};

int JPGEncoder::bytenew = 0;
int JPGEncoder::bytepos = 8;

Arr64 JPGEncoder::ZigZag(_ZigZag);
array<int, 17> JPGEncoder::std_dc_luminance_nrcodes(_std_dc_luminance_nrcodes);
array<int, 12> JPGEncoder::std_dc_luminance_values(_std_dc_luminance_values);
array<int, 17> JPGEncoder::std_ac_luminance_nrcodes(_std_ac_luminance_nrcodes);
array<int, 162> JPGEncoder::std_ac_luminance_values(_std_ac_luminance_values);

array<int, 17> JPGEncoder::std_dc_chrominance_nrcodes(_std_dc_chrominance_nrcodes);
array<int, 12> JPGEncoder::std_dc_chrominance_values(_std_dc_chrominance_values);
array<int, 17> JPGEncoder::std_ac_chrominance_nrcodes(_std_ac_chrominance_nrcodes);
array<int, 162> JPGEncoder::std_ac_chrominance_values(_std_ac_chrominance_values);

array<BitString, 65535> JPGEncoder::bitcode;
array<int, 65535> JPGEncoder::category;

} /* **** **** **** **** **** * namespace cv * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */

#endif // !JPEG_ENCODER_HPP_

