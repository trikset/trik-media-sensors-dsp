  #ifndef TRIK_VIDTRANSCODE_CV_INTERNAL_CV_BALL_DETECTOR_SEQPASS_HPP_
#define TRIK_VIDTRANSCODE_CV_INTERNAL_CV_BALL_DETECTOR_SEQPASS_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <cassert>
#include <cmath>
#include <c6x.h>
//#include <vlib.h>

extern "C" {
#include <include/IMG_ycbcr422pl_to_rgb565.h>
#include <ti/vlib/src/VLIB_image_rescale/VLIB_image_rescale.h>
}

#include "internal/stdcpp.hpp"
#include "trik_vidtranscode_cv.h"
#include "internal/cv_hsv_range_detector.hpp"


/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace cv /* **** **** **** **** **** */ {



#warning Eliminate global var
static uint8_t s_cb_in[640*480];
static uint8_t s_cr_in[640*480];
static uint8_t s_y_out[320*240];
static uint8_t s_cb_out[320*240];
static uint8_t s_cr_out[320*240];
static const short s_coeff[5] = { 0x2000, 0x2BDD, -0x0AC5, -0x1658, 0x3770 };

template <>
class BallDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422P, TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X> : public CVAlgorithm
{
  private:

    TrikCvImageDesc m_inImageDesc;
    TrikCvImageDesc m_outImageDesc;

    static uint16_t* restrict s_mult43_div;  // allocated from fast ram
    static uint16_t* restrict s_mult255_div; // allocated from fast ram

    static void __attribute__((always_inline)) writeOutputPixel(uint16_t* restrict _rgb565ptr,
                                                                const uint32_t _rgb888)
    {
      *_rgb565ptr = ((_rgb888>>19)&0x001f) | ((_rgb888>>5)&0x07e0) | ((_rgb888<<8)&0xf800);
    }

    void __attribute__((always_inline)) drawOutputPixelBound(const int32_t _srcCol,
                                                             const int32_t _srcRow,
                                                             const int32_t _srcColBot,
                                                             const int32_t _srcColTop,
                                                             const int32_t _srcRowBot,
                                                             const int32_t _srcRowTop,
                                                             const TrikCvImageBuffer& _outImage,
                                                             const uint32_t _rgb888) const
    {
      const int32_t srcCol = range<int32_t>(_srcColBot, _srcCol, _srcColTop);
      const int32_t srcRow = range<int32_t>(_srcRowBot, _srcRow, _srcRowTop);

      const int32_t dstRow = srcRow;
      const int32_t dstCol = srcCol;

      const uint32_t dstOfs = dstRow*m_outImageDesc.m_lineLength + dstCol*sizeof(uint16_t);
      writeOutputPixel(reinterpret_cast<uint16_t*>(_outImage.m_ptr+dstOfs), _rgb888);
    }

    void DEBUG_INLINE convertImageYuyvToRgb(const TrikCvImageBuffer& _inImage, TrikCvImageBuffer& _outImage)
    {
      const uint32_t width          = m_inImageDesc.m_width;
      const uint32_t height         = m_inImageDesc.m_height;
      const uint32_t imgSize        = width*height;

//separate Cb Cr
      uint8_t* restrict cb   = reinterpret_cast<uint8_t*>(s_cb_in);
      uint8_t* restrict cr   = reinterpret_cast<uint8_t*>(s_cr_in);
      const uint16_t* restrict CbCr = reinterpret_cast<const uint16_t*>(_inImage.m_ptr + 
                                                                    m_inImageDesc.m_lineLength*m_inImageDesc.m_height);
      #pragma MUST_ITERATE(8, ,8)
      for(int i = 0; i < imgSize; i++) {
          *(cb++) = static_cast<uint8_t>(*CbCr);
          *(cr++) = static_cast<uint8_t>((*CbCr) >> 8);
          CbCr++;
      }

//scale that motherf8ckers down!

      const uint8_t* restrict cr_in  = reinterpret_cast<const uint8_t*>(s_cr_in);
      uint8_t* restrict cr_out = reinterpret_cast<uint8_t*>(s_cr_out);
      VLIB_image_rescale(cr_in, cr_out, (1 << 13), width, height, 3);
      
      const uint8_t* restrict cb_in  = reinterpret_cast<const uint8_t*>(s_cb_in);
      uint8_t* restrict cb_out = reinterpret_cast<uint8_t*>(s_cb_out);
      VLIB_image_rescale(cb_in, cb_out, (1 << 13), width, height, 3);

      const uint8_t* restrict y_in  = reinterpret_cast<const uint8_t*>(_inImage.m_ptr);
      uint8_t* restrict y_out = reinterpret_cast<uint8_t*>(s_y_out);
      VLIB_image_rescale(y_in, y_out, (1 << 13), width, height, 3);

//yuv422pl to rgb565
      const short* restrict coeff = s_coeff;
      const unsigned char* restrict y_out2 = reinterpret_cast<const unsigned char*>(s_y_out);
      const unsigned char* restrict cb_out2  = reinterpret_cast<const unsigned char*>(s_cb_out);
      const unsigned char* restrict cr_out2  = reinterpret_cast<const unsigned char*>(s_cr_out);
      unsigned short* rgb565_out           = reinterpret_cast<unsigned short*>(_outImage.m_ptr);
      IMG_ycbcr422pl_to_rgb565(coeff, y_out2, cb_out2, cr_out2, rgb565_out, width*height);

    }


  public:
    virtual bool setup(const TrikCvImageDesc& _inImageDesc,
                       const TrikCvImageDesc& _outImageDesc,
                       int8_t* _fastRam, size_t _fastRamSize)
    {
      m_inImageDesc  = _inImageDesc;
      m_outImageDesc = _outImageDesc;

      if (   m_inImageDesc.m_width < 0
          || m_inImageDesc.m_height < 0
          || m_inImageDesc.m_width  % 32 != 0
          || m_inImageDesc.m_height % 4  != 0)
        return false;

      return true;
    }

    virtual bool run(const TrikCvImageBuffer& _inImage, TrikCvImageBuffer& _outImage,
                     const TrikCvAlgInArgs& _inArgs, TrikCvAlgOutArgs& _outArgs)
    {
      if (m_inImageDesc.m_height * m_inImageDesc.m_lineLength > _inImage.m_size)
        return false;
      if (m_outImageDesc.m_height * m_outImageDesc.m_lineLength > _outImage.m_size)
        return false;
      _outImage.m_size = m_outImageDesc.m_height * m_outImageDesc.m_lineLength;


#ifdef DEBUG_REPEAT
      for (unsigned repeat = 0; repeat < DEBUG_REPEAT; ++repeat) {
#endif
        if (m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0) {
          convertImageYuyvToRgb(_inImage, _outImage);
        }
#ifdef DEBUG_REPEAT
      } // repeat
#endif

      _outArgs.targetX = 0;
      _outArgs.targetY = 0;
      _outArgs.targetSize = 0;

      return true;
    }
};

uint16_t* restrict BallDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422P,
                                TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X>::s_mult43_div = NULL;
uint16_t* restrict BallDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422P,
                                TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X>::s_mult255_div = NULL;


} /* **** **** **** **** **** * namespace cv * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */


#endif // !TRIK_VIDTRANSCODE_CV_INTERNAL_CV_BALL_DETECTOR_SEQPASS_HPP_

