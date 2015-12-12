#ifndef TRIK_VIDTRANSCODE_CV_INTERNAL_CV_MOTION_DETECTOR_SEQPASS_HPP_
#define TRIK_VIDTRANSCODE_CV_INTERNAL_CV_MOTION_DETECTOR_SEQPASS_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <cassert>
#include <cmath>
#include <c6x.h>
//#include <vlib.h>

extern "C" {
#include <include/IMG_ycbcr422pl_to_rgb565.h>
#include <include/IMG_conv_5x5_i8_c8s.h>

#include "ti/vlib/src/common/VLIB_memory.h"
#include <ti/vlib/src/VLIB_image_rescale/VLIB_image_rescale.h>
#include <ti/vlib/src/VLIB_convertUYVYsemipl_to_YUVpl/VLIB_convertUYVYsemipl_to_YUVpl.h>
#include <ti/vlib/src/VLIB_Canny_Edge_Detection/VLIB_Canny_Edge_Detection.h>
#include <ti/vlib/src/VLIB_xyGradientsAndMagnitude/VLIB_xyGradientsAndMagnitude.h>

#include <ti/vlib/vlib.h>
}

#include "internal/stdcpp.hpp"
#include "trik_vidtranscode_cv.h"
#include "internal/cv_hsv_range_detector.hpp"


/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace cv /* **** **** **** **** **** */ {


#warning Eliminate global var
#define CAMERA_NOISE_S16      0x0A00        /* SQ12.3 */
#define THRESHOLD_FACTOR_S16  0x31ff        /* SQ4.11 */

SET_ALIGN(runningMean, 8)
static int16_t runningMean[320*240];

SET_ALIGN(runningVar, 8)
static int16_t runningVar[320*240];

static const short s_coeff[5] = { 0x2000, 0x2BDD, -0x0AC5, -0x1658, 0x3770 };
static const uint8_t s_minBlobArea = 625;
static const uint8_t s_connected8Flag = 1;


static uint16_t s_harrisScore[320*240];
static int8_t s_corners[320*240];
static uint8_t s_buffer[200];

template <>
class MotionDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422P, TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X> : public CVAlgorithm
{
  private:

    TrikCvImageDesc m_inImageDesc;
    TrikCvImageDesc m_outImageDesc;

    uint8_t m_scaleFactor;
    
    uint8_t *m_y_in;
    uint8_t *m_cb_in;
    uint8_t *m_cr_in;

    uint8_t *m_y_out;
    uint8_t *m_cb_out;
    uint8_t *m_cr_out;
    
    int16_t *m_gradX;
    int16_t *m_gradY;
    int16_t *m_mag;
    uint8_t *m_buffer;
	  
	  uint32_t *m_mask32packed;

    void *m_CCBuf;
	  int32_t m_maxBytesRequired;
    int32_t m_sizeOfCCHandle;
    
    VLIB_CCHandle   *m_handle;
    
    int32_t m_status;
    int32_t m_numCCs;
    
    int32_t m_targetX;
    int32_t m_targetY;
    int32_t m_targetSize;

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

    void __attribute__((always_inline)) drawCornerHighlight(const int32_t _srcCol, 
                                                            const int32_t _srcRow,
                                                            const TrikCvImageBuffer& _outImage,
                                                            const uint32_t _rgb888)
    {
      const int32_t widthBot  = 0;
      const int32_t widthTop  = m_inImageDesc.m_width-1;
      const int32_t heightBot = 0;
      const int32_t heightTop = m_inImageDesc.m_height-1;

      drawOutputPixelBound(_srcCol-1, _srcRow-1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol-1, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol-1, _srcRow+1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol, _srcRow-1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol, _srcRow+1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol+1, _srcRow-1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol+1, _srcRow, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
      drawOutputPixelBound(_srcCol+1, _srcRow+1, widthBot, widthTop, heightBot, heightTop, _outImage, _rgb888);
    }

    void __attribute__((always_inline)) drawHorizontalLine(const int32_t _srcCol1,
                                                           const int32_t _srcCol2,
                                                           const int32_t _srcRow,
                                                           const TrikCvImageBuffer& _outImage,
                                                           const uint32_t _rgb888) {
      uint16_t *restrict img_ptr = reinterpret_cast<uint16_t*>(_outImage.m_ptr) + m_outImageDesc.m_width*_srcRow + _srcCol1;
      for(int i = _srcCol1; i < _srcCol2; i++) {
        writeOutputPixel(img_ptr++, _rgb888);
      }
    }

    void __attribute__((always_inline)) drawVerticalLine(const int32_t _srcCol,
                                                         const int32_t _srcRow1,
                                                         const int32_t _srcRow2,
                                                         const TrikCvImageBuffer& _outImage,
                                                         const uint32_t _rgb888) {
      uint16_t *restrict img_ptr = reinterpret_cast<uint16_t*>(_outImage.m_ptr) + m_outImageDesc.m_width*_srcRow1 + _srcCol;
      for(int i = _srcRow1; i < _srcRow2; i++) {
        writeOutputPixel(img_ptr, _rgb888);
        img_ptr += m_outImageDesc.m_width;
      }
    }

    void __attribute__((always_inline)) drawRectangleBound(const int32_t _srcCol1,
                                                           const int32_t _srcCol2,
                                                           const int32_t _srcRow1,
                                                           const int32_t _srcRow2,
                                                           const TrikCvImageBuffer& _outImage,
                                                           const uint32_t _rgb888) {
      const int32_t widthBot  = 0;
      const int32_t widthTop  = m_inImageDesc.m_width-1;
      const int32_t heightBot = 0;
      const int32_t heightTop = m_inImageDesc.m_height-1;
      
      const int32_t srcCol1 = range<int32_t>(widthBot, _srcCol1, widthTop);
      const int32_t srcCol2 = range<int32_t>(widthBot, _srcCol2, widthTop);
      const int32_t srcRow1 = range<int32_t>(heightBot, _srcRow1, heightTop);
      const int32_t srcRow2 = range<int32_t>(heightBot, _srcRow2, heightTop);
      
      drawHorizontalLine(srcCol1, srcCol2, srcRow1, _outImage, _rgb888);
      drawHorizontalLine(srcCol1, srcCol2, srcRow2, _outImage, _rgb888);
      drawVerticalLine(srcCol1, srcRow1, srcRow2, _outImage, _rgb888);
      drawVerticalLine(srcCol2, srcRow1, srcRow2, _outImage, _rgb888);
    }

    void __attribute__((always_inline)) getMaxCC(VLIB_CC *max_CC)
    {
      VLIB_getNumCCs(m_handle, &m_numCCs);

      int max_area  = -1;
      VLIB_CC vlibBlob;
      for(int i = 0; i < m_numCCs; i++) {
        VLIB_getCCFeatures(m_handle, &vlibBlob, i);
        if(vlibBlob.area > max_area) {
          max_area = vlibBlob.area;
          *max_CC = vlibBlob;
        }
      }
    }


    void DEBUG_INLINE processYCbCr(const TrikCvImageBuffer& _inImage, TrikCvImageBuffer& _outImage)
    {
      const uint32_t width  = m_inImageDesc.m_width;
      const uint32_t height = m_inImageDesc.m_height;

//separate Cb Cr
      uint8_t* restrict separated_cb = reinterpret_cast<uint8_t*>(m_cb_in);
      uint8_t* restrict separated_cr = reinterpret_cast<uint8_t*>(m_cr_in);
      const uint8_t* restrict CbCr = reinterpret_cast<const uint8_t*>(_inImage.m_ptr + 
                                                                       m_inImageDesc.m_lineLength*height);
      VLIB_convertUYVYsemipl_to_YUVpl(CbCr, 
                                      width, width, height, 
                                      separated_cb, separated_cr);

      uint8_t *restrict inptr = reinterpret_cast<uint8_t *>(_inImage.m_ptr);
      uint8_t *pOut           = m_y_in;

//Frog vision
#if 1
      int16_t    thresholdFactor = THRESHOLD_FACTOR_S16 / 2;
      int16_t    thresholdGlobal = CAMERA_NOISE_S16 / 32;

      VLIB_subtractBackgroundS16(m_mask32packed,  /* new foreground mask to be computed */
                                 reinterpret_cast<const uint8_t*>(_inImage.m_ptr), /* the newest luma frame */
                                 runningMean,     /* running mean */
                                 runningVar,      /* running variance */
                                 thresholdGlobal, /* global threshold */
                                 thresholdFactor, /* multiplicative factor for the image threshold */
                                 width*height );  /* no. of pixels to be processed */

      /* initialize the running mean "image" */
      VLIB_initMeanWithLumaS16(runningMean, reinterpret_cast<const uint8_t*>(_inImage.m_ptr), width*height);

      /* initialize the running variance "image" */
      VLIB_initVarWithConstS16(runningVar, thresholdGlobal, width*height);

      VLIB_erode_bin_square(reinterpret_cast<uint8_t *>(m_mask32packed), reinterpret_cast<uint8_t *>(m_mask32packed), width*height, width);
      VLIB_dilate_bin_cross(reinterpret_cast<uint8_t *>(m_mask32packed), reinterpret_cast<uint8_t *>(m_mask32packed), width*height, width);

      m_status = VLIB_createConnectedComponentsList(m_handle,
                                                    width, height,
                                                    m_mask32packed,
                                                    s_minBlobArea,
                                                    s_connected8Flag);

      VLIB_getNumCCs(m_handle, &m_numCCs);
      
      VLIB_CC vlibBlob;
      getMaxCC(&vlibBlob);

      m_targetX    = (vlibBlob.xmin + vlibBlob.xmax)/2;
      m_targetY    = (vlibBlob.ymin + vlibBlob.ymax)/2;
      m_targetSize = vlibBlob.area;

/*
      m_status = VLIB_createCCMap8Bit(m_handle,
                                      reinterpret_cast<uint8_t *>(m_y_in),
                                      width,
                                      height);
*/

      VLIB_unpackMask32(m_mask32packed, m_y_in, width*height);
      pOut = m_y_in;
      for(int i = 0; i < width*height; i++)
        *(pOut++) *= 255;


#endif

#if 1
//scale that muttersaftsack down!
      uint8_t* y_out;
      uint8_t* cb_out;
      uint8_t* cr_out;

      if(m_scaleFactor > 1) {
        const uint8_t* restrict scale_y_in  = reinterpret_cast<const uint8_t*>(m_y_in);
        uint8_t* restrict scale_y_out = reinterpret_cast<uint8_t*>(m_y_out);
        VLIB_image_rescale(scale_y_in, scale_y_out, (1 << 13), width, height, 3);

        const uint8_t* restrict scale_cb_in  = reinterpret_cast<const uint8_t*>(m_cb_in);
        uint8_t* restrict scale_cb_out = reinterpret_cast<uint8_t*>(m_cb_out);
        VLIB_image_rescale(scale_cb_in, scale_cb_out, (1 << 13), width, height, 3);

        const uint8_t* restrict scale_cr_in  = reinterpret_cast<const uint8_t*>(m_cr_in);
        uint8_t* restrict scale_cr_out = reinterpret_cast<uint8_t*>(m_cr_out);
        VLIB_image_rescale(scale_cr_in, scale_cr_out, (1 << 13), width, height, 3);

        y_out   = m_y_out;
        cb_out  = m_cb_out;
        cr_out  = m_cr_out;
      } else {
        y_out   = m_y_in;
        cb_out  = m_cb_in;
        cr_out  = m_cr_in;
      }

      //yuv422pl to rgb565 to outImage
      const short* restrict coeff = s_coeff;
      unsigned short* rgb565_out  = reinterpret_cast<unsigned short*>(_outImage.m_ptr);
      IMG_ycbcr422pl_to_rgb565(coeff, reinterpret_cast<const unsigned char*>(y_out), 
                                      reinterpret_cast<const unsigned char*>(cb_out), 
                                      reinterpret_cast<const unsigned char*>(cr_out), rgb565_out, width*height);

      if(m_numCCs > 0) {
        drawRectangleBound(vlibBlob.xmin, vlibBlob.xmax, vlibBlob.ymin, vlibBlob.ymax, _outImage, 0xffff00);
        drawCornerHighlight(m_targetX, m_targetY, _outImage, 0xff0000);
      }
#endif
    }


  public:
    virtual bool setup(const TrikCvImageDesc& _inImageDesc,
                       const TrikCvImageDesc& _outImageDesc,
                       int8_t* _fastRam, size_t _fastRamSize)
    {
      m_inImageDesc  = _inImageDesc;
      m_outImageDesc = _outImageDesc;

      #define max(x, y) x > y ? x : y
      m_scaleFactor = std::round(max((double)m_inImageDesc.m_width/m_outImageDesc.m_width, (double)m_inImageDesc.m_height/m_outImageDesc.m_height));

      m_y_in  = (uint8_t *)VLIB_memalign(64, m_inImageDesc.m_width*m_inImageDesc.m_height*sizeof(uint8_t));
      m_cb_in = (uint8_t *)VLIB_memalign(64, m_inImageDesc.m_width*m_inImageDesc.m_height*sizeof(uint8_t));
      m_cr_in = (uint8_t *)VLIB_memalign(64, m_inImageDesc.m_width*m_inImageDesc.m_height*sizeof(uint8_t));

      m_y_out  = (uint8_t *)VLIB_memalign(64, m_outImageDesc.m_width*m_outImageDesc.m_height*sizeof(uint8_t));
      m_cb_out = (uint8_t *)VLIB_memalign(64, m_outImageDesc.m_width*m_outImageDesc.m_height*sizeof(uint8_t));
      m_cr_out = (uint8_t *)VLIB_memalign(64, m_outImageDesc.m_width*m_outImageDesc.m_height*sizeof(uint8_t));
/*      
      m_gradX    = (int16_t *)VLIB_memalign(32, m_inImageDesc.m_width*m_inImageDesc.m_height*sizeof(int16_t));
      m_gradY    = (int16_t *)VLIB_memalign(32, m_inImageDesc.m_width*m_inImageDesc.m_height*sizeof(int16_t));
      m_mag      = (int16_t *)VLIB_memalign(32, m_inImageDesc.m_width*m_inImageDesc.m_height*sizeof(int16_t));
      m_buffer   = (uint8_t *)VLIB_memalign(32, m_inImageDesc.m_width*m_inImageDesc.m_height*sizeof(uint8_t));
*/    

//Component Linker Zone      
      VLIB_calcConnectedComponentsMaxBufferSize(m_inImageDesc.m_width, m_inImageDesc.m_height,
                                                s_minBlobArea,
                                                &m_maxBytesRequired);

      m_CCBuf    = (void *)VLIB_memalign(32, m_maxBytesRequired);

      m_sizeOfCCHandle =  VLIB_GetSizeOfCCHandle();
      m_handle = (VLIB_CCHandle *)VLIB_memalign(8, m_sizeOfCCHandle);

      /* INITIALIZE FOREGROUND MASK AND MEMORY BUFFERS*/
      m_status = VLIB_initConnectedComponentsList(m_handle, m_CCBuf, m_maxBytesRequired);

//Mask! 
      m_mask32packed = (uint32_t *)VLIB_memalign(8, m_inImageDesc.m_width*m_inImageDesc.m_height*sizeof(uint32_t));

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
          processYCbCr(_inImage, _outImage);
        }
#ifdef DEBUG_REPEAT
      } // repeat
#endif

      if(m_numCCs > 0) {
        _outArgs.target[0].x = ((m_targetX - static_cast<int32_t>(m_inImageDesc.m_width) /2) * 100*2) / static_cast<int32_t>(m_inImageDesc.m_width);
        _outArgs.target[0].y = ((m_targetY - static_cast<int32_t>(m_inImageDesc.m_height) /2) * 100*2) / static_cast<int32_t>(m_inImageDesc.m_height);
        _outArgs.target[0].size = 100*m_targetSize/(m_inImageDesc.m_height*m_inImageDesc.m_width);
      } else {
        _outArgs.target[0].x = 0;
        _outArgs.target[0].y = 0;
        _outArgs.target[0].size = 0;
      }

      return true;
    }
};

uint16_t* restrict MotionDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422P,
                                TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X>::s_mult43_div = NULL;
uint16_t* restrict MotionDetector<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422P,
                                TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X>::s_mult255_div = NULL;


} /* **** **** **** **** **** * namespace cv * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */


#endif // !TRIK_VIDTRANSCODE_CV_INTERNAL_CV_MOTION_DETECTOR_SEQPASS_HPP_

