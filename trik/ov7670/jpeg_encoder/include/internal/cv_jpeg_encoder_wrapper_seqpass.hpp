#ifndef TRIK_VIDTRANSCODE_CV_INTERNAL_CV_JPEG_ENCODER_SEQPASS_HPP_
#define TRIK_VIDTRANSCODE_CV_INTERNAL_CV_JPEG_ENCODER_SEQPASS_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <cassert>
#include <cmath>
#include <c6x.h>
#include <string.h>
#include <stdio.h>
//#include <stdint.h>

#include "internal/stdcpp.hpp"
#include "internal/jpeg_encoder.hpp"
#include "trik_vidtranscode_cv.h"


/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace cv /* **** **** **** **** **** */ {

template <>
class JPGEncoderWrapper<TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_YUV422P, TRIK_VIDTRANSCODE_CV_VIDEO_FORMAT_RGB565X> : public CVAlgorithm
{
  private:
    JPGEncoder jpgEncoder;

    TrikCvImageDesc m_inImageDesc;
    TrikCvImageDesc m_outImageDesc;

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

      //jpgEncoder.init(10);

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

      jpgEncoder.init(_inArgs.jpgImageQuality, _inArgs.ifBlackAndWhite);//no checks needed for they are conducted in jpgEncoder.
      
#ifdef DEBUG_REPEAT
      for (unsigned repeat = 0; repeat < DEBUG_REPEAT; ++repeat) {
#endif
        if (m_inImageDesc.m_height > 0 && m_inImageDesc.m_width > 0) {

          _outArgs.size = jpgEncoder.encode(reinterpret_cast<uint8_t* >(_inImage.m_ptr), m_inImageDesc.m_width, m_inImageDesc.m_height,
                            reinterpret_cast<uint8_t* >(_outImage.m_ptr));
        }
#ifdef DEBUG_REPEAT
      } // repeat
#endif

      return true;
    }
};

} /* **** **** **** **** **** * namespace cv * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */


#endif // !TRIK_VIDTRANSCODE_CV_INTERNAL_CV_JPEG_ENCODER_SEQPASS_HPP_

