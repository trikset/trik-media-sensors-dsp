#ifndef TRIK_VIDTRANSCODE_CV_INTERNAL_JPEG_ENCODE_HPP_
#define TRIK_VIDTRANSCODE_CV_INTERNAL_JPEG_ENCODE_HPP_

#ifndef __cplusplus
#error C++-only header
#endif

#include <cassert>
#include <vector>
#include <c6x.h>

#include "internal/stdcpp.hpp"
#include "trik_vidtranscode_cv.h"


/* **** **** **** **** **** */ namespace trik /* **** **** **** **** **** */ {

/* **** **** **** **** **** */ namespace cv /* **** **** **** **** **** */ {


#if 0
#define DEBUG_INLINE __attribute__((noinline))
#else
#define DEBUG_INLINE __attribute__((always_inline))
#endif
//#define DEBUG_REPEAT 20


} /* **** **** **** **** **** * namespace cv * **** **** **** **** **** */

} /* **** **** **** **** **** * namespace trik * **** **** **** **** **** */

// include one of implementations
#include "internal/jpeg_encoder_reference.hpp"

#endif // !TRIK_VIDTRANSCODE_CV_INTERNAL_JPEG_ENCODE_HPP_
