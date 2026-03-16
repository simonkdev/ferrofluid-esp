
/* 
 Detect movement in sequential images using background subtraction.
 
 Very small (96x96) bitmaps are used both to provide image smoothing to reduce spurious motion changes 
 and to enable rapid processing
 Bitmaps can either be color or grayscale. Color requires triple memory
 of grayscale and more processing.

 The amount of change between images will depend on the frame rate.
 A faster frame rate will need a higher sensitivity

 When frame size is changed the OV2640 outputs a few glitched frames whilst it 
 makes the transition. These could be interpreted as spurious motion.

 Machine Learning can be incorporated to further discriminate when motion detection 
 has occurred by classsifying whether the object in the frame is of a particular
 type of interest, eg a human, animal, vehicle etc. 
 
 s60sc 2020, 2023, 2025
*/

#include "appGlobals.h"

//#if INCLUDE_TINYML
//#include TINY_ML_LIB
//#endif

#define RESIZE_DIM 96  // dimensions of resized motion bitmap
#define RESIZE_DIM_SQ (RESIZE_DIM * RESIZE_DIM) // pixels in bitmap
#define INACTIVE_COLOR 96 // color for inactive motion pixel
#define JPEG_QUAL 80 // % quality for generated motion detect jpeg
  
// motion recording parameters
bool dbgMotion = false;
int detectMotionFrames = 5; // min sequence of changed frames to confirm motion 
int detectNightFrames = 10; // frames of sequential darkness to avoid spurious day / night switching
// define region of interest, ie exclude top and bottom of image from movement detection if required
// divide image into detectNumBands horizontal bands, define start and end bands of interest, 1 = top
int detectNumBands = 10;
int detectStartBand = 3;
int detectEndBand = 8; // inclusive
int detectChangeThreshold = 15; // min difference in pixel comparison to indicate a change
uint8_t colorDepth; // set by depthColor config
static size_t stride;
bool mlUse = false; // whether to use ML for motion detection, requires INCLUDE_TINYML to be true
float mlProbability = 0.8; // minimum probability (0.0 - 1.0) for positive classification

uint8_t lightLevel; // Current ambient light level 
uint8_t nightSwitch = 20; // initial white level % for night/day switching
float motionVal = 8.0; // initial motion sensitivity setting
uint8_t* motionJpeg = NULL;
size_t motionJpegLen = 0;
static uint8_t* currBuff = NULL;

#ifndef AUXILIARY

static bool jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale);
//#endif

/**********************************************************************************/


bool isNight(uint8_t nightSwitch) {
  // check if night time for suspending recording
  // or for switching relay if enabled
  static bool nightTime = false;
  static uint16_t nightCnt = 0;
  if (nightTime) {
    if (lightLevel > nightSwitch) {
      // light image
      nightCnt--;
      // signal day time after given sequence of light frames
      if (nightCnt == 0) {
        nightTime = false;
        LOG_INF("Day time");
      }
    }
  } else {
    if (lightLevel < nightSwitch) {
      // dark image
      nightCnt++;
      // signal night time after given sequence of dark frames
      if (nightCnt > detectNightFrames) {
        nightTime = true;     
        LOG_INF("Night time"); 
      }
    }
  } 
  return nightTime;
}

static void rescaleImage(const uint8_t* input, int inputWidth, int inputHeight, uint8_t* output, int outputWidth, int outputHeight) {
  // use bilinear interpolation to resize image
  float xRatio = (float)inputWidth / (float)outputWidth;
  float yRatio = (float)inputHeight / (float)outputHeight;

  for (int i = 0; i < outputHeight; ++i) {
    for (int j = 0; j < outputWidth; ++j) {
      int xL = (int)floor(xRatio * j);
      int yL = (int)floor(yRatio * i);
      int xH = (int)ceil(xRatio * j);
      int yH = (int)ceil(yRatio * i);
      float xWeight = xRatio * j - xL;
      float yWeight = yRatio * i - yL;
      for (int channel = 0; channel < colorDepth; ++channel) {
        uint8_t a = input[(yL * inputWidth + xL) * colorDepth + channel];
        uint8_t b = input[(yL * inputWidth + xH) * colorDepth + channel];
        uint8_t c = input[(yH * inputWidth + xL) * colorDepth + channel];
        uint8_t d = input[(yH * inputWidth + xH) * colorDepth + channel];

        float pixel = a * (1 - xWeight) * (1 - yWeight) + b * xWeight * (1 - yWeight)
                    + c * yWeight * (1 - xWeight) + d * xWeight * yWeight;
        output[(i * outputWidth + j) * colorDepth + channel] = (uint8_t)pixel;
      }
    }
  }
}

static void rgbToGray(uint8_t* buffer, int width, int height) {
  // convert rgb buffer to grayscale in place
  for (int i = 0; i < width * height; ++i) {
    int index = i * 3;
    // Calculate grayscale value using luminance formula
    buffer[i] = (uint8_t)(((77 * buffer[index]) + (150 * buffer[index + 1]) + (29 * buffer[index + 2])) >> 8);
  }
}


bool checkMotion(camera_fb_t* fb, bool motionStatus, bool lightLevelOnly) {
  // check difference between current and previous image (subtract background)
  // convert image from JPEG to downscaled RGB888 or 8 bit grayscale bitmap
  if (fsizePtr > FRAMESIZE_SXGA) return false;
  uint32_t dTime = millis();
  uint32_t lux = 0;
  static uint32_t motionCnt = 0;
  static uint8_t fsizePtrPrev = 255; // initially invalid
  static uint8_t scaling, downsize;
  static uint16_t reducer;
  static int sampleWidth = 0, sampleHeight = 0;
  static uint8_t* rgbBuf = (uint8_t*)heap_caps_aligned_calloc(16, 1, frameData[FRAMESIZE_SXGA].frameWidth * frameData[FRAMESIZE_SXGA].frameHeight * RGB888_BYTES / 8, MALLOC_CAP_SPIRAM); // must be 16 byte aligned. Max size, no need to free

  // calculate parameters for sample size when resolution changes
  if (fsizePtr != fsizePtrPrev) {
    fsizePtrPrev = fsizePtr;
    scaling = frameData[fsizePtr].scaleFactor; 
    reducer = frameData[fsizePtr].sampleRate;
    downsize = pow(2, scaling) * reducer;
    stride = (colorDepth == RGB888_BYTES) ? GRAYSCALE_BYTES : RGB888_BYTES; // stride is inverse of colorDepth
    sampleWidth = frameData[fsizePtr].frameWidth / downsize;
    sampleHeight = frameData[fsizePtr].frameHeight / downsize;
  }
  if (!jpg2rgb((uint8_t*)fb->buf, fb->len, rgbBuf, scaling)) return motionStatus;
//#endif

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
  if (colorDepth == GRAYSCALE_BYTES) rgbToGray(rgbBuf, sampleWidth, sampleHeight);
#endif
  LOG_VRB("JPEG to rescaled %s bitmap conversion %u bytes in %lums", colorDepth == RGB888_BYTES ? "color" : "grayscale", sampleWidth * sampleHeight * colorDepth, millis() - dTime);
  
  // allocate buffer space on heap
  size_t resizeDimLen = RESIZE_DIM_SQ * colorDepth; // byte size of bitmap
  if (motionJpeg == NULL) motionJpeg = (uint8_t*)ps_malloc(32 * 1024);
  if (currBuff == NULL) currBuff = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
  static uint8_t* prevBuff = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
  static uint8_t* changeMap = (uint8_t*)ps_malloc(RESIZE_DIM_SQ * RGB888_BYTES);
  
  dTime = millis();
  rescaleImage(rgbBuf, sampleWidth, sampleHeight, currBuff, RESIZE_DIM, RESIZE_DIM);
  LOG_VRB("Bitmap rescale to %u bytes in %lums", resizeDimLen, millis() - dTime);
  // compare each pixel in current frame with previous frame 
  dTime = millis();
  int changeCount = 0;
  // set horizontal region of interest in image 
  uint16_t startPixel = (RESIZE_DIM*(detectStartBand-1)/detectNumBands) * RESIZE_DIM * colorDepth;
  uint16_t endPixel = (RESIZE_DIM*(detectEndBand)/detectNumBands) * RESIZE_DIM * colorDepth;
  int moveThreshold = ((endPixel-startPixel)/colorDepth) * (11-motionVal)/100; // number of changed pixels that constitute a movement
  for (int i = 0; i < resizeDimLen; i += colorDepth) {
    uint16_t currPix = 0, prevPix = 0;
    for (int j = 0; j < colorDepth; j++) {
      currPix += currBuff[i + j];
      prevPix += prevBuff[i + j];
    }
    currPix /= colorDepth;
    prevPix /= colorDepth;
    lux += currPix; // for calculating light level
    uint8_t pixVal = 255; // show active changed pixel as bright red color in changeMap image
    // set up display image for motion tracking debug
    if (dbgMotion) for (int j = 0; j < RGB888_BYTES; j++) changeMap[(i * stride) + j] = currPix; // grayscale
    // determine pixel change status
    if (abs((int)currPix - (int)prevPix) > detectChangeThreshold) {
      if (i > startPixel && i < endPixel) changeCount++; // number of changed pixels
      else pixVal = 80; // show inactive changed pixel as dark red color in changeMap image
      if (dbgMotion) {
        changeMap[(i * stride) + 2] = pixVal;
        for (int j = 0; j < RGB888_BYTES - 1; j++) changeMap[(i * stride) + j] = 0;
      }
    }
  }
  lightLevel = (lux*100)/(RESIZE_DIM_SQ*255); // light value as a %
  nightTime = isNight(nightSwitch);
  memcpy(prevBuff, currBuff, resizeDimLen); // save image for next comparison 
  LOG_VRB("Detected %u changes, threshold %u, light level %u, in %lums", changeCount, moveThreshold, lightLevel, millis() - dTime);
  if (lightLevelOnly) return false; // no motion checking, only calc of light level
  if (dbgMotion) {
    // show motion detection during streaming for tuning
    if (!motionJpegLen) {
      // ready to setup next movement map for streaming
      dTime = millis();
      // build jpeg of changeMap for debug streaming
      uint8_t* jpg_buf = NULL;
      if (!fmt2jpg(changeMap, resizeDimLen, RESIZE_DIM, RESIZE_DIM, PIXFORMAT_RGB888, JPEG_QUAL, &jpg_buf, &motionJpegLen))
        LOG_WRN("motionDetect: fmt2jpg() failed"); 
      memcpy(motionJpeg, jpg_buf, motionJpegLen); 
      free(jpg_buf);
      jpg_buf = NULL;
//#endif
      xSemaphoreGive(motionSemaphore);
      LOG_VRB("Created changeMap JPEG %d bytes in %lums", motionJpegLen, millis() - dTime);
    }
  } else {
    // normal motion detection
    dTime = millis();
    if (!nightTime && changeCount > moveThreshold) {
      LOG_VRB("### Change detected");
      motionCnt++; // number of consecutive changes
      // need minimum sequence of changes to signal valid movement
      if (!motionStatus && motionCnt >= detectMotionFrames) {
        LOG_VRB("***** Motion - START");
        motionStatus = true; // motion started
        dTime = millis();
      } 
    } else motionCnt = 0;
  
    if (motionStatus && !motionCnt) {
      // insufficient change or motion not classified
      LOG_VRB("***** Motion - STOP");
      motionStatus = false; // motion stopped
    } 
    if (motionStatus) LOG_VRB("*** Motion - ongoing %u frames", motionCnt);
  }
  
  if (dbgVerbose) checkMemory();  
  LOG_VRB("============================");
  // motionStatus indicates whether motion previously ongoing or not
  return nightTime ? false : motionStatus;
}


#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)

// based on jpg2rgb888() from esp32-camera/to_bmp.c for access to rescaling

static uint8_t work[3100]; // Default size is 3.1kB for JPEG decoder

static bool jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale) {
  esp_jpeg_image_cfg_t jpeg_cfg = {
      .indata = (uint8_t *)src,
      .indata_size = src_len,
      .outbuf = out,
      .outbuf_size = UINT32_MAX, // sic @todo: this is very bold assumption, keeping this like this for now, not to break existing code
      .out_format = JPEG_IMAGE_FORMAT_RGB888,
      .out_scale = (esp_jpeg_image_scale_t)scale,
      .flags = {.swap_color_bytes = 0},
      .advanced = {
        .working_buffer = work,
        .working_buffer_size = sizeof(work)
      }
  };
  esp_jpeg_image_output_t output_img = {};
  esp_err_t res = esp_jpeg_decode(&jpeg_cfg, &output_img);
  if (res != ESP_OK) LOG_WRN("jpg2rgb failure: %s", espErrMsg(res)); 
  return (res == ESP_OK) ? true : false;
}

#else

// for arduino-esp32 versions 3.2.1 or earlier

/************* copied and modified from esp32-camera/to_bmp.c to access jpg_scale_t *****************/

typedef struct {
  uint16_t width;
  uint16_t height;
  uint16_t data_offset;
  const uint8_t *input;
  uint8_t *output;
} rgb_jpg_decoder;

static bool _rgb_write(void * arg, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t *data) {
  // mpjpeg2sd: modified to generate 24 bit RGB or 8 bit grayscale
  rgb_jpg_decoder * jpeg = (rgb_jpg_decoder *)arg;
  if (!data){
    if (x == 0 && y == 0) {
      // write start
      jpeg->width = w;
      jpeg->height = h;
    } 
    return true;
  }

  size_t jw = jpeg->width*RGB888_BYTES;
  size_t t = y * jw;
  size_t b = t + (h * jw);
  size_t l = x * RGB888_BYTES;
  uint8_t *out = jpeg->output+jpeg->data_offset;
  uint8_t *o = out;
  size_t iy, ix;
  w *= RGB888_BYTES;

  for (iy=t; iy<b; iy+=jw) {
    o = out+(iy+l)/stride;
    for (ix=0; ix<w; ix+=RGB888_BYTES) {
      if (colorDepth == RGB888_BYTES) {
        o[ix] = data[ix+2];
        o[ix+1] = data[ix+1];
        o[ix+2] = data[ix];
      } else {
        uint16_t grayscale = (data[ix+2]+data[ix+1]+data[ix])/RGB888_BYTES;
        o[ix/RGB888_BYTES] = (uint8_t)grayscale;
      }
    }
    data+=w;
  }
  return true;
}

static unsigned int _jpg_read(void * arg, size_t index, uint8_t *buf, size_t len) {
  rgb_jpg_decoder * jpeg = (rgb_jpg_decoder *)arg;
  if (buf) memcpy(buf, jpeg->input + index, len);
  return len;
}

static bool jpg2rgb(const uint8_t* src, size_t src_len, uint8_t* out, uint8_t scale) {
  rgb_jpg_decoder jpeg;
  jpeg.width = 0;
  jpeg.height = 0;
  jpeg.input = src;
  jpeg.output = out;
  jpeg.data_offset = 0;
  esp_err_t res = esp_jpg_decode(src_len, (jpg_scale_t)scale, _jpg_read, _rgb_write, (void*)&jpeg);
  if (res != ESP_OK) LOG_WRN("jpg2rgb failure: %s", espErrMsg(res)); 
  return (res == ESP_OK) ? true : false;
}

#endif // ESP_ARDUINO_VERSION

//#endif // INCLUDE_NEW_JPG

#else 
// dummies
bool isNight(uint8_t nightSwitch) {return false;}

#endif // AUXILIARY

