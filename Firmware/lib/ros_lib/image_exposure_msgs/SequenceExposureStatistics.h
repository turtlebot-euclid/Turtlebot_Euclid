#ifndef _ROS_image_exposure_msgs_SequenceExposureStatistics_h
#define _ROS_image_exposure_msgs_SequenceExposureStatistics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "image_exposure_msgs/ImageExposureStatistics.h"

namespace image_exposure_msgs
{

  class SequenceExposureStatistics : public ros::Msg
  {
    public:
      uint8_t exposures_length;
      image_exposure_msgs::ImageExposureStatistics st_exposures;
      image_exposure_msgs::ImageExposureStatistics * exposures;
      uint32_t underExposed;
      uint32_t overExposed;
      float meanIrradiance;
      float minMeasurableIrradiance;
      float maxMeasurableIrradiance;
      float minObservedIrradiance;
      float maxObservedIrradiance;

    SequenceExposureStatistics():
      exposures_length(0), exposures(NULL),
      underExposed(0),
      overExposed(0),
      meanIrradiance(0),
      minMeasurableIrradiance(0),
      maxMeasurableIrradiance(0),
      minObservedIrradiance(0),
      maxObservedIrradiance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = exposures_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < exposures_length; i++){
      offset += this->exposures[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->underExposed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->underExposed >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->underExposed >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->underExposed >> (8 * 3)) & 0xFF;
      offset += sizeof(this->underExposed);
      *(outbuffer + offset + 0) = (this->overExposed >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->overExposed >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->overExposed >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->overExposed >> (8 * 3)) & 0xFF;
      offset += sizeof(this->overExposed);
      offset += serializeAvrFloat64(outbuffer + offset, this->meanIrradiance);
      offset += serializeAvrFloat64(outbuffer + offset, this->minMeasurableIrradiance);
      offset += serializeAvrFloat64(outbuffer + offset, this->maxMeasurableIrradiance);
      offset += serializeAvrFloat64(outbuffer + offset, this->minObservedIrradiance);
      offset += serializeAvrFloat64(outbuffer + offset, this->maxObservedIrradiance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t exposures_lengthT = *(inbuffer + offset++);
      if(exposures_lengthT > exposures_length)
        this->exposures = (image_exposure_msgs::ImageExposureStatistics*)realloc(this->exposures, exposures_lengthT * sizeof(image_exposure_msgs::ImageExposureStatistics));
      offset += 3;
      exposures_length = exposures_lengthT;
      for( uint8_t i = 0; i < exposures_length; i++){
      offset += this->st_exposures.deserialize(inbuffer + offset);
        memcpy( &(this->exposures[i]), &(this->st_exposures), sizeof(image_exposure_msgs::ImageExposureStatistics));
      }
      this->underExposed =  ((uint32_t) (*(inbuffer + offset)));
      this->underExposed |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->underExposed |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->underExposed |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->underExposed);
      this->overExposed =  ((uint32_t) (*(inbuffer + offset)));
      this->overExposed |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->overExposed |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->overExposed |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->overExposed);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->meanIrradiance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->minMeasurableIrradiance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->maxMeasurableIrradiance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->minObservedIrradiance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->maxObservedIrradiance));
     return offset;
    }

    const char * getType(){ return "image_exposure_msgs/SequenceExposureStatistics"; };
    const char * getMD5(){ return "2a4f3187c50e7b3544984e9e28ce4328"; };

  };

}
#endif