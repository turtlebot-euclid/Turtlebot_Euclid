#ifndef _ROS_image_exposure_msgs_ImageExposureStatistics_h
#define _ROS_image_exposure_msgs_ImageExposureStatistics_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "statistics_msgs/Stats1D.h"

namespace image_exposure_msgs
{

  class ImageExposureStatistics : public ros::Msg
  {
    public:
      ros::Time stamp;
      const char* time_reference;
      float shutterms;
      float gaindb;
      uint32_t underExposed;
      uint32_t overExposed;
      statistics_msgs::Stats1D pixelVal;
      statistics_msgs::Stats1D pixelAge;
      float meanIrradiance;
      float minMeasurableIrradiance;
      float maxMeasurableIrradiance;
      float minObservedIrradiance;
      float maxObservedIrradiance;

    ImageExposureStatistics():
      stamp(),
      time_reference(""),
      shutterms(0),
      gaindb(0),
      underExposed(0),
      overExposed(0),
      pixelVal(),
      pixelAge(),
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
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      uint32_t length_time_reference = strlen(this->time_reference);
      memcpy(outbuffer + offset, &length_time_reference, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->time_reference, length_time_reference);
      offset += length_time_reference;
      union {
        float real;
        uint32_t base;
      } u_shutterms;
      u_shutterms.real = this->shutterms;
      *(outbuffer + offset + 0) = (u_shutterms.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_shutterms.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_shutterms.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_shutterms.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->shutterms);
      union {
        float real;
        uint32_t base;
      } u_gaindb;
      u_gaindb.real = this->gaindb;
      *(outbuffer + offset + 0) = (u_gaindb.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gaindb.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gaindb.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gaindb.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gaindb);
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
      offset += this->pixelVal.serialize(outbuffer + offset);
      offset += this->pixelAge.serialize(outbuffer + offset);
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
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      uint32_t length_time_reference;
      memcpy(&length_time_reference, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_time_reference; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_time_reference-1]=0;
      this->time_reference = (char *)(inbuffer + offset-1);
      offset += length_time_reference;
      union {
        float real;
        uint32_t base;
      } u_shutterms;
      u_shutterms.base = 0;
      u_shutterms.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_shutterms.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_shutterms.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_shutterms.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->shutterms = u_shutterms.real;
      offset += sizeof(this->shutterms);
      union {
        float real;
        uint32_t base;
      } u_gaindb;
      u_gaindb.base = 0;
      u_gaindb.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gaindb.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gaindb.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gaindb.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gaindb = u_gaindb.real;
      offset += sizeof(this->gaindb);
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
      offset += this->pixelVal.deserialize(inbuffer + offset);
      offset += this->pixelAge.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->meanIrradiance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->minMeasurableIrradiance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->maxMeasurableIrradiance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->minObservedIrradiance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->maxObservedIrradiance));
     return offset;
    }

    const char * getType(){ return "image_exposure_msgs/ImageExposureStatistics"; };
    const char * getMD5(){ return "334dc170ce6287d1de470f25be78dd9e"; };

  };

}
#endif