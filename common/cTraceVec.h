// cTrace.h
#pragma once
#include <vector>
#include "cLcd.h"

class cTraceVec {
public:
  //{{{
  void addTrace (int16_t numSamples, int16_t averageSamples, int16_t chans) {
    for (int i = 0; i < chans; i++)
      mTraces.push_back (new cTrace (numSamples, averageSamples));
    }
  //}}}
  //{{{
  void addSample (int16_t trace, int16_t value) {
    if (trace < mTraces.size())
      mTraces[trace]->addSample (value);
    }
  //}}}
  //{{{
  void trigger() {
    for (auto trace : mTraces)
      trace->trigger();
    }
  //}}}
  //{{{
  void draw (cLcd* lcd, int16_t top, int16_t bottom) {

    int16_t height = (bottom - top - mTraces.size()) / mTraces.size();
    for (auto trace : mTraces) {
      trace->draw (lcd, top, top + height);
      top += height + 1;
      }
    }
  //}}}

private:
  //{{{
  class cTrace {
  public:
    //{{{
    cTrace (int16_t numSamples, int16_t averageSamples)
       : mNumSamples(numSamples), mAverageSamples(averageSamples) {

      mSamples = (int16_t*)malloc (numSamples*2);
      memset (mSamples, 0, numSamples*2);
      }
    //}}}

    //{{{
    void addSample (int16_t value) {

      if (value < mMin)
        mMin = value;
      if (value > mMax)
        mMax = value;

      mSamples[mCurSample++ % mNumSamples] = value;
      }
    //}}}
    //{{{
    void trigger() {

      mCurSample = 0;
      }
    //}}}
    //{{{
    void draw (cLcd* lcd, int16_t top, int16_t bottom) {

      int32_t range = mMax - mMin;
      int16_t height = bottom - top;

      int32_t sample = mCurSample - (lcd->getWidth() * mAverageSamples);
      for (int i = 0; i < lcd->getWidth(); i++) {
        int32_t value = 0;
        for (int j = 0; j < mAverageSamples; j++)
          value += sample+j > 0 ? mSamples[(sample+j) % mNumSamples] : 0;
        value = (value * height) / (range * mAverageSamples);
        if (value > 0)
          lcd->rectClipped (COL_WHITE, i, top + height/2 - value, 1, value);
        else
          lcd->rectClipped (COL_WHITE, i, top + height/2, 1, -value);
        sample += mAverageSamples;
        }
      }
    //}}}

  private:
    int16_t mNumSamples = 0;
    int16_t mAverageSamples = 1;
    int16_t* mSamples = nullptr;
    int32_t mCurSample = 0;
    int16_t mMin = 0x7FFF;
    int16_t mMax = -0x7FFF;
    };
  //}}}
  std::vector<cTrace*> mTraces;
  };
