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

      int32_t range = mMax > -mMin ? mMax : -mMin;
      int16_t height = bottom - top;

      lcd->rectClipped (COL_GREY, 0, top + height/2, lcd->getWidth(), 1);
      for (int i = 1000; i < mMax; i += 1000)
        lcd->rectClipped (COL_GREY,
                          0, top + height/2 - ((i * height) / (range * 2)), lcd->getWidth(), 1);
      for (int i = -1000; i > mMin; i -= 1000)
        lcd->rectClipped (COL_GREY,
                          0, top + height/2 - ((i * height) / (range * 2)), lcd->getWidth(), 1);

      int32_t sample = mCurSample - (lcd->getWidth() * mAverageSamples);
      for (int i = 0; i < lcd->getWidth(); i++) {
        int32_t value = 0;
        for (int j = 0; j < mAverageSamples; j++)
          value += sample+j > 0 ? mSamples[(sample+j) % mNumSamples] : 0;
        value = (value * height) / (range * 2 * mAverageSamples);
        if (value > 0)
          lcd->rectClipped (COL_WHITE, i, top + height/2 - value, 1, value);
        else
          lcd->rectClipped (COL_WHITE, i, top + height/2, 1, -value);
        sample += mAverageSamples;
        }

      lcd->text (COL_YELLOW, cWidget::getFontHeight(), dec(mSamples[(mCurSample-1) % mNumSamples],4),
                 lcd->getWidth() - 60, top + height/2, 60, cWidget::getFontHeight());
      lcd->text (COL_GREEN, cWidget::getFontHeight(), dec(mMax,4),
                 lcd->getWidth() - 60, top + height/2 - cWidget::getFontHeight(), 60, cWidget::getFontHeight());
      lcd->text (COL_RED, cWidget::getFontHeight(), dec(mMin,4),
                 lcd->getWidth() - 60, top + height/2 + cWidget::getFontHeight(), 60, cWidget::getFontHeight());
      }
    //}}}

  private:
    int16_t mNumSamples = 0;
    int16_t mAverageSamples = 1;
    int16_t* mSamples = nullptr;
    int32_t mCurSample = 0;
    int16_t mMin = 0;
    int16_t mMax = 0;
    };

  std::vector<cTrace*> mTraces;
  };
