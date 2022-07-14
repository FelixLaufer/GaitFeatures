#ifndef _FSM_PROCESSOR_H_
#define _FSM_PROCESSOR_H_

#include "DynamicSizeRingbuffer.h"

template <typename TIn, typename TStore, typename TOut, typename TState, typename TEvent>
class FSMProcessor
{
public:
  FSMProcessor(const size_t maxBufferSize = 1000)
    : state_(TState())
    , buffer_(DynamicSizeRingbuffer<TStore>(maxBufferSize))
    , states_(DynamicSizeRingbuffer<TState>(maxBufferSize))
    , events_(DynamicSizeRingbuffer<TEvent>(maxBufferSize))
    , framesProcessed_(0)
  {}

  virtual ~FSMProcessor()
  {}

  TState state() const { return state_; }

  std::vector<TState> recentStates(const size_t windowSize, const bool recentFirst = false) const
  {
    std::vector<TState> ret;
    for (int f = states_.size() - windowSize; f < states_.size(); ++f)
      ret.emplace_back(states_[f]);

    if (recentFirst)
      std::reverse(ret.begin(), ret.end());

    return ret;
  }

  TState recentState() const { return recentStates(1, true)[0]; }

  std::vector<TEvent> recentEvents(const size_t windowSize, const bool recentFirst = false) const
  {
    std::vector<TEvent> ret;
    for (int f = events_.size() - windowSize; f < events_.size(); ++f)
      ret.emplace_back(events_[f]);

    if (recentFirst)
      std::reverse(ret.begin(), ret.end());

    return ret;
  }

  TEvent recentEvent() const { return recentEvents(1, true)[0]; }

  std::vector<TStore>recent(const size_t windowSize, const bool recentFirst = false) const
  {
    std::vector<TStore> ret;
    for (int f = buffer_.size() - windowSize; f < buffer_.size(); ++f)
      ret.emplace_back(buffer_[f]);

    if (recentFirst)
      std::reverse(ret.begin(), ret.end());

    return ret;
  }

  virtual TOut process(const TIn& data) = 0;

protected:
  TState state_;
  DynamicSizeRingbuffer<TStore> buffer_;
  DynamicSizeRingbuffer<TState> states_;
  DynamicSizeRingbuffer<TEvent> events_;
  size_t framesProcessed_;
};

#endif
