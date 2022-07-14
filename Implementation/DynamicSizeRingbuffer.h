#ifndef _DYNAMIC_SIZE_RINGBUFFER_H_
#define _DYNAMIC_SIZE_RINGBUFFER_H_

#include <vector>
#include <cassert>

template <typename T>
class DynamicSizeRingbuffer : public std::vector<T>
{
public:
  DynamicSizeRingbuffer(unsigned int initialSize = 1) : std::vector<T>(), writeIdx_(0)
  {
    std::vector<T>::assign((std::max)(initialSize, 1u), T());
  }

  virtual ~DynamicSizeRingbuffer() {}

  inline void resize(const unsigned int newSize)
  {
    assert(newSize > 0);
    if (newSize == size())
      return;

    std::vector<T> tmp = *this;
    const unsigned int oldSize = size();

    std::vector<T>::resize(newSize, T());

    const unsigned int toCopy = std::min(newSize, static_cast<unsigned int>(oldSize));
    const unsigned int startOffset = writeIdx_+(oldSize-toCopy);
    for (unsigned int i = 0; i < toCopy; ++i)
      this->std::vector<T>::operator[](i) = (tmp[(startOffset+i)%tmp.size()]);

    writeIdx_ = toCopy;
  }

  void push(const T& t)
  {
    this->std::vector<T>::operator[](writeIdx_) = t;
    writeIdx_++;
    writeIdx_ %= size();
  }

  inline unsigned int size() const
  {
    return std::vector<T>::size();
  }

  inline const T& operator[](unsigned int idx) const
  {
    return this->at((writeIdx_+idx)%size());
  }

  inline T& operator[](unsigned int idx)
  {
    return this->at((writeIdx_+idx)%size());
  }

private:
    unsigned int writeIdx_;
};

#endif