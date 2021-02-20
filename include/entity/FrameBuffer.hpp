#ifndef INCLUDE_SCAN_FRAME_BUFFER_HPP
#define INCLUDE_SCAN_FRAME_BUFFER_HPP

#include <vector>
#include <cstdint>
#include <exception>
#include <iostream>
#include "Frame.hpp"

namespace Pharao
{

struct FrameBuffer
{
public:
    struct Iterator
    {
        friend struct FrameBuffer;
    public:
        Iterator()
        {
        }

        ~Iterator()
        {            
        }

        Frame& operator*() const
        {
            return *mIterator;
        }

        Frame* operator->() const
        {
            return &(*mIterator);
        }

        bool operator ==(const Iterator& other) const
        {
            return mIterator == other.mIterator;
        }

        bool operator !=(const Iterator& other) const
        {
            return !(other == *this);
        }

        Iterator& operator++(void)
        {
            if (mIterator == mBack)
            {
                mIterator = mEnd;
            }
            else
            {
                mIterator = mIterator == mEnd - 1 ? mBegin : mIterator + 1;
            }
            return *this;
        }

        Iterator operator++(int)
        {
            Iterator it = *this;

            if (mIterator == mBack)
            {
                mIterator = mEnd;
            }
            else
            {
                mIterator = mIterator == mEnd - 1 ? mBegin : mIterator + 1;
            }

            return it;
        }

        friend std::ostream& operator<<(std::ostream& os, const Iterator& iter)
        {
            os << "[" << static_cast<int>(iter.mIterator - iter.mBegin)
               << "," << static_cast<int>(iter.mEnd - iter.mIterator)
               << "," << static_cast<int>(iter.mBack - iter.mIterator) << "]";
            return os;
        }

    private:
        Iterator(const std::vector<Frame>::iterator& aIterator,
            const std::vector<Frame>::iterator& aBegin,
            const std::vector<Frame>::iterator& aEnd,
            const std::vector<Frame>::iterator& aBack)
            : mIterator(aIterator)
            , mBegin(aBegin)
            , mEnd(aEnd)
            , mBack(aBack)
        {
        }

        std::vector<Frame>::iterator mIterator;
        std::vector<Frame>::iterator mBegin;
        std::vector<Frame>::iterator mEnd;
        std::vector<Frame>::iterator mBack;
    };

    FrameBuffer()
    {
        mSize = 0;
        mFrontIter = mBuffer.end();
        mBackIter = mBuffer.end();
    }

    ~FrameBuffer()
    {
    }

    void set_capacity(const uint32_t aSize)
    {
        mBuffer.resize(aSize);
    }

    void set_capacity(const uint32_t aSize, const cv::Size& aCoarseCartesianSize,
        const cv::Size& aCoarseLogPolarSize, const cv::Size& aFineCartesianSize)
    {
        mBuffer.resize(aSize);
        for (auto& frame : mBuffer)
        {
            frame.mCoarseCartesianImage = cv::Mat(aCoarseCartesianSize, CV_32F);
            frame.mCoarseLogPolarImage = cv::Mat(aCoarseLogPolarSize, CV_32F);
            frame.mFineCartesianImage = cv::Mat(aFineCartesianSize, CV_32F);
        }
    }

    bool empty()
    {
        return mSize == 0;
    }

    bool full()
    {
        return mSize == mBuffer.size();
    }

    uint32_t size()
    {
        return mSize;
    }

    Iterator begin()
    {
        return Iterator(mFrontIter, mBuffer.begin(), mBuffer.end(), mBackIter);
    }

    Iterator end()
    {
        return Iterator(mBuffer.end(), mBuffer.begin(), mBuffer.end(), mBackIter);
    }

    Iterator iterator_at(int index)
    {
        if (index < 0 || mSize <= index)
        {
            std::cout << "Out of bounds access in frame buffer!" << std::endl;
            throw std::exception("Out of bounds access!");            
        }

        Iterator it(mFrontIter, mBuffer.begin(), mBuffer.end(), mBackIter);
        for (; index > 0; --index, ++it) ;
        return it;
    }

    Frame& front()
    {
        return *mFrontIter;
    }

    Frame& back()
    {
        return *mBackIter;
    }

    Frame& operator[](int index)
    {
        if (index < 0 || mSize <= index)
        {
            std::cout << "Out of bounds access in frame buffer!" << std::endl;
            throw std::exception("Out of bounds access!");
        }

        int diff = mBuffer.end() - mFrontIter;
        if (index >= diff)
        {
            int remaining = index - diff;
            return *(mBuffer.begin() + remaining);
        }
        else
        {
            return *(mFrontIter + index);
        }
    }

    void push_back(bool overwrite = false)
    {
        if (mSize == 0)
        {
            // When size == 0 all iterators are set to end()
            mFrontIter = mBuffer.begin();
            mBackIter = mBuffer.begin();
            ++mSize;
        }
        else if (mSize == mBuffer.size())
        {
            // Full, so we have to overwrite front iterator and shift it back
            next(mFrontIter);
            next(mBackIter);
        }
        else
        {
            next(mBackIter);
            ++mSize;
        }

        if (overwrite)
        {
            mBackIter->mCoarseCartesianImage.setTo(cv::Scalar(0));
            mBackIter->mCoarseLogPolarImage.setTo(cv::Scalar(0));
            mBackIter->mFineCartesianImage.setTo(cv::Scalar(0));
        }
    }

    void pop_front()
    {
        if (mSize > 1)
        {
            next(mFrontIter);
            --mSize;
        }
        else if (mSize == 1)
        {
            // We set all iterators to be end() when size == 0 after popping
            --mSize;
            mFrontIter = mBuffer.end();
            mBackIter = mBuffer.end();
        }
        // else mSize == 0 and nothing happens
    }

    void pop_back()
    {
        if (mSize > 1)
        {
            prev(mBackIter);
            --mSize;
        }
        else if (mSize == 1)
        {
            // We set all iterators to be end() when size == 0 after popping
            --mSize;
            mFrontIter = mBuffer.end();
            mBackIter = mBuffer.end();
        }
        // else mSize == 0 and nothing happens
    }

    void clear_until_next_keyframe()
    {
        // Implicitly assumes that first element of the buffer is the old keyframe,
        // which should be deleted regardless. Hence if mSize == 1 it will empty
        // the buffer and otherwise it will delete the old keyframe and delete
        // all elements in the buffer until it finds the next keyframe.
        if (mSize > 1)
        {
            next(mFrontIter);
            --mSize;

            while (mFrontIter != mBackIter && mFrontIter->mType != FrameType::KEYFRAME)
            {
                next(mFrontIter);
                --mSize;
            }

            if (mFrontIter == mBackIter && mFrontIter->mType != FrameType::KEYFRAME)
            {
                --mSize;
                mFrontIter = mBuffer.end();
                mBackIter = mBuffer.end();
            }
        }
        else if (mSize == 1)
        {
            --mSize;
            mFrontIter = mBuffer.end();
            mBackIter = mBuffer.end();
        }
    }

private:
    void next(std::vector<Frame>::iterator& it)
    {
        // check if we are at the end and wrap around
        if (it == mBuffer.end() - 1)
        {
            it = mBuffer.begin();
        }
        else
        {
            it++;
        }
    }

    void prev(std::vector<Frame>::iterator& it)
    {
        // check if we are at the beginning and wrap around
        if (it == mBuffer.begin())
        {
            it = mBuffer.end() - 1;
        }
        else
        {
            --it;
        }
    }

    std::vector<Frame> mBuffer;
    uint32_t mSize;
    std::vector<Frame>::iterator mFrontIter;
    std::vector<Frame>::iterator mBackIter;
};

}

#endif