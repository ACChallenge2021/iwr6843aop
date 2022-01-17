#ifndef SRC_TRIPLEBUFFER_H_
#define SRC_TRIPLEBUFFER_H_

#include <array>
#include <atomic>

namespace kria
{

    template<typename T>
    class TripleBuffer
    {
        public:
            TripleBuffer<T> ();

            // Get the current value to read
            T& read ();
            const T&read () const;

            // Write a new value
            void write (const T &newT);

            // Swap to the latest value for the consumer if any
            bool swapReadBuffer ();

            // Swap writer positions for producer and wait buffer
            void swapWriteBuffer ();

            // Wrapper to read the last available element (swapReadBuffer + read)
            T& readLast ();
            const T& readLast () const;

            // Wrapper to update with a new element (write + swapWriteBuffer)
            void update (const T &newT);

            // Get the current value to write to
            T& currentWriteValue ();
            const T& currentWriteValue () const;

        private:
            // Disable copyable behaviour
            TripleBuffer<T> (const TripleBuffer<T>&);
            TripleBuffer<T>& operator= (const TripleBuffer<T>&);

            // Check if the newWrite bit is 1
            bool isNewWrite (uint_fast8_t flags) const;

            // Swap consumer and wait buffer indexes
            uint_fast8_t swapReadWithWaitBuffer (uint_fast8_t flags);

            // Set newWrite to 1 and swap wait and producer buffer indexes
            uint_fast8_t newWriteSwapWaitWithWrite (uint_fast8_t flags);

            // 8 bit flags are (unused) (new write) (2x producer) (2x wait) (2x consumer)
            // new write flag  = (flags & 0x40)
            // producer buffer index = (flags & 0x30) >> 4
            // wait buffer index = (flags & 0xC) >> 2
            // consumer buffer index  = (flags & 0x3)
            mutable std::atomic_uint_fast8_t flags;

            std::array<T, 3> m_buffer;
    };



    template <typename T>
    inline TripleBuffer<T>::TripleBuffer()
        : m_buffer()
    {
        // Initially producer = 0, wait = 1 and consumer = 2
        flags.store(0x6, std::memory_order_relaxed);
    }

    template <typename T>
    inline T& TripleBuffer<T>::read()
    {
        // Read consumer m_buffer
        return m_buffer[flags.load(std::memory_order_consume) & 0x3];
    }

    template <typename T>
    inline const T& TripleBuffer<T>::read() const
    {
        // Read consumer m_buffer
        return m_buffer[flags.load(std::memory_order_consume) & 0x3];
    }

    template <typename T>
    inline void TripleBuffer<T>::write(const T& newT)
    {
        // Write into producer buffer
        m_buffer[(flags.load(std::memory_order_consume) & 0x30) >> 4] = newT;
    }



    template <typename T>
    inline T& TripleBuffer<T>::currentWriteValue()
    {
        // Return current producer buffer
        return m_buffer[(flags.load(std::memory_order_consume) & 0x30) >> 4];
    }



    template <typename T>
    inline const T& TripleBuffer<T>::currentWriteValue() const
    {
        // Return current producer buffer
        return m_buffer[(flags.load(std::memory_order_consume) & 0x30) >> 4];
    }



    template <typename T>
    inline bool TripleBuffer<T>::swapReadBuffer()
    {
        uint_fast8_t flagsNow(flags.load(std::memory_order_consume));

        do
        {
            // Nothing new, no need to swap
            if (!isNewWrite(flagsNow))
                return false;
        }
        while (!flags.compare_exchange_weak(flagsNow,
                                            swapReadWithWaitBuffer(flagsNow),
                                            std::memory_order_release,
                                            std::memory_order_consume));

        return true;
    }

    template <typename T>
    inline void TripleBuffer<T>::swapWriteBuffer()
    {

        uint_fast8_t flagsNow(flags.load(std::memory_order_consume));

        while (!flags.compare_exchange_weak(flagsNow,
                                            newWriteSwapWaitWithWrite(flagsNow),
                                            std::memory_order_release,
                                            std::memory_order_consume));
    }


    template <typename T>
    inline T& TripleBuffer<T>::readLast()
    {
        // Get most recent value
        swapReadBuffer();

        // Return value
        return read();
    }


    template <typename T>
    inline const T& TripleBuffer<T>::readLast() const
    {
        // Get most recent value
        swapReadBuffer();

        // Return value
        return read();
    }

    template <typename T>
    inline void TripleBuffer<T>::update(const T& newT)
    {
        // Write new value
        write(newT);

        // Swap producer/wait buffer positions for the next update
        swapWriteBuffer();
    }

    template <typename T>
    inline bool TripleBuffer<T>::isNewWrite(uint_fast8_t flags) const
    {
        // Check if the newWrite bit is 1
        return ((flags & 0x40) != 0);
    }

    template <typename T>
    inline uint_fast8_t TripleBuffer<T>::swapReadWithWaitBuffer(uint_fast8_t flags)
    {
        // Swap consumer with wait buffer
        return (flags & 0x30) | ((flags & 0x3) << 2) | ((flags & 0xC) >> 2);
    }

    template <typename T>
    inline uint_fast8_t TripleBuffer<T>::newWriteSwapWaitWithWrite(uint_fast8_t flags)
    {
        // Set newWrite bit to 1 and swap clean with dirty
        return 0x40 | ((flags & 0xC) << 2) | ((flags & 0x30) >> 2) | (flags & 0x3);
    }

} /* namespace kria */

#endif /* SRC_TRIPLEBUFFER_H_ */
