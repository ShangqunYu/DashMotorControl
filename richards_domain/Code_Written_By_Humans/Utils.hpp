#include <cstdint>
namespace Utils {

    uint32_t getBitMask(uint32_t lowerBoundInclusive, uint32_t upperBoundExclusive) {
        // Generates a bit-mask that can be used to extract a consecutive string
        // of bits from a uint32_t. For the purposes of this function, the index
        // of a given bit corresponds to the power of 2 that the bit represents
        // within the number. For example:
        //
        //      most sig bit          least sig bit
        //                |             |
        //    bit string: 0 1 0 1 0 1 0 1
        //         index: 7 6 5 4 3 2 1 0
        // 
        // I appologize to / empathize with those of you that instincively imagine
        // the '0'th element in a list being all the way on the left, but this way
        // of indexing bits seems to more cleanly map onto the intended use cases!


        uint32_t maskSize = upperBoundExclusive - lowerBoundInclusive;
        if (maskSize >= uint32_t(32)) {
            // Bit shifts become undefined behaviour when shifting by 
            // any amount that's equal to (or greater than) the number
            // of bits in the datatype. As such, we check for the special
            // case of wanting to mask every bit to avoid undefined behaviour
            // in the bit shifts we use for the general case below.

            // When masking every bit, we just want a uint32_t that has every
            // bit set to 1. We start by getting a uint32_t with every bit set to zero,
            // and then use the bitwise-not operator '~' to flip all those zeros to ones.
            return ~uint32_t(0);
        }

        // I think its easiest to see how this algorithm works by considering
        // an example. Let's consider how we would generate the following
        // bit mask on a uint8_t:
        // 
        // bit mask: 0 0 0 1 1 1 0 0
        //    index: 7 6 5 4 3 2 1 0
        //
        // This bit mask has:
        //   lowerBoundInclusive = 2;
        //   upperBoundExclusive = 5;
        //   maskSize = 3;
        //
        // We start by taking the number 1 (whose bit-string only has the rightmost bit set)
        // and shifting it to the left by 'maskSize', and then subtracting 1 from the resulting
        // number to get a corrctly sized string of 1's. Applying this to our example:
        // 
        //    1                    -->   0 0 0 0 0 0 0 1  // Start with the bit-string for '1',
        //   (1 << maskSize)       -->   0 0 0 0 1 0 0 0  // then shift by maskSize,
        //  ((1 << maskSize) - 1)  -->   0 0 0 0 0 1 1 1  // then subtract 1.
        //                               7 6 5 4 3 2 1 0
        //                                ^^^indexes^^^
        //
        // You can think about that last step as essentially "undoing" all the carries
        // that cascade together from adding 1 to a number that's [one less than a power
        // of two].
        //
        // Once we've generated our appropriately sized string of 1's, the only thing left
        // to do is to shift them into place! Once again, applying to our example:
        //
        //  ((1 << maskSize) - 1)                         -->   0 0 0 0 0 1 1 1  // Pick up where we left off,
        //  ((1 << maskSize) - 1) << lowerBoundInclusive  -->   0 0 0 1 1 1 0 0  // and shift into place!
        //                                                      7 6 5 4 3 2 1 0
        //                                                       ^^^indexes^^^
        //
        // And that matches the exact bit mask we set out to create, so we're done!
        // Hope that helps!
        uint32_t fencedOffZeros = (uint32_t(1) << maskSize);
        uint32_t maskAtLeastSigBit = (fencedOffZeros - uint32_t(1));
        uint32_t maskInCorrectPlace = (maskAtLeastSigBit << lowerBoundInclusive); 
        return maskInCorrectPlace;

        // Note: I've explicitly casted all integer literals to uint32_t to avoid
        //       any wonky behaviour that may result from C++ auto converting
        //       to a signed type anywhere, which would likely lead to
        //       incorrect results!
    }
}