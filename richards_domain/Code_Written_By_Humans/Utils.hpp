#include <cstdint>
namespace Utils {

    uint32_t getBitMask(uint32_t lowerBoundInclusive, uint32_t upperBoundExclusive) {
        // TODO: docs, I'm too tired right now
        uint32_t maskSize = upperBoundExclusive - lowerBoundInclusive;
        if (maskSize >= 32) {
            // avoid undefined behaviour of bit shifts
            // TODO: document
            uint32_t allZeros = 0;
            uint32_t allOnes = ~allZeros;
            return allOnes;
        }

        uint32_t one = 1;
        uint32_t fencedOffZeros = (one << maskSize);
        uint32_t maskAtLeastSigBit = (fencedOffZeros - one);
        uint32_t maskInCorrectPlace = (maskAtLeastSigBit << lowerBoundInclusive); 
        return maskInCorrectPlace;
    }
}