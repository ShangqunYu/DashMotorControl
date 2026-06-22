#include "MainProMax.hpp"
#include "CanBus.hpp"

extern "C" void mainProMax() {
    CanBus testBus = CanBus();
    testBus.sayHello();
}

// General Notes on Programming for STM32:
//
// 1) The primary entry point for programming the STM32 (at least for n00bs like us) is a program called
//    "STM32 CubeMX" that's provided by the company that makes the STM32 (the company itself is just called "ST").
//    It's a GUI for configuring all the I/O pins on the microcontroller, and then generating the corresponding
//    stater code that will actually do that configuration for you once uploaded to the device.
//    More deets here: https://www.st.com/content/st_com/en/stm32cubemx.html
//
//    Note that in the project setup wizard, they'll likely ask you for the exact "breed" of STM32 that you're working
//    with. Apparently there are TONS of different little variations that each provide a slightly different set of features.
//    The specifc one we're using is the STM32F446RE. Product page is here:
//    https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32f4-series/stm32f446/stm32f446re.html
//
// 2) Once you've got your auto-generated starter code, you can then open it up in your text editor of choice,
//    but there are a couple of "officially supported" options that are worth considering. The first is an
//    IDE provided by ST themselves, called "STM32 CubeIDE" (no "MX" this time, still plenty of "Cube" though).
//    The other option (which is the one we went with) is to just use VSCode, and install the ST-provided
//    "STM32 CubeIDE for VSCode" extension. Setup instructions and additional usage info can be found here:
//    https://www.youtube.com/watch?v=aWMni01XGeI
//    https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/1.0.1/en/docs/markup/getting_started/first_project_creation.html
//
// 3) The last (and most important!) thing you'll need is the documentation / set of APIs for each peripheral you...
//    ugh, I'll come back to this.
// 
// 1) The specific version we're using is the STM32F446RE
//    Product Page: https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32f4-series/stm32f446/stm32f446re.html
// 
// 2) The main documentation (as far as programming is concerned) is in the "Reference Manual", not the "User Manual"!
//    Link: https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
//    More details on how to use and read this reference manual (and some additional sources of programming documentation)
//    are outlined below, I just wanted to put the links for everything at the top.
//
// 

// https://dev.st.com/stm32cube-docs/stm32cubeide-vscode/1.0.1/en/docs/markup/basic_concepts/cmake.html#how-to-add-include-files-and-folders
// https://community.st.com/stm32-mcus-60/cmake-integration-in-stm32cubemx-and-usage-in-stm32cubeide-for-visual-studio-code-158397
// https://www.reddit.com/r/embedded/comments/18g3aae/stm32_learning_curve_for_beginners/
//
//
//
// We're using C++17 as of the last time I checked:
//
// https://stackoverflow.com/questions/49915424/what-does-the-cplusplus-macro-expand-to
// #if __cplusplus >= 202302L
//     #pragma message("Notice: Compiling with C++23 (or newer)")
// #elif __cplusplus >= 202002L
//     #pragma message("Notice: Compiling with C++20")
// #elif __cplusplus >= 201703L
//     #pragma message("Notice: Compiling with C++17")
// #elif __cplusplus >= 201402L
//     #pragma message("Notice: Compiling with C++14")
// #elif __cplusplus >= 201103L
//     #pragma message("Notice: Compiling with C++11")
// #else
//     #pragma message("Notice: Compiling with C++98 / C++03")
// #endif