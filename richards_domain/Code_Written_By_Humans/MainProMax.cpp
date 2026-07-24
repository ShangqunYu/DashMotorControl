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
// 3) The last (and most important!) thing you'll need before getting started is the documentation for each 
//    STM32 feature / peripheral that you want to use in your project. But before you go off and RTFM, you
//    should know that there are actually TWO primary sources of docs to be aware of, and that the one you reach
//    for depends on the level of abstraction you'd like to program with.
//
// 3) The last (and most important!) thing you'll need before getting started is the documentation for each 
//    STM32 feature / peripheral that you want to use in your project. But before you go off and RTFM, you
//    should know that there are actually TWO primary sources of docs to be aware of, 
//    depending on the level of abstraction you'd like to program with.
//
// 
// 3) The last (and most important!) thing you'll need before getting started is the documentation for each 
//    STM32 feature / peripheral that you want to use in your project. But before you go off and RTFM, you
//    should know that there are actually TWO primary sources of docs you should be aware of, 
//    depending on the level of abstraction you'd like to program with.
//
// 
// The first is
//    the "Reference Manual", which provides the lowest level of documentaiton, and answers the question of 
//    "How do I make X do Y?" with "Write <SOME SPECIFIC BIT STRING> to <SOME SPECIFIC REGISTER>".
//    It can be found here: https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
//
//    Higher level interfaces are provided by ST in the form of a library called the "HAL" (Hardware Abstraction Layer).
//    All higher-level interfaces are ultimately just wrappers that bottom out to these kinds of operations,
//    which I think is pretty cool! That being said, this project mostly uses the higher level interfaces
//    described in the next section, because they allow our code be;
//      - Easier to read (less squinting at bit shifts),
//    less error prone (don't need to keep track of as many hardware specific quirks), and more portable
//    (essentially little to no work if we decide to switch to a different breed of STM32 tomorrow).
//
//         - Improved readability.
//         - The ability to have semi-hardware-agnostic code
//           (at least with regards to different breeds of STM32 chips, which isn't much but its something).
//         - 
//      and avoiding 
//      
//      If you want to try programming this way, I think the only thing you need to do is to #include "stm32f446xx.h",
//      which is a ~15,000 line file specifying the address of each register you might need to write to / read from,
//      and then you should be good to go!
//      (If you want to check out the file yourself, it's in /STM32_CubeMX_Generated_Code/Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f446xx.h)
//      
//      
//      I haven't tried this myself yet, so therethough I haven't tried it myself, so there may be more to it. At the very least that's the starting point,
//      and I'd like to imagine it shouldn't be too much more work to get going with this route if you so choose.
// 
//      That being said, of We're using the slightly higher level interfaces described in the next paragraph, but if you want to try
//      programming at this levelThe documenation for the lowest (non-assembly) level of programming (upon which all higher level interfaces are built)
//           can be found in the "Reference Manual" for the specific breed of STM32 that you're using. Note that this
//           is different from the "User Manual" (which appears to just be a spec sheet)
//           
//
// 
// Because it would be too easy for us if
//    all the documenation was in one place, Depending on the level of abstraction
//    you'd like to program with, the docs can IThis documentaiton often comes in two
//    different levels depending on the level of abstraction that you'd like to program with:
// 
// n general, there are Broadly speaking, there are actually
//    a few different places to get the documentaiton for each pBecause they didn't want to make it
//    too easy for you, the documentation is actually spread out in a few different places
//    depending on the level of abstraction that you'd like to program with. There are actually a few different "levels"
//    of documentation that are provided depending on how high 
// 
// Unfortunately, it appears the documentation
//    is actually spread out in a few different places depending on the  documentation comes at a few different levels
//    of abstraction 
//    for each of the 
//    that you'll use for interacting with the STM32's various features / peripherals you want to use (along with any other supporting documentation). the interfaces / APIs and docs provided
//    for each of the STM32 features / peripherals you want to work with!
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