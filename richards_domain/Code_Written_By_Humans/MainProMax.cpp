#include "MainProMax.hpp"
#include "CanBus.hpp"

extern "C" void mainProMax() {
    CanBus testBus = CanBus();
    testBus.sayHello();
}

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