#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

// I needed to define a seperate header file for MainProMax.cpp
// because the C++ includes in that file (like #include <string>) were getting
// seen by C compiler at some point, and C would complain that those libs didn't exist!
// The solution was to make a header and include that, so C only ever saw the declaration
// of the function, and the actual definition is forced to be linked in later after compilation.
// 
// See also:
// https://stackoverflow.com/questions/43602910/extern-c-causing-an-error-expected-before-string-constant
// https://peterdaugaardrasmussen.com/2025/06/28/how-to-use-cpp-with-stm32-in-cube-ide/
// https://stackoverflow.com/questions/856636/effects-of-the-extern-keyword-on-c-functions
// https://stackoverflow.com/questions/1041866/what-is-the-effect-of-extern-c-in-c
// https://stackoverflow.com/questions/tagged/extern-c
// https://learn.microsoft.com/en-us/cpp/cpp/header-files-cpp?view=msvc-170
// 
// 
// 
//
void mainProMax();

#ifdef __cplusplus
}
#endif