#include <iostream>
#include <initializer_list>

template <typename T>
void log(T t) 
{
    std::cout << t << std::endl ;
}

template<typename T, typename... Args>
void log(T t, Args... args) // recursive variadic function
{
    std::cout << t ;
    
    log(args...) ;
}