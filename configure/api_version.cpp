// \file  api_version.cpp

#include <string>
#include <cstdio>
#include <ueye.h>

int main ( int argc, char** argv ) {

    if ( argc > 1 && std::string(argv[1]) == "-h") {
        printf("Outputs ueye api version on stdout (format: Mmmm,  M for major, m for minor).\n");
        printf("Option -d is for a more readable output.\n");
        return 0;
    }

    bool debug = argc > 1  && std::string(argv[1]) == "-d";

    INT version = is_GetDLLVersion();

    int major = ( version >> 24 ) & 0xFF;
    int minor = ( version >> 16 ) & 0xFF;
    int build = version & 0XFF;

    if ( debug ) {
       printf("Ueye-API Version: %d.%d\n", major, minor);
       printf("Build: %d\n", build);
    } else
        printf("%d%03d",major, minor);

    return 0;
}
